namespace BatteryWeldAOI.Core.Orchestration;

using System.Collections.Concurrent;
using BatteryWeldAOI.Core.Camera;
using BatteryWeldAOI.Core.Communication;
using BatteryWeldAOI.Core.Models;
using BatteryWeldAOI.Core.Pipeline;
using BatteryWeldAOI.Core.StateMachine;
using OpenCvSharp;

/// <summary>
/// 检测流程控制器（Stop-and-Go 编排核心）：
///   Idle -> [MoveTo -> 等待到位 -> Inspecting -> 拍照入队 -> Moving -> ...] -> Finished。
/// 图像入队后立刻移动到下一点，视觉在流水线中并行处理，
/// 单点节拍 ≈ 轴运动时间 + 曝光时间（而非 + 算法时间）。
/// 每包检测创建一个新实例（RunAsync 仅可调用一次）。
/// </summary>
public sealed class InspectionController
{
    private const int UploadRetryCount = 3;
    private static readonly TimeSpan UploadRetryDelay = TimeSpan.FromMilliseconds(100);
    private static readonly TimeSpan DrainTimeout = TimeSpan.FromSeconds(5);

    private readonly IPlcClient _plc;
    private readonly ICameraGrabber _camera;
    private readonly ImagePipeline _pipeline;
    private readonly MachineStateMachine _stateMachine;
    private readonly IReadOnlyList<WeldPoint> _points;
    private readonly ConcurrentBag<PointInspectionResult> _results = new();
    private readonly ConcurrentBag<int> _uploadFailedPoints = new();
    private readonly List<Task> _pendingUploads = new();
    private readonly object _uploadsLock = new();
    private readonly SemaphoreSlim _plcSendLock = new(1, 1);
    private int _runStarted;

    public event Action<string>? Log;

    /// <summary>单点拍照完成（已入流水线），用于界面标记"检测中"。</summary>
    public event Action<WeldPoint>? PointGrabbed;

    public InspectionController(IPlcClient plc, ICameraGrabber camera, ImagePipeline pipeline,
        MachineStateMachine stateMachine, IReadOnlyList<WeldPoint> points)
    {
        _plc = plc;
        _camera = camera;
        _pipeline = pipeline;
        _stateMachine = stateMachine;
        _points = points;
        _pipeline.ResultProcessed += OnResultProcessed;
    }

    /// <summary>
    /// 执行整包检测，返回报表。
    /// 被取消时停机（状态机进入 Stopped）并返回已完成点位的部分报表；
    /// 其他异常时状态机进入 Error 并重抛。
    /// </summary>
    public async Task<InspectionReport> RunAsync(CancellationToken ct)
    {
        if (Interlocked.Exchange(ref _runStarted, 1) == 1)
            throw new InvalidOperationException("RunAsync 只能调用一次，请为每包检测创建新的 InspectionController");

        var started = DateTime.UtcNow;
        try
        {
            await _plc.ConnectAsync(ct);
            Log?.Invoke("[流程] 已连接 PLC, 开始整包检测");

            foreach (var point in _points)
            {
                ct.ThrowIfCancellationRequested();

                if (!_stateMachine.TryTransition(MachineState.Moving))
                    throw new InvalidOperationException($"状态机拒绝进入 Moving（当前: {_stateMachine.Current}），中止以防撞机");

                await _plc.MoveToAsync(point.Index, ct);
                await _plc.WaitForArrivalAsync(point.Index, ct); // Stop：到位稳定

                if (!_stateMachine.TryTransition(MachineState.Inspecting))
                    throw new InvalidOperationException($"状态机拒绝进入 Inspecting（当前: {_stateMachine.Current}）");

                var image = _camera.TriggerGrab(point.Index); // Go：拍照
                _pipeline.Enqueue(point, image);               // 处理与运动解耦
                PointGrabbed?.Invoke(point);
            }

            if (!_stateMachine.TryTransition(MachineState.Finished))
                throw new InvalidOperationException($"状态机拒绝进入 Finished（当前: {_stateMachine.Current}）");

            await _plc.FinishAsync(ct);
            _pipeline.CompleteAdding();
            await _pipeline.Completion; // 等待流水线排空
            await FlushUploadsAsync();
            Log?.Invoke("[流程] 整包检测完成");

            return BuildReport(started, wasCancelled: false);
        }
        catch (OperationCanceledException)
        {
            _stateMachine.EmergencyStop(); // 任意状态 -> Stopped
            Log?.Invoke("[流程] 检测被操作员中止, 停机并输出部分结果");
            _pipeline.CompleteAdding();
            await DrainPipelineAsync();
            await FlushUploadsAsync();
            return BuildReport(started, wasCancelled: true);
        }
        catch (Exception)
        {
            _stateMachine.TryTransition(MachineState.Error);
            _pipeline.CompleteAdding();
            await DrainPipelineAsync();
            throw;
        }
    }

    /// <summary>视觉线程回调：收集结果并按序转发给 PLC。</summary>
    private void OnResultProcessed(PointInspectionResult result, Mat annotated)
    {
        _results.Add(result);
        var upload = ForwardToPlcAsync(result);
        lock (_uploadsLock)
        {
            _pendingUploads.Add(upload);
        }
    }

    private async Task ForwardToPlcAsync(PointInspectionResult result)
    {
        await _plcSendLock.WaitAsync();
        try
        {
            for (var attempt = 1; ; attempt++)
            {
                try
                {
                    await _plc.SendResultAsync(result, CancellationToken.None);
                    return;
                }
                catch (Exception ex) when (attempt < UploadRetryCount)
                {
                    Log?.Invoke($"[流程] 结果上传 PLC 失败({result.Point}) 第{attempt}/{UploadRetryCount}次: {ex.Message}, 重试...");
                    await Task.Delay(UploadRetryDelay);
                }
                catch (Exception ex)
                {
                    Log?.Invoke($"[流程] 结果上传 PLC 最终失败({result.Point}): {ex.Message}");
                    _uploadFailedPoints.Add(result.Point.Index);
                }
            }
        }
        finally
        {
            _plcSendLock.Release();
        }
    }

    /// <summary>等待全部上传任务结束，保证报表生成时上传失败数已确定。</summary>
    private async Task FlushUploadsAsync()
    {
        Task[] pending;
        lock (_uploadsLock)
        {
            pending = _pendingUploads.ToArray();
        }
        if (pending.Length > 0)
            await Task.WhenAll(pending);
    }

    private async Task DrainPipelineAsync()
    {
        try
        {
            await _pipeline.Completion.WaitAsync(DrainTimeout);
        }
        catch (TimeoutException)
        {
            Log?.Invoke("[流程] 等待流水线排空超时, 部分结果可能缺失");
        }
    }

    private InspectionReport BuildReport(DateTime started, bool wasCancelled) => new()
    {
        StartedAtUtc = started,
        FinishedAtUtc = DateTime.UtcNow,
        Results = _results.OrderBy(r => r.Point.Index).ToList(),
        WasCancelled = wasCancelled,
        UploadFailedCount = _uploadFailedPoints.Count
    };
}
