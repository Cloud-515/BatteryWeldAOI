namespace BatteryWeldAOI.Core.Communication;

using BatteryWeldAOI.Core.Models;

/// <summary>
/// 无 PLC 模式客户端：用于视频回放 / 相机直连的离线检测——
/// 没有运动轴，"到位"恒为真，结果不外发。
/// 让 InspectionController 无需改动即可脱离 PLC 运行。
/// </summary>
public sealed class NullPlcClient : IPlcClient
{
    public bool IsConnected => true;

    public Task ConnectAsync(CancellationToken ct) => Task.CompletedTask;

    public Task MoveToAsync(int pointIndex, CancellationToken ct) => Task.CompletedTask;

    public Task WaitForArrivalAsync(int pointIndex, CancellationToken ct) => Task.CompletedTask;

    public Task SendResultAsync(PointInspectionResult result, CancellationToken ct) => Task.CompletedTask;

    public Task FinishAsync(CancellationToken ct) => Task.CompletedTask;

    public void Dispose()
    {
    }
}
