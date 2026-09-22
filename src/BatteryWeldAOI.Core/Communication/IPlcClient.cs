namespace BatteryWeldAOI.Core.Communication;

using BatteryWeldAOI.Core.Models;

/// <summary>
/// 上位机侧的 PLC 客户端抽象。
/// </summary>
public interface IPlcClient : IDisposable
{
    bool IsConnected { get; }

    /// <summary>建立连接并完成握手。</summary>
    Task ConnectAsync(CancellationToken ct);

    /// <summary>下发运动指令：移动到指定点位。</summary>
    Task MoveToAsync(int pointIndex, CancellationToken ct);

    /// <summary>轮询等待到位信号（Stop-and-Go 模式的 "Stop"）。</summary>
    Task WaitForArrivalAsync(int pointIndex, CancellationToken ct);

    /// <summary>上传单点检测结果，供 PLC 决定打标/剔除。</summary>
    Task SendResultAsync(PointInspectionResult result, CancellationToken ct);

    /// <summary>通知整包检测结束。</summary>
    Task FinishAsync(CancellationToken ct);
}
