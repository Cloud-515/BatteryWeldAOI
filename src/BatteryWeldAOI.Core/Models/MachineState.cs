namespace BatteryWeldAOI.Core.Models;

/// <summary>
/// 设备状态机状态。
/// Idle(待机) -> Moving(轴运动中) -> Inspecting(检测中) -> Finished(整包完成)；
/// 任意状态可进入 Error / Stopped，Error/Stopped 仅可通过 Reset 回到 Idle。
/// </summary>
public enum MachineState
{
    Idle,
    Moving,
    Inspecting,
    Error,
    Stopped,
    Finished
}
