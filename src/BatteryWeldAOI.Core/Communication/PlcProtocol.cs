namespace BatteryWeldAOI.Core.Communication;

/// <summary>
/// 上位机-下位机文本协议（换行分隔，UTF-8）。
/// 真实项目可替换为 Modbus TCP / 西门子 S7 / 三菱 MC 协议，接口保持不变。
///
/// 上位机 -> PLC:
///   PING                      心跳
///   MOVE_TO {index}           移动到指定点位
///   READ                      读取轴状态
///   RESULT {index} OK|NG {dx} {dy} {defects}   上传单点检测结果
///   FINISH                    整包检测完成
///
/// PLC -> 上位机:
///   PONG
///   OK
///   MOVING {index}
///   ARRIVED {index}
///   DONE
/// </summary>
public static class PlcProtocol
{
    public const string Ping = "PING";
    public const string Pong = "PONG";
    public const string MoveTo = "MOVE_TO";
    public const string Read = "READ";
    public const string Result = "RESULT";
    public const string Finish = "FINISH";
    public const string AckOk = "OK";
    public const string StatusMoving = "MOVING";
    public const string StatusArrived = "ARRIVED";
    public const string Done = "DONE";
}
