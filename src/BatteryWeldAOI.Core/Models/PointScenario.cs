namespace BatteryWeldAOI.Core.Models;

/// <summary>
/// 单点检测的"剧本"：模拟相机用来渲染图像的真实场景（含真值偏移与缺陷）。
/// 真实设备中不存在此类——由相机实拍代替；它让本仿真项目可以自证算法精度。
/// </summary>
public sealed class PointScenario
{
    public required WeldPoint Point { get; init; }

    /// <summary>真实缺陷；null 表示该点合格。</summary>
    public WeldDefect? Defect { get; init; }

    /// <summary>焊缝相对极柱的真实 X 向偏移（毫米）。</summary>
    public double TrueOffsetXmm { get; init; }

    /// <summary>焊缝相对极柱的真实 Y 向偏移（毫米）。</summary>
    public double TrueOffsetYmm { get; init; }
}
