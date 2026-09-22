namespace BatteryWeldAOI.Core.Models;

/// <summary>
/// 焊点缺陷类型。
/// </summary>
public enum WeldDefect
{
    /// <summary>焊偏：焊缝中心相对极柱中心偏移超差。</summary>
    Misaligned,

    /// <summary>漏焊：ROI 内未找到有效焊缝。</summary>
    MissingWeld,

    /// <summary>炸焊：焊缝面积异常偏大、轮廓不规则。</summary>
    Blowout,

    /// <summary>焊渣飞溅：焊缝周边出现细小溅射颗粒。</summary>
    Spatter
}

/// <summary>
/// 单个待检测焊点的规划信息（对应电池包上一个电芯极柱）。
/// </summary>
public sealed class WeldPoint
{
    /// <summary>点位序号（从 0 开始，按设备固定运动顺序排列）。</summary>
    public int Index { get; init; }

    /// <summary>点位名称，如 "Cell03-POS"。</summary>
    public string Name { get; init; } = string.Empty;

    /// <summary>
    /// 该点位对应的原始图像来源（图片文件夹模式 = 文件名；视频模式 = "文件 #帧号"）。
    /// 仿真/真机模式下为 null。仅用于追溯与显示，不参与判定——
    /// 批量识别时只显示 Img007 无法定位到具体是哪个文件，异常点就没法复查。
    /// </summary>
    public string? SourceName { get; init; }

    /// <summary>该点在机台坐标系下的物理位置（毫米），仅用于模拟 PLC 运动。</summary>
    public double StageX { get; init; }

    /// <summary>该点在机台坐标系下的物理位置（毫米）。</summary>
    public double StageY { get; init; }

    public override string ToString() => $"[{Index}] {Name}";
}
