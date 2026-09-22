namespace BatteryWeldAOI.Core.Models;

/// <summary>
/// 缺陷名称文案。界面、报表与标注图共用同一份翻译，避免三处各自维护字符串表
/// （此前 WPF 与控制台各有一份 <c>TranslateDefect</c>，标注图上则干脆是英文枚举名）。
/// </summary>
public static class WeldDefectText
{
    /// <summary>单个缺陷的中文名。</summary>
    public static string Of(WeldDefect defect) => defect switch
    {
        WeldDefect.Misaligned => "焊偏",
        WeldDefect.MissingWeld => "漏焊",
        WeldDefect.Blowout => "炸焊",
        WeldDefect.Spatter => "焊渣飞溅",
        _ => defect.ToString()
    };

    /// <summary>多个缺陷的中文名（"、"分隔）；无缺陷返回 "-"。</summary>
    public static string Of(IReadOnlyList<WeldDefect> defects) =>
        defects.Count == 0 ? "-" : string.Join("、", defects.Select(Of));
}
