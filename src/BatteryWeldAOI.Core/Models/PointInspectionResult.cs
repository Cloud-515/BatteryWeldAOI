using OpenCvSharp;

namespace BatteryWeldAOI.Core.Models;

/// <summary>
/// 单点检测结论：包含像素/物理偏移量、缺陷列表与判定结果。
/// </summary>
public sealed class PointInspectionResult
{
    public required WeldPoint Point { get; init; }

    /// <summary>孔洞（极柱）中心，图像像素坐标 —— 检测基准。</summary>
    public Point2d PoleCenterPx { get; init; }

    /// <summary>孔洞半径（像素），用于结果标注。</summary>
    public double PoleRadiusPx { get; init; }

    /// <summary>焊环（熔核）中心，图像像素坐标。</summary>
    public Point2d WeldCenterPx { get; init; }

    /// <summary>焊环（熔核）外接圆半径（像素），用于结果标注。</summary>
    public double WeldRadiusPx { get; init; }

    /// <summary>
    /// 焊环边界对拟合圆的**支撑弧段比例**（0~1，0 表示未定位到焊环）。
    ///
    /// 72 个角向扇区中，落有内点（径向距离在中位半径 ±10% 内）的扇区占比。
    /// 这是本算法里唯一直接度量"圆心/半径可信度"的量：
    ///   · 真焊环的边界是一整圈，实测良品 40 帧 0.93~1.00、厂商 NG 大熔核 1.00；
    ///   · "环形光源反射细圈与熔核并块"的污染帧只有 0.64~0.94（圆心偏差最大的 4 帧
    ///     仅 0.64/0.68/0.78/0.81）。
    /// 低于 <see cref="InspectionConfig.MinRingArcSupport"/> 的帧会以"测量失败"返回，
    /// 此时本值就是失败原因里的那个数。界面上把它显示出来，操作员才能判断
    /// "这个点位是焊偏了、还是根本没测出来"。
    /// </summary>
    public double WeldArcSupport { get; init; }

    /// <summary>
    /// 熔核**内沿**半径（像素）。0 表示未取到（内沿不成环、镜像能量阈值不适用等）。
    ///
    /// 熔核是"有厚度的环"，只给外边界表达不出环宽。内沿在**纹理能量图**上按"粗糙起始半径"取
    /// （掩码里的内沿已被闭运算封死，取不到），实测内外圆心偏差中位 1.3px。
    /// 环宽 = <see cref="WeldRadiusPx"/> − 本值。
    /// </summary>
    public double WeldInnerRadiusPx { get; init; }

    /// <summary>熔核内沿圆心（像素）。仅在 <see cref="WeldInnerRadiusPx"/> &gt; 0 时有意义。</summary>
    public Point2d WeldInnerCenterPx { get; init; }

    /// <summary>
    /// 环宽（像素）= 外半径 − 内半径。两者任一未取到时为 0。
    /// 实测良品中位 344px，约占外半径的 65%（外半径 526px）。
    /// </summary>
    public double WeldRingWidthPx => WeldInnerRadiusPx > 0 && WeldRadiusPx > WeldInnerRadiusPx
        ? WeldRadiusPx - WeldInnerRadiusPx
        : 0;

    /// <summary>
    /// 上述像素几何量所在的图像尺寸。输入图短边超过
    /// <see cref="InspectionConfig.MaxWorkingBase"/> 时算法会先等比缩小再检测，
    /// 此时本值小于原图尺寸，<see cref="WeldInspectionAlgorithm.Annotate"/> 据此按比例还原到原图。
    /// </summary>
    public Size ProcessedSizePx { get; init; }

    /// <summary>
    /// **原始输入图**尺寸（本次检测开始时拿到的图幅）。
    ///
    /// 它与 <see cref="ProcessedSizePx"/> 一起决定工作分辨率坐标到原图坐标的换算
    /// （见 <see cref="SourceScale"/>）。界面上要让操作员拿坐标去图上核对，
    /// 必须给原图坐标——工作分辨率坐标是内部实现细节（20MP 现场图上会差约 8 倍），
    /// 拿它去图上量是量不到的。因此展示与报表一律用
    /// <see cref="PoleCenterSourcePx"/> / <see cref="WeldCenterSourcePx"/>。
    /// </summary>
    public Size SourceSizePx { get; init; }

    /// <summary>
    /// 工作分辨率坐标 → 原图坐标的缩放比。检测未做归一化（或原图就在工作分辨率以内）时为 1。
    /// </summary>
    public double SourceScale => ProcessedSizePx.Width > 0 && SourceSizePx.Width > 0
        ? SourceSizePx.Width / (double)ProcessedSizePx.Width
        : 1.0;

    /// <summary>极柱中心（原始输入图坐标）。</summary>
    public Point2d PoleCenterSourcePx => new(PoleCenterPx.X * SourceScale, PoleCenterPx.Y * SourceScale);

    /// <summary>焊环中心（原始输入图坐标）。未检出焊环时为 (0,0)，须先看 <see cref="HasValidOffset"/>。</summary>
    public Point2d WeldCenterSourcePx => new(WeldCenterPx.X * SourceScale, WeldCenterPx.Y * SourceScale);

    /// <summary>极柱半径（原始输入图像素）。</summary>
    public double PoleRadiusSourcePx => PoleRadiusPx * SourceScale;

    /// <summary>焊环半径（原始输入图像素）。</summary>
    public double WeldRadiusSourcePx => WeldRadiusPx * SourceScale;

    /// <summary>熔核内沿半径（原始输入图像素）。0 表示未取到。</summary>
    public double WeldInnerRadiusSourcePx => WeldInnerRadiusPx * SourceScale;

    /// <summary>环宽（原始输入图像素）。0 表示未能取到内沿。</summary>
    public double WeldRingWidthSourcePx => WeldRingWidthPx * SourceScale;

    /// <summary>焊环中心相对孔洞中心的 X 向偏移（毫米，含符号）。</summary>
    public double OffsetXmm { get; init; }

    /// <summary>焊环中心相对孔洞中心的 Y 向偏移（毫米，含符号）。</summary>
    public double OffsetYmm { get; init; }

    /// <summary>合成偏移距离：焊环中心到孔洞中心的距离（毫米）。</summary>
    public double OffsetDistanceMm => Math.Sqrt(OffsetXmm * OffsetXmm + OffsetYmm * OffsetYmm);

    /// <summary>检测到的缺陷列表；为空且无错误表示 OK。</summary>
    public IReadOnlyList<WeldDefect> Defects { get; init; } = Array.Empty<WeldDefect>();

    /// <summary>
    /// 本点测量失败（取图故障，或算法无法定位焊环/孔洞、几何尺寸超出合理区间）。
    /// 此时偏移量无效，既不判合格也不判不合格，必须人工复核，且不参与合格率统计。
    /// </summary>
    public bool HasError { get; init; }

    /// <summary>
    /// 诊断信息。两种情形会填写：测量失败（<see cref="HasError"/> 为 true）的原因，
    /// 或漏焊判定的依据（此时 <see cref="HasError"/> 为 false，是确定的缺陷而非测量失败）。
    /// 界面与报表一律以 HasError / Defects 为准，不要仅凭本字段非空就当成异常。
    /// </summary>
    public string? ErrorMessage { get; init; }

    public bool IsOk => !HasError && Defects.Count == 0;

    /// <summary>
    /// 本点的偏移量是否有物理意义（是否给出了一次真实测量）。
    ///
    /// 测量失败时偏移量无效；由"未找到焊环"造成的漏焊同样无效——焊环中心尚未确定，
    /// 偏移量恒为 0.000mm。界面、报表与标注图必须把这种情况显示为"—"，
    /// 否则操作员会把 0.000mm 当成"焊得很正"，恰好把最该关注的点位读反。
    /// （焊环找到、仅覆盖率不足的漏焊不属此类，其偏移量是真实测得的。）
    /// </summary>
    public bool HasValidOffset => !HasError && WeldRadiusPx > 0;

    /// <summary>本点图像处理耗时（毫秒）。</summary>
    public double ProcessingMs { get; init; }

    /// <summary>缺陷描述（用于报表与日志）。机器可读：缺陷枚举名，测量失败为 "ERROR:原因"。</summary>
    public string DefectText => HasError
        ? $"ERROR:{ErrorMessage}"
        : IsOk ? "OK" : string.Join(";", Defects);

    /// <summary>
    /// 缺陷描述的中文文案（界面、标注图与报表诊断用）。测量失败时为异常原因；
    /// 漏焊等"确定缺陷"会带上判定依据（实测覆盖率/填充率与下限），操作员据此判断
    /// 是"根本没焊"还是"焊了但环不密实"——只报一个"漏焊"是无法复核的。
    /// </summary>
    public string DefectTextZh => HasError
        ? $"异常：{ErrorMessage}"
        : IsOk ? "OK"
        : string.IsNullOrWhiteSpace(ErrorMessage)
            ? WeldDefectText.Of(Defects)
            : $"{WeldDefectText.Of(Defects)}：{ErrorMessage}";

    /// <summary>
    /// 把异常原因压缩成一行可读短原因：截到第一个括号/逗号之前。
    /// 完整诊断（如"焊环边界支撑弧段不足（仅覆盖 32% 圆周，下限 50%）"）写不进标注图，
    /// 挤在图上只会被截断成没人能读的半句话；短原因负责"是什么问题"，细节留给界面与报表。
    /// </summary>
    public static string ShortReason(string? message)
    {
        if (string.IsNullOrWhiteSpace(message))
            return "测量失败";

        var cut = message.Length;
        foreach (var delimiter in new[] { '（', '(', '，', ',', '；', ';', '：', ':' })
        {
            var index = message.IndexOf(delimiter);
            if (index > 0 && index < cut)
                cut = index;
        }
        return message[..cut].Trim();
    }

    public override string ToString()
    {
        if (HasError)
            return $"{Point} 异常({ErrorMessage})";
        if (IsOk)
            return $"{Point} OK   dX={OffsetXmm:+0.000;-0.000}mm dY={OffsetYmm:+0.000;-0.000}mm " +
                   $"D={OffsetDistanceMm:F3}mm ({ProcessingMs:F0}ms)";
        if (!HasValidOffset)
            return $"{Point} NG({DefectTextZh}) 偏移量不适用（未检出焊环） ({ProcessingMs:F0}ms)";
        return $"{Point} NG({DefectTextZh}) dX={OffsetXmm:+0.000;-0.000}mm " +
               $"dY={OffsetYmm:+0.000;-0.000}mm D={OffsetDistanceMm:F3}mm ({ProcessingMs:F0}ms)";
    }
}
