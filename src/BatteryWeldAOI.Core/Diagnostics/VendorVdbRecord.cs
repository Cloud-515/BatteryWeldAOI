namespace BatteryWeldAOI.Core.Diagnostics;

/// <summary>
/// 厂商 VDB 文件（<c>*.vdb</c>，魔数 <c>BDV.</c>）里记录的**单帧厂商测量结果**。
///
/// 这是现场样本目录（<c>tests/pp/**/VdbGui_pic</c>）中厂商软件留下的私有格式，
/// 此前项目只识别到魔数就当作"无公开规格，不可读"（见 PpSampleTests 的类注释）。
/// 实际可以读出圆心与半径，用来给本项目的焊环定位做参考基准。
///
/// 已实测确认（对应 2025-09-11 / 2025-09-12 两批，874 个 SN / 20955 帧）：
///   · <see cref="LocateCenterX"/>/<see cref="LocateCenterY"/> 是厂商"区域圆定位"工具输出的
///     定位圆心，坐标系是**原始图像像素**（如 5472×3648 图上的 2571.2 / 1734.4）。
///     与本项目算法测出的焊环中心对比（264 帧配对），偏差中位 10.4px、90 分位 59.7px，
///     而焊环半径约 525px——即中位偏差约 2% 半径。
///   · <see cref="LocateRadiusPx"/> 是厂商定位圆的半径（实测中位 481.5px），
///     与本项目测的焊环半径（约 525px）系统性相差 9%：两者取的"环边界"不是同一条，
///     互相替代前必须先定死边界口径。
///   · <see cref="ResultD"/> 是结果字符串里的第三个值，被**硬钳在 ±7.425**、可以为负，
///     现场语料里 46.4% 的记录 |D| &gt; 3、4.48% 撞到钳位。含义与单位未确定
///     （需要厂商的工具配置），不要拿它当测量量用。
/// </summary>
public sealed record VendorVdbRecord
{
    /// <summary>vdb 文件全路径。</summary>
    public required string FilePath { get; init; }

    /// <summary>vdb 文件名（含扩展名）。</summary>
    public required string FileName { get; init; }

    /// <summary>电池包序列号（从文件名解析），解析不到时为空串。</summary>
    public string Serial { get; init; } = string.Empty;

    /// <summary>
    /// 位置标记（<c>Pos0</c>/<c>Pos1</c>/<c>Pos2</c>）。
    /// 同一帧的三个位置标记**共享同一张嵌入图像**（实测逐像素差值为 0），
    /// 差别只在各自记录的对象上，因此它们不是三个不同的焊点、也不提供额外图像。
    /// </summary>
    public string Position { get; init; } = string.Empty;

    /// <summary>采集时间戳（文件名里 <c>_Scene</c> 之前的部分，如 "19-22-06 113"）。</summary>
    public string Timestamp { get; init; } = string.Empty;

    /// <summary>
    /// 厂商判定：true = 文件名带 <c>_NG_</c>，false = 带 <c>_OK_</c>，null = 文件名无标记。
    ///
    /// **判定结论只存在于文件名里**：对 40 个 OK 与 40 个 NG 的 vdb 头部做过逐字节比对，
    /// 找"OK 组恒为一个值、NG 组恒为另一个值"的字节，候选数为 0，文件内没有状态字段。
    /// 因此本字段的来源是文件名约定，不是文件内容。
    /// </summary>
    public bool? IsVendorNg { get; init; }

    /// <summary>厂商定位圆心 X（原始图像像素）。</summary>
    public double LocateCenterX { get; init; }

    /// <summary>厂商定位圆心 Y（原始图像像素）。</summary>
    public double LocateCenterY { get; init; }

    /// <summary>
    /// 厂商定位圆半径（原始图像像素）。取不到时为 <see cref="double.NaN"/>。
    ///
    /// 这个值是头部里紧跟在圆心双精度对之后的一个 double，**偏移是模板相关的**，
    /// 因此按"值与字符串里的 X/Y 相符"来定位而不是写死偏移；同一个头部里 X/Y 会出现两次
    /// （第二处后面跟的不是半径），这里取第一处匹配。厂商换模板后本字段可能取不到，
    /// 由单元测试的合理区间断言来暴露漂移。
    /// </summary>
    public double LocateRadiusPx { get; init; } = double.NaN;

    /// <summary>结果字符串里的第三个值（<c>X:..;Y:..;D:..</c> 的 D）。语义未确定，见类型注释。</summary>
    public double ResultD { get; init; }

    /// <summary>
    /// 头部长度 = 嵌入 JPEG 的起始偏移；<b>0 表示在探测范围内未找到</b>
    /// （Pos0 头部约 3461 字节可探到，Pos1/Pos2 头部可达数百 KB，超出探测长度）。
    /// 仅作结构诊断用，为 0 时测量值仍然有效。
    /// </summary>
    public int HeaderLength { get; init; }

    /// <summary>是否取到了定位半径（取不到不影响圆心可用性）。</summary>
    public bool HasLocateRadius => !double.IsNaN(LocateRadiusPx);
}
