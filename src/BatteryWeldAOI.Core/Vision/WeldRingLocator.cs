namespace BatteryWeldAOI.Core.Vision;

using BatteryWeldAOI.Core.Models;
using OpenCvSharp;

/// <summary>焊环定位失败的性质——决定调用方把它报成缺陷还是报成测量失败。</summary>
internal enum RingFailure
{
    /// <summary>未失败。</summary>
    None,

    /// <summary>确定没有焊点：视野里根本没有圆盘状粗糙区。这是可判定的缺陷（漏焊）。</summary>
    NoWeld,

    /// <summary>测不出来：图像条件或形状不支持可靠测量。偏移量无效，必须人工复核。</summary>
    Unmeasurable,
}

/// <summary>焊环（激光焊熔核）定位结果。</summary>
internal sealed class RingDetection : IDisposable
{
    /// <summary>是否成功定位到焊环。</summary>
    public bool Ok { get; init; }

    /// <summary>失败原因（Ok=false 时有效）。</summary>
    public string? Failure { get; init; }

    /// <summary>失败性质（Ok=false 时有效）。</summary>
    public RingFailure FailureKind { get; init; }

    /// <summary>熔核几何中心。</summary>
    public Point2d Center { get; internal set; }

    /// <summary>熔核外接圆半径（像素）。</summary>
    public double Radius { get; internal set; }

    /// <summary>半径 / base；同一视野下为常数。</summary>
    public double RadiusRatio { get; internal set; }

    /// <summary>圆度 = 距离变换峰值半径 / 等效半径。</summary>
    public double Roundness { get; init; }

    /// <summary>
    /// 边界对拟合圆的**支撑弧段比例**（0~1）：72 个角向扇区中，落有内点的扇区占比。
    /// 越高说明这条边界越像"一整圈"，因而拟合出的圆心越可信。
    /// 调用方在多个候选之间择优时以此为质量依据（见 WeldInspectionAlgorithm 的核宽阶梯）。
    /// </summary>
    public double ArcSupport { get; internal set; }

    /// <summary>
    /// 失败时仍然可用的**最佳候选圆**（掩码最大连通域的径向中位数拟合结果，在支撑弧段门限之前）。
    ///
    /// 调用方的**第五路径**（多起点重试）以它为中心做局部搜索——实测这帧的掩码质心离真值
    /// 9~23 工作像素，而正确盆地只覆盖约 3 工作像素宽，所以必须搜索而不能只换个估计器
    /// （见 FIXPLAN 第 11 节）。
    /// </summary>
    public CircleFit? BestCandidate { get; internal set; }

    /// <summary>
    /// 熔核区域的实心掩码（洞已填充），CV_8U 全图尺寸。
    /// </summary>
    public Mat? Solid { get; internal set; }

    /// <summary>
    /// 本次定位是**放宽了掩码边界支撑弧段门限**才通过的（<see cref="WeldRingLocator.Detect"/> 的
    /// <c>minArcSupport</c> 参数低于 <see cref="InspectionConfig.MinRingArcSupport"/>）。
    ///
    /// 这类结果的几何量（圆心/半径）可能完全正确，但"这条边界成一整圈"这件事**没有被证明**——
    /// 只能由调用方用**边界精修在 |∇I| 上量出的**支撑弧段去转正（见
    /// <see cref="WeldInspectionAlgorithm"/> 的第四路径）。在此之前不得参与判定。
    /// </summary>
    public bool MaskSupportRelaxed { get; internal set; }

    /// <summary>
    /// 用孔锚精修的结果替换圆几何，并把实心掩码**一并裁到新圆内**。
    ///
    /// 掩码必须一起裁：质量指标（填充率、环弧覆盖率、环内暗区）全部以"圆内"为口径，
    /// 圆变小而掩码不变会让填充率凭空虚增——实测旧圆 0.157×base、精修后 0.127×base，
    /// 同一块掩码的填充率会从约 1.0 涨到约 1.5，越过 <see cref="InspectionConfig.MaxRingFill"/>
    /// 变成"假炸焊"。裁完之后填充率的物理含义才成立："拟合圆内有多大比例是熔核"。
    /// </summary>
    public void ApplyRefinement(Point2d center, double radius, double arcSupport)
    {
        Center = center;
        Radius = radius;
        RadiusRatio = BaseSize > 0 ? radius / BaseSize : 0;
        ArcSupport = arcSupport;
        if (Solid is not null)
        {
            using var inside = new Mat(Solid.Size(), MatType.CV_8UC1, Scalar.Black);
            Cv2.Circle(inside, (Point)center, (int)Math.Round(radius), Scalar.White, -1);
            Cv2.BitwiseAnd(Solid, inside, Solid);
        }
    }

    /// <summary>纹理能量图 E，CV_32F 全图尺寸（飞溅计数复用，避免重复计算）。</summary>
    public Mat? Texture { get; init; }

    /// <summary>焊环二值化阈值。</summary>
    public double Threshold { get; init; }

    /// <summary>base = min(高, 宽)，所有尺度相对参数的分母。</summary>
    public int BaseSize { get; init; }

    public void Dispose()
    {
        Solid?.Dispose();
        Texture?.Dispose();
    }
}

/// <summary>
/// 以孔位（极柱）圆心为锚精修出的焊环圆。
/// </summary>
/// <param name="Circle">精修后的圆（圆心 + 半径），坐标系与输入灰度图一致。</param>
/// <param name="ArcSupport">参与拟合的射线占比（内点数 / 总射线数），语义与
/// <see cref="RingDetection.ArcSupport"/> 一致。</param>
/// <param name="EdgeRadius">全局选定的"熔核外沿"半径 R*（以孔心为原点），仅用于诊断。</param>
/// <param name="ResidualScatter">
/// 内点径向残差的标准差 ÷ 半径（**归一化残差**）。
///
/// 它是"这个圆心到底对不对"的**客观**度量，与"边界覆盖多少圆周"（<paramref name="ArcSupport"/>）
/// 是两件事：一条被系统性截断的边界可以覆盖 95% 圆周却残差很大（实测 `SN_...0004` 第 0 帧：
/// 第二起点给 arc=95% 但残差 0.0294，而主起点给 arc=82%、残差 **0.0224**）。
/// 两个起点仲裁时用它，正是为了不被"假性偏高的支撑弧段"骗到。
/// </param>
/// <param name="ClippedRatio">
/// 本次结果所在那一级的**截断比**：有效射线的边界落在搜索带外端 2 个上采样采样点以内的比例。
/// 越高说明 ρ 被低估、带太窄，边界被硬截在带外端——此时支撑弧段会**假性偏高**
/// （被截断的射线全部落在同一半径上）。它原本只在 <c>RefineBoundary</c> 内部用于逐级放宽带宽，
/// 现在随结果一起带出，供离线判别研究当候选量（见 FIXPLAN 第 10 节）。
/// </param>
internal readonly record struct RingRefinement(CircleFit Circle, double ArcSupport, double EdgeRadius,
    double ResidualScatter, double ClippedRatio);

/// <summary>
/// 焊环定位：用「纹理能量」把粗糙的激光焊熔核从光滑的金属表面里分出来。
///
/// 为什么是纹理而不是灰度：激光焊熔核表面是鱼鳞状凝固组织 + 飞溅颗粒，局部标准差极高；
/// 而极柱端面、孔壁、汇流排都是光滑金属，局部标准差极低。更关键的是——**灰度会随曝光漂移，
/// 纹理不会**。同一批样本里熔核有时比周围亮、有时比周围暗（取决于反光方向），
/// 但"粗糙"这一属性恒定不变。这就是本算法相对旧版（绝对灰度阈值 + 形态学背景差分）
/// 的根本改进：旧版参数只在合成图上成立，一上真机就全军覆没。
///
/// 流程：纹理能量图 → 自适应阈值（中位数 + k×(p99−中位数)）→ 闭/开运算串联鱼鳞斑 →
///       连通域筛选（面积、贴边剔除、圆度）→ 以最大内切圆圆心为种子的径向中位数圆拟合。
/// </summary>
internal static class WeldRingLocator
{
    /// <param name="gray8">已做 3×3 中值滤波的灰度图（调用方统一预处理，避免重复转换）。</param>
    /// <param name="preOpenKernelRatio">
    /// 闭运算**之前**的开运算核边长（× base），0 = 不做。用于删除薄结构（细线/薄环，
    /// 如环形光源反射、打标圈）。必须在闭运算之前：闭运算一旦把细圈与熔核桥接成一个实心圆盘，
    /// 之后任何形状统计量都分辨不出来。
    /// </param>
    /// <param name="dualScaleGate">是否启用"大窗能量也须超阈"的双尺度门控（只在重试路径上用）。</param>
    /// <param name="preCleanBorder">
    /// 是否在形态学**之前**先清除贴边连通域。**默认关闭**——它是"熔核被桥接进工件外框"这一类
    /// 污染的专用兜底，由调用方在既有两条路径都给不出可信结果时才启用（见
    /// <see cref="WeldInspectionAlgorithm"/> 的第三路径）。
    ///
    /// 为什么不能默认打开（2026-09-18 用合成夹具实测的教训）：当一帧**根本没有焊点**时，
    /// 纹理阈值会退化成"噪声决定"（实测合成夹具的无焊点帧 p99−中位只有约 3.6 灰阶，
    /// 阈值 0.9），于是**整幅台面**都成了"纹理显著区"，其连通域包围盒 1173×916（base=1024）
    /// 恰好落在边框禁区之内 3px。默认路径靠"闭运算把它与贴边窄带桥接在一起"从而按贴边剔除，
    /// 提前清除贴边窄带反而让这个巨大伪候选存活下来（实测 `WeldInspectionAlgorithmTests`
    /// 的 5 个 MissingWeld 用例因此由"漏焊"变成"测量失败"）。所以它只作为兜底，
    /// 并由调用方用"半径不得超过有效性上限"把这类退化候选挡回去。
    /// </param>
    /// <param name="minArcSupport">
    /// 边界支撑弧段门限的**覆写值**（0 = 用 <see cref="InspectionConfig.MinRingArcSupport"/>）。
    ///
    /// 只给"放宽"用：设一个比配置值更低的数，让"掩码轮廓不像一整圈、但尺度可能正确"的候选
    /// 能以 <see cref="RingDetection.MaskSupportRelaxed"/> = true 通过，交给调用方的
    /// **边界精修**去定性。为什么需要它：本门限是拿**主路径的实心掩码**标定的，
    /// 而激进薄结构删除之后剩下的掩码在鱼鳞斑之间是断裂的——实测现场 `SN_...0004` 第 10 帧，
    /// 激进档 k=0.060 给出的圆心与厂商 `.vdb` 只差 1.2px、半径比 0.1376（真值 0.1317），
    /// 但掩码轮廓落在中位半径 ±10% 的角向扇区占比只有 60%，够不着 0.82。
    /// 用它否决一个几何正确的候选属于**口径错配**；而放宽之后的正确性由精修在 |∇I| 上
    /// 量出的支撑弧段负责（与掩码稀疏程度无关），见 <see cref="WeldInspectionAlgorithm"/>。
    /// </param>
    public static RingDetection Detect(Mat gray8, InspectionConfig cfg, double preOpenKernelRatio = 0,
        bool dualScaleGate = false, bool preCleanBorder = false, double minArcSupport = 0)
    {
        var w = gray8.Width;
        var h = gray8.Height;
        var baseSize = Math.Min(h, w);
        if (baseSize < 64)
            return new RingDetection
            {
                Ok = false,
                Failure = "图像过小",
                FailureKind = RingFailure.Unmeasurable
            };

        // ---- 纹理能量图 ----
        var win = Geo.Odd(baseSize * cfg.TextureWindowRatio, 5, 15);
        var energy = TextureEnergy(gray8, win);

        // 边框置零：贴边的连通域是台面/背景边界，不是焊点。置零后它们既不会被阈值选中，
        // 也不会抬高 p99（p99 被抬高会连带把真焊环的阈值顶上去了）。
        var margin = (int)(baseSize * cfg.BorderMarginRatio);
        ZeroBorder(energy, margin);

        var median = Geo.Percentile32F(energy, 50, SampleStep(baseSize));
        var p99 = Geo.Percentile32F(energy, 99, SampleStep(baseSize));
        var threshold = median + cfg.TextureThresholdK * (p99 - median);

        using var mask = new Mat();
        Cv2.Compare(energy, new Scalar(threshold), mask, CmpType.GT);

        // 双尺度门控：要求**大窗**能量也超阈，把"细密灰斑噪声带"（表面污渍 / 环形光散斑）
        // 从熔核周围剔掉。鱼鳞斑是较大尺度的斑块，两个尺度都高；细散斑在大窗上被平均掉。
        // 详见 InspectionConfig.DualScaleWindowRatio 的实测记录。
        if (dualScaleGate && cfg.DualScaleWindowRatio > 0)
        {
            var coarseWindow = Geo.Odd(baseSize * cfg.DualScaleWindowRatio, 15, 61);
            using var coarse = TextureEnergy(gray8, coarseWindow);
            ZeroBorder(coarse, margin);
            var coarseMedian = Geo.Percentile32F(coarse, 50, SampleStep(baseSize));
            var coarseP99 = Geo.Percentile32F(coarse, 99, SampleStep(baseSize));
            var coarseThreshold = coarseMedian + cfg.DualScaleThresholdK * (coarseP99 - coarseMedian);
            using var coarseMask = new Mat();
            Cv2.Compare(coarse, new Scalar(coarseThreshold), coarseMask, CmpType.GT);
            Cv2.BitwiseAnd(mask, coarseMask, mask);
        }

        // ---- 贴边连通域清除（可选，必须在形态学**之前**）----
        //
        // 默认路径是**在闭运算之后**按包围盒贴边把整块剔除（见下面的 `bbox.Left <= margin`）。
        // 那条规则在"熔核与工件外框被闭运算桥接成一块"时会**把熔核一起扔掉**：
        // 实测 `SN_...0007` 第 10 帧（用户报的 Img010）原始掩码里熔核自成一域，
        // 而 21px 闭运算两次迭代（桥接距离约 40px）把它与工件外框连成一块 88903px、
        // 包围盒 556×383 的贴边连通域 → 候选数 0 → 整帧报"未找到纹理显著区域（无焊点）"，
        // 用户看到的就是"明明有焊环却判无焊点"。
        //
        // 结构性边缘（工件外框、线束、机加工台阶）在**原始掩码**里本来就是一整片贴边连通域，
        // 而熔核是**居中的独立连通域**（实测该帧熔核原始域包围盒 (252,149,178,179)、不贴边）。
        // 在形态学之前先清掉贴边连通域，"桥接"就失去了对象。实测（`SN_...0007` 24 帧）：
        // 22 帧逐像素不变，原先零候选的第 1、10 帧恢复检出。
        //
        // **只在兜底路径上启用**（默认 false），理由见 preCleanBorder 参数的说明。
        if (preCleanBorder)
            RemoveBorderComponents(mask, margin);

        // 先开运算删掉薄结构（细线/薄环，如环形光源反射、打标圈）：必须在闭运算**之前**，
        // 否则闭运算先把细圈与熔核桥接成一个实心圆盘，之后任何形状统计量都分辨不出来。
        // 详见 InspectionConfig.PreOpenKernelRatio。
        //
        // 上限从 21 放宽到 0.11×base（base=460 时 51px）：原先把核宽**硬钳在 21px**，
        // 导致 InspectionConfig.FallbackPreOpenKernelRatios 的核宽阶梯（21/27/34px）
        // 三级全被钳成同一个 21px —— 阶梯实际上是个空操作（2026-09-16 实测发现：
        // 加阶梯后全量指标一字未变）。51px 远小于熔核直径（约 132px = 2×0.144×base），
        // 因此放大核宽不会把熔核本体删掉；真正的安全网是调用方"只采纳半径落回族值带的结果"。
        if (preOpenKernelRatio > 0)
        {
            var maxPreOpenK = Math.Max(3, (int)Math.Round(baseSize * 0.11) | 1);
            var preOpenK = Geo.Odd(baseSize * preOpenKernelRatio, 3, maxPreOpenK);
            using var se = Cv2.GetStructuringElement(MorphShapes.Ellipse, new Size(preOpenK, preOpenK));
            Cv2.MorphologyEx(mask, mask, MorphTypes.Open, se);
        }

        // 闭运算串联鱼鳞斑，使其成为一个连通域。需要两次迭代——真机上有几个点位
        // 的熔核纹理较稀疏，一次迭代桥接不够，连通域碎裂，整点被判漏焊。
        // 代价是这个核宽会把 1.3 倍环径处的飞溅也并进来：连通域变成"熔核 + 一侧凸起"，
        // 此时任何以最小二乘初值为基准的圆拟合都会被凸起撑大（见 Geo.MedianRadiusCircle）。
        using (var closeK = Cv2.GetStructuringElement(MorphShapes.Ellipse,
                   new Size(Geo.Odd(baseSize * cfg.CloseKernelRatio, 3, 21), Geo.Odd(baseSize * cfg.CloseKernelRatio, 3, 21))))
        {
            Cv2.MorphologyEx(mask, mask, MorphTypes.Close, closeK, iterations: 2);
        }
        using (var openK = Cv2.GetStructuringElement(MorphShapes.Ellipse,
                   new Size(Geo.Odd(baseSize * cfg.OpenKernelRatio, 3, 15), Geo.Odd(baseSize * cfg.OpenKernelRatio, 3, 15))))
        {
            Cv2.MorphologyEx(mask, mask, MorphTypes.Open, openK);
        }

        // ---- 连通域筛选 ----
        using var labels = new Mat();
        using var stats = new Mat();
        using var centroids = new Mat();
        var count = Cv2.ConnectedComponentsWithStats(mask, labels, stats, centroids,
            PixelConnectivity.Connectivity8);

        var minArea = baseSize * baseSize * cfg.MinWeldBlobRatio;
        var candidates = new List<Candidate>();
        for (var i = 1; i < count; i++)
        {
            var area = stats.At<int>(i, (int)ConnectedComponentsTypes.Area);
            if (area < minArea)
                continue;

            var bbox = new Rect(
                stats.At<int>(i, (int)ConnectedComponentsTypes.Left),
                stats.At<int>(i, (int)ConnectedComponentsTypes.Top),
                stats.At<int>(i, (int)ConnectedComponentsTypes.Width),
                stats.At<int>(i, (int)ConnectedComponentsTypes.Height));

            // 贴边剔除：真实焊点完整落在视野内，被边框裁掉的一定是台面/背景
            if (bbox.Left <= margin || bbox.Top <= margin ||
                bbox.Right >= w - margin || bbox.Bottom >= h - margin)
                continue;

            using var sub = new Mat(labels, bbox);
            using var comp = new Mat();
            Cv2.Compare(sub, new Scalar(i), comp, CmpType.EQ);

            using var filled = FillHoles(comp);
            var filledArea = Cv2.CountNonZero(filled);
            if (filledArea <= 0)
                continue;

            using var dt = new Mat();
            Cv2.DistanceTransform(filled, dt, DistanceTypes.L2, DistanceTransformMasks.Mask5);
            Cv2.MinMaxLoc(dt, out _, out var dtMax, out _, out var dtPeak);
            var radiusEq = Math.Sqrt(filledArea / Math.PI);

            // 距离变换峰值即最大内切圆圆心。dt 的坐标系是包围盒局部，必须平移回全图，
            // 否则内切圆会整体偏到左上角（与拟合圆做对比时会凭空差出半个包围盒）。
            var dtCenter = new Point2d(bbox.X + dtPeak.X, bbox.Y + dtPeak.Y);

            candidates.Add(new Candidate(bbox, filled.Clone(), filledArea, radiusEq, dtMax, dtCenter));
        }

        var detection = new RingDetection
        {
            Texture = energy,
            Threshold = threshold,
            BaseSize = baseSize
        };

        if (candidates.Count == 0)
            return WithFailure(detection, "未找到纹理显著区域（无焊点）", RingFailure.NoWeld);

        // 圆度筛选：把焊环与车削纹、划痕、台面亮带区分开
        var discs = candidates
            .Where(c => c.Roundness >= cfg.MinWeldRoundness && c.Roundness <= cfg.MaxWeldRoundness)
            .ToList();
        if (discs.Count == 0)
        {
            DisposeAll(candidates);
            return WithFailure(detection, "纹理区域均不呈圆盘状", RingFailure.NoWeld);
        }

        // 取填洞面积最大者：真焊环与次大候选的等效半径实测相差 1.4 倍以上。
        //
        // **2026-09-18 试过加"实心度/未填洞内切半径"两道闸门，实测无收益且引入新失败，已全部回退**，
        // 记录原因：① 把排序改成"未填洞内切半径最大者"，在 `SN_...0007` 24 帧上与面积排序
        // **逐帧给出相同的选择**（零收益）；② 新增"实心度低于本帧最强候选 0.40 倍即剔除"的闸门
        // （本意是拦"细圈被 FillHoles 填成实心盘"的伪候选，实测该形态实心度 0.02 vs 真熔核 0.9，
        // 分离度确实够），但剔除候选会改变"谁被选中"，在合成夹具的对抗性无焊点帧上把
        // "漏焊"变成了别的结论。两者都没有现场数据上的收益，故不保留。
        // "细圈填洞块压过熔核"这一类仍由既有的**半径族值带触发重试 + 双尺度门控**处理，见
        // InspectionConfig.RingRadiusFamilyMaxRatio / DualScaleWindowRatio。
        var pick = discs.MaxBy(c => c.FilledArea)!;
        var roundness = pick.Roundness;

        // 内切圆圆心：凸起免疫，作为径向中位数拟合的起点。缺口会让它偏向本体另一侧，
        // 但下面的迭代不依赖这个起点准确（见 Geo.MedianRadiusCircle）。
        var inscribedCenter = pick.InscribedCenter;

        foreach (var c in candidates.Where(c => !ReferenceEquals(c, pick)).ToList())
        {
            candidates.Remove(c);
            c.Dispose();
        }

        using var solid = new Mat(new Size(w, h), MatType.CV_8UC1, Scalar.Black);
        using (var roi = new Mat(solid, pick.BBox))
            pick.Filled.CopyTo(roi);
        pick.Dispose();

        var contourPoints = LargestContourPoints(solid);
        if (contourPoints is null)
            return WithFailure(detection, "焊环轮廓提取失败", RingFailure.Unmeasurable);

        // 以最大内切圆圆心为种子做径向中位数拟合：凸起（并块）与缺口都拖不动它。
        var fit = Geo.MedianRadiusCircle(contourPoints, inscribedCenter);
        if (fit is null)
            return WithFailure(detection, "焊环圆拟合失败", RingFailure.Unmeasurable);

        // 边界对拟合圆的支持角不足 → 这个形状根本不是环（划痕弧、台面亮带粘连、断裂的碎块），
        // 拟合出的圆心/半径没有意义。此时报测量失败交人工复核，而不是给出一个自信的错误结论。
        // 实测 28 张真机良品最低支持角 0.60（15.png），故下限取 0.50 留出余量。
        //
        // **失败时仍把 Solid 与拟合圆一起带出**（2026-09-20 加）：调用方的第五路径
        // （多起点重试）需要"最佳候选圆"当搜索中心、并需要那块掩码去算质量指标。
        // 没有它们，第五路径就只能从零重建，而那正是本项要避免的重复。
        var supportFloor = minArcSupport > 0 ? minArcSupport : cfg.MinRingArcSupport;
        if (fit.Value.ArcSupport < supportFloor)
            return WithFailure(detection,
                $"焊环边界支撑弧段不足（仅覆盖 {fit.Value.ArcSupport:P0} 圆周，下限 {supportFloor:P0}）",
                RingFailure.Unmeasurable, solid, fit.Value.Circle);

        var circle = fit.Value.Circle;
        return new RingDetection
        {
            Ok = true,
            Center = circle.Center,
            Radius = circle.Radius,
            RadiusRatio = circle.Radius / baseSize,
            Roundness = roundness,
            ArcSupport = fit.Value.ArcSupport,
            MaskSupportRelaxed = supportFloor < cfg.MinRingArcSupport,
            Solid = solid.Clone(),
            Texture = energy,
            Threshold = threshold,
            BaseSize = baseSize
        };
    }

    /// <summary>
    /// **以熔核自身圆心为射线原点**做自洽边界精修——把红圈从"纹理掩码连通域的外缘"
    /// 搬到"熔核外沿那条真实的边"上。
    ///
    /// 为什么必须另起一条估计器：<see cref="Detect"/> 给出的是"纹理掩码连通域"的外接圆，
    /// 而掩码本身在**镜面冲白**的一侧是缺的、在**熔核外侧的细暗圈/散斑带**上是多的。
    /// 两个方向的污染都会把整体圆拟合的圆心推走，且推的方向跟着光照走。
    ///
    /// 本方法每一步都针对这两个污染源：
    ///   · 用 |∇I|（对明暗极性免疫）而不是灰度/纹理掩码的边界；
    ///   · 角向聚合用**中位数**（对"近一半圆周被冲白"免疫）；
    ///   · 逐射线半径只在全局选定的 R* 附近 ±bandRatio 内取（避免不同射线量到不同结构）；
    ///   · 圆心与外半径用一阶谐波拟合**同时**解出，并迭代到自洽。
    ///
    /// **射线原点必须是熔核自己的圆心，不能是孔心**（2026-09-18 修正，见
    /// <see cref="InspectionConfig.WeldRingRefineEdgeFraction"/> 的实测记录）：
    /// 以孔心为原点等于假设"熔核与极柱同心"，而"焊偏"这个量本身正是要检验这个假设；
    /// 于是估计器会锁死在所有与孔心同心的圆结构上（极柱端面边、暗缝隙、凹陷细暗圈、
    /// 冲白区的边界），把测得的偏移量抹平到接近零。本方法只以种子圆心为原点，
    /// 搜索带由**中位剖面里第一段显著平台**给出，与孔心位置无关。
    ///
    /// **边界判据是"纹理最后出现在哪里"，不是"梯度最大在哪里"**：中位剖面在真熔核外沿处
    /// 是一个台阶（外沿之外被冲白、没有梯度），而细圈会在更外面形成一根**孤立尖峰**
    /// （实测 342 vs 平台 67）。取"最外侧仍显著半径"会让 ρ 跳到细圈上，
    /// 红圈于是整体外扩并把圆心带向细圈一侧——这正是用户 2026-09-18 报的现象。
    ///
    /// 返回 null 表示"这次不精修"，调用方保持原结果。
    /// </summary>
    /// <param name="gray8">工作分辨率的灰度图（已中值滤波，与 <see cref="Detect"/> 同一张）。</param>
    /// <param name="seedCenter"><see cref="Detect"/> 给出的熔核圆心，迭代起点。</param>
    /// <param name="seedRadius"><see cref="Detect"/> 给出的熔核半径，搜索带的尺度基准。</param>
    /// <param name="altCenter">
    /// **第二起点**（传孔位圆心）。细圈污染帧上 <see cref="Detect"/> 给出的**种子圆心本身就是歪的**
    /// ——掩码被细圈/外侧碎斑带撑大后，最大内切圆圆心会偏向那一侧（实测偏 20~45px），
    /// 而自锚迭代会继承这个偏置。给估计器一个与之独立的起点（孔心；它与熔核大致同心但**不作为答案**），
    /// 两个起点各自收敛后取"边界更像圆"的那个，可以把这类帧救回来。
    /// 传 null（或不传）即退化为单起点行为。
    /// </param>
    public static RingRefinement? RefineBoundary(Mat gray8, Point2d seedCenter, double seedRadius,
        InspectionConfig cfg, Point2d? altCenter = null)
    {
        return RefineBoundary(gray8, seedCenter, seedRadius, cfg, altCenter, null);
    }

    /// <summary>
    /// 带**诊断回调**的重载：<paramref name="trace"/> 会在每一级外带宽上被调用一次，
    /// 报告两个起点各自的结果与分歧。仅供 <c>tools/Diag</c> 使用——
    /// 没有它就只能看到"最终选了谁"，看不到"阶梯有没有启动、放宽后有没有收敛"，
    /// 而本轮的修复正是靠这条信息才定位到根因的。
    /// </summary>
    public static RingRefinement? RefineBoundary(Mat gray8, Point2d seedCenter, double seedRadius,
        InspectionConfig cfg, Point2d? altCenter, Action<string>? trace)
    {
        var baseSize = Math.Min(gray8.Height, gray8.Width);
        if (baseSize < 64 || seedRadius < 4) return null;
        if (cfg.WeldRingRefineProfileOuterRatio <= cfg.WeldRingRefineProfileInnerRatio) return null;

        // ---- 外带宽逐级放宽（截断自适应）----
        // 见"截断比"的定义（Run 末尾）。窄带（默认外 0.08）是常态下的最优选择——它把熔核外侧那条
        // "细密灰斑噪声带/凹陷细暗圈"挡在搜索范围之外；但 ρ 被冲白区低估时窄带会把真边界截在带外端，
        // 于是必须放宽重跑。**只有在实测截断比超限时才逐级放宽**，健康帧根本不进第二级，
        // 所以这条阶梯不改变任何常态帧的行为（实测 `SN_...0007` 24 帧里只有第 1 帧走到第二级）。
        //
        // **2026-09-19 试过把"两个起点的分歧"也作为放宽的触发条件，实测被数据否定，已回退**：
        // 动机是黑环帧上窄带会把第二起点（孔心）困在一个偏小、但支撑弧段假性偏高的圆上
        // （`SN_...0004` 第 0 帧：窄带分歧 7.0px，放宽到 ±20% 后两个起点都收敛到同一个圆、
        // 与厂商差从 4.5px 收到 2.0px）。在 40 SN / 960 帧上实测：**28 帧变好、13 帧变差**——
        // 变差的正是当初靠第二起点救回的那两帧（`SN_...0003` 的 OK_933/OK_936，见 FIXPLAN 4.9），
        // 它们的分歧本就很小、不该被放宽；放宽后外侧散斑带进入搜索范围，圆心反而被推走
        // （+25~30px）。**尾部指标净退化：max 49.2px → 63.5px，半径比带外帧 0 → 1**。
        // 故本项不保留（诊断量 `RingRefinement.ResidualScatter` 与 trace 回调保留，
        // 它们只读不写、不改变任何行为）。
        var attempts = new List<RingRefinement?>();
        foreach (var bandOuter in BandOuterLadder(cfg))
        {
            var attempt = Attempt(bandOuter, out var clipped);
            if (attempt is not { } res)
            {
                trace?.Invoke($"    带宽 +{bandOuter:F2}: 无结果");
                continue;
            }
            trace?.Invoke($"    带宽 +{bandOuter:F2}: 采用 ({res.Circle.Center.X:F1},{res.Circle.Center.Y:F1})"
                          + $" r={res.Circle.Radius:F1} arc={res.ArcSupport:P0}"
                          + $" 截断比={clipped:F2} 残差={res.ResidualScatter:F4}");
            attempts.Add(res);
            if (clipped <= cfg.WeldRingRefineMaxClippedRatio)
                return res;
        }

        return attempts.Count > 0 ? attempts[0] : null;

        // 单级外带宽下的完整尝试：跑主起点，再按需跑第二起点并择优，返回选中结果的截断比。
        RingRefinement? Attempt(double bandOuterRatio, out double clippedRatio)
        {
            clippedRatio = 1.0;
            var primary = Run(seedCenter, bandOuterRatio, out var pClip);
            trace?.Invoke($"      [主起点 熔核心 ({seedCenter.X:F1},{seedCenter.Y:F1})] "
                          + Describe(primary, pClip));
            if (altCenter is not { } alt)
            {
                clippedRatio = pClip;
                return primary;
            }
            // 两个起点太近就没有交叉校验的意义（正常帧上种子圆心本来就准）
            if (Geo.Dist(alt, seedCenter) < 0.10 * seedRadius)
            {
                trace?.Invoke($"      第二起点距主起点仅 {Geo.Dist(alt, seedCenter):F1}px"
                              + $"（< {0.10 * seedRadius:F1}px），跳过");
                clippedRatio = pClip;
                return primary;
            }

            var secondary = Run(alt, bandOuterRatio, out var sClip);
            trace?.Invoke($"      [第二起点 孔心 ({alt.X:F1},{alt.Y:F1})] "
                          + Describe(secondary, sClip));
            if (secondary is not { } s)
            {
                clippedRatio = pClip;
                return primary;
            }
            if (primary is not { } p)
            {
                clippedRatio = sClip;
                return s;
            }

            // 选"边界更像圆"的那个（固定相对容差下的内点占比）。第二起点只有在**自身够可信**时才允许
            // 覆盖第一起点：半径必须落回族值带、支撑弧段达标、且严格优于第一起点——
            // 否则保留第一起点。实测第二起点偶发收敛到孔洞/内沿尺度的小圆（半径比 0.03~0.06），
            // 这两道闸门正好把它们挡在外面。
            //
            // **本规则保持不变**（2026-09-19 复核过）：试过用"归一化残差更小者胜"替换它，
            // 但实测两个起点的残差经常只差 0.0002（0.0302 vs 0.0304），没有区分力；
            // 也试过用"两起点分歧"触发带宽放宽（见上面阶梯的注释），40 SN / 960 帧上
            // 尾部指标净退化，已回退。详见 FIXPLAN 第 8 节。
            var sRatio = s.Circle.Radius / baseSize;
            // "未被闸门干预时本该胜出者"——即只看半径族值带 + 弧段的那条既有规则。
            // 截断比必须取它（见下面 clippedRatio 的说明），所以先把这一步单独算出来。
            var secondaryWouldWin = sRatio >= cfg.MinRingRadiusRatio
                                    && sRatio <= cfg.RingRadiusFamilyMaxRatio
                                    && s.ArcSupport >= cfg.MinRingArcSupport
                                    && s.ArcSupport > p.ArcSupport;
            var altTrustworthy = secondaryWouldWin;

            // ---- 边界粗糙度闸门（2026-09-20，FIXPLAN 第 9 节）----
            //
            // 上面那道"弧段更高者胜"会被**与孔同轴的机加工锐边**（工件上 30mm 凹陷台阶的
            // 阴影边，"黑环"）骗到：它在 |∇I| 上比焊道纹理强一倍（实测剖面峰值 117.5 vs 70.6），
            // 把孔锚起点吸走后给出一个假性偏高的支撑弧段（95.3% vs 82.4%），于是选错了那个圆。
            //
            // 物理上分得开：**焊道外沿是鱼鳞状凝固组织、必然粗糙；机加工边是锐边、必然光滑。**
            // 所以要求第二起点的边界粗糙度（ResidualScatter）不得低于主起点的
            // cfg.WeldRingRefineMinRoughnessRatio 倍。实测 OK_441 比值 0.43（该拦）、
            // OK_936 比值 1.60（该放）、OK_444 比值 0.74（该放），取 0.5 落在中间。
            //
            // 这一步**只投反对票**：不新增搜索、不放宽带宽、不改变任何几何，
            // 最坏后果是退回"只用主起点"的旧行为。
            if (altTrustworthy && cfg.WeldRingRefineMinRoughnessRatio > 0
                && p.ResidualScatter > 0
                && s.ResidualScatter < cfg.WeldRingRefineMinRoughnessRatio * p.ResidualScatter)
            {
                altTrustworthy = false;
                trace?.Invoke($"      → 拦下第二起点接管：其边界粗糙度 {s.ResidualScatter:F4}"
                              + $" < {cfg.WeldRingRefineMinRoughnessRatio:F2}×主起点 {p.ResidualScatter:F4}"
                              + "（像机加工锐边而非焊道纹理），保留主起点");
            }

            // ---- ρ 一致性闸门（2026-09-20，FIXPLAN 第 10 节）——**默认关闭，实测被否定** ----
            //
            // 它在 960 帧上是好判据（9 帧变好 1 帧变差、p95 17.1→14.7、max 不变），
            // 但**不能上线**：它唯一变差的那帧（`SN_...0003` 的 `16-48-48 544 / OK_933`）
            // 有独立的物理证据说明它判错了——该帧熔核半径必须 ≈489 原图像素
            // （同 SN 其余 20 帧 483~493 + 厂商 `.vdb` 489.1），而闸门保留的主起点给 527（+7.8%）。
            // 根因是 **ρ 比值只能测出"两个起点不一致"，测不出"是谁错了"**：
            // 该帧错的是主起点（种子被碎斑带撑歪），闸门却去惩罚正确的第二起点。
            // 详见 InspectionConfig.WeldRingRefineMinRhoRatio 与 FIXPLAN 第 10 节。
            if (altTrustworthy && cfg.WeldRingRefineMinRhoRatio > 0 && p.EdgeRadius > 0
                && s.EdgeRadius < cfg.WeldRingRefineMinRhoRatio * p.EdgeRadius)
            {
                altTrustworthy = false;
                trace?.Invoke($"      → 拦下第二起点接管：其 ρ={s.EdgeRadius:F1}"
                              + $" < {cfg.WeldRingRefineMinRhoRatio:F2}×主起点 ρ={p.EdgeRadius:F1}"
                              + "（两个起点量到的不是同一条边界），保留主起点");
            }

            trace?.Invoke($"      仲裁: 第二起点半径比={sRatio:F4}"
                          + $" (族值带 [{cfg.MinRingRadiusRatio:F3},{cfg.RingRadiusFamilyMaxRatio:F3}])"
                          + $" 弧段={s.ArcSupport:P1} vs 主起点 {p.ArcSupport:P1}"
                          + $" → {(altTrustworthy ? "第二起点胜出" : "保留主起点")}");

            // **截断比必须沿用"未被闸门干预时本该胜出者"的那一个**（`secondaryWouldWin`），
            // 不能换成被保留者的。这不是细节：截断比决定**要不要放宽带宽重跑**（见 BandOuterLadder），
            // 而闸门的职责只是"在本级带宽已算出的两个圆之间选一个"——它一旦改变搜索走向，
            // 就会把结果推向另一个（更宽的）带宽下的圆。实测 `SN_...1802` 的 `14-58-52 472`：
            // 主起点在本级的截断比 0.17 刚过 0.15 门限，被保留后触发放宽，
            // 于是采用的圆从"本级 62.8px 半径、离厂商 19.6px"变成"放宽级 64.6px 半径、
            // 离厂商 42.7px"——多出的 23px 偏差全部来自这个副作用，与闸门要拦的东西无关。
            // 沿用 sClip 后该帧回到 19.16px（若不动闸门本可以是 12.55px，即闸门本身只让它多偏 6.6px）。
            clippedRatio = secondaryWouldWin ? sClip : pClip;
            return altTrustworthy ? s : p;
        }

        // 诊断用：把一次 Run 的结果压成一行。trace 为 null 时不会被调用，
        // 因此对算法行为零影响（与 FIXPLAN 第 8.4 节"只加诊断、不改行为"的做法一致）。
        static string Describe(RingRefinement? r, double clipped) => r is { } v
            ? $"→ ({v.Circle.Center.X:F1},{v.Circle.Center.Y:F1}) r={v.Circle.Radius:F1}"
              + $" arc={v.ArcSupport:P1} 截断比={clipped:F2} 残差={v.ResidualScatter:F4}"
              + $" ρ={v.EdgeRadius:F1}"
            : $"→ 无结果（截断比={clipped:F2}）";

        // 从某个起点跑一遍完整估计（ROI 上采样 → 中位剖面定 ρ → 逐射线取最后一段显著样本 →
        // 一阶谐波拟合圆心与外半径 → 迭代至自洽）。
        RingRefinement? Run(Point2d origin, double bandOuterRatio, out double clippedRatio)
        {
            clippedRatio = 1.0;
            var rays = Math.Clamp(cfg.WeldRingRefineRays, 90, 1440);

            // ---- ROI 上采样：工作分辨率下熔核半径只有约 60px，3× 才有足够的径向/角向精度 ----
            var up = Math.Clamp(cfg.WeldRingRefineUpsample, 1, 4);
            var margin = (int)Math.Ceiling(seedRadius * cfg.WeldRingRefineProfileOuterRatio) + 8;
            var left = Math.Max(0, (int)origin.X - margin);
            var top = Math.Max(0, (int)origin.Y - margin);
            var right = Math.Min(gray8.Width, (int)origin.X + margin + 1);
            var bottom = Math.Min(gray8.Height, (int)origin.Y + margin + 1);
            if (right - left < 16 || bottom - top < 16) return null;

            using var roi = new Mat(gray8, new Rect(left, top, right - left, bottom - top));
            using var scaled = new Mat();
            Cv2.Resize(roi, scaled, new Size(), up, up,
                up > 1 ? InterpolationFlags.Cubic : InterpolationFlags.Nearest);
            using var magnitude = GradientMagnitude(scaled);
            var magIdx = magnitude.GetGenericIndexer<float>();
            var width = scaled.Width;
            var height = scaled.Height;

            var cos = new double[rays];
            var sin = new double[rays];
            for (var k = 0; k < rays; k++)
            {
                var angle = 2 * Math.PI * k / rays;
                cos[k] = Math.Cos(angle);
                sin[k] = Math.Sin(angle);
            }

            var valid = new bool[rays];
            var boundary = new double[rays];
            var samples = new float[rays];
            var center = new Point2d((origin.X - left) * up, (origin.Y - top) * up);
            var radius = seedRadius;          // 工作分辨率像素（未乘 up）
            var edge = 0.0;

            // 从**当前圆心**出发量一圈：
            //   1. 径向 |∇I| 中位剖面 g(r)（中位数对"最多一半圆周被冲白"免疫）；
            //   2. ρ = g(r) 里**从内向外第一段显著平台的外端**——这是"熔核纹理到哪儿结束"；
            //   3. 逐射线只在 ρ 附近窄带内取"最后一个显著样本"（不是带内梯度最大处）。
            bool Measure(Point2d origin)
            {
                // ---- 1+2. 中位剖面与平台外端 ----
                var lo = radius * cfg.WeldRingRefineProfileInnerRatio * up;
                var hi = radius * cfg.WeldRingRefineProfileOuterRatio * up;
                if (hi <= lo + 8) return false;
                var count = (int)Math.Floor(hi - lo) + 1;
                var profile = new double[count];
                for (var j = 0; j < count; j++)
                {
                    var r = lo + j;
                    for (var k = 0; k < rays; k++)
                        samples[k] = Sample(magIdx, origin, cos[k], sin[k], r, width, height);
                    Array.Sort(samples);
                    profile[j] = Geo.Percentile(samples, 50);
                }

                var profileMax = 0.0;
                foreach (var v in profile)
                    if (v > profileMax) profileMax = v;
                if (profileMax < 8) return false;    // 视野里没有边（纯平场）

                // 平台高度取"显著样本的 75 分位"而不是剖面最大值：细圈的尖峰可以比熔核平台高 5 倍
                // （实测帧 342 vs 67），用最大值当基准会把熔核平台整段判成"不显著"，
                // 于是 ρ 又会跑到细圈上——这正是本项要修的那个失败。
                var significant = new List<float>(count);
                foreach (var v in profile)
                    if (v > 0.05 * profileMax)
                        significant.Add((float)v);
                if (significant.Count < 8) return false;
                significant.Sort();
                var plateau = Geo.Percentile(significant, cfg.WeldRingRefinePlateauPercentile);
                if (plateau < 8) return false;

                var threshold = cfg.WeldRingRefineEdgeFraction * plateau;
                var minRun = Math.Max(3, (int)Math.Round(cfg.WeldRingRefineMinRunRatio * (hi - lo)));
                // 从内向外找**第一段**长度达标的连续显著区，取该段的**外端**作为 ρ。
                // 之后隔着实零区的第二个峰（反射细圈）不再考虑——细圈的尖峰可以比熔核平台还高，
                // 旧的"最外侧仍显著半径"判据因此会把细圈当成熔核外沿（用户现象：红圈套在细圆上）。
                var lastSignificant = -1;
                var run = 0;
                for (var j = 0; j < count; j++)
                {
                    if (profile[j] > threshold)
                    {
                        run++;
                        continue;
                    }
                    if (run >= minRun)
                    {
                        lastSignificant = j - 1;
                        break;
                    }
                    run = 0;
                }
                if (lastSignificant < 0 && run >= minRun)
                    lastSignificant = count - 1;
                if (lastSignificant < 0) return false;
                edge = lo + lastSignificant;

                // ---- 3. 逐射线在 ρ 窄带内取"最后一个显著样本" ----
                var bandLo = edge * (1 - cfg.WeldRingRefineBandInnerRatio);
                var bandHi = edge * (1 + bandOuterRatio);
                if (bandHi <= bandLo) return false;
                var bandCount = (int)Math.Floor(bandHi - bandLo) + 1;
                var rayValues = new float[bandCount];
                var deadLevel = cfg.WeldRingRefineDeadRayFraction * threshold;
                for (var k = 0; k < rays; k++)
                {
                    var peak = 0f;
                    for (var j = 0; j < bandCount; j++)
                    {
                        var v = Sample(magIdx, origin, cos[k], sin[k], bandLo + j, width, height);
                        rayValues[j] = v;
                        if (v > peak) peak = v;
                    }
                    // 整条带里都几乎没有梯度 = 这条射线落在镜面冲白区，没有可量的边
                    if (peak <= deadLevel)
                    {
                        boundary[k] = double.NaN;
                        valid[k] = false;
                        continue;
                    }
                    var rayThreshold = cfg.WeldRingRefinePerRayFraction * peak;
                    var bestR = double.NaN;
                    for (var j = bandCount - 1; j >= 0; j--)
                        if (rayValues[j] > rayThreshold)
                        {
                            bestR = bandLo + j;      // 从外向内找第一个显著样本 = 最后一段的外端
                            break;
                        }
                    boundary[k] = bestR;
                    valid[k] = !double.IsNaN(bestR);
                }
                return true;
            }

            var iterations = Math.Clamp(cfg.WeldRingRefineIterations, 1, 8);
            for (var it = 0; it < iterations; it++)
            {
                if (!Measure(center)) return null;

                // 一阶谐波稳健拟合 b(θ) ≈ R + ex·cosθ + ey·sinθ：
                // ex/ey 就是"当前圆心离真实圆心还有多远"，R 是新的半径（上采样像素）。
                if (!FitFirstHarmonic(boundary, valid, cos, sin, out var ex, out var ey,
                        out var fitRadius, out _))
                    return null;
                if (fitRadius < 4 * up) return null;

                center = new Point2d(center.X + ex, center.Y + ey);
                radius = fitRadius / up;
                if (Math.Sqrt(ex * ex + ey * ey) < cfg.WeldRingRefineConvergePx * up)
                    break;
            }

            // 末轮：以收敛后的圆心重新量一圈，并用**固定相对容差**判内点。
            // 不能用拟合内部那套 2σ 自适应容差：它会追着被截断的边界把容差放宽，
            // 从而把一条根本不成圈的边界也判成内点（见
            // <see cref="InspectionConfig.WeldRingRefineInlierTolerance"/>）。
            if (!Measure(center)) return null;
            var finalRadiusUp = radius * up;
            var tol = Math.Max(cfg.WeldRingRefineInlierTolerance * finalRadiusUp, 2.0 * up);
            var inliers = 0;
            var distances = new List<double>(rays);
            for (var k = 0; k < rays; k++)
            {
                if (!valid[k]) continue;
                if (Math.Abs(boundary[k] - finalRadiusUp) < tol)
                {
                    inliers++;
                    distances.Add(boundary[k]);
                }
            }
            var arcSupport = inliers / (double)rays;
            if (arcSupport < cfg.WeldRingRefineMinArcSupport || distances.Count < rays / 4)
                return null;

            // 截断比：有效射线的边界落在**外带边界 2 个上采样采样点以内**的比例。
            // ρ 由**全周中位剖面**给出，当熔核有一大段被镜面冲白（该段完全没有梯度）时，
            // "中位仍显著"的半径会被系统性低估——实测 `SN_...0007` 第 1 帧 ρ=450 而真值约 505，
            // 于是 [0.9, 1.08]×ρ 的窄带把可量到的边界**硬截在 484**：红圈偏小 23px、圆心偏 19px，
            // 而且被截断的射线全部落在同一个半径上，会把"支撑弧段"**假性抬高**（该帧 0.86，
     		// 而真实可信度只有 0.74）。健康帧实测截断比 0.00~0.09，该帧 0.45，分离度 5 倍以上。
            // 判据里的 2.0 是**上采样采样点的量化单位**（径向搜索步长），不是场景相关的常数。
            const double clipMarginSamples = 2.0;
            var bandHiUp = edge * (1 + bandOuterRatio);
            var clippedCount = 0;
            foreach (var d in distances)
                if (d > bandHiUp - clipMarginSamples)
                    clippedCount++;
            clippedRatio = distances.Count > 0 ? clippedCount / (double)distances.Count : 1.0;

            distances.Sort();
            var finalRadius = distances[distances.Count / 2] / up;
            if (finalRadius < 4) return null;

            // 归一化残差：内点到"以最终圆心为心、finalRadius 为半径"的径向标准差 ÷ 半径。
            // 这是两个起点仲裁时用的**客观**量（见 RingRefinement.ResidualScatter 的说明）。
            var finalRadiusUp2 = finalRadius * up;
            double sumSq = 0;
            foreach (var d in distances)
            {
                var e = d - finalRadiusUp2;
                sumSq += e * e;
            }
            var residualScatter = finalRadiusUp2 > 0
                ? Math.Sqrt(sumSq / distances.Count) / finalRadiusUp2
                : 1.0;

            // 半径相对种子的修正量上限：防止在"种子完全不对"时把圆搬到另一个结构上。
            if (seedRadius > 0 &&
                Math.Abs(finalRadius - seedRadius) / seedRadius > cfg.WeldRingRefineMaxRadiusCorrection)
                return null;

            var finalCenter = new Point2d(center.X / up + left, center.Y / up + top);
            return new RingRefinement(new CircleFit(finalCenter, finalRadius), arcSupport,
                edge / up, residualScatter, clippedRatio);
        }
    }

    /// <summary>Sobel 梯度幅值（CV_32F）。用幅值而非有符号梯度：熔核边界在有的帧是暗→亮、
    /// 有的帧是亮→暗（取决于反光方向），幅值对极性免疫。</summary>
    private static Mat GradientMagnitude(Mat gray8)
    {
        using var f = new Mat();
        gray8.ConvertTo(f, MatType.CV_32F);
        using var gx = new Mat();
        using var gy = new Mat();
        Cv2.Sobel(f, gx, MatType.CV_32F, 1, 0, ksize: 3);
        Cv2.Sobel(f, gy, MatType.CV_32F, 0, 1, ksize: 3);
        var magnitude = new Mat();
        Cv2.Magnitude(gx, gy, magnitude);
        return magnitude;
    }

    /// <summary>在 (center + r·(cos,sin)) 处采样，越界返回 0（越界即视野外，不是边）。</summary>
    private static float Sample(Mat.Indexer<float> idx, Point2d center, double cos, double sin,
        double r, int width, int height)
    {
        var x = (int)Math.Round(center.X + cos * r);
        var y = (int)Math.Round(center.Y + sin * r);
        if ((uint)x >= (uint)width || (uint)y >= (uint)height)
            return 0f;
        return idx[y, x];
    }

    /// <summary>
    /// 迭代重加权的一阶谐波（圆）拟合：解 b(θ) = R + ex·cosθ + ey·sinθ 的最小二乘，
    /// 按 2σ（下限 3px）剔点后重解，直到内点稳定。返回 false 表示有效点太少或法方程退化。
    ///
    /// 注意这里用的是**自适应**容差，只负责把圆心估稳（对尾部稳健）；
    /// "这条边界到底成不成圈"必须由调用方用**固定相对容差**另判
    /// （见 <see cref="InspectionConfig.WeldRingRefineInlierTolerance"/>），
    /// 否则自适应容差会追着被截断的边界放宽，把坏边界也判成内点。
    /// </summary>
    private static bool FitFirstHarmonic(double[] b, bool[] valid, double[] cos, double[] sin,
        out double ex, out double ey, out double radius, out double hitRatio)
    {
        ex = 0;
        ey = 0;
        radius = 0;
        hitRatio = 0;
        var n = b.Length;
        var keep = new bool[n];
        var kept = 0;
        for (var i = 0; i < n; i++)
        {
            keep[i] = valid[i];
            if (keep[i]) kept++;
        }
        const int minKept = 24;
        if (kept < minKept)
            return false;

        var residual = new double[n];
        var r = 0.0;
        for (var iteration = 0; iteration < 8; iteration++)
        {
            if (!SolveHarmonic(b, keep, cos, sin, out r, out ex, out ey))
                return false;

            double sum = 0, sumSq = 0;
            var count = 0;
            for (var i = 0; i < n; i++)
            {
                if (!keep[i]) continue;
                residual[i] = b[i] - (r + ex * cos[i] + ey * sin[i]);
                sum += residual[i];
                count++;
            }
            var mean = sum / count;
            for (var i = 0; i < n; i++)
                if (keep[i])
                    sumSq += (residual[i] - mean) * (residual[i] - mean);
            var sigma = Math.Sqrt(sumSq / count);
            var limit = Math.Max(2.0 * sigma, 3.0);

            var next = new bool[n];
            var nextCount = 0;
            for (var i = 0; i < n; i++)
            {
                next[i] = keep[i] && Math.Abs(residual[i] - mean) < limit;
                if (next[i]) nextCount++;
            }
            if (nextCount < minKept || nextCount == count)
                break;
            Array.Copy(next, keep, n);
            kept = nextCount;
        }

        hitRatio = kept / (double)n;
        radius = r;
        return true;
    }

    /// <summary>解 3×3 正规方程 [1 cos sin]·[R ex ey]ᵀ = b（只对 keep 的点求和）。</summary>
    private static bool SolveHarmonic(double[] b, bool[] keep, double[] cos, double[] sin,
        out double r0, out double ex, out double ey)
    {
        r0 = ex = ey = 0;
        double a00 = 0, a01 = 0, a02 = 0, a11 = 0, a12 = 0, a22 = 0;
        double y0 = 0, y1 = 0, y2 = 0;
        for (var i = 0; i < b.Length; i++)
        {
            if (!keep[i]) continue;
            var c = cos[i];
            var s = sin[i];
            a00 += 1;
            a01 += c;
            a02 += s;
            a11 += c * c;
            a12 += c * s;
            a22 += s * s;
            y0 += b[i];
            y1 += c * b[i];
            y2 += s * b[i];
        }
        var det = a00 * (a11 * a22 - a12 * a12)
                - a01 * (a01 * a22 - a12 * a02)
                + a02 * (a01 * a12 - a11 * a02);
        if (Math.Abs(det) < 1e-9)
            return false;
        r0 = (y0 * (a11 * a22 - a12 * a12)
            - a01 * (y1 * a22 - a12 * y2)
            + a02 * (y1 * a12 - a11 * y2)) / det;
        ex = (a00 * (y1 * a22 - a12 * y2)
            - y0 * (a01 * a22 - a12 * a02)
            + a02 * (a01 * y2 - y1 * a02)) / det;
        ey = (a00 * (a11 * y2 - y1 * a12)
            - a01 * (a01 * y2 - y1 * a02)
            + y0 * (a01 * a12 - a11 * a02)) / det;
        return true;
    }

    /// <summary>局部标准差：E = sqrt(box(g²) − box(g)²)。粗糙面高、光滑面低。</summary>
    private static Mat TextureEnergy(Mat gray8, int win)
    {
        using var f = new Mat();
        gray8.ConvertTo(f, MatType.CV_32F);
        using var f2 = new Mat();
        Cv2.Multiply(f, f, f2);

        using var mean = new Mat();
        using var meanSq = new Mat();
        Cv2.BoxFilter(f, mean, MatType.CV_32F, new Size(win, win));
        Cv2.BoxFilter(f2, meanSq, MatType.CV_32F, new Size(win, win));

        using var meanSquared = new Mat();
        Cv2.Multiply(mean, mean, meanSquared);

        var energy = new Mat();
        Cv2.Subtract(meanSq, meanSquared, energy);
        // 浮点舍入可能让平坦区出现极小负值，开方前必须截断到 0
        Cv2.Threshold(energy, energy, 0, 0, ThresholdTypes.Tozero);
        Cv2.Sqrt(energy, energy);
        return energy;
    }

    private static void ZeroBorder(Mat mat, int margin)
    {
        if (margin <= 0)
            return;
        var w = mat.Width;
        var h = mat.Height;
        using (var r = new Mat(mat, new Rect(0, 0, w, margin))) r.SetTo(Scalar.All(0));
        using (var r = new Mat(mat, new Rect(0, h - margin, w, margin))) r.SetTo(Scalar.All(0));
        using (var r = new Mat(mat, new Rect(0, 0, margin, h))) r.SetTo(Scalar.All(0));
        using (var r = new Mat(mat, new Rect(w - margin, 0, margin, h))) r.SetTo(Scalar.All(0));
    }

    /// <summary>
    /// 把**包围盒贴到边框禁区**的连通域整块清零。必须在形态学**之前**调用，理由见 <see cref="Detect"/> 里的实测记录：
    /// 闭运算会把熔核与贴边的工件外框桥接成一块，之后再按贴边剔除就会连熔核一起扔掉。
    /// margin ≤ 0 时不做任何事。
    /// </summary>
    private static void RemoveBorderComponents(Mat mask, int margin)
    {
        if (margin <= 0)
            return;
        var w = mask.Width;
        var h = mask.Height;
        using var labels = new Mat();
        using var stats = new Mat();
        using var centroids = new Mat();
        var count = Cv2.ConnectedComponentsWithStats(mask, labels, stats, centroids,
            PixelConnectivity.Connectivity8);
        for (var i = 1; i < count; i++)
        {
            var left = stats.At<int>(i, (int)ConnectedComponentsTypes.Left);
            var top = stats.At<int>(i, (int)ConnectedComponentsTypes.Top);
            var width = stats.At<int>(i, (int)ConnectedComponentsTypes.Width);
            var height = stats.At<int>(i, (int)ConnectedComponentsTypes.Height);
            if (left > margin && top > margin &&
                left + width < w - margin && top + height < h - margin)
                continue;

            // 用 labels==i 的掩码清零该连通域（ROI + SetTo 只对单通道 8U 成立，这里显式取掩码更稳）
            using var comp = new Mat();
            Cv2.Compare(labels, new Scalar(i), comp, CmpType.EQ);
            mask.SetTo(Scalar.All(0), comp);
        }
    }

    /// <summary>
    /// 精修的外带宽阶梯：先按配置的 <see cref="InspectionConfig.WeldRingRefineBandOuterRatio"/> 试，
    /// 截断比不达标时逐级用 <see cref="InspectionConfig.WeldRingRefineBandOuterFallbacks"/> 放宽重跑。
    /// 去重并升序，保证"只在必要时才放宽"且不会重复跑同一级。
    /// </summary>
    private static IEnumerable<double> BandOuterLadder(InspectionConfig cfg)
    {
        var values = new List<double> { cfg.WeldRingRefineBandOuterRatio };
        values.AddRange(cfg.WeldRingRefineBandOuterFallbacks);
        var seen = new HashSet<double>();
        foreach (var v in values.Where(v => v > 0).OrderBy(v => v))
            if (seen.Add(Math.Round(v, 6)))
                yield return v;
    }

    private static void DisposeAll(IEnumerable<Candidate> candidates)
    {
        foreach (var c in candidates)
            c.Dispose();
    }

    /// <summary>
    /// 填充连通域内部的孔洞。先补 1 像素边再泛洪：紧致的包围盒可能把连通域像素放在 (0,0)，
    /// 直接从那一点泛洪会把整个连通域当成背景（这是原型阶段的真实 bug）。
    /// </summary>
    private static Mat FillHoles(Mat bin)
    {
        using var padded = new Mat();
        Cv2.CopyMakeBorder(bin, padded, 1, 1, 1, 1, BorderTypes.Constant, Scalar.Black);
        using var flooded = padded.Clone();
        Cv2.FloodFill(flooded, new Point(0, 0), Scalar.White);

        using var holes = new Mat();
        Cv2.BitwiseNot(flooded, holes);
        using var inner = new Mat(holes, new Rect(1, 1, bin.Width, bin.Height));

        var result = new Mat();
        Cv2.BitwiseOr(bin, inner, result);
        Cv2.Threshold(result, result, 0, 255, ThresholdTypes.Binary);
        return result;
    }

    private static List<Point2d>? LargestContourPoints(Mat solid)
    {
        using var forContours = solid.Clone();
        Cv2.FindContours(forContours, out var contours, out _,
            RetrievalModes.External, ContourApproximationModes.ApproxNone);
        if (contours.Length == 0)
            return null;
        var biggest = contours.MaxBy(c => Cv2.ContourArea(c))!;
        return biggest.Select(p => new Point2d(p.X, p.Y)).ToList();
    }

    /// <summary>分位数抽样步长：纹理图本身是低频的（两次盒滤波），隔点抽样无影响。</summary>
    private static int SampleStep(int baseSize) => Math.Max(1, baseSize / 256);

    private static RingDetection WithFailure(RingDetection source, string reason, RingFailure kind,
        Mat? solid = null, CircleFit? bestCandidate = null) => new()
    {
        Ok = false,
        Failure = reason,
        FailureKind = kind,
        Solid = solid?.Clone(),
        BestCandidate = bestCandidate,
        Texture = source.Texture,
        Threshold = source.Threshold,
        BaseSize = source.BaseSize
    };
    private sealed class Candidate(Rect bbox, Mat filled, int filledArea,
        double radiusEq, double dtRadius, Point2d dtCenter) : IDisposable
    {
        public Rect BBox { get; } = bbox;
        public Mat Filled { get; } = filled;
        public int FilledArea { get; } = filledArea;

        /// <summary>圆度 = 最大内切圆半径 / 等效半径。实心圆盘 ≈1，细长条/划痕很小。</summary>
        public double Roundness { get; } = dtRadius / Math.Max(radiusEq, 1e-6);

        /// <summary>距离变换峰值位置（最大内切圆圆心），已从包围盒局部坐标平移回全图坐标。</summary>
        public Point2d InscribedCenter { get; } = dtCenter;

        public void Dispose() => Filled.Dispose();
    }
}
