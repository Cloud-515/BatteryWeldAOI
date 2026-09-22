namespace BatteryWeldAOI.Core.Vision;

using BatteryWeldAOI.Core.Models;
using OpenCvSharp;

/// <summary>孔位（端面上的灰盘）定位结果。</summary>
/// <param name="Center">孔位圆心（全图像素坐标）。这是焊偏判定的基准。</param>
/// <param name="Radius">孔位半径（像素）。</param>
/// <param name="ResidualPx">圆拟合残差标准差（像素）。</param>
/// <param name="ClusterSeparation">三个材质层的灰度间距（用于诊断，过小说明分层不可信）。</param>
internal sealed record HoleFit(Point2d Center, double Radius, double ResidualPx, double ClusterSeparation);

/// <summary>
/// 孔位（焊偏判定的基准）定位：端面上那个灰盘。
///
/// **为什么不能用"光滑区边界"来当孔位**（这是原来那套做法）：
/// 原来的做法是从焊环圆心发径向卡尺、找"光滑区向外遇到的第一个上升沿"。但焊环圆心不等于孔心
/// （实测 Img023 相距 97px，比孔半径 96px 还大），且孔沿在反光冲掉的一侧没有梯度——于是射线
/// 越过孔沿继续外走，落到更外面的熔核内沿上，把"孔位"量成 r=199px（真值约 96px）、
/// 圆心偏 100px+。后果是偏移量被系统性**压低**：Img023 报 0.152mm 而真值约 0.56mm，
/// 偏小的方向正是"该判 NG 却判合格"。实测换成孔位基准后偏移量中位由 0.228mm 升到 0.511mm。
///
/// **本方法按"三种材质"分层来找孔位，不含任何绝对灰度阈值**：
/// 焊环内部只有三种材质——**暗缝隙**（孔壁阴影，最暗）、**孔位端面**（灰盘，居中）、
/// **端面金属**（最亮，常被反光冲白）。因此对搜索区内的灰度直方图做三均值聚类，
/// 取**中间那一层**即孔位端面。层与层之间用相对间距校验（间距过小说明分层不可信，直接放弃），
/// 所以曝光/光照变化只会让三个层一起平移，不影响"取中间层"这个判据。
///
/// 实测（tests/pp）：孔位半径/焊环外半径 ≈ 0.18（Img023 为 91.6/517），
/// 不同端子（正/负/汇流排）的孔位尺寸不同，故只做宽区间合理性校验。
///
/// **2026-09-17 修正：残差闸门曾把 8 成以上的真孔位圆盘判掉，导致孔位基准整体失效。**
/// 用户报"标注的每个位置都偏了好多"（OK_432 帧：绿圈画在熔核纹理上而不是孔位上）。
/// 逐级复现后定位到：本方法**明明取到了正确的孔位圆盘**（工作图 (320.0, 220.5) r=9.9px，
/// 与目视的灰盘中心一致），却在残差闸门处被丢弃——残差 2.29px &gt;
/// <see cref="InspectionConfig.MaxHoleFitResidualRatio"/>×R = 0.12×9.9 = 1.19px。
/// 丢弃后 <see cref="WeldInspectionAlgorithm"/> 退回梯度卡尺，而卡尺的射线原点是**焊环圆心**：
/// 本产品孔心偏离焊环圆心约 20 工作像素（= 孔半径的 2 倍），于是射线越过孔位落到熔核内沿上，
/// 报出 r=20.2px（真值 9.9px）、圆心偏 26.6 工作像素（≈211 原始像素），
/// 偏移量被**系统性压低**：该帧报 0.324mm，孔位基准下为 0.948mm。
/// 实测这一失效覆盖 tests/pp 的 8 成帧（300 帧统计：能过残差闸门的仅 17%）。
/// 修法见 <see cref="InspectionConfig.MinHoleFitResidualPx"/>（残差闸门加绝对像素下限）。
///
/// **残留（据实记录，未做）**：中间灰度层取到的是孔位端面**被照亮的那一部分**，
/// 阴影侧（暗新月）落在暗层里被排除，因此
///   · 拟合半径系统性偏小约 20%（0.158×焊环半径 vs 目视约 0.19×）；
///   · 圆心被推离阴影约 1~2 工作像素（0.05~0.1mm），方向与光照固定相关。
/// 实测该偏差量级低于厂商自身复现性（同帧三个定位标记互差中位 24px ≈ 3 工作像素），
/// 故本轮只修"取到却被丢弃"，不改分割方式。要把阴影并进来需要新的判据
/// （"非高亮"区域会与熔核连成一片，"低纹理能量"在此尺度又被边缘污染，两条路都实测失败过）。
/// </summary>
internal static class HoleLocator
{
    /// <summary>直方图三均值聚类的迭代次数（在 256 档直方图上做，确定性收敛）。</summary>
    private const int ClusterIterations = 12;

    /// <summary>搜索区内参与统计的最少像素数，过少说明区域被裁得太小。</summary>
    private const int MinRegionPixels = 400;

    public static HoleFit? Locate(Mat gray8, Mat energy, double energyThreshold,
        Point2d ringCenter, double ringRadius, InspectionConfig cfg)
    {
        if (ringRadius < 12)
            return null;

        var rLimit = cfg.HoleSearchOuterRatio * ringRadius;
        if (rLimit < 8)
            return null;

        var width = gray8.Width;
        var height = gray8.Height;
        var x0 = Math.Max(0, (int)Math.Floor(ringCenter.X - rLimit));
        var x1 = Math.Min(width - 1, (int)Math.Ceiling(ringCenter.X + rLimit));
        var y0 = Math.Max(0, (int)Math.Floor(ringCenter.Y - rLimit));
        var y1 = Math.Min(height - 1, (int)Math.Ceiling(ringCenter.Y + rLimit));
        if (x1 - x0 < 8 || y1 - y0 < 8)
            return null;

        var idx = gray8.GetGenericIndexer<byte>();

        // 分层只在灰度直方图上做，**不掺纹理能量**：试过用熔核的能量阈值先筛"非粗糙"，
        // 结果孔位端面自身的细小斑点就把它的局部标准差顶到阈值附近、被整片筛掉，
        // 剩下的几乎全是被反光冲白的端面（实测该帧 80% 的"光滑"像素都是 255），
        // 三个聚类中心直接塌成一个 → 定位失败。
        // 熔核的粗糙纹理不碍事：它的中灰像素是细长的鱼鳞斑，随后会被"像不像圆盘"这道闸门剔掉。
        var histogram = new int[256];
        var region = 0;
        for (var y = y0; y <= y1; y++)
            for (var x = x0; x <= x1; x++)
            {
                var dx = x - ringCenter.X;
                var dy = y - ringCenter.Y;
                if (dx * dx + dy * dy > rLimit * rLimit)
                    continue;
                histogram[idx[y, x]]++;
                region++;
            }
        if (region < MinRegionPixels)
            return null;

        // ---- 分层：两种分法各出一批候选，按「旧行为优先」的顺序逐个尝试（2026-09-18）----
        // 只用递归 Otsu（先暗/亮二分、再在亮侧二分）隐含一个假设：**孔位端面是中间灰度层**。
        // 端面金属被反光冲白时该假设不成立——第一次 Otsu 的阈值会落在"孔位端面（约 180~215）"
        // 与"冲白金属（250+）"之间，灰盘被划进**暗层**，"中间层"只剩灰盘与冲白区之间的抗锯齿
        // 过渡带。实测两帧（Img000 / Img009）：中间层只占搜索区的 4.7% / 3.6%，3×3 开运算后
        // **无一个面积≥60px 的连通域** → 本方法返回 null → 调用方退回"以焊环圆心为原点"的
        // 梯度卡尺 → 孔位被量成冲白区边界（圆心偏 6.21mm、偏移量被压低 2.5 倍、界面仍报 OK）。
        //
        // 三类 Otsu（两个阈值**联合**优化）允许中间类自己落在任意位置，让"孔位端面"这一模态拿到
        // 自己的簇中心。实测两帧的中间层占比变为 25.7% / 28.8%，**沿用原有的"面积最大"选择规则**
        // 即给出正确的孔心：Img000 偏 0.54mm / Img009 偏 1.38mm（原型实测，`candrule.py`）。
        //
        // **不是替换而是追加**：灰度带按「递归 Otsu → 三类 Otsu」排序、带内仍是"面积从大到小"，
        // 于是不冲白的帧（92%）拿到的第一个候选与改动前**逐像素相同**，改动只在"旧路径给不出候选
        // （或给出的候选走不通）"时才生效。这一点是刻意保住的：替换分层判据会让全部良品帧的输入
        // 一起变，回归风险不可控。
        var bands = new List<(double Lo, double Hi, double Separation)>();
        var recursive = RecursiveOtsu(histogram, region, out var sepRecursive);
        if (recursive is not null && sepRecursive >= cfg.HoleMinClusterSeparation)
            bands.Add((recursive[0], recursive[1], sepRecursive));
        var threeClass = ThreeClassOtsu(histogram, region, out var sepThree);
        if (threeClass is not null && sepThree >= cfg.HoleMinClusterSeparation
            && (recursive is null || threeClass[0] != recursive[0] || threeClass[1] != recursive[1]))
            bands.Add((threeClass[0], threeClass[1], sepThree));
        if (bands.Count == 0)
            return null;

        // ---- 候选：带内按面积从大到小（与改动前同一规则），带间按上面的顺序拼接 ----
        // `IsLegacy` 标记该候选来自**改动前那条**分法：只有它可以按"中间层拟合"直接采纳（见下面的循环）。
        var candidates = new List<(Point2d Center, double Radius, double Residual, double Roundness,
            double Separation, bool IsLegacy)>();
        for (var bandIndex = 0; bandIndex < bands.Count; bandIndex++)
        {
            var band = bands[bandIndex];
            var isLegacy = bandIndex == 0 && recursive is not null
                           && band.Lo == recursive[0] && band.Hi == recursive[1];
            using var mask = new Mat(new Size(width, height), MatType.CV_8UC1, Scalar.Black);
            for (var y = y0; y <= y1; y++)
                for (var x = x0; x <= x1; x++)
                {
                    var dx = x - ringCenter.X;
                    var dy = y - ringCenter.Y;
                    if (dx * dx + dy * dy > rLimit * rLimit)
                        continue;
                    var v = idx[y, x];
                    if (v > band.Lo && v <= band.Hi)
                        mask.Set(y, x, (byte)255);
                }

            using (var se = Cv2.GetStructuringElement(MorphShapes.Ellipse, new Size(3, 3)))
            {
                Cv2.MorphologyEx(mask, mask, MorphTypes.Open, se);
            }

            using var labels = new Mat();
            using var stats = new Mat();
            using var centroids = new Mat();
            var count = Cv2.ConnectedComponentsWithStats(mask, labels, stats, centroids,
                PixelConnectivity.Connectivity8);

            var found = new List<(int Area, Point2d Center, double Radius, double Residual, double Roundness)>();
            for (var i = 1; i < count; i++)
            {
                var area = stats.At<int>(i, (int)ConnectedComponentsTypes.Area);
                if (area < 60)
                    continue;

                var bbox = new Rect(
                    stats.At<int>(i, (int)ConnectedComponentsTypes.Left),
                    stats.At<int>(i, (int)ConnectedComponentsTypes.Top),
                    stats.At<int>(i, (int)ConnectedComponentsTypes.Width),
                    stats.At<int>(i, (int)ConnectedComponentsTypes.Height));
                using var sub = new Mat(labels, bbox);
                using var comp = new Mat();
                Cv2.Compare(sub, new Scalar(i), comp, CmpType.EQ);

                // 形状指标（圆度 = 最大内切圆半径 / 等效半径）只记录，这里不筛
                using var dt = new Mat();
                Cv2.DistanceTransform(comp, dt, DistanceTypes.L2, DistanceTransformMasks.Mask5);
                Cv2.MinMaxLoc(dt, out _, out var dtMax, out _, out _);
                var radiusEq = Math.Sqrt(area / Math.PI);
                var roundness = dtMax / Math.Max(radiusEq, 1e-6);

                using var forContours = comp.Clone();
                Cv2.FindContours(forContours, out var contours, out _,
                    RetrievalModes.External, ContourApproximationModes.ApproxNone);
                if (contours.Length == 0)
                    continue;
                var biggest = contours.MaxBy(c => Cv2.ContourArea(c))!;
                var points = biggest.Select(p => new Point2d(p.X + bbox.X, p.Y + bbox.Y)).ToList();

                var fit = Geo.FitCircle(points);
                if (fit is null || fit.Value.Radius < 3)
                    continue;

                double acc = 0;
                foreach (var p in points)
                {
                    var d = Geo.Dist(p, fit.Value.Center) - fit.Value.Radius;
                    acc += d * d;
                }
                found.Add((area, fit.Value.Center, fit.Value.Radius,
                    Math.Sqrt(acc / points.Count), roundness));
            }

            foreach (var c in found.OrderByDescending(c => c.Area))
                candidates.Add((c.Center, c.Radius, c.Residual, c.Roundness, band.Separation, isLegacy));
        }

        if (candidates.Count == 0)
            return null;

        // 外径弧要的是"孔位端面 ↔ 端面金属"的亮度台阶，所以它的"亮层下界"必须取**端面金属的下界**
        // （递归 Otsu 的 t2），而不是中间层的上界：三类 Otsu 的中间层上界（实测约 215）落在孔位端面
        // 内部，拿它当"已成片高亮区"的判据会把种子圆自己的内部算进去，弧半径小于种子
        // → 被 FitOuterArc 的 1.15×种子 闸门拦掉。实测 Img000：三类 Hi=215、递归 t2=236。
        var brightLevel = recursive is not null ? recursive[1] : bands[0].Hi;

        // ---- 路径一：孔位外径弧（圆心与半径都来自孔位的真实可见边界）；取不到才退路径二 ----
        foreach (var cand in candidates)
        {
            // 闸门放宽默认开启（见 FitOuterArc 的 relaxGates 说明：实测净收益，真实语料逐像素不变）。
            var arc = FitOuterArc(gray8, idx, brightLevel, cand.Center, cand.Radius, ringRadius,
                cand.Separation, cfg, width, height);
            if (arc is not null)
                return arc;

            // ---- 路径二：中间层拟合，此时才用三态闸门校验 ----
            // **只对"改动前那条分法"的候选开放**（IsLegacy）：新增的三类 Otsu 带里，熔核内部的中灰
            // 碎片同样可能是"面积最大 + 圆度够 + 半径比正常"的块（实测 SN_...0033 的 15-52-39：
            // 选中左下角一块熔核碎片，圆心离焊环心 0.56×环半径，真孔位其实与焊环同心，
            // 于是报出 6.90mm 的假 NG）。真实孔位的边缘一定有"孔位端面 ↔ 端面金属"的亮度台阶、
            // 外径弧能拟合出来，熔核里的碎片没有——所以新带的候选**必须由外径弧验证**才允许采纳，
            // 通不过就继续试下一个候选（实测该帧的下一个候选就是那块同心灰盘）。
            if (!cand.IsLegacy)
                continue;

            // 圆度下限 0.6：掩码必须像圆盘（细月牙形状的中间层在路径一里已经被真实孔沿救走了，
            // 走到这里说明孔位边缘不可见，只能靠中间层自身的形状作证）。
            if (cand.Roundness < 0.6)
                continue;

            // 残差容差取"相对项"与"绝对像素下限"的较大者。工作分辨率下孔位圆盘只有约 10.6px 半径，
            // 残差被像素锯齿与灰度切边的粗糙度主导，0.12×R（≈1.27px）会把实测族群（0.64~3.55px）
            // 里 8 成以上的真孔位圆盘判掉，而那些帧退回梯度卡尺时射线原点还是焊环圆心——
            // 实测把孔位半径报成真值的两倍、圆心偏约 200 原始像素。详见
            // InspectionConfig.MinHoleFitResidualPx 的实测记录。
            var residualLimit = Math.Max(cfg.MaxHoleFitResidualRatio * cand.Radius, cfg.MinHoleFitResidualPx);
            if (cand.Residual > residualLimit)
                continue;

            var candRatio = cand.Radius / ringRadius;
            if (candRatio < cfg.MinHoleRadiusRatio || candRatio > cfg.MaxHoleRadiusRatio)
                continue;

            return new HoleFit(cand.Center, cand.Radius, cand.Residual, cand.Separation);
        }

        return null;
    }

    /// <summary>
    /// 孔位**外径**：用"孔位端面与端面金属的交界弧"拟合孔位的真实可见边界。
    ///
    /// 为什么需要它：中间灰度层取到的是孔位端面**被照亮的部分**，阴影侧（暗新月）落在暗层里被排除，
    /// 于是拟合半径系统性地只有真值的约 6 成。实测 OK_432（工作图）：中间层拟合 r=9.92px，
    /// 而孔位与端面金属的真实交界圆是 r=16.56px——**圆心两者只差 0.75px（6 原始像素），
    /// 差的是半径**。半径偏小会连带三件事：界面上"极柱 r"偏小、孔/环比被压低到族值带之外、
    /// 熔核内沿的搜索起点（1.15×孔位半径）落进孔内导致环宽取不到。
    ///
    /// 判据：孔位端面与**端面金属**（被反光冲白的高亮区）之间是一条亮度台阶，是可见的圆弧，
    /// 而孔位与熔核之间没有台阶（同为中低灰）。所以从中间层圆心向外逐角度找"高亮起始半径"，
    /// 只保留外接高亮区**足够大**（≥0.01×base²，即端面金属本体而不是熔核上的反光碎点）的方向——
    /// 少了这一条限制，熔核里的高亮点会被当成边界，实测把圆心拉偏 31 原始像素（弧拟合看似残差
    /// 0.7px 却整体偏大 23%）。实测该弧覆盖约 180°（两个约 112° 的扇区），残差 0.33px。
    ///
    /// 取不到可靠弧（覆盖不足、残差过大、半径不在合理倍数内、半径比不像孔位）时返回 null，
    /// 由调用方按原有三态闸门退回中间层拟合——返回 null 而不是在这里兜底，是因为调用方还要
    /// 用圆度闸门区分"细月牙形状的中间层（此时孔沿本应可见，取不到弧说明有问题）"与正常情况。
    /// </summary>
    /// <param name="relaxGates">
    /// 是否放宽两道按"种子只有孔位端面一小部分"标定的闸门（半径增长 1.15→1.05、
    /// 并启用"弧必须精化种子"判据）。**默认 true**，且实测放宽是净收益：
    ///   · 合成夹具：既有失败由 **29 例降到 10 例**（放宽修好了 19 例"弧被 1.15 判无效 →
    ///     退到中间层拟合/卡尺"的帧），且这 10 例是回滚后那 29 例的子集（没有新增失败）；
    ///   · tests/pp 真实语料 20 SN / 435 帧：**逐像素不变**（这些帧的弧结果不落在 1.05~1.15 之间）。
    /// 之所以能放宽：种子（中间层掩码）在冲白工况下往往已接近整个孔位端面，弧只把它精修进去
    /// 十几个百分点（实测 SN_...0033 的 15-52-39：种子 r=13.5、弧 r=15.29，+13%），
    /// 会被 1.15 判无效，循环于是走到一块熔核碎片上凑出假圆（6.90mm 假 NG）；
    /// 而"弧把圆心搬走"这种真正的风险，由新增的种子一致性判据单独拦住。
    /// </param>
    private static HoleFit? FitOuterArc(Mat gray8, Mat.Indexer<byte> idx, double brightLevel,
        Point2d seedCenter, double seedRadius, double ringRadius, double separation,
        InspectionConfig cfg, int width, int height, bool relaxGates = true)
    {
        var r0 = seedRadius;
        if (r0 < 3 || cfg.HoleOuterArcMinCoverageRatio <= 0)
            return null;

        // 端面金属 = 比"亮层下界"更亮的像素。只接受**成片**的高亮（≥0.002×base²，工作图 460 短边时约 420px），
        // 熔核上的反光碎点必须排除，否则弧会被引到熔核上去。
        //
        // 这个面积下限是第三轮实测调下来的：原取 0.01×base²（约 2100px）过大——孔位旁边那一片
        // 端面金属并不总是连成一大块，实测 20-56-00 731 的孔沿外侧只有 **897px** 的高亮片
        // （外侧亮延续 5~15px，无疑是端面而不是反光），却因此被判掉，覆盖率 0/180、只好退回卡尺。
        // 实测反光碎点只有 11~178px 量级，0.002×base² 把两者分开；此外还要求向外**连续亮 ≥4px**
        // （见下），两条合起来才是"这是端面金属"的判据。
        var baseSize = Math.Min(width, height);
        var minPlateArea = Math.Max(40, baseSize * baseSize * 0.002);
        using var bright = new Mat();
        Cv2.Compare(gray8, new Scalar(brightLevel), bright, CmpType.GT);
        using var plateLabels = new Mat();
        using var plateStats = new Mat();
        using var plateCentroids = new Mat();
        var plateCount = Cv2.ConnectedComponentsWithStats(bright, plateLabels, plateStats,
            plateCentroids, PixelConnectivity.Connectivity8);

        var rInner = Math.Max(2.0, 0.45 * r0);
        // 搜索上限取**孔位半径的物理上限**（MaxHoleRadiusRatio × 焊环半径），而不是"种子半径的若干倍"。
        // 这一点是第三轮实测踩出来的：种子（中间层）在光照不利时只有真值的一半左右，
        // 用它定上限会把真孔沿挡在搜索范围之外——实测 20-56-00 731 的弧覆盖率为 **0/180**，
        // 根因就是种子半径 8.18px 把上限压到 21.3px，而真孔沿在 26px 处；同一帧的中间层圆度
        // 只有 0.52（细月牙），两条路都走不通，只能退回卡尺。改用物理上限后该帧走外径弧。
        var rOuter = cfg.MaxHoleRadiusRatio * ringRadius;
        if (rOuter - rInner < 3)
            return null;

        const int rays = 180;
        const double step = 0.5;
        var arc = new List<Point2d>(rays);
        for (var k = 0; k < rays; k++)
        {
            var angle = 2 * Math.PI * k / rays;
            var ca = Math.Cos(angle);
            var sa = Math.Sin(angle);
            for (var r = rInner; r <= rOuter; r += step)
            {
                var x = (int)Math.Round(seedCenter.X + ca * r);
                var y = (int)Math.Round(seedCenter.Y + sa * r);
                if (x < 2 || x >= width - 2 || y < 2 || y >= height - 2)
                    break;
                if (idx[y, x] <= brightLevel)
                    continue;

                // 向外再走 4px 必须一直是同一块"大高亮区"
                var runOk = true;
                for (var extra = 0.5; extra <= 4.0; extra += 0.5)
                {
                    var xx = (int)Math.Round(seedCenter.X + ca * (r + extra));
                    var yy = (int)Math.Round(seedCenter.Y + sa * (r + extra));
                    if (xx < 1 || xx >= width - 1 || yy < 1 || yy >= height - 1)
                    {
                        runOk = false;
                        break;
                    }
                    var label = plateLabels.At<int>(yy, xx);
                    if (label <= 0 || label >= plateCount ||
                        plateStats.At<int>(label, (int)ConnectedComponentsTypes.Area) < minPlateArea)
                    {
                        runOk = false;
                        break;
                    }
                }
                if (runOk)
                    arc.Add(new Point2d(seedCenter.X + ca * r, seedCenter.Y + sa * r));
                break;
            }
        }

        if (arc.Count < rays * cfg.HoleOuterArcMinCoverageRatio)
            return null;

        var fit = Geo.FitCircle(arc);
        if (fit is null)
            return null;
        var arcRadius = fit.Value.Radius;
        // 弧的半径上限、以及"弧必须精化种子而不是另找一个圆"这两道判据。与焊环半径的比值
        // 必须落在孔位族值带内——它同时把"种子落错地方、弧拟合上了别的圆"挡在外面。
        //
        // 上限用 `MaxPinArcRadiusRatioOfRing`（0.30）而不是 `MaxHoleRadiusRatio`（0.40）：
        // 实测 Img003（0040 的 OK_450）上弧拟合锁到了**远处那片被反光冲白的端面金属区**的边界，
        // 给出 2.32×种子、0.400×焊环半径——旧上限 0.45 拦不住，界面于是画出一个圆心偏
        // 156 原始像素（3.7mm）、半径 1.8 倍的绿圈。现场 48 帧实测该路径的族值是 0.216~0.241，
        // 0.30 既有 24% 余量又能拦住这一帧。详见 InspectionConfig.MaxPinArcRadiusRatioOfRing。
        //
        // 半径增长门限：(relaxGates ? 1.05 : 1.15)。**默认放宽到 1.05**——实测这是净收益
        // （合成夹具既有失败 29→10 例、tests/pp 真实语料 435 帧逐像素不变），详见上面
        // relaxGates 参数的说明；relaxGates=false 保留原口径，供将来做对照实验。
        if (arcRadius < (relaxGates ? 1.05 : 1.15) * r0)
            return null;
        var arcRatio = arcRadius / ringRadius;
        if (arcRatio < cfg.MinHoleRadiusRatio || arcRatio > cfg.MaxPinArcRadiusRatioOfRing)
            return null;

        // 弧必须是"精化这个种子"，不是"另找一个圆"：拟合圆心离种子圆心的距离不得超过
        // `max(1.0×种子半径, 6px)`。
        //
        // 为什么需要（2026-09-18 实测）：候选是**按面积从大到小**逐个尝试的，而种子可能落在
        // 一块熔核碎片上。SN_...0033 的 15-52-39 就是：真孔位（面积最大的那个候选）的弧给
        // r=15.29，只差 1.4% 没够到 `1.15×种子半径`（15.5）的门限而被判无效，循环于是走到
        // 下一块碎片（种子半径仅 4.6px），弧在那块碎片上凑出一个 r=15.29、圆心离真值 0.56×环半径
        // 的圆并通过了全部闸门——报出 6.90mm 的**假 NG**（真孔位其实与焊环同心）。
        // 真孔位的弧只把圆心挪 0.35×种子半径（Img000 实测 3.4px / 9.6px）；碎片上那个"圆"
        // 的圆心离种子 6× 种子半径。1.0× 这条门限两侧各有约 3 倍余量。
        // 绝对下限 6px 是给"种子本身很小"留的余地（种子半径 3px 时按比例算只有 3px，过严）。
        if (relaxGates && Geo.Dist(fit.Value.Center, seedCenter) > Math.Max(r0, 6.0))
            return null;

        double acc = 0;
        foreach (var p in arc)
        {
            var d = Geo.Dist(p, fit.Value.Center) - arcRadius;
            acc += d * d;
        }
        var residual = Math.Sqrt(acc / arc.Count);
        if (residual > cfg.HoleOuterArcResidualRatio * arcRadius)
            return null;

        return new HoleFit(fit.Value.Center, arcRadius, residual, separation);
    }

    /// <summary>
    /// 三类 Otsu：**两个阈值联合优化**（类间方差最大），返回中间类的灰度区间 (tLow, tHigh]。
    ///
    /// 与 <see cref="RecursiveOtsu"/> 的区别是本质的：递归法是"先二分、再在亮侧二分"，
    /// 于是中间类**只能落在亮侧内部**；三类联合优化允许中间类自己落在任意位置。
    /// 端面金属被冲白时这个区别是决定性的——递归法把孔位端面划进暗层、中间层退化成抗锯齿
    /// 过渡带（实测占搜索区 4.7%/3.6%，开运算后无连通域），联合优化则让孔位端面拿到自己的
    /// 簇中心（实测 25.7%/28.8%）。见 <see cref="Locate"/> 里的实测记录。
    ///
    /// 在 256 档直方图上用前缀和穷举两个阈值（约 3.2 万次组合），确定性、无随机性、无绝对灰度阈值。
    /// </summary>
    private static int[]? ThreeClassOtsu(int[] histogram, int total, out double separation)
    {
        separation = 0;
        if (total <= 0)
            return null;

        var cumSum = new double[257];
        var cumCount = new long[257];
        for (var v = 0; v < 256; v++)
        {
            cumSum[v + 1] = cumSum[v] + (double)v * histogram[v];
            cumCount[v + 1] = cumCount[v] + histogram[v];
        }

        var bestVariance = -1.0;
        var bestLow = -1;
        var bestHigh = -1;
        for (var t1 = 1; t1 < 255; t1++)
        {
            var n1 = cumCount[t1];
            if (n1 == 0)
                continue;
            var m1 = cumSum[t1] / n1;
            for (var t2 = t1 + 1; t2 < 255; t2++)
            {
                var n2 = cumCount[t2] - n1;
                if (n2 == 0)
                    continue;
                var n3 = total - cumCount[t2];
                if (n3 == 0)
                    continue;
                var m2 = (cumSum[t2] - cumSum[t1]) / n2;
                var m3 = (cumSum[256] - cumSum[t2]) / n3;
                var variance = (double)n1 * n2 * (m1 - m2) * (m1 - m2)
                             + (double)n1 * n3 * (m1 - m3) * (m1 - m3)
                             + (double)n2 * n3 * (m2 - m3) * (m2 - m3);
                if (variance > bestVariance)
                {
                    bestVariance = variance;
                    bestLow = t1;
                    bestHigh = t2;
                }
            }
        }

        if (bestLow < 0)
            return null;
        separation = Math.Min(bestLow, bestHigh - bestLow);
        return new[] { bestLow, bestHigh };
    }

    /// <summary>
    /// 在 256 档直方图上做三均值聚类，返回三个中心（升序）与相邻层的**最小间距**。
    /// 初始中心取 p20 / p50 / p80 分位，迭代为标准的 Lloyd 步骤，确定性收敛、无随机性。
    /// </summary>
    private static double[]? KMeans3(int[] histogram, out double separation)
    {
        separation = 0;
        var total = 0;
        for (var v = 0; v < 256; v++)
            total += histogram[v];
        if (total == 0)
            return null;

        var centers = new double[3];
        centers[0] = PercentileValue(histogram, total, 20);
        centers[1] = PercentileValue(histogram, total, 50);
        centers[2] = PercentileValue(histogram, total, 80);
        if (centers[1] - centers[0] < 1 || centers[2] - centers[1] < 1)
            return null;

        var sums = new double[3];
        var counts = new double[3];
        for (var iteration = 0; iteration < ClusterIterations; iteration++)
        {
            Array.Clear(sums);
            Array.Clear(counts);
            for (var v = 0; v < 256; v++)
            {
                var n = histogram[v];
                if (n == 0)
                    continue;
                var k = Nearest(centers, v);
                sums[k] += (double)v * n;
                counts[k] += n;
            }
            var moved = 0.0;
            for (var k = 0; k < 3; k++)
            {
                if (counts[k] < total * 0.01)
                    return null;   // 某一层几乎为空 → 分层不成立（例如视野里只有两种材质）
                var next = sums[k] / counts[k];
                moved += Math.Abs(next - centers[k]);
                centers[k] = next;
            }
            Array.Sort(centers);
            if (moved < 0.05)
                break;
        }

        separation = Math.Min(centers[1] - centers[0], centers[2] - centers[1]);
        return centers;
    }

    /// <summary>
    /// 递归 Otsu 分出三层，返回 (孔位端面灰度下界, 上界) 与**两处阈值的较小间距**。
    ///
    /// 第一步在 [0,255] 上求 Otsu 阈值 t1（暗侧 / 亮侧）；第二步只在 (t1,255] 上求 t2。
    /// 孔位端面的灰度区间取 (t1, t2]。这样得到的两个边界都是**相对本图灰度分布**的，
    /// 曝光/增益变化时随之平移，不含任何绝对灰度常数。
    /// </summary>
    private static double[]? RecursiveOtsu(int[] histogram, int total, out double separation)
    {
        separation = 0;
        var t1 = OtsuThreshold(histogram, total, 0, 255);
        if (t1 <= 0)
            return null;

        var upper = total - PrefixCount(histogram, t1);
        if (upper <= 0)
            return null;
        var t2 = OtsuThreshold(histogram, upper, t1 + 1, 255);
        if (t2 <= t1)
            return null;

        separation = Math.Min(t1 - 0, t2 - t1);
        return new[] { (double)t1, (double)t2 };
    }

    /// <summary>在直方图的 [lo, hi] 档上求 Otsu 阈值（类间方差最大）。</summary>
    private static int OtsuThreshold(int[] histogram, int total, int lo, int hi)
    {
        double sum = 0;
        for (var v = lo; v <= hi; v++)
            sum += (double)v * histogram[v];

        double sumBackground = 0;
        var weightBackground = 0;
        var best = -1;
        var bestVariance = -1.0;
        for (var t = lo; t < hi; t++)
        {
            weightBackground += histogram[t];
            if (weightBackground == 0)
                continue;
            var weightForeground = total - weightBackground;
            if (weightForeground == 0)
                break;

            sumBackground += (double)t * histogram[t];
            var meanBackground = sumBackground / weightBackground;
            var meanForeground = (sum - sumBackground) / weightForeground;
            var variance = (double)weightBackground * weightForeground
                         * (meanBackground - meanForeground) * (meanBackground - meanForeground);
            if (variance > bestVariance)
            {
                bestVariance = variance;
                best = t;
            }
        }
        return best;
    }

    private static int PrefixCount(int[] histogram, int threshold)
    {
        var count = 0;
        for (var v = 0; v <= threshold && v < 256; v++)
            count += histogram[v];
        return count;
    }

    private static int Nearest(double[] centers, int value)
    {
        var best = 0;
        var bestDistance = double.MaxValue;
        for (var k = 0; k < centers.Length; k++)
        {
            var d = Math.Abs(centers[k] - value);
            if (d < bestDistance)
            {
                bestDistance = d;
                best = k;
            }
        }
        return best;
    }

    /// <summary>按累计计数取灰度分位值（直方图档位）。</summary>
    private static double PercentileValue(int[] histogram, int total, double pct)
    {
        var target = total * pct / 100.0;
        double acc = 0;
        for (var v = 0; v < 256; v++)
        {
            acc += histogram[v];
            if (acc >= target)
                return v;
        }
        return 255;
    }
}
