namespace BatteryWeldAOI.Core.Vision;

using BatteryWeldAOI.Core.Models;
using OpenCvSharp;

/// <summary>焊环质量指标。全部为相对量，与分辨率、曝光无关。</summary>
/// <param name="Coverage">环弧角度覆盖率（0~1）。72 个角度扇区中被熔核像素覆盖的比例。</param>
/// <param name="Fill">填充率 = 熔核面积 / (πR²)。未成形偏低，熔融过量偏高。</param>
/// <param name="PoreTotalRatio">环内暗区总面积 / (πR²)。仅作诊断输出，不参与判定（见下）。</param>
/// <param name="PoreMaxRatio">最大单个暗区面积 / (πR²)。</param>
/// <param name="SpatterAreaRatio">环外飞溅总面积 / base²。</param>
/// <param name="SpatterCount">环外飞溅颗粒数。</param>
internal readonly record struct WeldMetrics(
    double Coverage, double Fill,
    double PoreTotalRatio, double PoreMaxRatio,
    double SpatterAreaRatio, int SpatterCount);

/// <summary>
/// 焊环质量度量。
///
/// 关于暗区（气孔/熔洞）判据的说明——这是原型阶段验证后**主动放弃**的一个判据：
/// 曾尝试用「环内暗区面积占比」检测炸焊熔洞，但在良品样本上实测暗区恒为 6.4%±0.8%
/// （那是固定方向的阴影，不是缺陷），且**任何基于区域自身直方图分位数的阈值都是自适应的**：
/// 缺陷越大，分位数越落在缺陷内部，阈值随之被拉低，检测面积反而缩小。
/// 实测把 0.35R 的合成熔洞注入良品图，p12 判据的检出面积几乎不变（0.075 → 0.077），
/// 完全失效。改用「熔核-基底对比度锚定」的阈值同样失败——熔核自身的鱼鳞纹理亮度分布
/// 比熔核与基底的对比度还宽，会把半个熔核判成暗区。
/// 因此这里只测量、不判定，指标随结果一并输出，供后续用真实炸焊样本重新标定。
/// </summary>
internal static class WeldQualityMeter
{
    /// <summary>角度覆盖率分档数。</summary>
    private const int CoverageBins = 72;

    public static WeldMetrics Measure(
        Mat gray8,
        Mat solidMask,
        Mat energy,
        double energyThreshold,
        Point2d center,
        double radius,
        int baseSize)
    {
        var width = gray8.Width;
        var height = gray8.Height;
        var idealArea = Math.PI * radius * radius;

        // ---- 角度覆盖率 ----
        // 用熔核掩码（而非轮廓）分档：环形熔核的**外轮廓**永远是闭合的一圈，
        // 断弧信息只存在于掩码里，用轮廓会恒得 100%。
        var hit = new bool[CoverageBins];
        var reach = radius * 1.15;
        var solidIdx = solidMask.GetGenericIndexer<byte>();
        var yFrom = Math.Max(0, (int)Math.Floor(center.Y - reach));
        var yTo = Math.Min(height - 1, (int)Math.Ceiling(center.Y + reach));
        for (var y = yFrom; y <= yTo; y++)
        {
            var dy = y - center.Y;
            var halfSquared = reach * reach - dy * dy;
            if (halfSquared <= 0)
                continue;
            var half = Math.Sqrt(halfSquared);
            var xFrom = Math.Max(0, (int)Math.Ceiling(center.X - half));
            var xTo = Math.Min(width - 1, (int)Math.Floor(center.X + half));
            for (var x = xFrom; x <= xTo; x++)
            {
                if (solidIdx[y, x] == 0)
                    continue;
                var angle = Math.Atan2(dy, x - center.X);
                var bin = (int)((angle + Math.PI) / (2 * Math.PI) * CoverageBins);
                hit[Math.Clamp(bin, 0, CoverageBins - 1)] = true;
            }
        }
        var coverage = hit.Count(b => b) / (double)CoverageBins;

        // ---- 填充率 ----
        var fill = Cv2.CountNonZero(solidMask) / idealArea;

        // ---- 环内暗区 ----
        var (poreTotal, poreMax) = MeasurePores(gray8, center, radius, baseSize);

        // ---- 环外飞溅 ----
        var (spatterArea, spatterCount) = MeasureSpatter(
            energy, energyThreshold, center, radius, baseSize);

        return new WeldMetrics(
            coverage, fill,
            poreTotal / idealArea, poreMax / idealArea,
            spatterArea / ((double)baseSize * baseSize), spatterCount);
    }

    private static (double Total, double Max) MeasurePores(
        Mat gray8, Point2d center, double radius, int baseSize)
    {
        using var smoothed = new Mat();
        Cv2.GaussianBlur(gray8, smoothed, new Size(0, 0), Math.Max(1.0, radius * 0.05));

        using var inside = new Mat(new Size(gray8.Width, gray8.Height), MatType.CV_8UC1, Scalar.Black);
        Cv2.Circle(inside, (Point)center, (int)(radius * 0.80), Scalar.White, -1);

        var darkLevel = Geo.MaskedPercentile(smoothed, inside, 12, Math.Max(1, baseSize / 384));

        using var dark = new Mat();
        Cv2.Compare(smoothed, new Scalar(darkLevel), dark, CmpType.LT);
        Cv2.BitwiseAnd(dark, inside, dark);

        var kernel = Geo.Odd(radius * 0.10, 3, 31);
        using (var se = Cv2.GetStructuringElement(MorphShapes.Ellipse, new Size(kernel, kernel)))
            Cv2.MorphologyEx(dark, dark, MorphTypes.Open, se);

        using var labels = new Mat();
        using var stats = new Mat();
        using var centroids = new Mat();
        var count = Cv2.ConnectedComponentsWithStats(dark, labels, stats, centroids,
            PixelConnectivity.Connectivity8);

        double total = 0;
        var max = 0.0;
        for (var i = 1; i < count; i++)
        {
            var area = stats.At<int>(i, (int)ConnectedComponentsTypes.Area);
            total += area;
            if (area > max)
                max = area;
        }
        return (total, max);
    }

    private static (double Area, int Count) MeasureSpatter(
        Mat energy, double threshold, Point2d center, double radius, int baseSize)
    {
        using var spatter = new Mat();
        Cv2.Compare(energy, new Scalar(threshold), spatter, CmpType.GT);

        // 只统计焊环之外的颗粒：环内是熔核本体，不是飞溅
        using var outside = new Mat(new Size(energy.Width, energy.Height), MatType.CV_8UC1, Scalar.White);
        Cv2.Circle(outside, (Point)center, (int)(radius * 1.12), Scalar.Black, -1);
        Cv2.BitwiseAnd(spatter, outside, spatter);

        var kernel = Geo.Odd(baseSize * 0.012, 3, 9);
        using (var se = Cv2.GetStructuringElement(MorphShapes.Ellipse, new Size(kernel, kernel)))
            Cv2.MorphologyEx(spatter, spatter, MorphTypes.Open, se);

        using var labels = new Mat();
        using var stats = new Mat();
        using var centroids = new Mat();
        var count = Cv2.ConnectedComponentsWithStats(spatter, labels, stats, centroids,
            PixelConnectivity.Connectivity8);

        var minArea = baseSize * (double)baseSize * 0.00005;
        double area = 0;
        var blobs = 0;
        for (var i = 1; i < count; i++)
        {
            var a = stats.At<int>(i, (int)ConnectedComponentsTypes.Area);
            if (a < minArea)
                continue;
            area += a;
            blobs++;
        }
        return (area, blobs);
    }
}
