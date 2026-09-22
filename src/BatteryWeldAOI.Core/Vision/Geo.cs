namespace BatteryWeldAOI.Core.Vision;

using OpenCvSharp;

/// <summary>拟合圆：圆心（像素）+ 半径（像素）。</summary>
internal readonly record struct CircleFit(Point2d Center, double Radius);

/// <summary>径向中位数圆拟合结果，附带形状自洽性指标。</summary>
/// <param name="Circle">拟合圆。</param>
/// <param name="ArcSupport">内点覆盖的角向扇区比例（0~1）。1 表示整圈边界都支持这个圆。</param>
/// <param name="ResidualScatter">内点径向残差标准差 / 半径。</param>
internal readonly record struct RadialCircleFit(CircleFit Circle, double ArcSupport, double ResidualScatter);

/// <summary>
/// 检测算法共用的几何与统计小工具。
/// 所有分位数都走"抽样 + 排序"而非直方图，是为了避免依赖 OpenCV 的 CalcHist 绑定，
/// 同时保证与原型验证阶段（Python/NumPy）的数值一致。
/// </summary>
internal static class Geo
{
    /// <summary>取奇数核尺寸并夹在 [lo, hi]。</summary>
    public static int Odd(double v, int lo, int hi) =>
        Math.Max(lo, Math.Min(hi, (int)Math.Round(v) | 1));

    /// <summary>线性插值分位数（输入必须已升序）。等价于 numpy.percentile 的默认插值。</summary>
    public static double Percentile(IReadOnlyList<float> sorted, double pct)
    {
        if (sorted.Count == 0)
            return 0;
        if (sorted.Count == 1)
            return sorted[0];
        var pos = pct / 100.0 * (sorted.Count - 1);
        var lo = (int)Math.Floor(pos);
        var hi = Math.Min(lo + 1, sorted.Count - 1);
        var t = pos - lo;
        return sorted[lo] * (1 - t) + sorted[hi] * t;
    }

    /// <summary>总体标准差（ddof = 0，与 numpy.std 默认一致）。</summary>
    public static double StdDev(IReadOnlyList<float> v, int count = -1)
    {
        var n = count < 0 ? v.Count : Math.Min(count, v.Count);
        if (n == 0)
            return 0;
        double sum = 0;
        for (var i = 0; i < n; i++)
            sum += v[i];
        var mean = sum / n;
        double acc = 0;
        for (var i = 0; i < n; i++)
        {
            var d = v[i] - mean;
            acc += d * d;
        }
        return Math.Sqrt(acc / n);
    }

    /// <summary>
    /// Kasa 最小二乘圆拟合：解 [2x 2y 1]·[cx cy c]ᵀ = x²+y² 的正规方程。
    /// 先减去质心再拟合，避免像素坐标平方后正规方程病态（数量级 1e12 会吃掉 double 的有效位）。
    /// </summary>
    public static CircleFit? FitCircle(IReadOnlyList<Point2d> pts)
    {
        var n = pts.Count;
        if (n < 3)
            return null;

        double mx = 0, my = 0;
        foreach (var p in pts)
        {
            mx += p.X;
            my += p.Y;
        }
        mx /= n;
        my /= n;

        double sxx = 0, sxy = 0, syy = 0, sx = 0, sy = 0;
        double bx = 0, by = 0, bc = 0;
        foreach (var p in pts)
        {
            double x = p.X - mx, y = p.Y - my;
            var r2 = x * x + y * y;
            sxx += 4 * x * x;
            sxy += 4 * x * y;
            syy += 4 * y * y;
            sx += 2 * x;
            sy += 2 * y;
            bx += 2 * x * r2;
            by += 2 * y * r2;
            bc += r2;
        }

        var det = sxx * (syy * n - sy * sy)
                - sxy * (sxy * n - sy * sx)
                + sx * (sxy * sy - syy * sx);
        if (Math.Abs(det) < 1e-9)
            return null;

        var cx = (bx * (syy * n - sy * sy)
                - sxy * (by * n - sy * bc)
                + sx * (by * sy - syy * bc)) / det;
        var cy = (sxx * (by * n - sy * bc)
                - bx * (sxy * n - sy * sx)
                + sx * (sxy * bc - by * sx)) / det;
        var c = (sxx * (syy * bc - by * sy)
               - sxy * (sxy * bc - by * sx)
               + bx * (sxy * sy - syy * sx)) / det;

        var rr = c + cx * cx + cy * cy;
        if (rr <= 0)
            return null;
        return new CircleFit(new Point2d(cx + mx, cy + my), Math.Sqrt(rr));
    }

    /// <summary>
    /// 径向中位数圆拟合：以 seed 为起点，反复做
    /// 「取轮廓点到圆心的距离中位数当半径 → 丢掉偏离中位半径超过 tolFrac·R 的点 →
    ///   用剩余点重拟合圆心」，直到圆心收敛。
    ///
    /// 为什么不用「最小二乘初值 + 迭代剔点」（本类曾用过、已在真机上被证伪）：
    /// 那条路的容差带是以**已经被污染**的半径为基准算出来的。连通域上粘了凸起
    /// （飞溅颗粒、划痕弧、加工纹并入熔核）时，最小二乘初值先被凸起撑大，容差随之变宽，
    /// 凸起就被当成正常点收下，最终收敛成一个又大又偏的折中圆。真机良品 27.png 实测：
    /// 半径 68.9 → 84.8px（虚增 23%），圆心偏移 19.4px——而该图短边只有 429px，
    /// 1.0mm 公差折算下来仅 13.7px，拟合误差比整个公差还大，偏移量读数完全不可信。
    /// 更糟的是圆心偏移会连带把孔洞卡尺的射线原点带偏，孔洞也跟着跑（实拍同一张图上
    /// 孔洞从 r=15px 正确位置跑到 r=29px 的暗弧上）。
    ///
    /// 中位数对两侧尾部同时稳健：凸起把少数点的径向距离推大、缺口把少数点推小，
    /// 两者都不改变中位数。所以半径先站稳，再用它剔点，凸起和缺口都拖不动圆心。
    ///
    /// seed 取距离变换峰值（最大内切圆圆心）：它按定义对凸起免疫，是最好的起点；
    /// 缺口会把它推离本体中心，但半径中位数不受这个偏移影响——圆心偏 e 只把径向距离
    /// 展宽到 R±e，中位数仍≈R——几轮迭代后圆心就回到本体中心。
    /// 实测该偏移量在 28 张真机图上对 20 张的影响 &lt;0.2px，只在并块图上有 6~18px 的修正。
    /// </summary>
    /// <param name="pts">轮廓点（全图像素坐标）。</param>
    /// <param name="seed">迭代起点，取最大内切圆圆心。</param>
    /// <param name="tolFrac">内点判定容差（× 中位半径）。实测 0.06~0.15 之间结果变化 ≤2px。</param>
    /// <param name="iters">圆心收敛迭代上限。</param>
    public static RadialCircleFit? MedianRadiusCircle(IReadOnlyList<Point2d> pts, Point2d seed,
        double tolFrac = 0.10, int iters = 12)
    {
        if (pts.Count < minKeep)
            return null;

        var distances = new double[pts.Count];
        var scratch = new double[pts.Count];
        var center = seed;
        var radius = 0.0;

        for (var it = 0; it < iters; it++)
        {
            radius = RadiusAndDistances(pts, center, distances, scratch);

            var tol = Math.Max(tolFrac * radius, 1.5);
            var keep = new List<Point2d>(pts.Count);
            for (var i = 0; i < pts.Count; i++)
                if (Math.Abs(distances[i] - radius) < tol)
                    keep.Add(pts[i]);
            if (keep.Count < minKeep)
                break;

            var next = FitCircle(keep);
            if (next is null)
                break;
            var moved = Dist(next.Value.Center, center);
            center = next.Value.Center;
            if (moved < 0.05)
                break;
        }

        // 末轮：以收敛后的圆心重算半径（循环可能是在更新圆心之后 break 的），并统计自洽性
        radius = RadiusAndDistances(pts, center, distances, scratch);
        if (radius < 1)
            return null;

        var tolFinal = Math.Max(tolFrac * radius, 1.5);
        var hit = new bool[ArcBins];
        double sum = 0;
        var kept = 0;
        for (var i = 0; i < pts.Count; i++)
        {
            if (Math.Abs(distances[i] - radius) >= tolFinal)
                continue;
            kept++;
            sum += distances[i];
            var angle = Math.Atan2(pts[i].Y - center.Y, pts[i].X - center.X);
            hit[Math.Clamp((int)((angle + Math.PI) / (2 * Math.PI) * ArcBins), 0, ArcBins - 1)] = true;
        }
        if (kept == 0)
            return null;

        var mean = sum / kept;
        double acc = 0;
        for (var i = 0; i < pts.Count; i++)
            if (Math.Abs(distances[i] - radius) < tolFinal)
                acc += (distances[i] - mean) * (distances[i] - mean);

        return new RadialCircleFit(
            new CircleFit(center, radius),
            hit.Count(x => x) / (double)ArcBins,
            Math.Sqrt(acc / kept) / radius);
    }

    /// <summary>角向自洽性统计的分档数，与 <see cref="WeldQualityMeter"/> 的环覆盖率口径一致。</summary>
    private const int ArcBins = 72;

    /// <summary>最小内点数：少于此数拟合无意义（三点定圆毫无约束力）。</summary>
    private const int minKeep = 12;

    /// <summary>
    /// 计算各轮廓点到 center 的距离，并返回其**中位数**作为半径。
    /// distances 保持与 pts 同序（后面还要按 i 取值），排序走 scratch 副本。
    /// </summary>
    private static double RadiusAndDistances(IReadOnlyList<Point2d> pts, Point2d center,
        double[] distances, double[] scratch)
    {
        for (var i = 0; i < pts.Count; i++)
            distances[i] = Dist(pts[i], center);
        Array.Copy(distances, scratch, pts.Count);
        Array.Sort(scratch, 0, pts.Count);
        var mid = pts.Count / 2;
        return (pts.Count & 1) == 1 ? scratch[mid] : 0.5 * (scratch[mid - 1] + scratch[mid]);
    }

    public static double Dist(Point2d a, Point2d b) =>
        Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y));

    /// <summary>
    /// 掩码内像素值的分位数（8U 单通道）。按 step 抽样——焊缝纹理图与高斯平滑图
    /// 本身都是低频的，隔点抽样对分位数没有可测影响，却把开销降到 1/step²。
    /// mask 为 null 表示全图。
    /// </summary>
    public static double MaskedPercentile(Mat src8, Mat? mask, double pct, int step)
    {
        step = Math.Max(1, step);
        var samples = new List<float>(4096);
        var idx = src8.GetGenericIndexer<byte>();
        var midx = mask?.GetGenericIndexer<byte>();
        for (var y = 0; y < src8.Height; y += step)
            for (var x = 0; x < src8.Width; x += step)
                if (midx is null || midx[y, x] != 0)
                    samples.Add(idx[y, x]);
        if (samples.Count == 0)
            return 0;
        samples.Sort();
        return Percentile(samples, pct);
    }

    /// <summary>32F 单通道 Mat 的全局分位数（抽样）。</summary>
    public static double Percentile32F(Mat src32, double pct, int step)
    {
        step = Math.Max(1, step);
        var samples = new List<float>(Math.Max(1024, src32.Rows * src32.Cols / (step * step)));
        var idx = src32.GetGenericIndexer<float>();
        for (var y = 0; y < src32.Rows; y += step)
            for (var x = 0; x < src32.Cols; x += step)
                samples.Add(idx[y, x]);
        if (samples.Count == 0)
            return 0;
        samples.Sort();
        return Percentile(samples, pct);
    }
}
