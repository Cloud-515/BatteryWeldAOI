namespace BatteryWeldAOI.Core.Vision;

using BatteryWeldAOI.Core.Models;
using OpenCvSharp;

/// <summary>孔洞（极柱/灰圆）定位结果：正弦拟合 r(θ) = a + b·cosθ + c·sinθ。</summary>
/// <param name="Center">拟合圆心（全图像素坐标）。</param>
/// <param name="Radius">拟合平均半径 q。</param>
/// <param name="OffsetPx">拟合中心相对射线原点初值的偏移量 D = √(b²+c²)。</param>
/// <param name="ResidualPx">剔除离群射线后的残差标准差。</param>
/// <param name="UsedRays">参与拟合的射线数。</param>
/// <param name="TotalRays">总射线数。</param>
internal sealed record PinFit(Point2d Center, double Radius, double OffsetPx,
    double ResidualPx, int UsedRays, int TotalRays);

/// <summary>
/// 孔洞 / 极柱端面（"灰圆"）定位：逐射线自适应梯度卡尺 + 正弦拟合。
///
/// 物理场景：极柱端面是一块**光滑的灰盘**，外面依次是深色缝隙与粗糙熔核。
/// 从灰盘中心向外走一条射线，梯度幅值的形态必然是「盘内平坦（低）→ 盘沿阶跃（陡增）
/// → 缝隙/熔核（高）」。所以盘沿 = 从盘内平坦段向外遇到的第一个显著上升沿。
///
/// 三个关键设计（每一个都是原型阶段踩坑换来的）：
///  1. **用梯度而不是灰度**：灰盘有时比熔核亮、有时比它暗（取决于反光），
///     但边缘处的梯度幅值恒在。梯度算子天然对极性免疫。
///  2. **逐射线自适应阈值**：tau = p10 + frac·(p95 − p10)。全局固定阈值必然失败——
///     反光强的点位整条射线梯度都高，反光弱的点位整条都低。
///  3. **搜索起点取射线内部的梯度极小点**：从中心直接向外找"第一次超过阈值"，
///     会在盘内噪声上立刻触发。先找到盘内最平坦处再向外找沿，才稳定。
///
/// **本轮改造（2026-09-16）：灰圆不再依赖焊环的圆心与半径。**
/// 改动前射线原点与全部尺度都取自焊环（<c>ringCenter</c> / <c>ringRadius</c>），
/// 于是焊环一旦被外围反光细圈带偏，灰圆的搜索窗整片搬到无关区域，产生误差级联——
/// 实测 Img000 上焊环圆心偏 202.6px，灰圆也随之错，最终把偏移量报成 0.396mm（真值约 0.81mm）。
/// 现在：
///   · 尺度基准改用**灰圆自身的族值半径**（<see cref="InspectionConfig.PinRadiusFamilyRatio"/> × base），
///     焊环半径偏 30% 也不会再牵动这里的搜索窗、平滑尺度与离群门限；
///   · 射线原点只把焊环圆心当**初值**，随后**以自身拟合结果重新锚定并迭代**（最多 3 轮，
///     取残差最小的一轮），因此初值偏差不会把结果绑架过去；
///   · 与焊环之间**不加任何同心约束**——两者的偏心量正是要测的"焊偏"，强行同心会把它抹平。
///
/// 之所以仍用正弦拟而不是"取若干边缘点拟合圆"：正弦拟对缺边、局部反光导致的射线丢失
/// 天然鲁棒（缺哪条射线只影响那一条的权重）。
/// </summary>
internal static class PinLocator
{
    /// <summary>梯度平滑尺度（× 灰圆族值半径）——必须与盘沿的边缘尺度匹配。</summary>
    private const double SmoothingWidthRatio = 0.020;

    /// <summary>残差剔除的迭代次数上限。</summary>
    private const int TrimIterations = 8;

    /// <summary>以自身结果重锚定射线原点的最大轮数。</summary>
    private const int ReAnchorPasses = 3;

    /// <summary>
    /// 允许重锚定迭代的最小灰圆半径（**绝对像素**）。
    ///
    /// 为什么用绝对像素而不是相对 base 的比例：重锚定的收益取决于"盘沿跨多少个采样点"
    /// （射线步长固定 0.5px），这是**绝对像素尺度**的问题。实测两侧分界很清楚：
    ///   · 合成夹具的灰圆半径约 16.8px → 迭代能显著改善大偏移（D/q ≈ 0.9）下的圆心精度
    ///     （不迭代时焊偏量把真值 0.70mm 量成 0.865mm，超出 0.05mm 精度断言）；
    ///   · tests/tp 的 15.png 灰圆只有约 10.7px（低分辨率，盘径仅约 6px）→ 迭代反而把圆心带偏
    ///     （偏移量从 0.674mm 翻成 1.295mm 的伪"焊偏"）。
    /// 取 12px 把两者分开。
    /// </summary>
    private const double ReAnchorMinRadiusPx = 12.0;

    /// <summary>只采纳"显著更好"的后续轮：残差改善不到这个比例就保留前一轮（防噪声驱动位移）。</summary>
    private const double ReAnchorMinImprovement = 0.80;

    /// <summary>重新锚定的收敛判据（圆心移动小于该值即停，像素）。</summary>
    private const double ReAnchorTolerancePx = 0.5;

    /// <param name="gray8">已做 3×3 中值滤波的灰度图（与焊环检测共用同一预处理结果）。</param>
    /// <param name="initialCenter">
    /// 射线原点**初值**（一般给焊环圆心）。只作初值：拟合会以自身结果重锚定迭代，
    /// 因此初值偏一些不会绑架结果。</param>
    /// <param name="baseSize">工作图短边（min(高,宽)），用于按族值换算灰圆自身的尺度基准。</param>
    public static PinFit? Locate(Mat gray8, Point2d initialCenter, int baseSize, InspectionConfig cfg)
    {
        if (baseSize < 64)
            return null;

        // 灰圆自身的尺度基准：族值半径 × base。用它而不是焊环半径，是"两个地标各自独立测量"的关键。
        var familyRadius = baseSize * cfg.PinRadiusFamilyRatio;
        if (familyRadius < 4)
            return null;

        // 尺度基准 = 灰圆自身族值半径（不取焊环半径，见类型注释）。
        // 注意：**射线原点目前仍是焊环圆心**——这是已知缺陷（见 R23）：
        // 焊环圆心离灰圆心可达 97px、比灰盘半径还大，此时卡尺会在"灰盘边界被反光冲掉"的一侧
        // 落到熔核内沿上，把灰圆半径量成 199px（真值约 96px）。已试过两种粗定位判据均无效：
        // 局部灰度带（种子落在盘边缘，带退化）与"被暗缝隙包围的孤岛"（该帧缝隙只是一段弧、
        // 不闭合，任何阈值下孤岛数都是 0）。
        var scaleRadius = familyRadius;
        var origin = initialCenter;

        using var gray32 = new Mat();
        gray8.ConvertTo(gray32, MatType.CV_32F);

        using var smooth = new Mat();
        Cv2.GaussianBlur(gray32, smooth, new Size(0, 0),
            Math.Max(0.8, scaleRadius * cfg.PinGradientSigmaRatio));

        using var gx = new Mat();
        using var gy = new Mat();
        Cv2.Sobel(smooth, gx, MatType.CV_32F, 1, 0, ksize: 3);
        Cv2.Sobel(smooth, gy, MatType.CV_32F, 0, 1, ksize: 3);
        using var mag = new Mat();
        Cv2.Magnitude(gx, gy, mag);

        // 以自身结果重锚定并迭代：单轮返回会让结果被初值偏差带偏一档（大偏移时尤甚，
        // 因为正弦模型在 D/q 大时本身有偏）；迭代到原点落在灰圆心附近后，逐射线
        // "盘内平坦段"的假设才真正成立。
        //
        // 两道闸门，都是实测踩出来的：
        //  1. **灰圆够大才迭代**（ReAnchorMinRadiusPx）：低分辨率盘沿只有几个像素时，
        //     迭代是噪声驱动的位移——tests/tp 的 15.png 因此把偏移量从 0.674mm 翻成 1.295mm；
        //  2. **只采纳显著更好的后续轮**（ReAnchorMinImprovement）：残差没改善够就停在前一轮。
        PinFit? best = null;
        for (var pass = 0; pass < ReAnchorPasses; pass++)
        {
            var fit = FitOnce(mag, origin, scaleRadius, cfg);
            if (fit is null)
                break;

            if (best is null)
            {
                best = fit;
                if (fit.Radius < ReAnchorMinRadiusPx)
                    break;   // 灰圆太小，不迭代
            }
            else if (fit.ResidualPx < best.ResidualPx * ReAnchorMinImprovement)
            {
                best = fit;
            }
            else
            {
                break;   // 后续轮不再显著更好，停在这里
            }

            var moved = Geo.Dist(fit.Center, origin);
            origin = fit.Center;
            if (moved < ReAnchorTolerancePx)
                break;
        }

        return best;
    }

    /// <summary>从给定原点出发做一轮卡尺 + 正弦拟合。尺度一律取自 <paramref name="familyRadius"/>。</summary>
    private static PinFit? FitOnce(Mat mag, Point2d origin, double familyRadius, InspectionConfig cfg)
    {
        var width = mag.Width;
        var height = mag.Height;
        var magIdx = mag.GetGenericIndexer<float>();

        // ---- 采样半径序列 ----
        // 搜索窗也按灰圆族值半径表达，因而与焊环半径无关。
        // rStart 必须落在盘内平坦区（见 InspectionConfig.PinSearchInnerRatioOfFamily 的说明）。
        var rStart = Math.Max(2.0, cfg.PinSearchInnerRatioOfFamily * familyRadius);
        var rEnd = cfg.PinSearchOuterRatioOfFamily * familyRadius;
        var span = rEnd - rStart;
        if (span < 4)
            return null;

        var step = Math.Max(0.5, span / 80.0);
        var radiusCount = (int)Math.Floor(span / step) + 1;
        if (radiusCount < 8)
            return null;

        var radii = new double[radiusCount];
        for (var i = 0; i < radiusCount; i++)
            radii[i] = rStart + i * step;

        // 离群门限按搜索窗宽度表达（原为 0.22×焊环半径，这里换成同量级的自尺度量，
        // 避免把"尺度基准换成灰圆半径"这件事连带改变残差剔除的宽严）。
        var trimLimitPx = 0.42 * span;

        var kernelWidth = Geo.Odd(familyRadius * SmoothingWidthRatio, 3, 31);
        var angles = new List<double>(cfg.PinCaliperRays);
        var radiiHit = new List<double>(cfg.PinCaliperRays);

        var sampleX = new int[radiusCount];
        var sampleY = new int[radiusCount];
        var samples = new double[radiusCount];
        var smoothed = new double[radiusCount];
        var sorted = new double[radiusCount];

        for (var k = 0; k < cfg.PinCaliperRays; k++)
        {
            var angle = 2 * Math.PI * k / cfg.PinCaliperRays;
            var ca = Math.Cos(angle);
            var sa = Math.Sin(angle);

            var hitCount = 0;
            for (var i = 0; i < radiusCount; i++)
            {
                var px = (int)Math.Round(origin.X + ca * radii[i]);
                var py = (int)Math.Round(origin.Y + sa * radii[i]);
                if (px < 1 || px >= width - 1 || py < 1 || py >= height - 1)
                    continue;
                sampleX[hitCount] = px;
                sampleY[hitCount] = py;
                samples[hitCount] = magIdx[py, px];
                hitCount++;
            }
            if (hitCount < 8)
                continue;

            // 沿射线做一次移动平均，抑制单像素噪声引起的假沿。
            // 边缘用 numpy 'same' 的零填充语义（越界样本按 0 计入但不补归一化），
            // 与原型验证时逐位一致。
            MovingAverageSame(samples, hitCount, kernelWidth, smoothed);

            Array.Copy(smoothed, sorted, hitCount);
            Array.Sort(sorted, 0, hitCount);
            var sortedSpan = new ReadOnlySpan<double>(sorted, 0, hitCount);
            var vLo = Percentile(sortedSpan, 10);
            var vHi = Percentile(sortedSpan, 95);
            var rise = vHi - vLo;
            if (rise < 1e-6)
                continue;

            // 噪声底：射线最内侧若干样本（盘内平坦区）的标准差
            var noiseCount = Math.Max(3, hitCount / 8);
            var noise = StdDev(smoothed, noiseCount) + 1e-6;
            if (rise < cfg.PinCaliperMinSnr * noise)
                continue;

            var tau = vLo + cfg.PinCaliperLevel * rise;

            // 起点 = 射线前半段最平坦处（盘内），避免在盘内噪声上误触发
            var searchLimit = Math.Max(3, hitCount / 2);
            var start = 0;
            for (var i = 1; i < searchLimit; i++)
                if (smoothed[i] < smoothed[start])
                    start = i;

            var crossing = -1;
            for (var i = start; i < hitCount; i++)
                if (smoothed[i] > tau)
                {
                    crossing = i;
                    break;
                }
            if (crossing <= 0)
                continue;

            // 亚像素：跨界点附近的抛物线峰值
            var j0 = Math.Max(1, crossing - 2);
            var j1 = Math.Min(hitCount - 2, crossing + 3);
            var peak = j0;
            for (var i = j0; i <= j1; i++)
                if (smoothed[i] > smoothed[peak])
                    peak = i;

            var y0 = smoothed[peak - 1];
            var y1 = smoothed[peak];
            var y2 = smoothed[peak + 1];
            var den = y0 - 2 * y1 + y2;
            var offset = Math.Abs(den) > 1e-9 ? 0.5 * (y0 - y2) / den : 0.0;
            offset = Math.Clamp(offset, -1, 1);

            angles.Add(angle);
            radiiHit.Add(radii[peak] + offset * step);
        }

        var total = cfg.PinCaliperRays;
        if (angles.Count < total * 0.35)
            return null;

        // ---- 迭代剔除离群射线后做正弦拟合 ----
        var theta = angles.ToArray();
        var r0 = radiiHit.ToArray();
        var keep = new bool[theta.Length];
        Array.Fill(keep, true);

        double a = 0, b = 0, c = 0;
        var residual = new double[theta.Length];
        for (var iteration = 0; iteration < TrimIterations; iteration++)
        {
            if (!FitSinusoid(theta, r0, keep, out a, out b, out c))
                return null;

            for (var i = 0; i < theta.Length; i++)
                residual[i] = r0[i] - (a + b * Math.Cos(theta[i]) + c * Math.Sin(theta[i]));

            var kept = new List<double>(theta.Length);
            for (var i = 0; i < theta.Length; i++)
                if (keep[i])
                    kept.Add(residual[i]);
            if (kept.Count < 4)
                return null;
            var spread = StdDev(kept) + 1e-6;

            var next = new bool[theta.Length];
            var nextCount = 0;
            for (var i = 0; i < theta.Length; i++)
            {
                next[i] = Math.Abs(residual[i]) < 2.5 * spread
                          && Math.Abs(residual[i]) < trimLimitPx;
                if (next[i])
                    nextCount++;
            }

            var unchanged = nextCount == kept.Count;
            if (nextCount < 12 || unchanged)
            {
                if (nextCount >= 12)
                    keep = next;
                break;
            }
            keep = next;
        }

        if (!FitSinusoid(theta, r0, keep, out a, out b, out c))
            return null;

        var used = 0;
        var finalResiduals = new List<double>(theta.Length);
        for (var i = 0; i < theta.Length; i++)
        {
            if (!keep[i])
                continue;
            used++;
            finalResiduals.Add(r0[i] - (a + b * Math.Cos(theta[i]) + c * Math.Sin(theta[i])));
        }
        if (used < 12)
            return null;

        return new PinFit(
            new Point2d(origin.X + b, origin.Y + c),
            a,
            Math.Sqrt(b * b + c * c),
            StdDev(finalResiduals),
            used,
            total);
    }

    /// <summary>
    /// 三参数最小二乘 r = a + b·cosθ + c·sinθ 的正规方程解。
    /// 基函数有界（1、cos、sin），正规方程天然良态，无需 SVD。
    /// </summary>
    private static bool FitSinusoid(double[] theta, double[] r, bool[] keep,
        out double a, out double b, out double c)
    {
        a = b = c = 0;
        double n = 0, sc = 0, ss = 0, scc = 0, scs = 0, sss = 0;
        double sr = 0, src = 0, srs = 0;
        for (var i = 0; i < theta.Length; i++)
        {
            if (!keep[i])
                continue;
            var ct = Math.Cos(theta[i]);
            var st = Math.Sin(theta[i]);
            n += 1;
            sc += ct;
            ss += st;
            scc += ct * ct;
            scs += ct * st;
            sss += st * st;
            sr += r[i];
            src += r[i] * ct;
            srs += r[i] * st;
        }
        if (n < 3)
            return false;

        var det = n * (scc * sss - scs * scs)
                - sc * (sc * sss - scs * ss)
                + ss * (sc * scs - scc * ss);
        if (Math.Abs(det) < 1e-9)
            return false;

        a = (sr * (scc * sss - scs * scs)
           - sc * (src * sss - scs * srs)
           + ss * (src * scs - scc * srs)) / det;
        b = (n * (src * sss - scs * srs)
           - sr * (sc * sss - scs * ss)
           + ss * (sc * srs - src * ss)) / det;
        c = (n * (scc * srs - src * scs)
           - sc * (sc * srs - src * ss)
           + sr * (sc * scs - scc * ss)) / det;
        return true;
    }

    /// <summary>numpy.convolve(mode="same") 的零填充移动平均（核宽为奇数时居中）。</summary>
    private static void MovingAverageSame(double[] src, int count, int kernelWidth, double[] dst)
    {
        var half = kernelWidth / 2;
        var scale = 1.0 / kernelWidth;
        for (var i = 0; i < count; i++)
        {
            double sum = 0;
            for (var j = 0; j < kernelWidth; j++)
            {
                var k = i + j - half;
                if (k >= 0 && k < count)
                    sum += src[k];
            }
            dst[i] = sum * scale;
        }
    }

    private static double Percentile(ReadOnlySpan<double> sorted, double pct)
    {
        var pos = pct / 100.0 * (sorted.Length - 1);
        var lo = (int)Math.Floor(pos);
        var hi = Math.Min(lo + 1, sorted.Length - 1);
        var t = pos - lo;
        return sorted[lo] * (1 - t) + sorted[hi] * t;
    }

    private static double StdDev(double[] v, int count)
    {
        if (count <= 0)
            return 0;
        double sum = 0;
        for (var i = 0; i < count; i++)
            sum += v[i];
        var mean = sum / count;
        double acc = 0;
        for (var i = 0; i < count; i++)
        {
            var d = v[i] - mean;
            acc += d * d;
        }
        return Math.Sqrt(acc / count);
    }

    private static double StdDev(List<double> v)
    {
        if (v.Count == 0)
            return 0;
        double sum = 0;
        foreach (var x in v)
            sum += x;
        var mean = sum / v.Count;
        double acc = 0;
        foreach (var x in v)
        {
            var d = x - mean;
            acc += d * d;
        }
        return Math.Sqrt(acc / v.Count);
    }
}
