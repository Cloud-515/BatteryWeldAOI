namespace BatteryWeldAOI.Core.Vision;

using BatteryWeldAOI.Core.Models;
using OpenCvSharp;

/// <summary>熔核内沿（焊环的内边界）定位结果。</summary>
/// <param name="Center">内沿拟合圆心（与焊环外边界应为同心——实测偏差中位 1.3px）。</param>
/// <param name="Radius">内沿半径（像素）。</param>
/// <param name="ResidualPx">内沿圆拟合的残差标准差（像素）。</param>
/// <param name="ConcentricDeviationPx">与焊环外边界圆心的偏差（像素）。</param>
/// <param name="UsedRays">命中内沿的射线数。</param>
internal sealed record WeldInnerFit(Point2d Center, double Radius, double ResidualPx,
    double ConcentricDeviationPx, int UsedRays);

/// <summary>
/// 熔核**内沿**定位——焊环是"有厚度的环"，只有外边界表达不出环宽。
///
/// 为什么不能从掩码里取内沿：<see cref="WeldRingLocator"/> 的闭运算（0.045×base、两次迭代）
/// 会把约 20px 的极柱孔一起封死，实测 OK_312 与 Img000 的"最大内切圆/等效半径"都是 0.97
/// （即后处理掩码是**实心盘**），内沿信息在掩码里已经不存在了。
///
/// 因此在**纹理能量图**（<see cref="RingDetection.Texture"/>）上直接找"粗糙起始半径"：
/// 从焊环圆心向外走一条射线，内侧是光滑的极柱端面与暗缝隙（能量低），外侧是粗糙熔核（能量高），
/// 第一次持续超过二值化阈值处即熔核内沿。
///
/// 起点必须**跳过极柱及其边缘带**（取 1.15×极柱半径）：极柱边缘是亮→暗阶跃，局部标准差同样很高，
/// 否则会把它误当成内沿。这里正好复用了本轮"灰圆独立测量"的成果——极柱半径现在可信。
///
/// 实测（tests/pp 前 10 帧，工作分辨率）：内/外半径比中位 0.347（0.228~0.428）、
/// 环宽中位 344px（外半径 526px 时，即环宽约占外半径 65%）、
/// 内沿拟合残差中位 1.75px、**内外圆心偏差中位 1.3px**——即两条边确实同心，
/// 这与"熔核的内沿与外沿同心于激光中心"这一物理事实一致。
/// 反过来，内外圆心偏差大就是污染的指示器：两个残留伪候选帧（外半径 717/705px）的偏差达 48.8px。
/// </summary>
internal static class WeldInnerLocator
{
    /// <param name="energy">纹理能量图（CV_32F，与焊环检测同一张，已做边框置零）。</param>
    /// <param name="threshold">焊环二值化阈值（与 <see cref="RingDetection.Threshold"/> 同值）。</param>
    /// <param name="ringCenter">焊环外边界拟合圆心（射线原点）。</param>
    /// <param name="ringRadius">焊环外边界拟合半径（搜索外界的基准）。</param>
    /// <param name="pinCenter">灰圆（孔位）中心——用于剔除"被孔位挡住、量不到内沿"的射线。</param>
    /// <param name="pinRadius">灰圆半径（跳过半径 = <see cref="InspectionConfig.WeldInnerSearchStartRatio"/>×本值）。</param>
    /// <param name="rays">射线数。</param>
    public static WeldInnerFit? Locate(Mat energy, double threshold, Point2d ringCenter,
        double ringRadius, Point2d pinCenter, double pinRadius, InspectionConfig cfg, int rays)
    {
        if (ringRadius < 12 || pinRadius <= 0 || rays < 8)
            return null;

        var rStart = Math.Max(2.0, cfg.WeldInnerSearchStartRatio * pinRadius);
        var rEnd = cfg.WeldInnerSearchOuterRatio * ringRadius;
        if (rEnd - rStart < 6)
            return null;

        // ---- 被孔位挡住的射线必须整条弃用，而不是"把起点挪到孔位之外" ----
        // 用户 2026-09-17 指出橙色圈没有贴合焊环内边缘。根因是本方法给极柱的"让位"方式：
        // 原先只是把**起点半径**提到 1.15×灰圆半径，隐含假设"孔位在焊环圆心附近"，于是起点
        // 恰好落在真内沿附近——"第一次超阈值"命中的其实是起点本身（旧值比率 0.374 与起点半径
        // 比率 0.339 几乎相同，即那个"内沿"主要来自起点）。孔位基准修正后灰圆半径翻倍（9.9→16.6px），
        // 起点从 11.4px 提到 19.1px，又刚好压在真内沿（约 0.37×焊环半径 ≈ 25px）旁边，
        // 报出的内/外半径比 0.28~0.31、仍然偏小且随孔位半径漂移。
        //
        // 几何事实：孔心偏离焊环圆心约 0.3×焊环半径（= 孔半径的 1.3 倍），因此**孔位那一侧的
        // 内沿被孔位圆盘遮住，本来就量不到**；其余方向（约 2/3 的角度）内沿是可见的。
        // 所以正确做法是：搜索窗与孔位圆盘相交的射线**直接弃用**，只拟合真正看得见的那部分弧。
        // 圆拟合不需要整圈——实测可用弧约 240°，足以定圆心与半径。
        var skipRadius = cfg.WeldInnerSearchStartRatio * pinRadius;

        var width = energy.Width;
        var height = energy.Height;
        var energyIdx = energy.GetGenericIndexer<float>();

        var step = 0.5;
        var kernelWidth = Geo.Odd(ringRadius * 0.02, 3, 31);
        var half = kernelWidth / 2;

        var points = new List<Point2d>(rays);
        var scratch = new double[2048];
        var smoothed = new double[2048];
        var usableRays = 0;

        for (var k = 0; k < rays; k++)
        {
            var angle = 2 * Math.PI * k / rays;
            var ca = Math.Cos(angle);
            var sa = Math.Sin(angle);

            // 射线与孔位圆盘是否相交：把孔心投影到射线上取最近点，再看最近距离
            var toPin = new Point2d(pinCenter.X - ringCenter.X, pinCenter.Y - ringCenter.Y);
            var along = toPin.X * ca + toPin.Y * sa;
            var clamped = Math.Clamp(along, rStart, rEnd);
            var closestX = ringCenter.X + ca * clamped - pinCenter.X;
            var closestY = ringCenter.Y + sa * clamped - pinCenter.Y;
            if (closestX * closestX + closestY * closestY < skipRadius * skipRadius)
                continue;   // 这条射线在搜索窗内穿过孔位，内沿被遮住

            usableRays++;

            var count = 0;
            var radiusAt = new double[2048];
            for (var r = rStart; r <= rEnd && count < scratch.Length; r += step)
            {
                var x = (int)Math.Round(ringCenter.X + ca * r);
                var y = (int)Math.Round(ringCenter.Y + sa * r);
                if (x < 1 || x >= width - 1 || y < 1 || y >= height - 1)
                    continue;
                scratch[count] = energyIdx[y, x];
                radiusAt[count] = r;
                count++;
            }
            if (count < 10)
                continue;

            // 沿射线做移动平均，抑制单点（鱼鳞斑内部）噪声引起的假沿
            for (var i = 0; i < count; i++)
            {
                double sum = 0;
                for (var j = 0; j < kernelWidth; j++)
                {
                    var m = i + j - half;
                    if (m >= 0 && m < count)
                        sum += scratch[m];
                }
                smoothed[i] = sum / kernelWidth;
            }

            // 第一个**持续**超过阈值的样本（用 r 而非索引，因为越界样本被跳过了）。
            //
            // **必须"持续"而不是"第一次"**（2026-09-17 修正，这是"橙圈不贴合内沿"的直接原因）：
            // 极柱边缘是亮→暗阶跃，局部标准差（纹理能量）本来就很高，搜索起点附近的第一个样本
            // 往往就已经超过阈值——于是"第一次超阈值"命中的是**起点本身**，返回的"内沿半径"
            // 恒等于 `1.15×极柱半径`，环宽成了纯人工产物。实测两帧完全吻合这个恒等式：
            //   · Img010：内半径 159.5 = 1.15 × 138.4（极柱半径）；
            //   · Img003：内半径 241.1 = 1.15 × 209.9。
            // 现在要求连续 `persist` 个样本（对应 WeldInnerMinRunPx 个工作像素）都在阈值之上，
            // 单点/窄带的高能量（极柱边缘、鱼鳞斑内部噪点）不再被当成内沿。
            var persist = Math.Max(2, (int)Math.Round(cfg.WeldInnerMinRunPx / step));
            for (var i = 0; i + persist <= count; i++)
            {
                var run = true;
                for (var j = 0; j < persist; j++)
                {
                    if (smoothed[i + j] <= threshold)
                    {
                        run = false;
                        break;
                    }
                }
                if (!run)
                    continue;

                points.Add(new Point2d(ringCenter.X + ca * radiusAt[i],
                                       ringCenter.Y + sa * radiusAt[i]));
                break;
            }
        }

        // 命中射线太少说明内沿不成环（缺口过大、或能量阈值不适用）。
        // 下限按**可用射线**（搜索窗不与孔位相交的那些）计，而不是按总射线数：
        // 孔位一侧本来就被遮住，用总数作分母会把"内沿正常但孔位偏大"的帧误判成不成环。
        if (points.Count < Math.Max(8, usableRays * 0.5))
            return null;

        var fit = Geo.FitCircle(points);
        if (fit is null || fit.Value.Radius < 1)
            return null;

        var center = fit.Value.Center;
        var radius = fit.Value.Radius;

        // 假环宽守卫：拟合半径与搜索起点几乎相同，说明"内沿"就是起点本身
        // （极柱边缘的能量让每条射线一开始就超阈），不是一条真实的边。
        // 宁可返回 null 让界面显示"—"，也不要输出一个等于 `1.15×极柱半径` 的假环宽。
        if (Math.Abs(radius - rStart) < 1.0)
            return null;

        // 口径守卫：真内沿的实测内/外半径比约 0.31~0.37，且必然大于极柱半径。
        // 落在 0.6×外半径之外说明拟合到的是外边界附近的纹理，不是内沿。
        if (radius > 0.6 * ringRadius)
            return null;
        double acc = 0;
        foreach (var p in points)
        {
            var d = Geo.Dist(p, center) - radius;
            acc += d * d;
        }
        var residual = Math.Sqrt(acc / points.Count);
        var concentric = Geo.Dist(center, ringCenter);

        return new WeldInnerFit(center, radius, residual, concentric, points.Count);
    }
}
