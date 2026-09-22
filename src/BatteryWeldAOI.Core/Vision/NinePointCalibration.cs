namespace BatteryWeldAOI.Core.Vision;

using OpenCvSharp;

/// <summary>
/// 九点标定：用 9 组「像素坐标 -> 机台物理坐标(毫米)」样本
/// 最小二乘拟合仿射变换（含旋转/缩放/平移，可吸收镜头畸变与安装偏角）。
/// 标定后视觉测得的像素偏差即可换算为物理偏差，或反算补焊坐标。
/// </summary>
public sealed class NinePointCalibration
{
    // x_mm = A*x_px + B*y_px + C ; y_mm = D*x_px + E*y_px + F
    private readonly double _a, _b, _c, _d, _e, _f;

    /// <summary>构造时若提供 9 组样本则直接拟合（内部按最小二乘求解）。</summary>
    public NinePointCalibration(IReadOnlyList<(Point2d Pixel, Point2d Mm)> pairs)
    {
        if (pairs == null || pairs.Count < 3)
            throw new ArgumentException("标定至少需要 3 组点对", nameof(pairs));

        (_a, _b, _c) = SolveLeastSquares(pairs, p => p.Pixel.X, p => p.Pixel.Y, p => p.Mm.X);
        (_d, _e, _f) = SolveLeastSquares(pairs, p => p.Pixel.X, p => p.Pixel.Y, p => p.Mm.Y);
    }

    /// <summary>
    /// 纯缩放标定（无旋转、无平移）：把 <paramref name="mmPerPixel"/> 这一个像素当量变成标定记录。
    ///
    /// 存在的理由：像素当量原本在**四个地方各写了一份**——`InspectionConfig.MmPerPixel`
    /// （只有 App/WPF 的 <c>DemoPlanBuilder.BuildCalibration</c> 消费它）、`tools/VdbImport`
    /// 硬编码的 `(1000,0)→(25,0)`、`tools/Diag` 里同样的字面量、以及现场数据测试里各写一遍。
    /// 2026-09-17 按现场实测把当量从 0.025 改成 0.1003 时，只有第一处跟着变了，
    /// 于是**同一批现场图在界面里与在诊断工具/测试里会报出差 4 倍的毫米数**（实测踩到）。
    /// 现场数据一律用本工厂从配置当量取值，不要再写 `0.025` 这类字面量。
    /// </summary>
    public static NinePointCalibration PureScale(double mmPerPixel) => new(new List<(Point2d, Point2d)>
    {
        (new Point2d(0, 0), new Point2d(0, 0)),
        (new Point2d(1000, 0), new Point2d(1000 * mmPerPixel, 0)),
        (new Point2d(0, 1000), new Point2d(0, 1000 * mmPerPixel)),
    });

    private NinePointCalibration(double a, double b, double c, double d, double e, double f)
    {
        _a = a; _b = b; _c = c; _d = d; _e = e; _f = f;
    }

    /// <summary>像素坐标 -> 物理坐标（毫米）。</summary>
    public Point2d Transform(Point2d pixel) => new(
        _a * pixel.X + _b * pixel.Y + _c,
        _d * pixel.X + _e * pixel.Y + _f);

    /// <summary>返回本变换的逆变换（物理坐标 -> 像素坐标），用于生成标定用运动轨迹。</summary>
    public NinePointCalibration Invert()
    {
        var det = _a * _e - _b * _d;
        if (Math.Abs(det) < 1e-12)
            throw new InvalidOperationException("仿射矩阵奇异，无法求逆（标定样本共线?）");

        double ia = _e / det, ib = -_b / det, id = -_d / det, ie = _a / det;
        return new NinePointCalibration(
            ia, ib, -(ia * _c + ib * _f),
            id, ie, -(id * _c + ie * _f));
    }

    /// <summary>
    /// 法方程最小二乘：拟合 z = a*x + b*y + c。
    /// 正规方程为 3x3 线性方程组，用克莱姆法则求解。
    /// </summary>
    private static (double A, double B, double C) SolveLeastSquares(
        IReadOnlyList<(Point2d Pixel, Point2d Mm)> pairs,
        Func<(Point2d Pixel, Point2d Mm), double> x,
        Func<(Point2d Pixel, Point2d Mm), double> y,
        Func<(Point2d Pixel, Point2d Mm), double> z)
    {
        double sxx = 0, sxy = 0, sx = 0, syy = 0, sy = 0, n = pairs.Count;
        double szx = 0, szy = 0, sz = 0;
        foreach (var p in pairs)
        {
            var px = x(p); var py = y(p); var pz = z(p);
            sxx += px * px; sxy += px * py; sx += px;
            syy += py * py; sy += py;
            szx += px * pz; szy += py * pz; sz += pz;
        }

        // | sxx sxy sx | |a|   |szx|
        // | sxy syy sy | |b| = |szy|
        // | sx  sy  n  | |c|   |sz |
        double m00 = sxx, m01 = sxy, m02 = sx;
        double m10 = sxy, m11 = syy, m12 = sy;
        double m20 = sx, m21 = sy, m22 = n;

        var (a, b, c) = Solve3(m00, m01, m02, m10, m11, m12, m20, m21, m22, szx, szy, sz);
        return (a, b, c);
    }

    /// <summary>
    /// Kasa 最小二乘圆拟合：x²+y² + a·x + b·y + c = 0，圆心 (-a/2, -b/2)。
    /// 供卡尺边缘精化与焊缝环拟合复用；点数不足或退化时返回 null。
    /// </summary>
    public static (Point2d Center, double Radius)? FitCircle(IReadOnlyList<Point2d> pts)
    {
        if (pts.Count < 8)
            return null;

        double sxx = 0, syy = 0, sxy = 0, sx = 0, sy = 0, sqx = 0, sqy = 0, sq = 0;
        foreach (var p in pts)
        {
            var q = p.X * p.X + p.Y * p.Y;
            sxx += p.X * p.X;
            syy += p.Y * p.Y;
            sxy += p.X * p.Y;
            sx += p.X;
            sy += p.Y;
            sqx += p.X * q;
            sqy += p.Y * q;
            sq += q;
        }

        try
        {
            var n = (double)pts.Count;
            var (a, b, c) = Solve3(
                sxx, sxy, sx, sxy, syy, sy, sx, sy, n, -sqx, -sqy, -sq);
            var center = new Point2d(-a / 2, -b / 2);
            // 圆方程 x²+y²+ax+by+c=0 → r² = center² − c（注意 c 的符号，
            // 此前误写为 center² + c，在绝对像素坐标下会得到 ~1100px 的荒谬半径）
            var radius = Math.Sqrt(Math.Max(1.0, center.X * center.X + center.Y * center.Y - c));
            return (center, radius);
        }
        catch (InvalidOperationException)
        {
            return null; // 点共线等退化情形
        }
    }

    /// <summary>克莱姆法则求解 3x3 线性方程组（亦供卡尺圆拟合复用）。</summary>
    internal static (double, double, double) Solve3(
        double m00, double m01, double m02,
        double m10, double m11, double m12,
        double m20, double m21, double m22,
        double r0, double r1, double r2)
    {
        double Det(double a0, double a1, double a2, double b0, double b1, double b2, double c0, double c1, double c2)
            => a0 * (b1 * c2 - b2 * c1) - a1 * (b0 * c2 - b2 * c0) + a2 * (b0 * c1 - b1 * c0);

        var det = Det(m00, m01, m02, m10, m11, m12, m20, m21, m22);
        if (Math.Abs(det) < 1e-12)
            throw new InvalidOperationException("标定样本退化（共线），无法求解仿射变换");

        var da = Det(r0, m01, m02, r1, m11, m12, r2, m21, m22);
        var db = Det(m00, r0, m02, m10, r1, m12, m20, r2, m22);
        var dc = Det(m00, m01, r0, m10, m11, r1, m20, m21, r2);
        return (da / det, db / det, dc / det);
    }
}
