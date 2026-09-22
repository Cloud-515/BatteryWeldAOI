namespace BatteryWeldAOI.Core.Vision;

using BatteryWeldAOI.Core.Models;
using OpenCvSharp;

/// <summary>
/// 合成焊点场景渲染器：把「光滑基底 + 粗糙的圆形熔核 + 压在熔核中央的暗孔洞」
/// 这套**真机几何**画到图像上。
///
/// 生产侧的 <see cref="ImageGenerator"/>（仿真相机，干净场景）与测试侧的
/// 对抗性生成器共用本类，保证"现场看到的几何"与"回归测试验证的几何"永远是同一个。
///
/// 几何比例取自真机样本实测（见 RealSampleTests）：
///   熔核半径 = 0.155~0.195 × 短边；孔洞半径 = 0.235~0.275 × 熔核半径。
/// 孔心是基准（真值），熔核中心 = 孔心 + 待测偏移，二者之差即焊偏量。
///
/// 历史教训（本类存在的理由）：之前两个生成器画的都是"大极柱 + 柱面上的小焊环"，
/// 几何与真机正好相反，熔核中央根本没有孔洞，卡尺无孔可找。算法在那套图上
/// "验证通过"，一上真机 28 个点位全部误判 NG。几何必须是共享的单一事实来源。
///
/// 一个必须守住的约束：**孔洞与熔核的灰度差要足够大**（≥100 灰阶）。
/// 卡尺阈值是逐射线自适应的（≈熔核纹理梯度的一半），孔壁边缘若弱于熔核自身的
/// 鱼鳞纹理梯度，就找不到孔——这是几何比例之外最容易踩的坑。
/// </summary>
public static class WeldSceneRenderer
{
    /// <summary>
    /// 熔核半径 / 短边（**渲染值**）：真机 28 样本实测 0.1554~0.1977。
    ///
    /// **2026-09-17 曾试改 0.1225 后又回退，记录原因**：把夹具对齐到 20MP 现场图实测的族值
    /// （0.1428~0.1466）后，合成缺陷检出率发生大面积回归（`Spatter_IsDetected*` 等 10 余个用例
    /// 由 NG 变 OK）。根因是**缺陷是"按熔核半径画"的**——飞溅距离 `ringR×1.55~1.95`、
    /// 高光 `ringR×1.30~1.85`、炸焊坑 `ringR×0.25`、鱼鳞斑 `ringR×0.045`。
    /// 熔核缩小 19% 会连带把飞溅颗粒缩到 `MinSpatterBlobRatio` 之下、把炸焊坑缩到
    /// `MaxPoreAreaRatio` 之下，于是整套合成缺陷判据同时失效。**夹具的参数是一个整体标定，
    /// 不能只动其中一个**；要对齐现场尺度，必须连同全部缺陷尺寸与判据一起重标，本轮不做。
    ///
    /// 因此族值带（<see cref="InspectionConfig.RingRadiusFamilyMaxRatio"/>）保持宽松、
    /// 只当"触发重试"的信号，不承担"夹具 vs 现场"的判别职责；真正的可信度判据是
    /// **边界支撑弧段**（见 <see cref="WeldInspectionAlgorithm.IsTrustworthyRing"/>）。
    /// </summary>
    public const double NominalRingRadiusRatio = 0.155;

    public const double RingRadiusRatioJitter = 0.040;

    /// <summary>孔洞半径 / 熔核半径：真机实测均值 0.258（区间 0.215~0.349）。</summary>
    public const double PinRadiusRatio = 0.235;
    public const double PinRadiusRatioJitter = 0.040;

    /// <summary>炸焊熔核半径 / 短边：判据阈值 0.26，尺寸上限 0.32。</summary>
    public const double BlowoutRingRadiusRatio = 0.275;

    /// <summary>鱼鳞斑灰度散布（半幅）。决定熔核纹理能量的强弱。</summary>
    public const int NuggetBlobSpread = 45;

    /// <summary>场景几何（与灰度、光照无关，可单独用于断言）。</summary>
    /// <param name="HoleCenter">孔洞中心 = 基准（真值）。</param>
    /// <param name="HoleRadius">孔洞半径（像素）。</param>
    /// <param name="WeldCenter">熔核中心 = 孔心 + 待测偏移。</param>
    /// <param name="RingRadius">熔核半径（像素）。</param>
    public readonly record struct SceneGeometry(
        Point2d HoleCenter, double HoleRadius,
        Point2d WeldCenter, double RingRadius);

    /// <summary>按真机比例生成几何。孔心在视野内小幅漂移，模拟重复定位误差。</summary>
    public static SceneGeometry BuildGeometry(PointScenario scenario, int width, int height,
        Random rng, NinePointCalibration mmToPixel)
    {
        var baseSize = Math.Min(width, height);
        var nominalRingR = baseSize * (NominalRingRadiusRatio + RingRadiusRatioJitter * rng.NextDouble());
        var holeR = nominalRingR * (PinRadiusRatio + PinRadiusRatioJitter * rng.NextDouble());

        var blowout = scenario.Defect == WeldDefect.Blowout;
        var ringR = blowout ? baseSize * BlowoutRingRadiusRatio : nominalRingR;
        if (blowout)
            holeR *= 1.25; // 炸焊喷射会冲大孔口

        var holeCenter = new Point2d(
            width / 2.0 + rng.Next(-30, 31),
            height / 2.0 + rng.Next(-30, 31));
        var offsetPx = mmToPixel.Transform(new Point2d(scenario.TrueOffsetXmm, scenario.TrueOffsetYmm));

        return new SceneGeometry(holeCenter, holeR, holeCenter + offsetPx, ringR);
    }

    /// <summary>
    /// 粗糙熔核：铺底后按抖动极坐标网格密布鱼鳞斑。
    /// 必须"密布"而不是"随撒"——稀疏抖动会在阈值掩码上开洞，连通域碎成几块，
    /// 焊环检测会直接失败。
    /// </summary>
    public static void DrawNugget(Mat img, Point2d center, double ringR, int gray, int spread, Random rng)
    {
        Cv2.Circle(img, (Point)center, (int)Math.Round(ringR), Mono(gray), -1);

        var blobR = Math.Max(4.0, ringR * 0.045);
        var step = blobR * 1.15;
        var rings = Math.Max(1, (int)Math.Ceiling(ringR / step));
        for (var k = 0; k <= rings; k++)
        {
            var radius = ringR * k / rings;
            var count = Math.Max(6, (int)Math.Round(2 * Math.PI * radius / step));
            var phase = rng.NextDouble() * 2 * Math.PI;
            for (var i = 0; i < count; i++)
            {
                var angle = phase + 2 * Math.PI * i / count + (rng.NextDouble() - 0.5) * 0.6;
                var rr = Math.Max(0, radius + (rng.NextDouble() - 0.5) * step * 0.7);
                var p = new Point(
                    (int)(center.X + Math.Cos(angle) * rr),
                    (int)(center.Y + Math.Sin(angle) * rr));
                var g = Math.Clamp(gray + rng.Next(-spread, spread + 1), 0, 255);
                Cv2.Circle(img, p, (int)Math.Round(blobR * (0.75 + 0.5 * rng.NextDouble())), Mono(g), -1);
            }
        }
    }

    /// <summary>孔洞：暗孔腔。最后画，压在熔核之上（真机就是熔核中间打了一个孔）。</summary>
    public static void DrawHole(Mat img, Point2d center, double holeR, int gray)
    {
        Cv2.Circle(img, (Point)center, (int)Math.Round(holeR), Mono(gray), -1);
    }

    /// <summary>炸焊熔洞：熔核中央的不规则暗色凹坑。</summary>
    public static void DrawCrater(Mat img, Point2d center, double ringR, int holeGray, Random rng)
    {
        var craterR = ringR * (0.25 + 0.10 * rng.NextDouble());
        var gray = (int)(holeGray * (1.2 + 0.6 * rng.NextDouble()));
        var lumps = 7 + rng.Next(0, 5);
        for (var i = 0; i < lumps; i++)
        {
            var angle = rng.NextDouble() * 2 * Math.PI;
            var dist = craterR * (0.3 + 0.5 * rng.NextDouble());
            var p = new Point(
                (int)(center.X + Math.Cos(angle) * dist),
                (int)(center.Y + Math.Sin(angle) * dist));
            var g = Math.Clamp(gray + rng.Next(-25, 26), 0, 255);
            Cv2.Circle(img, p, (int)(craterR * (0.35 + 0.3 * rng.NextDouble())), Mono(g), -1);
        }
    }

    /// <summary>
    /// 飞溅：环外密布粗糙颗粒（与熔核同极性——物理上就是熔核甩出来的金属）。
    ///
    /// 颗粒必须**彼此分离**（单颗粒、不做多瓣堆叠），否则会连成一片、数量骤减，
    /// 越不过"颗粒数"判据。落点也要留出足够间隙：焊环检测的闭运算桥接距离约 4%×短边，
    /// 落在 1.3 倍环径处的颗粒会被并进熔核，拟合半径虚增——真机 27.png 就是这个现象，
    /// 合成场景里表现为飞溅被标成炸焊。1.55 倍以外才安全。
    /// 颗粒半径 ≥7px：质检环节的形态学开运算（≈1.2%×短边）会滤掉更小的颗粒。
    /// </summary>
    public static void DrawSpatter(Mat img, Point2d center, double ringR, int plateGray,
        bool bright, Random rng)
    {
        var count = 45 + rng.Next(0, 21);
        for (var i = 0; i < count; i++)
        {
            var angle = rng.NextDouble() * 2 * Math.PI;
            var dist = ringR * (1.55 + 0.40 * rng.NextDouble());
            var p = new Point(
                (int)(center.X + Math.Cos(angle) * dist),
                (int)(center.Y + Math.Sin(angle) * dist));
            // 与基底拉开对比度，保证单颗粒也有足够的纹理能量
            var g = bright
                ? Math.Min(255, plateGray + 45 + rng.Next(0, 51))
                : Math.Max(0, plateGray - 45 - rng.Next(0, 51));
            Cv2.Circle(img, p, 7 + rng.Next(0, 5), Mono(g), -1);
        }
    }

    /// <summary>
    /// 金属表面局部镜面高光。暗熔核场景取强高光（与熔核极性不同，算法应天然排除）；
    /// 亮熔核场景取弱高光（代表现场已用偏振光消强反光）。返回场景最亮值。
    /// </summary>
    public static int DrawHighlights(Mat img, Point2d center, double ringR, int plateGray,
        bool bright, Random rng)
    {
        var highlightGray = bright ? Math.Min(255, plateGray + 45) : 244 + rng.Next(0, 12);
        for (var i = 0; i < 3; i++)
        {
            var angle = rng.NextDouble() * 2 * Math.PI;
            var dist = ringR * (1.30 + rng.NextDouble() * 0.55);
            var p = new Point(
                (int)(center.X + Math.Cos(angle) * dist),
                (int)(center.Y + Math.Sin(angle) * dist));
            Cv2.Ellipse(img, p, new Size(6 + rng.Next(0, 9), 3 + rng.Next(0, 6)),
                rng.Next(0, 180), 0, 360, Mono(highlightGray), -1);
        }
        return highlightGray;
    }

    /// <summary>低对比度脏污：灰度起伏仅 ±8~18，低于纹理阈值，不得触发任何缺陷。</summary>
    public static void DrawDirt(Mat img, Point2d center, double ringR, int plateGray, Random rng)
    {
        var count = 6 + rng.Next(0, 5);
        for (var i = 0; i < count; i++)
        {
            var angle = rng.NextDouble() * 2 * Math.PI;
            var dist = ringR * (1.20 + rng.NextDouble() * 0.9);
            var p = new Point(
                (int)(center.X + Math.Cos(angle) * dist),
                (int)(center.Y + Math.Sin(angle) * dist));
            var g = Math.Clamp(plateGray + (rng.Next(2) == 0 ? 1 : -1) * (8 + rng.Next(0, 11)), 0, 255);
            Cv2.Circle(img, p, 5 + rng.Next(0, 12), Mono(g), -1);
        }
    }

    private static Scalar Mono(int gray) => new(gray, gray, gray);
}
