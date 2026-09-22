using BatteryWeldAOI.Core.Models;
using BatteryWeldAOI.Core.Vision;
using OpenCvSharp;

namespace BatteryWeldAOI.Core.Tests;

/// <summary>
/// 对抗性合成图像生成器：在 <see cref="WeldSceneRenderer"/> 的**真机几何**之上叠加真实成像干扰，
/// 验证算法不含"对着合成图调出来的常数"。
///
/// 几何全部取自共享渲染器（熔核半径 = 0.155~0.195 × 短边，孔洞半径 = 0.235~0.275 × 熔核半径），
/// 与生产侧的 <see cref="ImageGenerator"/> 同源，二者的场景拓扑不可能再各自漂移。
/// （旧版这里画的是"大极柱 + 柱面上的小焊环"——几何与真机正好相反，熔核中央没有孔洞，
///   卡尺无孔可找，因此完全无法验证本算法。历史教训见 WeldSceneRenderer 的类注释。）
///
/// 本类负责的干扰：随机曝光（0.7~1.4 倍，按场景最亮值做饱和保护）、线性光照梯度（±25%）、
/// 渐晕（0~25%）、基底/熔核反照率随机、焊缝极性（熔核比基底亮/暗，各 50%）、
/// 镜面高光、低对比度脏污、传感器噪声（σ=4~9）。
/// 场景（几何/反照率/极性）与光照用两个独立种子驱动，支持"同场景不同光照"的不变性测试。
///
/// 缺陷场景：漏焊——不画熔核；炸焊——熔核放大到 0.275×短边 + 中央熔洞；
///           焊偏——熔核整体偏离孔心；飞溅——环外密布粗糙颗粒。
/// </summary>
public sealed class RealisticImageGenerator
{
    private readonly InspectionConfig _config;
    private readonly NinePointCalibration _mmToPx;

    public RealisticImageGenerator(InspectionConfig config, NinePointCalibration pixelToMm)
    {
        _config = config;
        _mmToPx = pixelToMm.Invert();
    }

    /// <summary>渲染一张"真实感"干扰图像（调用方负责 Dispose 返回的 Mat）。</summary>
    /// <param name="sceneSeed">场景种子：孔心位置/半径、反照率、极性、缺陷几何。</param>
    /// <param name="illumSeed">光照种子：曝光、梯度、渐晕、噪声。</param>
    /// <param name="forceBrightWeld">强制熔核极性（null = 由场景种子随机决定）。</param>
    public Mat Render(PointScenario scenario, int sceneSeed, int illumSeed, bool? forceBrightWeld = null)
    {
        var rng = new Random(sceneSeed);
        var w = _config.ImageWidth;
        var h = _config.ImageHeight;
        var geo = WeldSceneRenderer.BuildGeometry(scenario, w, h, rng, _mmToPx);

        // ---- 场景属性 ----
        var bright = forceBrightWeld ?? rng.Next(2) == 0;
        // 熔核与基底的灰度必须反号，才能验证算法不依赖对比度极性
        var plateGray = bright ? 128 + rng.Next(0, 28) : 188 + rng.Next(0, 33);
        var nuggetGray = bright ? 186 + rng.Next(0, 22) : 142 + rng.Next(0, 24);
        // 孔腔不受光，恒为暗；与熔核的灰度差远大于熔核自身的纹理起伏
        var holeGray = 18 + rng.Next(0, 25);

        var img = new Mat(h, w, MatType.CV_8UC3, Mono(plateGray));
        var sceneMax = plateGray;

        if (scenario.Defect != WeldDefect.MissingWeld)
        {
            WeldSceneRenderer.DrawNugget(img, geo.WeldCenter, geo.RingRadius,
                nuggetGray, WeldSceneRenderer.NuggetBlobSpread, rng);
            sceneMax = Math.Max(sceneMax, nuggetGray + WeldSceneRenderer.NuggetBlobSpread);
            if (scenario.Defect == WeldDefect.Blowout)
                WeldSceneRenderer.DrawCrater(img, geo.WeldCenter, geo.RingRadius, holeGray, rng);
        }

        // 孔洞最后画：它压在熔核之上（真机就是熔核中间打了一个孔）
        WeldSceneRenderer.DrawHole(img, geo.HoleCenter, geo.HoleRadius, holeGray);

        if (scenario.Defect == WeldDefect.Spatter)
            WeldSceneRenderer.DrawSpatter(img, geo.WeldCenter, geo.RingRadius, plateGray, bright, rng);

        sceneMax = Math.Max(sceneMax,
            WeldSceneRenderer.DrawHighlights(img, geo.WeldCenter, geo.RingRadius, plateGray, bright, rng));
        WeldSceneRenderer.DrawDirt(img, geo.WeldCenter, geo.RingRadius, plateGray, rng);

        ApplyIllumination(img, illumSeed, sceneMax);
        return img;
    }

    /// <summary>施加光照与传感器干扰：曝光 × 光照梯度 × 渐晕 + 高斯噪声（逐像素）。
    /// protectedMax 非空时按"该场景值经梯度峰值放大后不超过 240"压低曝光——
    /// 饱和会把熔核纹理抹平，那样测的就不是算法而是相机动态范围了。</summary>
    private static void ApplyIllumination(Mat img, int seed, int? protectedMax = null)
    {
        var rng = new Random(seed);
        var exposure = 0.7 + 0.7 * rng.NextDouble();
        var gradX = (rng.NextDouble() - 0.5) * 0.5;
        var gradY = (rng.NextDouble() - 0.5) * 0.5;
        var vignette = rng.NextDouble() * 0.25;
        var sigma = 4 + 5 * rng.NextDouble();

        if (protectedMax is { } max)
            exposure = Math.Min(exposure, 240.0 / (max * (1 + (Math.Abs(gradX) + Math.Abs(gradY)) / 2)));

        var w = img.Width;
        var h = img.Height;
        var cx = w / 2.0;
        var cy = h / 2.0;
        var maxR = Math.Sqrt(cx * cx + cy * cy);

        img.ConvertTo(img, MatType.CV_32FC3);
        for (var y = 0; y < h; y++)
        {
            for (var x = 0; x < w; x++)
            {
                var f = exposure * (1 + gradX * (x / (double)w - 0.5) + gradY * (y / (double)h - 0.5));
                var rr = Math.Sqrt((x - cx) * (x - cx) + (y - cy) * (y - cy)) / maxR;
                f *= 1 - vignette * rr * rr;

                // 三个均匀分布之和近似高斯
                var noise = (rng.NextDouble() + rng.NextDouble() + rng.NextDouble() - 1.5) * sigma;
                var v = img.At<Vec3f>(y, x);
                img.Set(y, x, new Vec3f(
                    (float)Math.Clamp(v.Item0 * f + noise, 0, 255),
                    (float)Math.Clamp(v.Item1 * f + noise, 0, 255),
                    (float)Math.Clamp(v.Item2 * f + noise, 0, 255)));
            }
        }
        img.ConvertTo(img, MatType.CV_8UC3);
    }

    private static Scalar Mono(int gray) => new(gray, gray, gray);
}
