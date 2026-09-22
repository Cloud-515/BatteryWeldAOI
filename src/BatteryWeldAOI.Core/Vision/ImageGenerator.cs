namespace BatteryWeldAOI.Core.Vision;

using BatteryWeldAOI.Core.Models;
using OpenCvSharp;

/// <summary>
/// 合成图像生成器（生产侧）：按剧本渲染一张"光滑金属端面 + 粗糙圆形熔核 + 中央暗孔洞"的图像，
/// 供仿真相机、控制台演示程序与 WPF 演示模式使用。
///
/// 几何由 <see cref="WeldSceneRenderer"/> 统一提供——它与回归测试用的对抗性生成器共用
/// 同一套比例常数，杜绝"在测试图上验证通过、真机上几何完全不同"。
/// 本类只叠加**温和**的成像干扰（轻微曝光起伏 + 传感器噪声），用于演示与流程联调；
/// 严苛的光照梯度、极性翻转、镜面高光等干扰由测试侧的对抗性生成器覆盖。
/// </summary>
public sealed class ImageGenerator
{
    private readonly InspectionConfig _config;
    private readonly NinePointCalibration _mmToPx;

    public ImageGenerator(InspectionConfig config, NinePointCalibration pixelToMm)
    {
        _config = config;
        _mmToPx = pixelToMm.Invert();
    }

    /// <summary>渲染一张合成图像（调用方负责 Dispose 返回的 Mat）。</summary>
    public Mat Render(PointScenario scenario, Random rng)
    {
        var w = _config.ImageWidth;
        var h = _config.ImageHeight;
        var geo = WeldSceneRenderer.BuildGeometry(scenario, w, h, rng, _mmToPx);

        // 铝熔核在环形光下比端面亮；孔腔不受光，恒暗。
        // 孔洞与熔核的灰度差必须够大（这里 ~150 灰阶），否则卡尺找不到孔壁。
        var plateGray = 128 + rng.Next(0, 24);
        var nuggetGray = Math.Min(255, plateGray + 58 + rng.Next(0, 20));
        var holeGray = 18 + rng.Next(0, 25);

        var img = new Mat(h, w, MatType.CV_8UC3, Mono(plateGray));

        if (scenario.Defect != WeldDefect.MissingWeld)
        {
            WeldSceneRenderer.DrawNugget(img, geo.WeldCenter, geo.RingRadius,
                nuggetGray, WeldSceneRenderer.NuggetBlobSpread, rng);
            if (scenario.Defect == WeldDefect.Blowout)
                WeldSceneRenderer.DrawCrater(img, geo.WeldCenter, geo.RingRadius, holeGray, rng);
        }

        // 孔洞最后画：它压在熔核之上（真机就是熔核中间打了一个孔）
        WeldSceneRenderer.DrawHole(img, geo.HoleCenter, geo.HoleRadius, holeGray);

        if (scenario.Defect == WeldDefect.Spatter)
            WeldSceneRenderer.DrawSpatter(img, geo.WeldCenter, geo.RingRadius, plateGray, bright: true, rng);

        WeldSceneRenderer.DrawHighlights(img, geo.WeldCenter, geo.RingRadius, plateGray, bright: true, rng);
        AddSensorNoise(img, sigma: 3);

        return img;
    }

    private static void AddSensorNoise(Mat img, double sigma)
    {
        using var noise = new Mat(img.Size(), MatType.CV_32FC3, Scalar.All(0));
        Cv2.Randn(noise, Scalar.All(0), Scalar.All(sigma));
        using var img32 = new Mat();
        img.ConvertTo(img32, MatType.CV_32FC3);
        Cv2.Add(img32, noise, img32);
        img32.ConvertTo(img, MatType.CV_8UC3);
    }

    private static Scalar Mono(int gray) => new(gray, gray, gray);
}
