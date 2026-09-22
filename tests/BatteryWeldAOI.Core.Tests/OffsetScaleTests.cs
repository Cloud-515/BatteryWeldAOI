using BatteryWeldAOI.Core.Models;
using BatteryWeldAOI.Core.Vision;
using OpenCvSharp;
using Xunit;
using Xunit.Abstractions;

namespace BatteryWeldAOI.Core.Tests;

/// <summary>
/// 毫米折算模型与像素坐标系口径的回归测试。
///
/// 这里守的是两个"不说清就会被误读"的地方：
///
/// 1. **毫米折算必须是各向同性的**。旧实现对 X 用 <c>ImageWidth÷图宽</c>、对 Y 用
///    <c>ImageHeight÷图高</c> 分轴折算，于是输入图与配置视野宽高比不一致时，
///    同一段物理距离在 X/Y 上被折成不同的毫米数。现场图（5472×3648，宽高比 1.50）
///    配默认配置（1280×1024，宽高比 1.25）实测两轴系数 1.855 vs 2.226，**差 20%**——
///    也就是偏移量的方向被歪曲 20%，而 1.0mm 公差判定就建立在这个合成值上。
///    <see cref="OffsetMillimetres_UseOneScaleForBothAxes"/> 用"mm 偏移 ÷ 像素偏移
///    在两轴上必须相等"来钉住它（旧实现会以 1.20 倍的差距失败）。
///
/// 2. **给操作员的像素坐标必须是原图坐标**。算法在输入图较大时先缩到工作分辨率再检测，
///    若把内部坐标直接显示出来，操作员拿它去图上核对是量不到的
///    （20MP 现场图上与显示值差约 8 倍），"标注圈是不是画偏了"这类问题就没法自查。
/// </summary>
public class OffsetScaleTests
{
    private readonly ITestOutputHelper _output;
    private readonly InspectionConfig _config = new();
    private readonly WeldInspectionAlgorithm _algorithm;

    public OffsetScaleTests(ITestOutputHelper output)
    {
        _output = output;
        // 纯缩放标定，当量取自配置（0.1003 mm/标定像素，2026-09-17 现场实测重标）。
        // 必须是纯缩放：本类的"两轴比值相等"断言只在标定不含旋转时成立——
        // 带旋转的仿射会把 X/Y 分量互相混合，那是标定的性质，不是折算模型的问题。
        _algorithm = new WeldInspectionAlgorithm(_config,
            NinePointCalibration.PureScale(_config.MmPerPixel));
    }

    // -----------------------------------------------------------------------------------

    [Fact]
    public void OffsetMillimetres_UseOneScaleForBothAxes()
    {
        var files = FindPpFrames(limit: 12);
        if (files.Count == 0)
        {
            _output.WriteLine("未找到 tests/pp 样本，跳过。");
            return;
        }

        var checkedFrames = 0;
        foreach (var file in files)
        {
            using var image = Cv2.ImRead(file, ImreadModes.Color);
            Assert.False(image.Empty(), $"{Path.GetFileName(file)}: 读取失败");

            var result = _algorithm.Inspect(image, new WeldPoint { Index = 0, Name = "t" });
            if (!result.HasValidOffset)
                continue;

            // 像素偏移与毫米偏移必须成**同一个**比例。
            //
            // **2026-09-17 改用原图坐标**：孔锚精修（InspectionConfig.WeldRingRefineEdgeFraction）
            // 把测量偏置去掉之后，真实偏移只剩 0.2~0.3mm——在工作分辨率（短边 460）上不到 2 个像素，
            // 比值会被取整完全主导（原判据 `|dxPx| < 2 就跳过` 因此把所有帧都跳过了）。
            // 换算到原图坐标后分量是 8~20px，比值有判别力；两轴同系数这一性质与坐标系无关。
            var dxPx = result.WeldCenterSourcePx.X - result.PoleCenterSourcePx.X;
            var dyPx = result.WeldCenterSourcePx.Y - result.PoleCenterSourcePx.Y;
            if (Math.Abs(dxPx) < 2 || Math.Abs(dyPx) < 2)
                continue;   // 分量太小则比值由舍入主导，不具判别力

            var kx = result.OffsetXmm / dxPx;
            var ky = result.OffsetYmm / dyPx;
            checkedFrames++;

            // 单一系数：这正是旧实现差 20% 的地方（1.855 vs 2.226）
            Assert.Equal(kx, ky, 6);

            // 系数本身 = 配置当量 × (配置宽 ÷ **原图**宽)，两轴同一个式子
            var expected = _config.MmPerPixel * _config.ImageWidth / result.SourceSizePx.Width;
            Assert.Equal(expected, kx, 8);

            _output.WriteLine($"{Path.GetFileName(file)[..14]} 原图{result.SourceSizePx.Width}px " +
                              $"kX={kx:F6} kY={ky:F6}（相等）");
        }

        Assert.True(checkedFrames > 0, "没有一帧的偏移量分量足够大，无法验证各向同性");
        _output.WriteLine($"已在 {checkedFrames} 帧上验证两轴同系数");
    }

    [Fact]
    public void SourceCoordinates_MapBackToOriginalImage()
    {
        var files = FindPpFrames(limit: 3);
        if (files.Count == 0)
        {
            _output.WriteLine("未找到 tests/pp 样本，跳过。");
            return;
        }

        foreach (var file in files)
        {
            using var image = Cv2.ImRead(file, ImreadModes.Color);
            Assert.False(image.Empty());

            var result = _algorithm.Inspect(image, new WeldPoint { Index = 0, Name = "t" });

            // 原图尺寸必须是**原始输入图**，而不是被归一化后的工作图
            Assert.Equal(image.Width, result.SourceSizePx.Width);
            Assert.Equal(image.Height, result.SourceSizePx.Height);
            if (result.ProcessedSizePx.Width > 0)
                Assert.Equal(image.Width / (double)result.ProcessedSizePx.Width, result.SourceScale, 6);

            if (result.WeldRadiusPx > 0)
            {
                // 换算关系必须一致，且结果落在原图范围内
                Assert.Equal(result.WeldCenterPx.X * result.SourceScale, result.WeldCenterSourcePx.X, 6);
                Assert.Equal(result.WeldCenterPx.Y * result.SourceScale, result.WeldCenterSourcePx.Y, 6);
                Assert.Equal(result.WeldRadiusPx * result.SourceScale, result.WeldRadiusSourcePx, 6);
                Assert.InRange(result.WeldCenterSourcePx.X, 0, image.Width);
                Assert.InRange(result.WeldCenterSourcePx.Y, 0, image.Height);
                Assert.InRange(result.PoleCenterSourcePx.X, 0, image.Width);
                Assert.InRange(result.PoleCenterSourcePx.Y, 0, image.Height);

                _output.WriteLine($"{Path.GetFileName(file)[..14]} 原图{image.Width}×{image.Height} " +
                                  $"工作{result.ProcessedSizePx.Width}px 缩放比{result.SourceScale:F3} " +
                                  $"焊缝中心(原图)=({result.WeldCenterSourcePx.X:F1}, {result.WeldCenterSourcePx.Y:F1}) " +
                                  $"r={result.WeldRadiusSourcePx:F1}");
            }
        }
    }

    [Fact]
    public void SyntheticImage_SourceCoordinatesUseConfiguredSize_NotWorkSize()
    {
        var calibration = new NinePointCalibration(new List<(Point2d, Point2d)>
        {
            (new Point2d(0, 0), new Point2d(0, 0)),
            (new Point2d(1000, 0), new Point2d(25, 0)),
            (new Point2d(0, 1000), new Point2d(0, 25)),
        });
        var generator = new ImageGenerator(_config, calibration);
        using var image = generator.Render(
            new PointScenario
            {
                Point = new WeldPoint { Index = 0, Name = "ok0" },
                Defect = null,
                TrueOffsetXmm = 0,
                TrueOffsetYmm = 0,
            },
            new Random(1));

        var result = _algorithm.Inspect(image, new WeldPoint { Index = 0, Name = "ok0" });

        // 合成图按配置分辨率渲染（1280×1024），短边 1024 **同样**超过 MaxWorkingBase=460，
        // 因此它一样会被归一化。这里要钉的是：喂给界面/报表的"原图坐标"必须是 1280×1024 口径，
        // 而不是内部那张 575×460 的工作图口径——否则操作员拿坐标去图上量会差 2.2 倍。
        Assert.Equal(_config.ImageWidth, result.SourceSizePx.Width);
        Assert.Equal(_config.ImageHeight, result.SourceSizePx.Height);
        Assert.Equal(_config.MaxWorkingBase,
            Math.Min(result.ProcessedSizePx.Width, result.ProcessedSizePx.Height));
        Assert.Equal(image.Width / (double)result.ProcessedSizePx.Width, result.SourceScale, 6);

        // 换算关系成立且落在原图范围内
        Assert.Equal(result.WeldCenterPx.X * result.SourceScale, result.WeldCenterSourcePx.X, 6);
        Assert.Equal(result.WeldCenterPx.Y * result.SourceScale, result.WeldCenterSourcePx.Y, 6);
        Assert.InRange(result.WeldCenterSourcePx.X, 0, image.Width);
        Assert.InRange(result.WeldCenterSourcePx.Y, 0, image.Height);

        _output.WriteLine($"合成图 {image.Width}×{image.Height} → 工作图 {result.ProcessedSizePx.Width}×{result.ProcessedSizePx.Height}，" +
                          $"缩放比 {result.SourceScale:F3}，焊缝中心(原图)=({result.WeldCenterSourcePx.X:F1}, {result.WeldCenterSourcePx.Y:F1})");
    }

    [Fact]
    public void FovAspectDeviation_QuantifiesTheAxisSkewItReplaced()
    {
        // 配置 1280×1024（宽高比 1.25）跑 5472×3648（1.50）的图：差 20%，
        // 与旧版分轴折算的两轴系数之比 2.226/1.855 = 1.20 完全一致
        Assert.Equal(0.20, _config.FovAspectDeviation(5472, 3648), 3);

        // 宽高比一致（只是分辨率不同）→ 0，此时折算模型是自洽的
        Assert.Equal(0.0, _config.FovAspectDeviation(2560, 2048), 6);

        // 退化输入不应抛异常
        Assert.Equal(0.0, _config.FovAspectDeviation(0, 0), 6);
    }

    // -----------------------------------------------------------------------------------

    /// <summary>取 tests/pp 下的 jpg 样本（按路径排序，保证可复现）；找不到目录时返回空。</summary>
    private static List<string> FindPpFrames(int limit)
    {
        var dir = new DirectoryInfo(AppContext.BaseDirectory);
        while (dir is not null)
        {
            var candidate = Path.Combine(dir.FullName, "tests", "pp");
            if (Directory.Exists(candidate))
            {
                return Directory.EnumerateFiles(candidate, "*.jpg", SearchOption.AllDirectories)
                    .OrderBy(f => f, StringComparer.Ordinal)
                    .Take(limit)
                    .ToList();
            }
            dir = dir.Parent;
        }
        return new List<string>();
    }
}
