using BatteryWeldAOI.Core.Models;
using BatteryWeldAOI.Core.Vision;
using OpenCvSharp;
using Xunit;
using Xunit.Abstractions;

namespace BatteryWeldAOI.Core.Tests;

/// <summary>
/// 熔核**内沿**与**环宽**的回归测试。
///
/// 为什么需要它：熔核是"有厚度的环"，只给外边界表达不出环宽；而掩码里的内沿已被闭运算封死
/// （实测后处理掩码是实心盘，"最大内切圆/等效半径"=0.97），内沿只能在纹理能量图上按
/// "粗糙起始半径"取（见 <c>WeldInnerLocator</c>）。
///
/// 实测基线（tests/pp 前 10 帧，工作分辨率）：内/外半径比 0.228~0.428（中位 0.347）、
/// 环宽中位 344px（外半径 526px，约占 65%）、内沿拟合残差中位 1.75px、
/// **内外圆心偏差中位 1.3px**——两条边确实同心，与"熔核内外沿同心于激光中心"的物理事实一致。
/// 而含反射细圈残留的帧该偏差达 48.8px，故它同时是污染指示器。
/// </summary>
public class RingWidthTests
{
    private readonly ITestOutputHelper _output;
    private readonly InspectionConfig _config = new();
    private readonly WeldInspectionAlgorithm _algorithm;

    public RingWidthTests(ITestOutputHelper output)
    {
        _output = output;
        _algorithm = new WeldInspectionAlgorithm(_config, new NinePointCalibration(
            new List<(Point2d, Point2d)>
            {
                (new Point2d(0, 0), new Point2d(0, 0)),
                (new Point2d(1000, 0), new Point2d(25, 0)),
                (new Point2d(0, 1000), new Point2d(0, 25)),
            }));
    }

    /// <summary>
    /// 现实图上的不变式：只要取到内沿，就必须满足 0 &lt; 内半径 &lt; 外半径、环宽 = 外 − 内、
    /// 且内/外比值落在实测族值范围内。
    /// </summary>
    [Fact]
    public void InnerRadius_IsInsideOuterRadius_AndWidthIsConsistent()
    {
        var files = FindPpFrames(limit: 25);
        if (files.Count == 0)
        {
            _output.WriteLine("未找到 tests/pp 样本，跳过。");
            return;
        }

        var withInner = 0;
        var ratios = new List<double>();
        foreach (var file in files)
        {
            using var image = Cv2.ImRead(file, ImreadModes.Color);
            Assert.False(image.Empty());

            var result = _algorithm.Inspect(image, new WeldPoint { Index = 0, Name = "t" });
            if (result.WeldRadiusPx <= 0 || result.WeldInnerRadiusPx <= 0)
                continue;

            withInner++;
            Assert.True(result.WeldInnerRadiusPx < result.WeldRadiusPx,
                $"{Path.GetFileName(file)}: 内半径 {result.WeldInnerRadiusPx:F1} 不小于外半径 {result.WeldRadiusPx:F1}");
            Assert.True(result.WeldInnerRadiusPx > 0);

            // 环宽恒等式（外 − 内），以及原图坐标口径的一致性
            Assert.Equal(result.WeldRadiusPx - result.WeldInnerRadiusPx, result.WeldRingWidthPx, 6);
            Assert.Equal(result.WeldRingWidthPx * result.SourceScale, result.WeldRingWidthSourcePx, 6);
            Assert.Equal(result.WeldInnerRadiusPx * result.SourceScale, result.WeldInnerRadiusSourcePx, 6);

            // 实测内/外 0.228~0.428；放宽到 [0.15, 0.60] 作为回归护栏
            var ratio = result.WeldInnerRadiusPx / result.WeldRadiusPx;
            ratios.Add(ratio);
            Assert.InRange(ratio, 0.15, 0.60);

            // 内沿圆心落在原图内
            Assert.InRange(result.WeldInnerCenterPx.X * result.SourceScale, 0, image.Width);
            Assert.InRange(result.WeldInnerCenterPx.Y * result.SourceScale, 0, image.Height);
        }

        // 若一帧都取不到内沿，说明该功能已静默失效（阈值/起点参数被改坏）
        Assert.True(withInner > 0,
            $"检查了 {files.Count} 帧，一帧都没取到熔核内沿——该功能可能已失效");
        _output.WriteLine($"{withInner}/{files.Count} 帧取到熔核内沿；内/外半径比 " +
                          $"{ratios.Min():F3}~{ratios.Max():F3}");
    }

    /// <summary>
    /// 内外圆心偏差过大时不得输出环宽（含反射细圈残留的帧实测偏差达 48.8px，会把环宽算歪），
    /// 但也**不判测量失败**——环宽只作诊断量，不参与判定。
    /// </summary>
    [Fact]
    public void InnerRadius_IsOmitted_WhenNotConcentric()
    {
        var files = FindPpFrames(limit: 25);
        if (files.Count == 0)
        {
            _output.WriteLine("未找到 tests/pp 样本，跳过。");
            return;
        }

        foreach (var file in files)
        {
            using var image = Cv2.ImRead(file, ImreadModes.Color);
            var result = _algorithm.Inspect(image, new WeldPoint { Index = 0, Name = "t" });
            if (result.WeldRadiusPx <= 0 || result.WeldInnerRadiusPx <= 0)
                continue;

            // 输出的内沿必须满足同心门限
            var deviation = Math.Sqrt(
                Math.Pow(result.WeldInnerCenterPx.X - result.WeldCenterPx.X, 2) +
                Math.Pow(result.WeldInnerCenterPx.Y - result.WeldCenterPx.Y, 2));
            Assert.True(deviation <= _config.WeldInnerMaxConcentricDeviationRatio * result.WeldRadiusPx + 1e-6,
                $"{Path.GetFileName(file)}: 内沿同心偏差 {deviation:F1}px 超过门限，" +
                $"却仍输出了环宽 {result.WeldRingWidthPx:F1}px");

            // 环宽是诊断量：它的有无不应改变判定
            Assert.False(result.HasError, $"{Path.GetFileName(file)}: 测量失败不应与内沿有关");
        }
    }

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
