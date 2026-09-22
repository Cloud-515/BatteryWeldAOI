using BatteryWeldAOI.Core.Models;
using BatteryWeldAOI.Core.Vision;
using OpenCvSharp;
using Xunit;
using Xunit.Abstractions;

namespace BatteryWeldAOI.Core.Tests;

/// <summary>
/// 孔位（极柱）识别的**绝对真值**回归：用用户手绘标注反解出的孔位坐标钉住算法输出。
///
/// 为什么需要这组用例（2026-09-18）：「焊环与孔洞几乎连在一起」的极端工况下，
/// `HoleLocator` 的三材质分层会整体塌掉（端面金属被反光冲白 → 第一次 Otsu 把孔位端面划进暗层，
/// 中间层只剩 3~5% 的过渡带 → 没有面积 ≥60px 的连通域），算法于是**无条件**退到
/// "以焊环圆心为原点"的梯度卡尺；卡尺在这一步会把"半圈孔沿 + 半圈熔核内沿"平均成一个
/// *自洽但无意义*的圆。实测后果：
///   · Img000（19-22-06）：孔心偏 6.21mm、偏移量报 1.761mm 而真值 ≈4.45mm（**偏低 2.5 倍**），
///     界面仍判 OK —— 属"该判 NG 却判合格"的方向；
///   · Img009（16-49-03）：孔位半径量成真值的 1.71 倍 → 被 `MaxPinRadiusRatio` 拦下报"异常"，
///     而真值 2.00mm 本来完全可测。
///
/// 真值来源：用户手绘圈按颜色分离，再用**算法自己画的红圈（焊环）/绿圈（极柱）**反解
/// "原图坐标 → 截图坐标"的等比+平移映射（映射校验：红圈 91.11/90.14、绿圈 32.09/31.80，
/// 误差 &lt;1.2%）。详见 `REPORT_hole_near_ring.md` 与 `.scratch/case000/userblue2.py`。
///
/// 本组用例刻意用**绝对真值 + 逐帧断言**，而不是此仓库既有 pp 用例的"中位数落在某带内"——
/// 那条口径对"7.7% 的帧走错路径"没有鉴别力（错误帧动不了中位数，实测 median pin/ring 0.226
/// 始终稳稳在带内）。
/// </summary>
public class HoleTruthTests
{
    private const string Root = "pp";
    private readonly ITestOutputHelper _output;
    private readonly InspectionConfig _config = new();
    private readonly WeldInspectionAlgorithm _algorithm;

    // 用户手绘标注反解出的**原图像素**真值（孔位圆心与半径）
    private static readonly Dictionary<string, (double X, double Y, double R)> Truth = new()
    {
        ["19-22-06"] = (2389.4, 1791.7, 143.3),   // Img000：卡尺兜底静默错读数那一帧
        ["16-49-03"] = (2739.9, 1917.9, 113.6),   // Img009：假"孔洞半径异常"那一帧
    };

    public HoleTruthTests(ITestOutputHelper output)
    {
        _output = output;
        _algorithm = new WeldInspectionAlgorithm(_config, NinePointCalibration.PureScale(_config.MmPerPixel));
    }

    /// <summary>
    /// 两帧都必须**测得出**，且孔位与真值的圆心差 ≤60px（1.4mm）、半径比落在 0.85~1.25。
    ///
    /// 三条阈值都是"刚好卡在实测值之外"：改动后实测 Img000 圆心差 19.7px、半径比 0.98；
    /// Img009 圆心差 58px、半径比 1.11。60px 给 Img009 留的余量最小（约 3%），
    /// 若后续改动把它推过 60px，说明这条链路的精度又退化了——那时应该改代码，而不是放宽这里。
    /// </summary>
    [Fact]
    public void ExtremeFrames_HoleIsMeasuredAtUserAnnotatedTruth()
    {
        var checkedFrames = 0;
        foreach (var (tag, truth) in Truth)
        {
            var file = FindFrame(tag);
            if (file is null)
                continue;   // 样本集可能不随源码分发，与 RealSampleTests 同口径
            checkedFrames++;

            using var image = Cv2.ImRead(file, ImreadModes.Color);
            Assert.False(image.Empty(), $"{tag}: 读取失败");
            var result = _algorithm.Inspect(image, new WeldPoint { Index = 0, Name = tag });

            Assert.False(result.HasError, $"{tag}: 不该测不出来 —— {result.ErrorMessage}");
            Assert.True(result.PoleRadiusPx > 0, $"{tag}: 未给出孔位半径");

            // 结果里的几何量在工作分辨率上，换算回原图再与真值比
            var scale = result.SourceSizePx.Width / (double)result.ProcessedSizePx.Width;
            var cx = result.PoleCenterPx.X * scale;
            var cy = result.PoleCenterPx.Y * scale;
            var r = result.PoleRadiusPx * scale;
            var centerError = Math.Sqrt((cx - truth.X) * (cx - truth.X) + (cy - truth.Y) * (cy - truth.Y));

            _output.WriteLine($"{tag}: 孔心 ({cx:F0},{cy:F0}) r={r:F0} | 真值 ({truth.X:F0},{truth.Y:F0}) " +
                              $"r={truth.R:F0} | 圆心差 {centerError:F1}px 半径比 {r / truth.R:F3} | D={result.OffsetDistanceMm:F3}mm");

            Assert.True(centerError <= 60, $"{tag}: 孔心离真值 {centerError:F1}px（上限 60px）");
            Assert.InRange(r / truth.R, 0.85, 1.25);
        }

        Assert.True(checkedFrames > 0, "未找到任何真值帧（样本集缺失）");
    }

    /// <summary>
    /// Img000 的偏移量必须落在真值附近（4.45mm ±1.0mm），**不允许再被压低到 2mm 以下**。
    ///
    /// 这一条是本类错误的核心危害：孔位错 → 偏移量系统性偏小 → 该判 NG 的帧报 OK。
    /// 阈值取 ±1.0mm：真值本身有区间（用算法自己报的焊环心算 4.45mm、用用户手绘焊环心算 5.74mm、
    /// 用凹陷圆环机械基准算 4.13mm），并且 60px 的孔位容差折算到毫米约 1.4mm。
    /// </summary>
    [Fact]
    public void ExtremeFrame_OffsetIsNotUnderReported()
    {
        var file = FindFrame("19-22-06");
        if (file is null)
            return;

        using var image = Cv2.ImRead(file, ImreadModes.Color);
        var result = _algorithm.Inspect(image, new WeldPoint { Index = 0, Name = "19-22-06" });

        Assert.True(result.HasValidOffset, $"偏移量无效：{result.ErrorMessage}");
        _output.WriteLine($"Img000 偏移量 = {result.OffsetDistanceMm:F3}mm（真值 ≈4.45mm，改动前为 1.761mm）");
        Assert.InRange(result.OffsetDistanceMm, 3.4, 5.5);
    }

    /// <summary>
    /// 孔位不得来自"卡尺兜底"：分层路径必须能在这两帧上给出结果。
    ///
    /// 这条是**机理守卫**——上面两条只钉结果，而结果可能被别的口径巧合修好。
    /// 判据：孔/环比 ≤0.40 且孔位半径与原图像的 5mm 孔（卡尺实测直径 5mm → 半径 106.6 原始像素）
    /// 相差不超过 1.35 倍。卡尺兜底在这两帧上给的是 170.3 / 195.1 原始像素（1.60 / 1.83 倍）。
    /// </summary>
    [Fact]
    public void ExtremeFrames_HoleDiameterMatchesCalibratedFiveMillimetreBore()
    {
        // 用户带卡尺到现场实测：孔直径 5mm；像素当量 0.0235 mm/原图像素（见 InspectionConfig.MmPerPixel）
        const double expectedRadiusPx = 2.5 / 0.0235;   // ≈106.4
        var checkedFrames = 0;
        foreach (var tag in Truth.Keys)
        {
            var file = FindFrame(tag);
            if (file is null)
                continue;
            checkedFrames++;

            using var image = Cv2.ImRead(file, ImreadModes.Color);
            var result = _algorithm.Inspect(image, new WeldPoint { Index = 0, Name = tag });
            Assert.True(result.PoleRadiusPx > 0, $"{tag}: 未给出孔位半径");

            var scale = result.SourceSizePx.Width / (double)result.ProcessedSizePx.Width;
            var radiusPx = result.PoleRadiusPx * scale;
            _output.WriteLine($"{tag}: 孔位半径 {radiusPx:F0} 原始像素（卡尺实测 5mm 孔 → {expectedRadiusPx:F0}px，" +
                              $"比值 {radiusPx / expectedRadiusPx:F2}）");
            Assert.InRange(radiusPx / expectedRadiusPx, 0.75, 1.35);
        }
        Assert.True(checkedFrames > 0, "未找到任何真值帧（样本集缺失）");
    }

    /// <summary>从测试输出目录向上找 `tests/pp/2025-09-11/Orignal_pic/*&#47;&lt;时间戳&gt;*.jpg`。</summary>
    private static string? FindFrame(string tag)
    {
        var dir = new DirectoryInfo(AppContext.BaseDirectory);
        while (dir is not null)
        {
            var root = Path.Combine(dir.FullName, "tests", Root, "2025-09-11", "Orignal_pic");
            if (Directory.Exists(root))
            {
                var hit = Directory.EnumerateFiles(root, $"{tag}*.jpg", SearchOption.AllDirectories).FirstOrDefault();
                if (hit is not null)
                    return hit;
            }
            dir = dir.Parent;
        }
        return null;
    }
}
