using BatteryWeldAOI.Core.Diagnostics;
using BatteryWeldAOI.Core.Models;
using BatteryWeldAOI.Core.Vision;
using OpenCvSharp;
using Xunit;
using Xunit.Abstractions;

namespace BatteryWeldAOI.Core.Tests;

/// <summary>
/// 焊环定位精度回归：把本项目测出的焊环中心与**厂商 VDB 里的圆定位结果**逐帧对齐比较。
///
/// 为什么需要这条测试：在此之前，"定位准不准"只能靠间接指标判断——
/// PpSampleTests 要求 24/24 全合格、PpBatchRegressionTests 要求误判率 ≤2%，
/// 两者量的都是**判定结果**，定位偏差多少完全看不见。厂商 VDB 提供了 20955 帧覆盖、
/// 零人工标注的定位圆心（见 <see cref="VendorVdbReader"/>），于是定位精度第一次变成可量化指标。
///
/// 实测基线（2025-09-11 批次前 20 个 SN / 478 帧参与比较）：
///   · 偏差中位 11.1px、均值 23.6、p90 67.4、p99 171.4、最大 215.3（焊环半径约 528px，中位偏差约 2.1% 半径）
///   · 我们未定位到焊环 2/480 = 0.42%
///
/// 参照系：厂商自己在**同一张图**上换三个位置标记（Pos0/1/2）得到的圆心互差，中位就有 24.4px
/// （20955 帧统计）；也就是说本测试的"与厂商一致性"里天然含约 10~20px 的基准噪声。
///
/// **2026-09-17 判据重构（第一次）。** 起因：用户报"红圈框选偏移，而且同一个工件在不同帧里
/// 偏的方向还不一样"。当时把厂商一致性降级为"崩溃守卫"，改盯**同一工件跨帧的自洽性**
/// （刚体工件平移过视野时"焊环心 − 孔心"必须不变）。
///
/// **2026-09-18 判据再重构（第二次，据实记录一次被指标误导的过程）。**
/// 上面那条"跨帧自洽"判据**奖励了一个错误的修法**：那一轮把精修的射线原点设成**孔心**，
/// 于是估计器把焊环圆心钉在孔心上，跨帧离散度立刻从 269px 掉到 25px——指标变好看了，
/// 但变好的是"测量值被压成了常数"，而不是"测量变准了"。用户随后用手绘蓝圈指出红圈仍偏
/// （实测圆心差 34px、半径 +31px）。事后核对：
///   · 以孔心为原点等于假设"熔核与极柱同心"，而**"焊偏"正是要检验这个前提**；
///   · 同时搜索带太窄（±15%×族值中位）且内点容差被 2.0 工作像素的下限顶到与带半宽同量级，
///     那道可信度闸门形同虚设（实测支撑弧段恒在 0.84~0.99）。
/// 所以本文件现在**不再用跨帧一致性当精度指标**（它对"把测量压成常数"没有鉴别力），
/// 改用**与厂商圆心的一致性**——这一轮它才第一次成为有效指标，因为已经用第三条独立证据
/// （对"粗糙区外边界"做 RANSAC 圆拟合，19 帧可比）确认过：厂商的圆就是熔核外沿，
/// 两者逐帧只差中位 4px。三方交叉核对的结果与本算法逐帧差中位 2.7px。
///
/// 实测（2025-09-11 前 20 个 SN / 480 帧，与厂商 `.vdb` 逐帧比较）：
///   · 圆心差 中位 **2.7px**、p90 **7.0px**、最大 70.5px（焊环半径约 485px，中位差 0.6% 半径）；
///   · 对照：修复前（孔锚精修）中位 73px、中间版（自锚 + 带内 argmax）中位 10.8px。
///
/// 默认跑前 8 个 SN 目录（约 30s，保证常规回归能覆盖到）；用环境变量放大或跳过：
///   set BWAOI_PP_AGREE=40     跑前 40 个 SN 目录（约 2~3 分钟，横向证据更足）
///   set BWAOI_PP_AGREE=0      跳过本测试（只在需要极快回归时用）
///
/// 样本集可能不随源码分发；找不到目录时跳过而非失败（与 RealSampleTests 同口径）。
/// </summary>
public class PpLocatorAgreementTests
{
    /// <summary>与厂商圆心的偏差**中位数**上限（像素）。实测 2.7px，取 15px 留 5 倍余量。
    /// 它拦的是"整条链路退化"（归一化失效、孔位基准确失、精修搬到别的结构上）。</summary>
    private const double MaxMedianDeltaPx = 15.0;

    /// <summary>与厂商圆心的偏差 **p90** 上限（像素）。实测 7.0px，取 30px 留 4 倍余量。
    /// 只看中位数会漏掉"多数帧好、少数帧崩"的形态。</summary>
    private const double MaxP90DeltaPx = 30.0;

    /// <summary>定位失败率上限。实测 3.1%，留约 1.6 倍余量——它拦的是"整批定位崩掉"，不是正常波动。</summary>
    private const double MaxLocateFailureRatio = 0.05;

    private readonly ITestOutputHelper _output;

    public PpLocatorAgreementTests(ITestOutputHelper output) => _output = output;

    [Fact]
    public void OurRingCenter_AgreesWithVendorLocateCenter()
    {
        var sampleCount = ReadInt("BWAOI_PP_AGREE", 8);
        if (sampleCount <= 0)
        {
            _output.WriteLine("BWAOI_PP_AGREE=0，跳过。");
            return;
        }

        var pairs = FindSamplePairs(sampleCount);
        if (pairs.Count == 0)
        {
            _output.WriteLine("未找到 tests/pp 下的 vdb/jpg 配对样本，跳过。");
            return;
        }

        var config = new InspectionConfig();
        var algorithm = new WeldInspectionAlgorithm(config, new NinePointCalibration(new List<(Point2d, Point2d)>
        {
            (new Point2d(0, 0), new Point2d(0, 0)),
            (new Point2d(1000, 0), new Point2d(25, 0)),
            (new Point2d(0, 1000), new Point2d(0, 25)),
        }));

        var deltas = new List<double>();
        var radii = new List<double>();
        var failures = new List<string>();
        var worst = new List<(double Delta, string Name)>();

        foreach (var (vdb, jpg) in pairs)
        {
            Assert.True(VendorVdbReader.TryRead(vdb, out var vendor, out var parseError),
                $"{Path.GetFileName(vdb)}: {parseError}");

            using var image = Cv2.ImRead(jpg, ImreadModes.Color);
            Assert.False(image.Empty(), $"{Path.GetFileName(jpg)}: 图像读取失败");

            var result = algorithm.Inspect(image,
                new WeldPoint { Index = 0, Name = Path.GetFileNameWithoutExtension(jpg) });
            var agreement = VendorLocatorAgreement.Compare(result, vendor!);

            if (agreement is not { } a)
            {
                failures.Add($"{Path.GetFileName(jpg)}: 未定位到焊环 — {result.ErrorMessage ?? result.DefectText}");
                continue;
            }

            deltas.Add(a.DeltaPx);
            radii.Add(a.OurRadiusPx);
            worst.Add((a.DeltaPx, Path.GetFileName(jpg)));
        }

        Assert.True(deltas.Count > 0, "没有一帧参与比较：" + string.Join("; ", failures));

        var median = Percentile(deltas, 50);
        var p90 = Percentile(deltas, 90);
        var medianRadius = Percentile(radii, 50);
        _output.WriteLine($"参与比较 {deltas.Count}/{pairs.Count} 帧；未定位 {failures.Count}（{failures.Count / (double)pairs.Count:P2}）");
        _output.WriteLine($"与厂商圆心偏差 中位 {median:F1}  均值 {deltas.Average():F1}  p90 {p90:F1}  " +
                          $"最大 {deltas.Max():F1}  （本算法焊环半径中位 {medianRadius:F0}px，中位偏差 {median / medianRadius:P1} 半径）");
        _output.WriteLine("参考：厂商自身在同一图上的三个位置标记互差中位 24.4px；" +
                          "本轮已用独立纹理边界拟合确认厂商圆即熔核外沿（逐帧差中位 4px）");
        foreach (var (delta, name) in worst.OrderByDescending(w => w.Delta).Take(5))
            _output.WriteLine($"  最差 {delta,7:F1}px  {name}");

        // ---- 守卫一：与厂商圆心的中位偏差（精度指标，见类注释）----
        Assert.True(median <= MaxMedianDeltaPx,
            $"焊环中心与厂商的偏差中位数 {median:F1}px 超过上限 {MaxMedianDeltaPx}px（参与比较 {deltas.Count} 帧）");
        // ---- 守卫二：p90（抓"多数帧好、少数帧崩"）----
        Assert.True(p90 <= MaxP90DeltaPx,
            $"焊环中心与厂商的偏差 p90 {p90:F1}px 超过上限 {MaxP90DeltaPx}px");

        var failureRatio = failures.Count / (double)pairs.Count;
        Assert.True(failureRatio <= MaxLocateFailureRatio,
            $"定位失败率 {failureRatio:P2} 超过上限 {MaxLocateFailureRatio:P0}（{failures.Count}/{pairs.Count}）:\n  "
            + string.Join("\n  ", failures.Take(20)));
    }

    /// <summary>
    /// 取前 <paramref name="snLimit"/> 个 SN 目录里的 vdb/jpg 配对。
    /// 只有 Pos0 存在导出的 jpg；Pos1/Pos2 的 vdb 内嵌的是同一张图，不提供额外图像数据。
    /// </summary>
    private static List<(string Vdb, string Jpg)> FindSamplePairs(int snLimit)
    {
        var root = FindPpRoot();
        if (root is null)
            return new List<(string, string)>();

        return Directory.EnumerateFiles(root, "*.vdb", SearchOption.AllDirectories)
            .Where(f => f.Contains("VdbGui_pic", StringComparison.OrdinalIgnoreCase))
            .Where(f => Path.GetFileName(f).Contains("Pos0", StringComparison.Ordinal))
            .GroupBy(f => Path.GetDirectoryName(f)!, StringComparer.Ordinal)
            .OrderBy(g => g.Key, StringComparer.Ordinal)
            .Take(snLimit)
            .SelectMany(g => g.OrderBy(f => f, StringComparer.Ordinal))
            .Select(vdb => (vdb, Path.ChangeExtension(
                vdb.Replace("VdbGui_pic", "Orignal_pic", StringComparison.OrdinalIgnoreCase), ".jpg")))
            .Where(p => File.Exists(p.Item2))
            .ToList();
    }

    /// <summary>线性插值分位数（与 Core 的 Geo.Percentile 同口径）。</summary>
    private static double Percentile(List<double> values, double pct)
    {
        var sorted = values.OrderBy(v => v).ToList();
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

    private static int ReadInt(string name, int fallback) =>
        int.TryParse(Environment.GetEnvironmentVariable(name), out var v) ? v : fallback;

    private static string? FindPpRoot()
    {
        var dir = new DirectoryInfo(AppContext.BaseDirectory);
        while (dir is not null)
        {
            var candidate = Path.Combine(dir.FullName, "tests", "pp");
            if (Directory.Exists(candidate))
                return candidate;
            dir = dir.Parent;
        }
        return null;
    }
}
