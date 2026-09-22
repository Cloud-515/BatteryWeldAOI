using BatteryWeldAOI.Core.Models;
using BatteryWeldAOI.Core.Vision;
using OpenCvSharp;
using Xunit;
using Xunit.Abstractions;

namespace BatteryWeldAOI.Core.Tests;

/// <summary>
/// 现场样本批量回归（**按需运行**）：tests/pp 下每个 SN 目录的 24 帧都是**厂商判良品**，
/// 因此任何非 OK 都是误判。本测试按目录批量跑算法，统计误判率并列出误判帧及其判定依据，
/// 用来回答"改了判据/加了兜底之后，误判是变多还是变少了"——这是单点测试给不出的横向证据。
///
/// 运行方式（默认跳过，避免拖慢常规回归）：
///   set BWAOI_PP_BATCH=40        扫前 40 个 SN 目录（约 960 帧，2~4 分钟）
///   set BWAOI_PP_MAX_FP=0.06     良品被判缺陷的比例上限（默认 6%，见下）
///
/// 两条断言分工不同，别把它们混为一谈：
///   · **偏移量中位数必须落在 1.70~2.30mm**（紧，且不依赖公差标定）——它抓的是**测量**退化：
///     孔位基准退回梯度卡尺会把 D 中位压低（旧当量下实测由 0.48 掉到 0.26）；
///   · **良品被判缺陷的比例 ≤ 6%**（松）——它取决于 `OffsetToleranceMm`（现为 6.6mm，
///     由本参考集推出，见该配置项），20 SN 参考集的期望值是 **0**。
///
/// 历史（这批样本全部是厂商判良品，因此任何 NG 都要解释清楚）：
///   · 2026-09-17 之前：NG 0 帧、测量失败 1 帧。当时的"零 NG"是假象——孔位基准（`HoleLocator`
///     的残差闸门）把 8 成真孔位圆盘判掉、退回梯度卡尺，量到的是熔核内沿，偏移量被系统性
///     压低到公差以内（同一批 12 帧厂商判 NG 的样本只检出 2 帧，见 NgSampleTests）。
///   · 修正孔位基准后：NG 27 帧（2.81%）、测量失败 0。
///   · 再加孔位外径补全（中间层只覆盖孔位端面被照亮的那一半，用与端面金属的交界弧补出真外径）：
///     NG 38 帧（**3.96%**）、测量失败 0；偏移量中位 0.476mm、p90 0.934mm（**旧当量 0.025**）。
///   · **2026-09-17 修正当量（0.025→0.1003，即 0.0235mm/原图像素）后**：同一批 20 SN / 480 帧
///     实测偏移量中位 1.989mm、p90 3.780、max 5.763——即上面那 38 帧"按 1.0mm 越线"的读数
///     本身是偏小 4 倍的（真值 1.5~4mm），现在按由本集推出的 6.6mm 公差判，**NG 0 帧**。
/// 测量失败必须为 0：这批帧的成像条件都足以测出结果，"测不出来"出现在这里就是定位链退化的信号。
/// </summary>
public class PpBatchRegressionTests
{
    private readonly ITestOutputHelper _output;

    public PpBatchRegressionTests(ITestOutputHelper output) => _output = output;

    [Fact]
    public void VendorOkFrames_AreNotReportedAsDefects()
    {
        var root = FindSampleRoot();
        if (root is null)
        {
            _output.WriteLine("未找到 tests/pp 样本目录，跳过。");
            return;
        }

        var limit = ReadInt("BWAOI_PP_BATCH", 0);
        if (limit <= 0)
        {
            _output.WriteLine("未设置 BWAOI_PP_BATCH，跳过批量回归（按需运行，见类注释）。");
            return;
        }

        var maxFalsePositiveRatio = ReadDouble("BWAOI_PP_MAX_FP", 0.06);

        var config = new InspectionConfig();
        var algorithm = new WeldInspectionAlgorithm(config, NinePointCalibration.PureScale(config.MmPerPixel));

        var directories = Directory.GetDirectories(root)
            .OrderBy(d => Path.GetFileName(d), StringComparer.Ordinal)
            .Take(limit)
            .ToList();

        int frames = 0, ok = 0, defects = 0, unmeasurable = 0;
        var problems = new List<string>();
        var offsets = new List<double>();

        foreach (var directory in directories)
        {
            var files = Directory.GetFiles(directory, "*.jpg")
                .OrderBy(f => Path.GetFileName(f), StringComparer.Ordinal)
                .ToList();

            foreach (var file in files)
            {
                var name = Path.GetFileName(file);
                var sceneAt = name.IndexOf("_Scene", StringComparison.Ordinal);
                var tag = sceneAt > 0 ? name[..sceneAt] : name;

                using var image = Cv2.ImRead(file, ImreadModes.Color);
                if (image.Empty())
                {
                    problems.Add($"{tag} 读取失败");
                    continue;
                }

                frames++;
                var result = algorithm.Inspect(image, new WeldPoint { Index = 0, Name = tag });
                if (result.HasValidOffset)
                    offsets.Add(result.OffsetDistanceMm);
                if (result.IsOk)
                {
                    ok++;
                }
                else if (result.HasError)
                {
                    unmeasurable++;
                    if (problems.Count < 60)
                        problems.Add($"{Path.GetFileName(directory)[3..]} / {tag}: 测量失败 — {result.ErrorMessage}");
                }
                else
                {
                    defects++;
                    problems.Add($"{Path.GetFileName(directory)[3..]} / {tag}: 误判 NG — {result.DefectTextZh}");
                }
            }
        }

        var fpRatio = frames == 0 ? 0 : defects / (double)frames;
        _output.WriteLine($"扫描 {directories.Count} 个 SN 目录 / {frames} 帧：" +
                          $"合格 {ok}（{ok / (double)Math.Max(1, frames):P1}）、" +
                          $"误判 NG {defects}（{fpRatio:P2}）、测量失败 {unmeasurable}（{unmeasurable / (double)Math.Max(1, frames):P1}）");
        if (problems.Count > 0)
            _output.WriteLine("异常帧清单:\n  " + string.Join("\n  ", problems));

        // ---- 断言一（紧）：偏移量的量级必须与"孔位基准已修正 + 当量已修正"的实测一致 ----
        // 这是本套样本里**最能抓住孔位基准退化**的量，而且它不依赖公差标定（D 是测量值）：
        // 孔位基准退回梯度卡尺时（射线原点是焊环圆心，量到熔核内沿），偏移量被系统性压低。
        // 实测（旧当量 0.025）：修正前 0.262mm、修正后 0.476mm；
        // 换算到修正后的当量 0.1003（×4.012）：**中位 1.989mm**（20 SN / 480 帧全量实测）。
        // 区间 [1.70, 2.30] 两侧都留了余量，退化到卡尺会让它掉到 ~1.05（0.262 × 4.012）。
        Assert.True(offsets.Count > 0, "没有任何一帧给出有效偏移量");
        offsets.Sort();
        var medianOffset = offsets[offsets.Count / 2];
        Assert.InRange(medianOffset, 1.70, 2.30);

        // ---- 断言二（松）：良品被判缺陷的比例 ----
        // 上限比实测松，只用于拦住"某次改动让大批帧变 NG"（例如归一化失效时的 24/24 全 NG）。
        // 期望值是 0：`OffsetToleranceMm` = 6.6mm 就是由本参考集推出的（最大值 × 1.15）。
        // 若这里出现成片的 NG，说明公差与当量又被改脱钩了，先去看那两个配置项的注释再动阈值。
        Assert.True(fpRatio <= maxFalsePositiveRatio,
            $"良品被误判为缺陷的比例 {fpRatio:P2} 超过上限 {maxFalsePositiveRatio:P2}（{defects}/{frames}）:\n  " +
            string.Join("\n  ", problems.Where(p => p.Contains("误判 NG")).Take(30)));

        // 测量失败必须为 0：这批帧的成像条件都足以测出结果，"测不出来"出现在这里就是定位链退化的信号
        // （2026-09-17 修正前有 1 帧，原因是孔位基准把孔半径量成真值的两倍后触发了"孔洞半径异常"）。
        Assert.True(unmeasurable == 0,
            $"{unmeasurable} 帧测量失败（要求 0）:\n  " +
            string.Join("\n  ", problems.Where(p => p.Contains("测量失败")).Take(30)));
    }

    /// <summary>从测试输出目录向上查找 tests/pp/2025-09-11/Orignal_pic（与 PpSampleTests 同口径）。</summary>
    private static string? FindSampleRoot()
    {
        var dir = new DirectoryInfo(AppContext.BaseDirectory);
        while (dir is not null)
        {
            var candidate = Path.Combine(dir.FullName, "tests", "pp", "2025-09-11", "Orignal_pic");
            if (Directory.Exists(candidate))
                return candidate;
            dir = dir.Parent;
        }
        return null;
    }

    private static int ReadInt(string name, int fallback) =>
        int.TryParse(Environment.GetEnvironmentVariable(name), out var v) ? v : fallback;

    private static double ReadDouble(string name, double fallback) =>
        double.TryParse(Environment.GetEnvironmentVariable(name), out var v) ? v : fallback;
}
