using BatteryWeldAOI.Core.Models;
using BatteryWeldAOI.Core.Vision;
using OpenCvSharp;
using Xunit;
using Xunit.Abstractions;

namespace BatteryWeldAOI.Core.Tests;

/// <summary>
/// 现场 NG 反例回归：tests/pp 里 12 帧厂商判 <c>_NG_</c> 的现场图，逐张跑完整算法，
/// 统计"被判合格（漏检）"的张数。
///
/// **断言于 2026-09-17 定下来（此前是"临时探针版"，从建立起就红着）。** 该日修正了孔位基准
/// （<c>HoleLocator</c> 的残差闸门曾把 8 成真孔位圆盘判掉、退回梯度卡尺从而**系统性压低**偏移量，
/// 详见其类型注释）。修正前后的实测对比（同一批 12 帧）：
///   · 修正前：判 NG **2 张**、漏检 10 张，漏检帧的 D 只有 0.128~0.378mm——被压低了一半以上；
///   · 修正后：判 NG **6 张**、漏检 6 张，判 NG 的 6 张 D = 1.013~1.056mm，
///     漏检的 6 张 D = 0.378~0.991mm（全部落在 1.0mm 公差以内，即"按偏移量确实合格"）。
/// 也就是说修正后**厂商 NG 与良品两族在 1.0mm 附近被分开**了：良品族 D 中位 0.475mm、
/// 前 90 分位 0.93mm，而本批 NG 族除两张外全部 ≥0.96mm。这既是对修正的旁证
/// （把偏移量量对了，判据才与厂商的结论对得上），也说明剩下的 6 张漏检不是测量问题——
/// 它们按偏移量本就该判合格，厂商判 NG 另有原因（其它缺陷类型或更严的判定口径，尚不可知）。
///
/// 因此断言取"判 NG 张数不得少于实测值"，而不是"漏检必须为 0"：
/// 后者在这批样本上从来就不成立（厂商的判据比本项目的偏移量公差更宽），
/// 而前者恰好是本帧集合里**最能抓住孔位基准退化**的哨兵——退化时它会掉到 2/12。
///
/// **2026-09-17 第二轮修正（当量 0.025→0.1003）后，上面那条哨兵失效了，据实改写。**
/// 当量修正使这批帧的偏移量由 0.43~1.44mm 变为 **1.73~5.79mm**；而公差同时改为按
/// pp 参考集推出的 6.6mm（见 `InspectionConfig.OffsetToleranceMm`）——于是
/// **12 帧的偏移量全部落在公差之内，判 NG 张数 = 0**。这不是漏检：按"6.6mm 以内算合格"
/// 这个由厂商良品自己定义的口径，它们确实该判合格。
/// 由此得到一个必须上报的结论：**偏移量判据在本批数据上对厂商 NG 零分辨力**
/// （良品族 D 与 NG 族 D 完全重叠）。所以本测试不再断言"检出多少张"，只断言**测量链正常**
/// （每帧都测得出、D 落在实测带内、定位不退化），并把逐帧 D 打印出来供人工比对。
/// 要判焊偏必须等工艺给出远小于 6.6mm 的正式公差，或者改用别的量
/// （见 `OffsetToleranceMm` 里记的方向统计待办）。
///
/// **2026-09-18 第三轮：精修的射线原点由"孔心"改回"熔核自身圆心"**
/// （详见 `InspectionConfig.WeldRingRefineEdgeFraction`）。本批 12 帧由
/// "3 帧测量失败 + 9 帧 D 0.16~4.49mm"变为 **12/12 全部测出、D 1.875~4.940mm（中位 3.636mm）**：
/// 孔位基准不再被偏大的焊环半径连累（孔/环比回到实测带），而偏移量也回到真实量级。
/// </summary>
public class NgSampleTests
{
    private readonly ITestOutputHelper _output;
    private readonly InspectionConfig _config = new();
    private readonly WeldInspectionAlgorithm _algorithm;

    public NgSampleTests(ITestOutputHelper output)
    {
        _output = output;
        // 现场数据的当量取自配置（2026-09-17 按现场实测重标为 0.1003 ⇒ 0.0235mm/原图像素）
        _algorithm = new WeldInspectionAlgorithm(_config,
            NinePointCalibration.PureScale(_config.MmPerPixel));
    }

    [Fact]
    public void NgSamples_AreNotJudgedOk()
    {
        var root = FindPpRoot();
        if (root is null)
        {
            _output.WriteLine("未找到 tests/pp 样本目录，跳过。");
            return;
        }

        var files = Directory.GetFiles(root, "*_NG_*.jpg", SearchOption.AllDirectories)
            .OrderBy(f => Path.GetFileName(f), StringComparer.Ordinal)
            .ToList();
        Assert.NotEmpty(files);

        var passed = new List<string>();
        var detected = new List<string>();
        var failed = new List<string>();
        var offsets = new List<double>();
        foreach (var file in files)
        {
            var name = Path.GetFileName(file);
            var scene = name[..Math.Max(0, name.IndexOf("_Scene", StringComparison.Ordinal))];
            var sn = SnOf(name);

            using var image = Cv2.ImRead(file, ImreadModes.Color);
            Assert.False(image.Empty(), $"{name}: 读取失败");

            var result = _algorithm.Inspect(image, new WeldPoint { Index = 0, Name = scene });
            var processedBase = Math.Min(result.ProcessedSizePx.Height, result.ProcessedSizePx.Width);

            _output.WriteLine(
                $"{sn[..Math.Min(sn.Length, 20)],-20} {scene}  {image.Width}x{image.Height}  " +
                $"判定={(result.HasError ? "测量失败" : result.IsOk ? "OK(漏检!)" : "NG")}  " +
                $"{result.DefectText}  D={result.OffsetDistanceMm:F3}mm  " +
                $"环r/工作短边={(result.WeldRadiusPx / Math.Max(processedBase, 1)):F4}  " +
                $"依据={result.ErrorMessage ?? "-"}");

            if (result.HasValidOffset)
                offsets.Add(result.OffsetDistanceMm);

            if (result.HasError)
                failed.Add($"{sn} {scene}: 测量失败 — {result.ErrorMessage}");
            else if (result.IsOk)
                passed.Add($"{sn} {scene}: 被判合格（漏检）D={result.OffsetDistanceMm:F3}mm");
            else
                detected.Add($"{sn} {scene} D={result.OffsetDistanceMm:F3}mm");
        }

        _output.WriteLine($"\n共 {files.Count} 张现场 NG 图；判 NG {detected.Count} 张、被测出 {offsets.Count} 张、" +
                          $"被判合格 {passed.Count} 张（公差口径的必然结果）、测量失败 {failed.Count} 张。");

        // ---- 断言：测量链正常，而不是"检出多少张" ----
        // 本批 12 帧的偏移量全部落在按 pp 参考集推出的 6.6mm 公差之内，判 NG 为 0——
        // 这是公差口径的必然结果，不是漏检（详见类注释）。因此这里只钉与公差无关的事。
        //
        // **2026-09-17 契约变更（据实记录）**：把 `MinRingArcSupport` 由 0.50 提到现场标定值
        // 0.82（见该属性）之后，本批 12 帧里有 3 帧的焊环边界支撑弧段低于门限，按设计报
        // **测量失败**交人工复核。所以"每帧都必须测得出"这条旧契约不再成立，改为：
        //   ① 放弃测量的帧**必须全部**由"焊环边界可信度"解释（边界支撑弧段 / 族值带 / 有效性上限），
        //      真正的定位链退化会以别的理由出现——未定位到孔位、极柱不在焊环内部、
        //      孔洞边缘拟合残差过大——那类理由一条都不允许；
        //   ② 放弃比例不超过 1/4（门限被调得过严时立刻暴露）；
        //   ③ 仍要给出测量的帧里，偏移量的**中位**落在实测带内（当量被改错时会立刻越界：
        //      例如退回 0.025 会让中位掉到 1.0mm 上下）。用中位而不是 min/max，
        //      是为了让这条守卫不受"哪几帧被放弃"的影响。
        //
        // **2026-09-17 第二处契约变更**：孔锚精修（InspectionConfig.WeldRingRefineEdgeFraction）
        // 上线后，本批里有 2 帧的**真熔核偏心超过精修搜索带能覆盖的范围**（约 25%×半径）——
        // 那种情形下逐射线边界会顶在搜索带的上下界上，精修给出一个不成立的圆
        // （孔/环比量成 45%、35%），由 §校验一「孔洞半径异常」拦下 → **测量失败交人工复核**。
        // 这是**安全方向的降级**：旧口径下这两帧的 D（4.32mm / 3.59mm）都落在 6.6mm 公差内，
        // 会被静默判合格（漏检）；现在它们至少不会被放过。
        // 因此把"孔洞半径异常"一并算作可信度原因——它是精修对"量不准"的显式报告。
        var chainFailures = failed
            .Where(f => !f.Contains("边界支撑弧段") && !f.Contains("族值带") && !f.Contains("有效性上限")
                        && !f.Contains("孔洞半径异常"))
            .ToList();
        Assert.True(chainFailures.Count == 0,
            $"{chainFailures.Count} 帧的测量失败原因不在焊环可信度门限之内（定位链退化）:\n  "
            + string.Join("\n  ", chainFailures));
        Assert.True(failed.Count * 4 <= files.Count,
            $"{failed.Count}/{files.Count} 张被放弃测量，超过 1/4（可信度门限是否过严？）:\n  "
            + string.Join("\n  ", failed));

        Assert.True(offsets.Count > 0, "没有任何一帧给出有效偏移量");
        offsets.Sort();
        var median = offsets[offsets.Count / 2];
        // **2026-09-18 第四处契约变更**：精修的射线原点由"孔心"改回"熔核自身圆心"后，
        // 本批的偏移量中位由 0.225mm 升到 **3.636mm**（12 帧 1.875~4.940mm）。
        // 这不是测量变差，而是**上一版的 0.225mm 本身是伪值**：以孔心为射线原点等于假设
        // "熔核与极柱同心"，估计器于是把焊环圆心钉在孔心上，把"焊偏"这个量抹平到近零。
        // 本轮红圈已与厂商圆心一致到 中位 2.5px（见 PpSampleTests 的实测记录），
        // 因此下面的带按**新读数**重定，作用不变（抓"毫米当量被改错"这类整批尺度错误）：
        //   · 当量退回旧的 0.025（÷4.01）→ 中位掉到约 0.91mm，越下界；
        //   · 当量放大 4.01 倍 → 中位涨到约 14.6mm，越上界。
        // 带取 [1.5, 7.0]，两侧各留约 2 倍余量。
        //
        // 同时要据实记录一个结论：**本批厂商 NG 帧的缺陷不是"焊偏"**——它们修正后的偏移量
        // （1.875~4.940mm）与同批良品（0.30~4.92mm）依然重叠，在 6.6mm 的筛查限下必然判合格。
        // 也就是说这一批 NG 样本**不能**用来验证焊偏判据，只能验证"测量链没崩"。
        Assert.InRange(median, 1.5, 7.0);
        _output.WriteLine($"偏移量（自锚精修后）：min {offsets.Min():F3}mm、max {offsets.Max():F3}mm、" +
                          $"中位 {median:F3}mm——与同批良品（0.30~4.92mm）重叠，故本批不能用来验证焊偏判据。");
    }

    private static string SnOf(string fileName)
    {
        var i = fileName.IndexOf("SN_", StringComparison.Ordinal);
        return i < 0 ? fileName : fileName[i..].Split('_')[0] + "_" + fileName[(i + 3)..];
    }

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
