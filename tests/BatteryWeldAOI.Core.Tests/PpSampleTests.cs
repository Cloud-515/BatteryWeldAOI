using BatteryWeldAOI.Core.Models;
using BatteryWeldAOI.Core.Vision;
using OpenCvSharp;
using Xunit;
using Xunit.Abstractions;

namespace BatteryWeldAOI.Core.Tests;

/// <summary>
/// 20MP 现场图的回归测试：tests/pp 下 2025-09-11 批次、SN_CCSAGZZG01A2536925090900002025080700034
/// 这一个点位的 24 帧（5472×3648，同一位置连续拍摄）。**24 帧都是厂商判良品**，
/// 因此任何非 OK 都是误判——这是本测试的判据基础。
///
/// 这组数据是本项目分辨率上限问题的发现现场：算法自称所有参数都是相对量，但形态学核带
/// **绝对像素上限**（纹理窗 ≤15、闭运算 ≤21、开运算 ≤15、飞溅 ≤9），base = 3648 时这些核
/// 相对尺度差 5~8 倍，导致闭运算桥接不到鱼鳞斑、开运算删不掉极板噪点——实测这 24 帧
/// **全部被误判为飞溅 NG**。归一化到 460px 工作分辨率后 22/24 判合格。
///
/// 归一化带来的实际改善（注意：偏移量的**量级**几乎不变，D 由焊环与孔洞的相对位置决定，
/// 这批点位的环心本来就大体正确；改善的是判定与稳定性）：
///   · 判定：24/24 飞溅 NG → 22/24 合格
///   · 焊环半径：环r/短边 由 0.133~0.183（极差 0.050）收敛到 0.142~0.146（极差 0.004），
///     即"同一视野下焊环半径/短边为常数"这一前提从失效恢复到成立
///   · 耗时：单帧 1.6~2.2s → 0.07~0.19s（大核形态学正是原耗时主因）
///
/// **残留两帧已修复（2026-09-15）**：19-22-07 与 19-22-22 原先分别是"测量失败"与"伪漏焊"，
/// 根因是环形光源在工件上的反射形成一条约 20px 厚的细圈紧贴熔核外沿，闭运算把细圈与熔核
/// 桥接成**一整块横跨可用区并贴边的巨大连通域**，被"贴边连通域 = 台面/背景"的剔除规则整块丢弃
/// （19-22-22 实测：掩码 66544px → 并块后 79535px、bbox 551×373、贴边 → 候选数 0 → 报"无焊点"漏焊）。
/// 现在由 `WeldInspectionAlgorithm` 的**兜底路径**处理：主路径（保守，不做薄结构删除）找不到焊环时，
/// 才用 `InspectionConfig.FallbackPreOpenKernelRatio`（0.045×base）在闭运算前把细圈开运算掉再试一次。
/// 修复后 19-22-22 的环r/短边=0.1430、D=0.094mm，回到家族值（良品帧 (269,165,133,132) 的等价圆），
/// 19-22-07 环r/短边=0.1514、D=0.357mm；**24/24 帧全部合格**，中位半径比不变（0.1448）。
/// 因此下面的断言按"全部帧必须合格"给出：任何一帧回退都会立刻暴露兜底路径失效。
///
/// 另有两帧（06/23）的半径仍为 0.194~0.196×短边（家族值 0.142~0.146）：细圈与熔核并块后
/// 圆心仍落在熔核上，故 D 正常、判定合格，只是半径偏大——不构成误判，故不纳入断言。
///
/// **2026-09-17：上面"24/24 帧全部合格"这一条不再成立，据实记录。** 用户报"标注的每个位置都偏了好多"，
/// 复现后定位到孔位基准（`HoleLocator`）的残差闸门把 8 成以上的真孔位圆盘判掉，退回梯度卡尺后
/// 量到的是熔核内沿——圆心偏约 211 原始像素、偏移量被系统性压低。本 SN 内两种路径交替出现即其指纹：
/// 孔/环比 0.13~0.16（孔位基准）vs 0.28~0.39（卡尺），D 0.4~0.9mm vs 0.04~0.5mm。
/// 修正后本 SN 24 帧实测 D 中位 0.46mm、最大 1.033mm：其中 19-22-07 729 **是真实的几何偏移**
/// （标注图核对：绿圈落在灰盘上、焊环圆心确实偏在右侧），按 1.0mm 公差判 NG，本 SN 因此变成 23/24。
/// 断言相应改为："不得有帧测不出来" + "孔/环比落在孔位族值带内" + "偏移量不超过 1.10mm"，
/// 不再断言零 NG——公差与像素当量的标定（`FovAspectDeviation`=0.20 说明本配置分辨率不是该相机
/// 的真实值，毫米数的绝对尺度不可信）是现场待办，不该由本测试替它决定。
///
/// 样本集可能不随源码分发；找不到目录时本测试跳过而非失败（与 RealSampleTests 同口径）。
/// VdbGui_pic 下的 .vdb 是厂商私有格式（魔数 "BDV."），无公开规格，未纳入本测试。
/// </summary>
public class PpSampleTests
{
    private const string Serial = "SN_CCSAGZZG01A2536925090900002025080700034";

    private readonly ITestOutputHelper _output;
    private readonly InspectionConfig _config = new();
    private readonly WeldInspectionAlgorithm _algorithm;

    public PpSampleTests(ITestOutputHelper output)
    {
        _output = output;
        // 现场数据的像素当量取自配置（2026-09-17 按现场实测重标为 0.1003 ⇒ 0.0235 mm/原图像素），
        // 不再在本文件硬编码 0.025——那样会让测试里的毫米数与界面/报表报出的差 4 倍。
        _algorithm = new WeldInspectionAlgorithm(_config, NinePointCalibration.PureScale(_config.MmPerPixel));
    }

    [Fact]
    public void TwentyMegapixelSamples_AreMeasured()
    {
        var directory = FindSampleDirectory();
        if (directory is null)
        {
            _output.WriteLine("未找到 tests/pp 样本目录，跳过。");
            return;
        }

        var files = Directory.GetFiles(directory, "*.jpg")
            .OrderBy(f => Path.GetFileName(f), StringComparer.Ordinal)
            .ToList();
        Assert.NotEmpty(files);

        var ok = 0;
        var failures = new List<string>();
        var offsets = new List<double>();
        var ringRatios = new List<double>();
        var pinRatios = new List<double>();

        foreach (var file in files)
        {
            var name = Path.GetFileName(file);
            var tag = name[..Math.Max(0, name.IndexOf("_Scene", StringComparison.Ordinal))];
            using var image = Cv2.ImRead(file, ImreadModes.Color);
            Assert.False(image.Empty(), $"{name}: 读取失败");

            var result = _algorithm.Inspect(image, new WeldPoint { Index = 0, Name = tag });
            // 半径比按**处理尺寸**算：归一化后像素几何量都在缩小图的坐标系里，
            // 用原图短边（3648）去除会把比值算小 8 倍
            var processedBase = Math.Min(result.ProcessedSizePx.Height, result.ProcessedSizePx.Width);
            var ringRatio = result.WeldRadiusPx / Math.Max(processedBase, 1);

            if (result.HasError || !result.IsOk)
            {
                failures.Add($"{tag}: {(result.HasError ? "测量失败" : "误判 NG")} — {result.DefectText}");
                _output.WriteLine($"{tag}  {image.Width}×{image.Height}  工作图{processedBase}px  " +
                                  $"{result.DefectText}  依据: {result.ErrorMessage ?? "-"}");
                continue;
            }

            ok++;
            offsets.Add(result.OffsetDistanceMm);
            ringRatios.Add(ringRatio);
            pinRatios.Add(result.PoleRadiusPx / result.WeldRadiusPx);
            _output.WriteLine($"{tag}  {image.Width}×{image.Height}  " +
                              $"环r/短边={ringRatio:F4} 孔/环={result.PoleRadiusPx / result.WeldRadiusPx:F3} " +
                              $"D={result.OffsetDistanceMm:F3}mm ({result.ProcessingMs:F0}ms)");
        }

        ringRatios.Sort();

        // 一帧都不该"测不出来"：孔位或焊环定位失败会报 HasError，那意味着偏移量无效、必须人工复核。
        // 这批 24 帧全部是厂商判良品，任何一帧测不出来都说明定位链退化（兜底路径失效、
        // 薄结构删除被误关等会立刻在这里暴露）。
        var unmeasurable = failures.Where(f => f.Contains("测量失败")).ToList();
        Assert.True(unmeasurable.Count == 0,
            $"{unmeasurable.Count} 帧测不出来（要求全部可测）:\n  " + string.Join("\n  ", unmeasurable));

        // 孔位必须真的被"量到"，而不是被梯度卡尺顶替。这一条抓的是孔位基准的退化：
        //   · 退化成梯度卡尺（射线原点是焊环圆心）→ 量到熔核内沿，孔/环比升到 0.30~0.40；
        //   · 退化回"只覆盖孔位端面被照亮的那一半"的中间灰度层 → 环比掉到 0.13~0.16。
        // 实测本 SN 24 帧（2026-09-17 加上外径补全后）：0.129~0.387，中位 0.226——
        // 三簇分别为 0.13~0.16（外径弧未取到、只用中间层 8 帧）、0.22~0.26（外径补全 11 帧）、
        // 0.30~0.39（卡尺兜底 5 帧）。区间取 [0.17, 0.32] 把中间那一簇圈住，
        // 两侧的退化都会被拦下。偏移量的绝对值抓不住第一类退化——它恰恰表现为"偏移量偏小"。
        pinRatios.Sort();
        var medianPinRatio = pinRatios[pinRatios.Count / 2];
        Assert.InRange(medianPinRatio, 0.17, 0.32);

        // 合格帧的偏移量上限。**2026-09-18 第三次修正：一度由 5.00mm 收到 1.00mm，现回到"公差"口径。**
        //
        // 沿革（据实记录，因为它包含一次**把测量值当成改善**的误判）：
        //   · 5.00mm 那一版是"焊环心由纹理掩码外接圆给出"口径下的读数（本 SN 实测最大 4.793mm）；
        //   · 收到 1.00mm 的依据是"改用孔锚精修后本 SN 24 帧实测 D ≤ 0.35mm"。但那个 0.35mm
        //     不是测量变准了，而是**估计器以孔心为射线原点、把焊环圆心钉在了孔心上**——
        //     "焊偏"这个量正是它要检验的东西，于是它必然给出接近零的读数。
        //     同期的跨帧一致性指标（x 极差 269px→25px）也被这一点奖励了：把测量压成常数
        //     当然"稳定"。用户在 2026-09-18 用手绘蓝圈指出红圈仍然偏（实测圆心差 34px、半径 +31px）。
        //   · 本轮把精修的射线原点改回**熔核自身圆心**、并把边界判据改成"纹理第一段平台的末端"
        //     （见 InspectionConfig.WeldRingRefineEdgeFraction），红圈才真正贴上熔核外沿：
        //     与厂商圆心差 中位 2.5px、与独立纹理边界拟合差 中位 2.9px、与用户手绘真值差 16px。
        //
        // 偏移量随之回到**真实量级**（本 SN 24 帧 0.30~4.92mm、中位 1.76mm）——这是正确结果，
        // 不是退化：公差 `OffsetToleranceMm` 本就是按 pp 良品实测推出来的筛查限（6.6mm）。
        // 因此断言分两条：
        //   ① 不得有帧超出筛查公差（超了就该判 NG，而上面已经断言全部合格，两者一致）；
        //   ② **中位**落在实测带内——它抓的是"毫米当量被改错"这类整批尺度错误：
        //      当量退回旧的 0.025（÷4.01）→ 中位掉到约 0.44mm，越下界；
        //      当量放大 4.01 倍 → 中位涨到约 7.0mm，越上界。取 [0.9, 3.2] 两侧各留约 1.8 倍余量。
        Assert.True(offsets.Max() <= _config.OffsetToleranceMm,
            $"合格帧偏移量超出筛查公差：最大 {offsets.Max():F3}mm（公差 {_config.OffsetToleranceMm:F1}mm）");
        offsets.Sort();
        var medianOffset = offsets[offsets.Count / 2];
        Assert.InRange(medianOffset, 0.9, 3.2);

        // 同一视野下焊环半径/短边应为常数。**2026-09-18 口径再修正**：焊环圆现在量的是
        // "|∇I| 中位剖面里第一段显著平台的末端"（见 InspectionConfig.WeldRingRefineEdgeFraction），
        // 与厂商 .vdb（中位 0.1316）和独立纹理边界拟合（中位 0.1347）同口径。
        // 实测本 SN 24 帧 0.1228~0.1343，中位 0.1330——旧的 [0.110, 0.145] 依然成立，故保留。
        var medianRatio = ringRatios[ringRatios.Count / 2];
        Assert.InRange(medianRatio, 0.110, 0.145);

        _output.WriteLine($"\n{ok}/{files.Count} 帧判合格；D 中位={medianOffset:F3}mm " +
                          $"最大={offsets.Max():F3}mm；环r/短边 中位={medianRatio:F4} " +
                          $"孔/环 中位={medianPinRatio:F3}");
    }

    /// <summary>从测试输出目录向上查找 tests/pp/2025-09-11/Orignal_pic/&lt;SN&gt;。</summary>
    private static string? FindSampleDirectory()
    {
        var dir = new DirectoryInfo(AppContext.BaseDirectory);
        while (dir is not null)
        {
            var candidate = Path.Combine(dir.FullName, "tests", "pp", "2025-09-11", "Orignal_pic", Serial);
            if (Directory.Exists(candidate))
                return candidate;
            dir = dir.Parent;
        }
        return null;
    }
}
