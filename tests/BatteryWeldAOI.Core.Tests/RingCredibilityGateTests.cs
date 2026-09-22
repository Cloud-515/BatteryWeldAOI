namespace BatteryWeldAOI.Core.Tests;

using BatteryWeldAOI.Core.Models;
using BatteryWeldAOI.Core.Vision;
using OpenCvSharp;
using Xunit;
using Xunit.Abstractions;

/// <summary>
/// 焊环**可信度门限**的现场回归（FIXPLAN 的 T1.9）。
///
/// 为什么必须有这个用例：本仓库此前的鲁棒性回归全部建立在两套**合成夹具**上
/// （<c>ImageGenerator</c> / <c>RealisticImageGenerator</c>），而"环形光源反射细圈与熔核并块"
/// 这个失效形态在合成夹具里根本不存在——夹具不画细圈。于是算法在夹具上全绿、一上现场图就
/// 把圆心画偏（用户 2026-09-17 报的正是这个）。本用例把现场真实帧钉进来当夹具。
///
/// 钉住的行为（2026-09-17 修复后）：
///   · `Img010 / OK_459`（SN ...0070，用户报的帧）：焊环边界被细圈污染、只有 68% 圆周，
///     必须报**测量失败**（交人工复核），**不得**判合格、**不得**伪造成"漏焊"、
///     也不得给出一个偏移量。修复前它输出圆心 (2574.7, 1950.3)、半径 578.1，
///     与厂商 `.vdb` 的 (2647.1, 1911.0)、481.6 相差 82 原始像素 = 1.94mm，半径偏大 20%。
///   · 同 SN 的良品帧必须照常测出，且焊环半径落回族值 0.143 附近（说明门限没有误伤良品）。
///
/// 这组断言依赖 tests/pp 现场样本存在；样本缺失时跳过（与 PpSampleTests 一致）。
/// </summary>
public sealed class RingCredibilityGateTests
{
    private readonly ITestOutputHelper _output;
    private readonly InspectionConfig _config = new();
    private readonly WeldInspectionAlgorithm _algorithm;

    public RingCredibilityGateTests(ITestOutputHelper output)
    {
        _output = output;
        // 纯缩放标定：本用例只关心定位与判定，不关心毫米当量的绝对正确性
        var calibration = new NinePointCalibration(new List<(Point2d Pixel, Point2d Mm)>
        {
            (new(0, 0), new(0, 0)),
            (new(1000, 0), new(25, 0)),
            (new(0, 1000), new(0, 25)),
        });
        _algorithm = new WeldInspectionAlgorithm(_config, calibration);
    }

    /// <summary>
    /// 用户报的那一帧（`21-57-36 299 / OK_459`）：**焊环必须被测出来，且必须与厂商圆吻合**。
    ///
    /// **2026-09-18 断言契约更新（记录为什么）**：本用例原先断言"这一帧必须报测量失败"——
    /// 那是当时**测不出来**的事实记录（细圈与工件外框把熔核桥接成一个贴边连通域，
    /// 被"贴边=背景"规则连熔核一起扔掉，候选数 0）。但用户 2026-09-18 再次报的正是这一帧
    /// （"明明有焊环却报异常"），说明"报测量失败"并非期望结果。第三路径（形态学之前先清贴边连通域，
    /// 见 `InspectionConfig.EnableRingBorderCleanupFallback`）落地后本帧测出来了：
    /// 圆心 (2646.0,1909.6)、半径 485.7（原图像素），与厂商 `.vdb` 的 (2647.1,1911.0)、481.6
    /// **只差 1.8px / +0.85%**，目视与熔核外沿贴合。
    ///
    /// 于是契约改成"**两种结局都合法，但都不许自信地给错**"（与
    /// <see cref="OversizedPoleRadiusFrame_NeverOutputsOffsetFromWrongReference"/> 同一写法）：
    ///   · 结局一（本机当前实际走的）：给出了测量 → 圆心必须对上厂商圆、半径同口径、
    ///     边界支撑弧段必须达标，且**不得**判成缺陷（这是厂商良品帧）；
    ///   · 结局二（若某台机器的成像条件回到测不出来的状态）：必须报测量失败、
    ///     不输出偏移量、且写明原因。**绝不允许**的是"给出一个偏 80px 的自信圆"。
    /// </summary>
    [Fact]
    public void PollutedBoundaryFrame_IsMeasuredAgainstVendorCircle_OrReportedAsFailure()
    {
        var file = FindSample("21-57-36 299", "_OK_459");
        if (file is null)
        {
            _output.WriteLine("未找到现场样本 tests/pp，跳过。");
            return;
        }

        using var image = Cv2.ImRead(file, ImreadModes.Color);
        Assert.False(image.Empty(), $"{Path.GetFileName(file)}: 读取失败");

        var result = _algorithm.Inspect(image, new WeldPoint { Index = 0, Name = "OK_459" });
        _output.WriteLine($"判定={result.DefectTextZh}  焊环圆心(原图)=({result.WeldCenterSourcePx.X:F1}," +
                          $"{result.WeldCenterSourcePx.Y:F1}) r={result.WeldRadiusSourcePx:F1}  " +
                          $"支撑弧段={result.WeldArcSupport:P0}  依据={result.ErrorMessage ?? "-"}");

        // ① 无论哪种结局，都不得把这个良品帧伪造成确定缺陷（"漏焊"是最容易被误报的那一类）
        Assert.DoesNotContain(WeldDefect.MissingWeld, result.Defects);
        Assert.DoesNotContain(WeldDefect.Misaligned, result.Defects);

        if (result.HasError)
        {
            // 结局二：测不出来 → 交人工复核，不输出物理量，且必须写明原因
            Assert.False(result.HasValidOffset);
            Assert.False(string.IsNullOrWhiteSpace(result.ErrorMessage));
            return;
        }

        // 结局一：给出了测量 → 必须与厂商 `.vdb` 的圆吻合（厂商在本帧的定位：圆心 (2647.1,1911.0)、
        // 半径 481.6 原图像素）。容差按"厂商自身在同一张图上换位置标记的互差中位 24.4px"取 30px。
        Assert.False(result.IsOk && result.Defects.Count > 0);
        var dx = result.WeldCenterSourcePx.X - 2647.1;
        var dy = result.WeldCenterSourcePx.Y - 1911.0;
        var deviation = Math.Sqrt(dx * dx + dy * dy);
        Assert.True(deviation <= 30,
            $"焊环圆心偏离厂商圆 {deviation:F1} 原始像素（上限 30）——" +
            "旧版（本次修复前）这里直接报测量失败，更早的版本偏 82px（1.94mm）");
        Assert.InRange(result.WeldRadiusSourcePx / 481.6, 0.92, 1.10);
        Assert.True(result.WeldArcSupport >= _config.MinRingArcSupport,
            $"焊环边界支撑弧段只有 {result.WeldArcSupport:P0}，未达门限 {_config.MinRingArcSupport:P0}");
    }

    /// <summary>同 SN 的良品帧必须照常测出，且焊环半径落回族值附近——门限不能误伤正常焊点。</summary>
    [Theory]
    [InlineData("23-00-16 305", "_OK_492")]
    [InlineData("23-00-25 977", "_OK_510")]
    public void CleanFrame_IsStillMeasured(string scene, string suffix)
    {
        var file = FindSample(scene, suffix);
        if (file is null)
        {
            _output.WriteLine("未找到现场样本 tests/pp，跳过。");
            return;
        }

        using var image = Cv2.ImRead(file, ImreadModes.Color);
        Assert.False(image.Empty(), $"{Path.GetFileName(file)}: 读取失败");

        var result = _algorithm.Inspect(image, new WeldPoint { Index = 0, Name = scene });
        var processedBase = Math.Max(1, Math.Min(result.ProcessedSizePx.Height, result.ProcessedSizePx.Width));
        var ringRatio = result.WeldRadiusPx / processedBase;
        _output.WriteLine($"{scene}: 判定={result.DefectTextZh}  环r/工作短边={ringRatio:F4}  " +
                          $"支撑弧段={result.WeldArcSupport:P0}  D={result.OffsetDistanceMm:F3}mm");

        Assert.False(result.HasError, $"{scene}: 良品帧不该报测量失败（{result.ErrorMessage}）");
        Assert.True(result.WeldRadiusPx > 0, $"{scene}: 未定位到焊环");
        // **2026-09-17 半径口径修正**：焊环圆现在由"以孔心为射线原点、用梯度幅值量出的那条边界"
        // 精修给出（见 InspectionConfig.WeldRingRefineEdgeFraction），量的是**熔核外沿本身**；
        // 旧口径量的是"纹理掩码连通域的外缘"，它把熔核外侧的散斑/凹陷细暗圈一起吸进来，
        // 半径系统性偏大 9%（旧 0.143~0.147，厂商 .vdb 为 0.1316）。
        // 修正后实测本两帧 0.1246 / 0.1266，与厂商同口径；区间取 0.110~0.145。
        Assert.InRange(ringRatio, 0.110, 0.145);
        // 良品的边界必须是一整圈
        Assert.True(result.WeldArcSupport >= _config.MinRingArcSupport,
            $"{scene}: 良品帧的支撑弧段只有 {result.WeldArcSupport:P0}");
    }

    /// <summary>
    /// 用户报的**另一帧**（Img003 / 0040 的 OK_450）：极柱圈被量成真值的 1.8 倍、圆心偏 3.7mm。
    ///
    /// 成因是"孔位外径弧"锁到了那片被反光冲白的端面金属区边界（2.32×种子、0.400×焊环半径），
    /// 旧上限 0.45 拦不住；而现在它要么被拦下后走兜底路径/报测量失败，要么必须给出一个
    /// **圆心正确**的绿圈。这条断言对两种结局都成立，钉住的是"不许拿一个偏 3.7mm 的基准
    /// 去算偏移量"。
    /// </summary>
    [Fact]
    public void OversizedPoleRadiusFrame_NeverOutputsOffsetFromWrongReference()
    {
        var file = FindSample("22-59-54 194", "_OK_450");
        if (file is null)
        {
            _output.WriteLine("未找到现场样本 tests/pp，跳过。");
            return;
        }

        using var image = Cv2.ImRead(file, ImreadModes.Color);
        Assert.False(image.Empty(), $"{Path.GetFileName(file)}: 读取失败");

        var result = _algorithm.Inspect(image, new WeldPoint { Index = 0, Name = "OK_450" });
        var ratio = result.WeldRadiusPx > 0 ? result.PoleRadiusPx / result.WeldRadiusPx : 0;
        _output.WriteLine($"判定={result.DefectTextZh}  孔/环={ratio:F3}  " +
                          $"极柱圆心(原图)=({result.PoleCenterSourcePx.X:F0},{result.PoleCenterSourcePx.Y:F0}) r={result.PoleRadiusSourcePx:F0}");

        if (result.HasError)
        {
            // 结局一：报测量失败（旧值 0.400 越界）——交人工复核，不输出偏移量
            Assert.False(result.HasValidOffset);
            Assert.False(string.IsNullOrWhiteSpace(result.ErrorMessage));
            return;
        }

        // 结局二：给出了测量，那么绿圈必须在实测族值带内，且圆心要对得上真实极柱盘
        // （真实盘圆心 ≈ (2552.7, 1714.3) 原图像素、半径 ≈116，由 12 倍放大目视核对得到）
        Assert.InRange(ratio, _config.MinPinRadiusRatio, _config.MaxPinRadiusRatio);
        var dx = result.PoleCenterSourcePx.X - 2552.7;
        var dy = result.PoleCenterSourcePx.Y - 1714.3;
        Assert.True(Math.Sqrt(dx * dx + dy * dy) <= 40,
            $"极柱圆心偏离真实极柱盘 {(Math.Sqrt(dx * dx + dy * dy)):F0} 原始像素（上限 40）——" +
            "旧版这里是 156 像素，正是用户报的'绿圈框选偏移'");
    }

    /// <summary>
    /// 用户 2026-09-19 报的**第二张**"未找到纹理显著区域"截图：`SN_...0004` 第 10 帧
    /// （`23-00-05 185 / OK_471`）。它与 <see cref="PollutedBoundaryFrame_IsMeasuredAgainstVendorCircle_OrReportedAsFailure"/>
    /// 那一帧**表面现象相同、机制完全不同**，所以必须单独钉住：
    ///
    ///   · 前者（`SN_...0007` OK_459）：熔核在**阈值层**是独立的不贴边连通域，
    ///     是 21px 闭运算把它与工件外框桥接成一块 → 第三路径（形态学之前先清贴边）能救它；
    ///   · 本帧（`SN_...0004` OK_471）：熔核在**阈值层就已经**与工件外框连成同一个贴边连通域
    ///     （原始掩码实测：最大连通域 area=48574、bbox=(155,23,421,371)，焊环与它在同一域内），
    ///     所以第三路径把它连同熔核一起清掉，只能剩下一个 429px 的碎斑（支撑弧段 12%）。
    ///
    /// 本帧真正卡住的是**另一件事**：同一 SN 的其余 23 帧熔核纹理致密、掩码实心（支撑弧段 93%~100%），
    /// 只有这一帧纹理稀疏，掩码在鱼鳞斑之间断裂。激进薄结构删除 k=0.060 一级给出的圆其实很准
    /// （圆心 (335.6,234.8)、半径比 0.1376），厂商 `.vdb` 是 (335.0,233.8)、半径比 0.1317
    /// ——**圆心只差 1.2px**，但"掩码轮廓落在中位半径±10% 的角向扇区占比"只有 60%，够不着 0.82。
    /// 这道门限是拿主路径的**实心掩码**标定的，用它否决"掩码稀疏"的候选属于口径错配。
    ///
    /// 第四路径（见 <see cref="InspectionConfig.MinRingFallbackArcSupport"/>）因此把掩码门限放宽到 0.50，
    /// 但要求结果**必须**由 <c>RefineBoundary</c> 在 |∇I| 上转正（支撑弧段 ≥ 0.82）、
    /// 且只作用于"本来就会报测量失败"的帧。本用例钉的就是这条链路的产出。
    /// </summary>
    [Fact]
    public void SparseMaskFrame_IsCertifiedByBoundaryRefinement_AndMatchesVendorCircle()
    {
        var file = FindSample("23-00-05 185", "_OK_471",
            "SN_CCSAGZZG01A2536925090900004025080700034");
        if (file is null)
        {
            _output.WriteLine("未找到现场样本 tests/pp，跳过。");
            return;
        }

        using var image = Cv2.ImRead(file, ImreadModes.Color);
        Assert.False(image.Empty(), $"{Path.GetFileName(file)}: 读取失败");

        var result = _algorithm.Inspect(image, new WeldPoint { Index = 0, Name = "OK_471" });
        _output.WriteLine($"判定={result.DefectTextZh}  焊环圆心(原图)=({result.WeldCenterSourcePx.X:F1}," +
                          $"{result.WeldCenterSourcePx.Y:F1}) r={result.WeldRadiusSourcePx:F1}  " +
                          $"支撑弧段={result.WeldArcSupport:P0}  依据={result.ErrorMessage ?? "-"}");

        // 本帧是固定的磁盘数据、成像条件不会自变，所以这里可以要一个**确定的**结论：
        // 必须被量出来。第四路径就是为它加的——退回"测不出来"说明那条路径失效了。
        Assert.False(result.HasError,
            $"这一帧必须被测出来（原结论：{result.ErrorMessage}）——" +
            "它正是 InspectionConfig.MinRingFallbackArcSupport 记录的用例");
        Assert.False(result.IsOk && result.Defects.Count > 0);

        // 与厂商 `.vdb` 对照（本帧厂商圆：圆心 (2656.5,1854.5)、半径 480.6 原图像素）。
        // 容差按"厂商自身在同一张图上换位置标记的互差中位 24.4px"取 30px。
        var dx = result.WeldCenterSourcePx.X - 2656.5;
        var dy = result.WeldCenterSourcePx.Y - 1854.5;
        var deviation = Math.Sqrt(dx * dx + dy * dy);
        Assert.True(deviation <= 30,
            $"焊环圆心偏离厂商圆 {deviation:F1} 原始像素（上限 30）");
        Assert.InRange(result.WeldRadiusSourcePx / 480.6, 0.92, 1.10);
        Assert.True(result.WeldArcSupport >= _config.MinRingArcSupport,
            $"焊环边界支撑弧段只有 {result.WeldArcSupport:P0}，未达门限 {_config.MinRingArcSupport:P0}" +
            "——第四路径的产出来自精修，必须带上精修自己量出的支撑弧段");
    }

    /// <summary>
    /// 第四路径的**安全边界**：候选能进这条路径，靠的是"掩码边界不成圈"被放宽了门限，
    /// 因此**只有**边界精修能在 |∇I| 上证明它成圈才允许进入判定；证明不了必须退回测量失败。
    ///
    /// 钉的是 `SN_...0502` 第 10 帧（`21-39-16 665 / OK_597`）——它是本批 40 SN / 960 帧里
    /// 唯一一帧被第四路径"捞起来"却**不该**被采纳的：激进档给出的候选圆心与厂商 `.vdb`
    /// 相差 **144px**，若直接采纳就是一个自信的错读数。实测精修给它的边界只有 70% 圆周，
    /// 低于 0.82 → 必须报测量失败、不输出偏移量。
    ///
    /// 断言契约（与本文件其它用例同写法）：**两种结局都合法，但都不许自信地给错**——
    ///   · 若给出测量：圆心必须与厂商圆吻合（≤30px）；
    ///   · 若报测量失败：不得输出偏移量、必须写明原因。
    /// 本帧当前实际走的是后者。
    /// </summary>
    [Fact]
    public void UncertifiedFallbackCandidate_IsNeverReportedAsAMeasurement()
    {
        var file = FindSample("21-39-16 665", "_OK_597",
            "SN_CCSAGZZG01A2536925090900005025080700034");
        if (file is null)
        {
            _output.WriteLine("未找到现场样本 tests/pp，跳过。");
            return;
        }

        using var image = Cv2.ImRead(file, ImreadModes.Color);
        Assert.False(image.Empty(), $"{Path.GetFileName(file)}: 读取失败");

        var result = _algorithm.Inspect(image, new WeldPoint { Index = 0, Name = "OK_597" });
        _output.WriteLine($"判定={result.DefectTextZh}  焊环圆心(原图)=({result.WeldCenterSourcePx.X:F1}," +
                          $"{result.WeldCenterSourcePx.Y:F1}) r={result.WeldRadiusSourcePx:F1}  " +
                          $"支撑弧段={result.WeldArcSupport:P0}  依据={result.ErrorMessage ?? "-"}");

        // 厂商良品帧：不得被判成确定缺陷
        Assert.DoesNotContain(WeldDefect.MissingWeld, result.Defects);
        Assert.DoesNotContain(WeldDefect.Misaligned, result.Defects);

        if (result.HasError)
        {
            Assert.False(result.HasValidOffset);
            Assert.False(string.IsNullOrWhiteSpace(result.ErrorMessage));
            return;
        }

        // 若某天精修能把它认下来，那也必须认对：厂商圆 (2636.8,1875.0) r=493.2 原图像素
        var dx = result.WeldCenterSourcePx.X - 2636.8;
        var dy = result.WeldCenterSourcePx.Y - 1875.0;
        var deviation = Math.Sqrt(dx * dx + dy * dy);
        Assert.True(deviation <= 30,
            $"焊环圆心偏离厂商圆 {deviation:F1} 原始像素（上限 30）——" +
            "第四路径的候选偏厂商 144px，只有精修把它认回来才允许输出");
        Assert.InRange(result.WeldRadiusSourcePx / 493.2, 0.92, 1.10);
    }

    /// <summary>
    /// 用户 2026-09-19 报的**第三张**截图（`Img001` / `SN_...0004` 第 1 帧 / `22-59-50 992 / OK_444`）：
    /// "黑环导致焊环识别存在偏移"。
    ///
    /// **黑环**是工件上直径 30mm 机加工凹陷台阶的阴影边（第 6.1 节已确认它不是焊环），
    /// 与**孔同轴**（圆心差 1.2 工作像素）。本轮实测确认它**会进入纹理掩码**
    /// （本帧掩码外接半径 81.0 = 黑环半径；抹掉黑环后降到 68.3），
    /// 于是精修时以**孔心**为起点的那一路会被它困住，收敛到一个"自洽但偏小"的圆，
    /// 而它的**支撑弧段假性偏高**（95% vs 主起点的 90%），仲裁规则（支撑弧段更高者胜）于是选错。
    ///
    /// 本帧实测偏差 13.5 原始像素（厂商 `.vdb`：(2677.0,1795.9) r=502.0），在 30px 容差内，
    /// 故这里按"必须测出且与厂商吻合"断言——它是**能达标的**那一帧。
    /// 同 SN 第 0 帧（`OK_441`）偏 35.8px，超出 30px，见
    /// <see cref="BlackRingWorstFrame_IsMeasuredWithinWidenedTolerance"/>。
    /// </summary>
    [Fact]
    public void BlackRingFrame_IsMeasuredWithinVendorTolerance()
    {
        var file = FindSample("22-59-50 992", "_OK_444",
            "SN_CCSAGZZG01A2536925090900004025080700034");
        if (file is null)
        {
            _output.WriteLine("未找到现场样本 tests/pp，跳过。");
            return;
        }

        using var image = Cv2.ImRead(file, ImreadModes.Color);
        Assert.False(image.Empty(), $"{Path.GetFileName(file)}: 读取失败");

        var result = _algorithm.Inspect(image, new WeldPoint { Index = 0, Name = "OK_444" });
        _output.WriteLine($"判定={result.DefectTextZh}  焊环圆心(原图)=({result.WeldCenterSourcePx.X:F1}," +
                          $"{result.WeldCenterSourcePx.Y:F1}) r={result.WeldRadiusSourcePx:F1}  " +
                          $"支撑弧段={result.WeldArcSupport:P0}  依据={result.ErrorMessage ?? "-"}");

        Assert.False(result.HasError, $"这一帧必须被测出来（{result.ErrorMessage}）");
        Assert.DoesNotContain(WeldDefect.MissingWeld, result.Defects);
        Assert.DoesNotContain(WeldDefect.Misaligned, result.Defects);

        var dx = result.WeldCenterSourcePx.X - 2677.0;
        var dy = result.WeldCenterSourcePx.Y - 1795.9;
        var deviation = Math.Sqrt(dx * dx + dy * dy);
        Assert.True(deviation <= 30,
            $"焊环圆心偏离厂商圆 {deviation:F1} 原始像素（上限 30）");
        Assert.InRange(result.WeldRadiusSourcePx / 502.0, 0.92, 1.10);
    }

    /// <summary>
    /// 黑环帧里**最坏的那一帧**（`SN_...0004` 第 0 帧 / `22-59-49 478 / OK_441`）。
    ///
    /// **2026-09-20 已修好，容差由 60px 收紧到 30px**（FIXPLAN 第 9 节）。
    ///
    /// 原实测偏差 **35.8 原始像素**（厂商 `.vdb`：(2618.1,1771.4)）。成因不是"两个起点谁更圆"，
    /// 而是**仲裁规则被一条与孔同轴的机加工锐边骗到**：工件上 30mm 凹陷台阶的阴影边（"黑环"）
    /// 在 |∇I| 上比焊道纹理强一倍（剖面峰值 117.5 vs 70.6），把**孔锚起点**吸走，收敛到
    /// (326.3,221.0) r=60.5，而它的支撑弧段**假性偏高**（95.3% vs 主起点 82.4%），于是被选中；
    /// 主起点给的是 (332.4,224.5) r=65.9，离厂商圆只差 2.5 工作像素（**是对的**）。
    ///
    /// 修法是 `InspectionConfig.WeldRingRefineMinRoughnessRatio`（边界粗糙度闸门）：
    /// **焊道外沿是鱼鳞状凝固组织、必然粗糙；机加工边是锐边、必然光滑**，故要求第二起点的
    /// 边界粗糙度不得低于主起点的 0.5 倍。本帧实测比值 **0.43**（该拦），
    /// 而 4.9 节靠第二起点救回的 `OK_936` 是 **1.60**（该放）。
    ///
    /// 修复后本帧采用主起点，偏差 **19.9 原始像素**。40 SN / 960 帧 A/B：3 帧变好、1 帧变差，
    /// 尾部指标（max 49.2px、p90 6.8px、半径比带外 0 帧）**一字未变**——与被否定的那两次修法
    /// （max 49.2 → 63.5px）的关键区别就在这里。
    ///
    /// 容差取 30px 与同文件其它黑环/细圈用例一致（依据：厂商自身在同一张图上换位置标记的
    /// 互差中位 24.4px）。实测 19.9px 有 1.5 倍余量。
    /// </summary>
    [Fact]
    public void BlackRingWorstFrame_IsMeasuredWithinVendorTolerance()
    {
        var file = FindSample("22-59-49 478", "_OK_441",
            "SN_CCSAGZZG01A2536925090900004025080700034");
        if (file is null)
        {
            _output.WriteLine("未找到现场样本 tests/pp，跳过。");
            return;
        }

        using var image = Cv2.ImRead(file, ImreadModes.Color);
        Assert.False(image.Empty(), $"{Path.GetFileName(file)}: 读取失败");

        var result = _algorithm.Inspect(image, new WeldPoint { Index = 0, Name = "OK_441" });
        _output.WriteLine($"判定={result.DefectTextZh}  焊环圆心(原图)=({result.WeldCenterSourcePx.X:F1}," +
                          $"{result.WeldCenterSourcePx.Y:F1}) r={result.WeldRadiusSourcePx:F1}  " +
                          $"支撑弧段={result.WeldArcSupport:P0}  依据={result.ErrorMessage ?? "-"}");

        Assert.False(result.HasError, $"这一帧必须被测出来（{result.ErrorMessage}）");
        Assert.DoesNotContain(WeldDefect.MissingWeld, result.Defects);
        Assert.DoesNotContain(WeldDefect.Misaligned, result.Defects);

        var dx = result.WeldCenterSourcePx.X - 2618.1;
        var dy = result.WeldCenterSourcePx.Y - 1771.4;
        var deviation = Math.Sqrt(dx * dx + dy * dy);
        Assert.True(deviation <= 30,
            $"焊环圆心偏离厂商圆 {deviation:F1} 原始像素（上限 30）——" +
            "修复前这里是 35.8px（黑环把孔锚起点吸走，见 WeldRingRefineMinRoughnessRatio 的说明）");
        Assert.InRange(result.WeldRadiusSourcePx / 482.4, 0.92, 1.10);
    }

    /// <summary>
    /// 用户 2026-09-20 报的那一帧（`SN_...0003` 第 10 帧 / `16-49-04 649 / OK_963`，截图 D=3.305mm）。
    ///
    /// **它的偏移不是黑环造成的**（实测）：黑环在 r≈80 工作px（直径 29.98mm，与孔同轴，
    /// 圆心离我们孔心 13.5 原图像素 = 0.32mm），而**孔锚精修的搜索带是 [55.0, 66.0] 工作px**——
    /// 黑环在带外 14px，够不到，不可能困住精修。本帧真正的原因是**仲裁规则**：
    /// 主起点给 (327.3,240.0) r=65.0，离厂商圆只差 **5.7 原始像素**；
    /// 第二起点给 (331.0,240.8) r=62.3，差 26.4px——但它的支撑弧段更高（95.0% vs 77.2%），
    /// 于是"弧段更高者胜"选了错的那个。
    ///
    /// 这一帧同时**结掉了第 4.9 节的一个悬案**：4.9 记录它"由 5.9px 变为 26.7px，
    /// 两帧的独立纹理真值被细圈污染，无法判定哪个更对"——厂商 `.vdb` 圆站在主起点一边，
    /// 即 4.9 引入"第二起点"时在本帧上是净退步。
    ///
    /// **本用例钉的是"它必须被测出来、且偏差在容差内"**（实测 26.4px，容差 30px）。
    /// 曾有一条判据（`WeldRingRefineMinRhoRatio`，ρ 一致性闸门）能把它收到 5.9px，
    /// 但那条判据在 `OK_933` 上会让熔核半径变成物理上不可能的 527px（该帧必须 ≈489px），
    /// 故**默认关闭**——详见 `InspectionConfig.WeldRingRefineMinRhoRatio` 与 FIXPLAN 第 10 节。
    /// 若将来有人补上"按产品标定的熔核半径带"，那条判据就能安全启用，届时本断言应收紧到 10px。
    /// </summary>
    [Fact]
    public void BlackRingUnrelatedFrame_OffsetWasArbitrationNotRing_IsNowMeasuredAgainstVendor()
    {
        var file = FindSample("16-49-04 649", "_OK_963",
            "SN_CCSAGZZG01A2536925090900003025080700034");
        if (file is null)
        {
            _output.WriteLine("未找到现场样本 tests/pp，跳过。");
            return;
        }

        using var image = Cv2.ImRead(file, ImreadModes.Color);
        Assert.False(image.Empty(), $"{Path.GetFileName(file)}: 读取失败");

        var result = _algorithm.Inspect(image, new WeldPoint { Index = 0, Name = "OK_963" });
        _output.WriteLine($"判定={result.DefectTextZh}  焊环圆心(原图)=({result.WeldCenterSourcePx.X:F1}," +
                          $"{result.WeldCenterSourcePx.Y:F1}) r={result.WeldRadiusSourcePx:F1}  " +
                          $"支撑弧段={result.WeldArcSupport:P0}  依据={result.ErrorMessage ?? "-"}");

        Assert.False(result.HasError, $"这一帧必须被测出来（{result.ErrorMessage}）");
        Assert.DoesNotContain(WeldDefect.MissingWeld, result.Defects);
        Assert.DoesNotContain(WeldDefect.Misaligned, result.Defects);

        // 厂商 `.vdb`：圆心 (2600.5,1900.4)、半径 508.5 原图像素。容差 30px 与同文件其它用例一致
        var dx = result.WeldCenterSourcePx.X - 2600.5;
        var dy = result.WeldCenterSourcePx.Y - 1900.4;
        var deviation = Math.Sqrt(dx * dx + dy * dy);
        Assert.True(deviation <= 30,
            $"焊环圆心偏离厂商圆 {deviation:F1} 原始像素（上限 30）——" +
            "本帧实测 26.4px（仲裁选了弧段更高但锁错边界的第二起点；主起点本来只差 5.7px）");
        Assert.InRange(result.WeldRadiusSourcePx / 508.5, 0.92, 1.10);
    }

    /// <summary>
    /// **第五路径（多起点重试）要救的那两帧**——它们此前报"焊环候选未被边界精修确认"，
    /// 但实测**熔核外沿完全可测，只是掩码算出的候选圆心偏了**（FIXPLAN 第 11 节）。
    ///
    /// 机理（实测，不是推测）：
    /// · `21-39-16 665`：用厂商 `.vdb` 的圆当种子跑我们的精修 → 支撑弧段 **99.2%**、离厂商 18px；
    ///   用掩码质心当种子 → 落进 r=535、弧段 70% 的错误盆地；两个种子**只差 2.5 工作像素**
    ///   （分叉，所以换个"更好的估计器"没用，必须搜索）；
    /// · `17-01-23 304`：掩码质心离厂商圆心 **22.7 工作像素**，径向扫描显示正确盆地在 8px 之外。
    ///
    /// 第五路径用**尺度相对网格**（步长 0.13×半径，{0,±1,±2}×步长 = 25 个种子）搜索，
    /// **只采纳过 0.82 弧段门限的结果**、取弧段最高者——错误盆地的弧段只有 68~74%，过不了门。
    /// 两帧实测：`21-39-16 665` 弧段 99%、离厂商 18.4px；`17-01-23 304` 弧段 95%、离厂商 3.7px。
    ///
    /// 这两条断言钉住"它们必须被测出来、且圆心对得上厂商圆"。
    /// 若将来有人把第五路径关掉（`MultiSeedRetryStepRatio = 0`），它们会失败——这正是它们存在的意义。
    /// </summary>
    [Theory]
    [InlineData("21-39-16 665", "_OK_597", "SN_CCSAGZZG01A2536925090900005025080700034",
        2636.8, 1875.0, 493.2)]
    [InlineData("17-01-23 304", "_OK_537", "SN_CCSAGZZG01A2536925090900028025080700034",
        2609.5, 1874.9, 477.2)]
    public void MaskCenterBiasedFrame_IsRescuedByMultiSeedRetry(string scene, string suffix,
        string sn, double vx, double vy, double vr)
    {
        var file = FindSample(scene, suffix, sn);
        if (file is null)
        {
            _output.WriteLine("未找到现场样本 tests/pp，跳过。");
            return;
        }

        using var image = Cv2.ImRead(file, ImreadModes.Color);
        Assert.False(image.Empty(), $"{Path.GetFileName(file)}: 读取失败");

        var result = _algorithm.Inspect(image, new WeldPoint { Index = 0, Name = scene });
        _output.WriteLine($"{scene}: 判定={result.DefectTextZh}  " +
                          $"焊环圆心(原图)=({result.WeldCenterSourcePx.X:F1}," +
                          $"{result.WeldCenterSourcePx.Y:F1}) r={result.WeldRadiusSourcePx:F1}  " +
                          $"支撑弧段={result.WeldArcSupport:P0}  依据={result.ErrorMessage ?? "-"}");

        Assert.False(result.HasError,
            $"{scene}: 必须被测出来（第五路径就是为它加的；原结论：{result.ErrorMessage}）");
        Assert.DoesNotContain(WeldDefect.MissingWeld, result.Defects);
        Assert.DoesNotContain(WeldDefect.Misaligned, result.Defects);

        var dx = result.WeldCenterSourcePx.X - vx;
        var dy = result.WeldCenterSourcePx.Y - vy;
        var deviation = Math.Sqrt(dx * dx + dy * dy);
        Assert.True(deviation <= 30,
            $"{scene}: 焊环圆心偏离厂商圆 {deviation:F1} 原始像素（上限 30）");
        Assert.InRange(result.WeldRadiusSourcePx / vr, 0.92, 1.10);
        // 第五路径的产出必须自带精修量出的支撑弧段，且必须达标
        Assert.True(result.WeldArcSupport >= _config.MinRingArcSupport,
            $"{scene}: 支撑弧段只有 {result.WeldArcSupport:P0}，未达门限");
    }

    /// <summary>在 tests/pp 下按"场景时间戳 + 文件名后缀"找那一帧（跨 SN 目录搜索）。</summary>
    private static string? FindSample(string scene, string suffix, string? sn = null)
    {
        var root = FindPpRoot();
        if (root is null)
            return null;

        return Directory.EnumerateFiles(root, "*.jpg", SearchOption.AllDirectories)
            .FirstOrDefault(f =>
                Path.GetFileName(f).StartsWith(scene, StringComparison.Ordinal) &&
                Path.GetFileName(f).Contains(suffix, StringComparison.Ordinal) &&
                (sn is null || Path.GetFileName(f).Contains(sn, StringComparison.Ordinal)));
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
