namespace BatteryWeldAOI.Core.Tests;

using BatteryWeldAOI.Core.Models;
using BatteryWeldAOI.Core.Vision;
using OpenCvSharp;
using Xunit;
using Xunit.Abstractions;

/// <summary>
/// 红圈（焊环外圈）**边界判据**的现场回归——钉住用户 2026-09-18 报的
/// "有细圆时红圈被细圆带偏、圆心往那边走、圈也变大"。
///
/// ## 被钉住的失效形态（两帧，各代表一种细圈污染）
///
/// 1. **细圈与熔核并块**（`SN_...0067` 第 0/1 帧，用户截图 Img000/Img001）：
///    环形光源在工件上的反射细圈与熔核的纹理掩码连成一片，`Detect` 给出的种子半径
///    冲到 579.9 / 542.1（真值约 480）。旧版"孔锚精修"以**孔心**为射线原点，
///    把圆心钉回了孔心（用户实测圆心差 34px、半径 +31px）；中间版改成以熔核自身圆心为原点后
///    半径仍偏大，因为"带内梯度最大处"落在过渡带内部、还会被**真边界外侧**的碎斑/细圈抓走。
/// 2. **细圈被填洞成实心盘**（`SN_...0002` 第 10 帧，用户截图 Img010）：
///    `Detect` 直接把细圈内部填成实心盘，种子半径 **654.7**（真值约 486），
///    而且 654.7/3648 = 0.1795 &lt; `RingRadiusFamilyMaxRatio`(0.18)，**连兜底路径都没被触发**，
///    红圈整圈套在细圆上。旧版精修在 ±22% 的带里量不到真边界（0.78×654.7 = 511 &gt; 486），
///    于是原样返回 654.7。
///
/// ## 修法与判据
///
/// 精修的射线原点改为**熔核自身圆心**，边界半径取"径向 |∇I| 中位剖面里**第一段**显著平台的外端"
/// ——即"纹理最后出现在哪里"，而不是"梯度最大在哪里"。实测两帧的中位剖面：
/// 真熔核外沿是一段 46~80 的平台终止于 r≈467，其外是**恒 0 的平滑区**；
/// 第 2 帧在 r≈640 另有一根中位 **342** 的孤立尖峰（细圈），平台判据不会走到那里去。
/// 详见 `InspectionConfig.WeldRingRefineEdgeFraction`。
///
/// 断言里的期望值是**实测值**（原图像素），并已在同一帧上与厂商 `.vdb` 的圆定位、
/// 以及与独立的"粗糙区外边界 RANSAC 圆拟合"三方交叉核对过（见 FIXPLAN 本轮记录）。
/// 样本缺失时跳过（与 PpSampleTests / RingCredibilityGateTests 一致）。
/// </summary>
public sealed class WeldRingBoundaryTests
{
    private readonly ITestOutputHelper _output;
    private readonly InspectionConfig _config = new();
    private readonly WeldInspectionAlgorithm _algorithm;

    public WeldRingBoundaryTests(ITestOutputHelper output)
    {
        _output = output;
        _algorithm = new WeldInspectionAlgorithm(_config,
            NinePointCalibration.PureScale(_config.MmPerPixel));
    }

    /// <summary>
    /// 细圈与熔核并块的两帧（用户报的 Img000 / Img001）。红圈必须贴住熔核外沿，
    /// 不许被细圈拉到 540~660 那一档去。
    ///
    /// 期望圆心：**Img000 用用户 2026-09-18 手绘的蓝圈**（他从截图里圈出"正确的范围"，
    /// 反解到原图坐标是 (2632.2, 1763.0) R=471.9）；Img001 用户未标注，用厂商 `.vdb`。
    /// 需要注意这两条基准在 Img000 上**本身差 34px**（厂商 (2654.4,1743.0)）——
    /// 本帧恰好是两者分歧最大的一帧，因此断言对"离基准多远"给出 40px 的容忍，
    /// 只钉住"不许偏 100px 以上"这个量级（旧版实测 105~133px）。
    /// </summary>
    [Theory]
    [InlineData("09-37-03 664", "_OK_480", 2632.2, 1763.0, 471.9)]   // 用户截图 Img000，用户手绘真值
    [InlineData("09-37-05 318", "_OK_483", 2671.8, 1797.0, 482.5)]   // 用户截图 Img001，厂商 .vdb
    public void ThinRingMergedFrame_RedCircleHugsNugget(string scene, string suffix,
        double expectedX, double expectedY, double expectedRadius)
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
        var cx = result.WeldCenterSourcePx.X;
        var cy = result.WeldCenterSourcePx.Y;
        var r = result.WeldRadiusSourcePx;
        var delta = Math.Sqrt((cx - expectedX) * (cx - expectedX) + (cy - expectedY) * (cy - expectedY));
        _output.WriteLine($"{scene}: 判定={result.DefectTextZh}  圆心=({cx:F1},{cy:F1}) R={r:F1}  " +
                          $"支撑弧段={result.WeldArcSupport:P0}  与期望圆心差={delta:F1}px  " +
                          $"（期望 R≈{expectedRadius:F0}）");

        Assert.False(result.HasError, $"{scene}: 不该报测量失败（{result.ErrorMessage}）");
        Assert.True(r > 0, $"{scene}: 未定位到焊环");

        // 半径必须落在真熔核那一档。旧版实测 542~580（并块残留）、654.7（填洞成实心盘），
        // 而真熔核外沿实测 479~486——本带上界 515 把两档污染都拦在外面。
        Assert.InRange(r, 450, 515);

        // 圆心必须与厂商 `.vdb` 的圆定位（已与独立纹理边界拟合交叉核对）一致到 40px 以内。
        // 旧版（孔锚精修）实测 105~133px。
        Assert.True(delta <= 40,
            $"{scene}: 红圈圆心偏离真熔核中心 {delta:F0}px（上限 40）——" +
            "旧版这里是 105~133px，正是用户报的'红圈被细圆带偏'");
    }

    /// <summary>
    /// 细圈被填洞成实心盘的那一帧（用户截图 Img010，`SN_...0002` 第 10 帧）。
    /// 种子半径 654.7 = 0.1795×短边，恰好从族值带上界 0.18 下面溜过去，
    /// 兜底路径不触发；修好它的责任完全在精修的边界判据上。
    /// </summary>
    [Fact]
    public void FilledReflectionRingFrame_DoesNotLockOntoTheRing()
    {
        var file = FindSample("19-22-22 370", "_OK_285");
        if (file is null)
        {
            _output.WriteLine("未找到现场样本 tests/pp，跳过。");
            return;
        }

        using var image = Cv2.ImRead(file, ImreadModes.Color);
        Assert.False(image.Empty(), $"{Path.GetFileName(file)}: 读取失败");

        var result = _algorithm.Inspect(image, new WeldPoint { Index = 0, Name = "OK_285" });
        var processedBase = Math.Max(1, Math.Min(result.ProcessedSizePx.Height, result.ProcessedSizePx.Width));
        var ratio = result.WeldRadiusPx / processedBase;
        _output.WriteLine($"19-22-22 370: 判定={result.DefectTextZh}  环r/工作短边={ratio:F4}  " +
                          $"半径(原图)={result.WeldRadiusSourcePx:F1}px  支撑弧段={result.WeldArcSupport:P0}");

        Assert.False(result.HasError, $"不该报测量失败（{result.ErrorMessage}）");
        // 填洞实心盘的伪半径实测 0.1795×短边；真熔核 0.1332。带的上界 0.16 落在两者之间。
        Assert.InRange(ratio, 0.110, 0.160);
    }

    /// <summary>
    /// **第二起点**要救的那两帧（用户 2026-09-18 第二次反馈的 Img000/Img001，
    /// `SN_...0003` 第 0/1 帧，OK_933 / OK_936）。
    ///
    /// 这一对与上面两帧的失效机理不同：细圈**没有**被并块，但 `Detect` 给出的**种子圆心本身**
    /// 被细圈/外侧碎斑带撑歪了 39~45px；自锚迭代从歪的圆心出发，会收敛到"碎斑带的外沿"上
    /// （实测第 0 帧 R=527.2、圆心偏 41px）。同一 SN 其余 20 帧的熔核半径都在 483~493
    /// （刚体工件、同一个焊点），所以第 0 帧的 527 就是错的。
    ///
    /// 修法：把**孔心**作为第二起点再跑一遍，两遍取"边界更像圆"（固定容差下内点占比更高）的那个，
    /// 且只接受半径落回族值带、支撑弧段达标的第二结果——实测两个坏帧的第二结果支撑弧段
    /// 0.94~0.95 明显高于第一结果的 0.70~0.83，而偶发收敛到孔洞尺度（半径比 0.03~0.06）
    /// 的伪结果被族值带挡住。
    /// </summary>
    [Theory]
    [InlineData("16-48-48 544", "_OK_933", 489.0, 45.0)]   // Img000：修前 R=527.2、圆心偏 41px
    [InlineData("16-48-50 168", "_OK_936", 494.3, 20.0)]   // Img001：修前圆心偏 42.8px
    public void SeedCenterBiasedBySpeckleBand_SecondStartRecoversIt(string scene, string suffix,
        double expectedRadius, double centerTolerancePx)
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
        var r = result.WeldRadiusSourcePx;
        _output.WriteLine($"{scene}: 判定={result.DefectTextZh}  R={r:F1}（同 SN 其余 20 帧 483~493）  " +
                          $"支撑弧段={result.WeldArcSupport:P0}  期望 R≈{expectedRadius:F0}");

        Assert.False(result.HasError, $"{scene}: 不该报测量失败（{result.ErrorMessage}）");

        // 半径必须与"同一 SN 其余 20 帧"一致（刚体工件的熔核尺寸是常数）。
        // 修前实测 527.2（+8.5%）、476.0，本带 [465,510] 把前者挡在外面。
        Assert.InRange(r, 465, 510);

        // 圆心：以厂商 `.vdb`（已与独立纹理边界拟合交叉核对到中位 4px）为参照
        var vx = FindVendorCenter(scene);
        if (vx is { } v)
        {
            var d = Math.Sqrt((result.WeldCenterSourcePx.X - v.X) * (result.WeldCenterSourcePx.X - v.X) +
                              (result.WeldCenterSourcePx.Y - v.Y) * (result.WeldCenterSourcePx.Y - v.Y));
            _output.WriteLine($"   圆心=({result.WeldCenterSourcePx.X:F1},{result.WeldCenterSourcePx.Y:F1})  " +
                              $"厂商=({v.X:F1},{v.Y:F1})  差={d:F1}px");
            Assert.True(d <= centerTolerancePx,
                $"{scene}: 红圈圆心偏离厂商圆 {d:F0}px（上限 {centerTolerancePx:F0}）");
        }
    }

    /// <summary>按场景时间戳在同一语料里找厂商 `.vdb` 的圆定位圆心（找不到返回 null）。</summary>
    private static System.Drawing.PointF? FindVendorCenter(string scene)
    {
        var dir = new DirectoryInfo(AppContext.BaseDirectory);
        while (dir is not null)
        {
            var root = Path.Combine(dir.FullName, "tests", "pp");
            if (Directory.Exists(root))
            {
                var vdb = Directory.EnumerateFiles(root, "*.vdb", SearchOption.AllDirectories)
                    .FirstOrDefault(f => Path.GetFileName(f).StartsWith(scene, StringComparison.Ordinal));
                if (vdb is not null && BatteryWeldAOI.Core.Diagnostics.VendorVdbReader
                        .TryRead(vdb, out var rec, out _) && rec is not null)
                    return new System.Drawing.PointF((float)rec.LocateCenterX, (float)rec.LocateCenterY);
                return null;
            }
            dir = dir.Parent;
        }
        return null;
    }

    private static string? FindSample(string scene, string suffix)
    {
        var dir = new DirectoryInfo(AppContext.BaseDirectory);
        while (dir is not null)
        {
            var root = Path.Combine(dir.FullName, "tests", "pp");
            if (Directory.Exists(root))
            {
                return Directory.EnumerateFiles(root, "*.jpg", SearchOption.AllDirectories)
                    .FirstOrDefault(f =>
                        Path.GetFileName(f).StartsWith(scene, StringComparison.Ordinal) &&
                        Path.GetFileName(f).Contains(suffix, StringComparison.Ordinal));
            }
            dir = dir.Parent;
        }
        return null;
    }
}
