using BatteryWeldAOI.Core.Models;
using BatteryWeldAOI.Core.Vision;
using OpenCvSharp;
using Xunit;

namespace BatteryWeldAOI.Core.Tests;

/// <summary>
/// 真实成像鲁棒性闭环测试：
/// 用 RealisticImageGenerator 注入随机曝光/光照梯度/渐晕/反照率/极性/脏污/噪声，
/// 验证算法对四类缺陷的判定与偏移测量不随光照条件漂移。
/// 场景种子与光照种子独立，"同场景不同光照"结果必须一致（不变性测试）。
///
/// 关于本组测试的偏移公差取 0.5mm（而非配置默认的 1.0mm）：
/// 正弦拟合模型要求**熔核中心落在孔洞之内**（射线从熔核中心出发，若起点在孔外，
/// 背向孔的那一半射线根本打不到孔壁，拟合退化）。按真机实测比例，
/// 孔洞半径 ≈ 0.26 × 熔核半径 ≈ 1.0mm，即算法可测偏移上限约 0.8mm——
/// 与默认公差 1.0mm 已经重合。因此这里用 0.5mm 公差把测试放在模型的有效区间内，
/// 同时把"默认公差偏大"这一标定风险暴露出来（见 InspectionConfig.OffsetToleranceMm）。
/// </summary>
public class RealisticRobustnessTests
{
    // 本组测试**显式关闭工作分辨率归一化**（MaxWorkingBase = 0），即让算法在配置的
    // 1280×1024 上原生工作——那是本套合成夹具被标定时的情形（各形态学核被绝对上限冻结成
    // 15/21/15/9px 的固定值，与 base 无关）。这不是生产的默认路径：生产默认
    // MaxWorkingBase = 460（见 InspectionConfig 的说明），20MP 相机靠它才可用。
    // 为什么本套夹具不改到 460 去测生产路径：生成器用**绝对像素尺寸**画飞溅颗粒/反光斑/
    // 脏污（WeldSceneRenderer.DrawSpatter 的 3~15px 椭圆），这些特征只在 1024 边下与
    // 算法参数匹配；实测把夹具整体搬到 460 会造成 34 处失配（12→34），
    // 而真正的生产路径由 RealSampleTests（现场 28 张）与 PpSampleTests（20MP 24 帧）覆盖。
    // 要让本套夹具在 460 下可用，需先把生成器改成尺度相对（与算法同口径），这是独立的一项工作。
    private readonly InspectionConfig _config = new()
    {
        OffsetToleranceMm = 0.5,
        MaxWorkingBase = 0,
        // 边界支撑弧段门限是**随成像条件标定的配方参数**（见 InspectionConfig.MinRingArcSupport）。
        // 生产默认 0.82 是在现场 20MP 图（5472×3648，归一化到 460）上标定的：良品 40 帧 0.93~1.00、
        // 厂商 NG 大熔核 1.00，而"环形光源反射细圈与熔核并块"的污染帧只有 0.51~0.94
        // （圆心偏差最大的 4 帧 0.64/0.68/0.78/0.81）。
        // 本夹具是**另一套成像**：熔核由散点鱼鳞斑绘制，边界天生不如真机的连续粗糙区完整；
        // 且本类用 MaxWorkingBase=0 关闭归一化（合成图原生即工作分辨率）。故按自己的历史值 0.50 标定。
        // 现场那套门限的守卫测试见 FIXPLAN 的 T1.9（用 tests/pp 的真实帧）。
        MinRingArcSupport = 0.50,
        // 与 WeldInspectionAlgorithmTests 同口径：合成夹具的熔核半径（0.18~0.22×短边）落在
        // 本产品实测族值带（0.142~0.147）之外，而"孔锚精修"的搜索带上界锚在 1.15×族值中位，
        // 会让每条射线顶在带的边缘、把圆心拉回孔心，从而抹平夹具注入的真实偏移。
        // 夹具也不模拟精修要治的成像现象（镜面冲白抹掉熔核一侧的纹理、熔核外侧的凹陷细暗圈），
        // 故本套语料显式关闭精修；精修由 tests/pp 真实帧的测试覆盖。
        WeldRingRefineEdgeFraction = 0,
        // 同 WeldInspectionAlgorithmTests：卡尺兜底的有效性判据是按现场 20MP 良品标定的配方参数，
        // 本套合成语料上孔位本来就走卡尺且卡尺是准的，判据会把正常结果误判为"不可信"，
        // 故显式关闭；该判据由 tests/pp 真实帧的用例覆盖（HoleTruthTests / PpSampleTests）。
        CaliperMaxResidualRatio = double.MaxValue,
        CaliperMaxRadiusRatioOfFamily = double.MaxValue,
    };
    private readonly RealisticImageGenerator _generator;
    private readonly WeldInspectionAlgorithm _algorithm;

    public RealisticRobustnessTests()
    {
        // 纯缩放标定（0.025 mm/px），方便数值断言
        var pairs = new List<(Point2d Pixel, Point2d Mm)>
        {
            (new Point2d(0, 0), new Point2d(0, 0)),
            (new Point2d(1000, 0), new Point2d(25, 0)),
            (new Point2d(0, 1000), new Point2d(0, 25)),
        };
        var calibration = new NinePointCalibration(pairs);
        _generator = new RealisticImageGenerator(_config, calibration);
        _algorithm = new WeldInspectionAlgorithm(_config, calibration);
    }

    public static TheoryData<int> Seeds => new() { 1, 2, 3, 4, 5, 6 };

    [Theory]
    [MemberData(nameof(Seeds))]
    public void OkPoint_RandomIllumination_IsOk_AndMeasuresOffset(int seed)
    {
        var (result, image) = Inspect(seed, seed * 7 + 3, defect: null, 0.18, -0.12);
        using (image)
        {
            Assert.True(result.IsOk,
                $"OK 点误判: {result.DefectText}, D={result.OffsetDistanceMm:F3}mm");
            Assert.True(Math.Abs(result.OffsetXmm - 0.18) < 0.08,
                $"X 偏移测量误差过大: 真值 0.18, 实测 {result.OffsetXmm:F3}");
            Assert.True(Math.Abs(result.OffsetYmm - (-0.12)) < 0.08,
                $"Y 偏移测量误差过大: 真值 -0.12, 实测 {result.OffsetYmm:F3}");
        }
    }

    [Theory]
    [MemberData(nameof(Seeds))]
    public void BrightWeld_OkPoint_IsOk_AndMeasuresOffset(int seed)
    {
        var (result, image) = Inspect(seed, seed * 5 + 11, defect: null, 0.22, 0.10, forceBright: true);
        using (image)
        {
            Assert.True(result.IsOk,
                $"亮焊缝 OK 点误判: {result.DefectText}, D={result.OffsetDistanceMm:F3}mm");
            Assert.True(Math.Abs(result.OffsetXmm - 0.22) < 0.08,
                $"亮焊缝 X 偏移误差过大: 真值 0.22, 实测 {result.OffsetXmm:F3}");
            Assert.True(Math.Abs(result.OffsetYmm - 0.10) < 0.08,
                $"亮焊缝 Y 偏移误差过大: 真值 0.10, 实测 {result.OffsetYmm:F3}");
        }
    }

    [Theory]
    [MemberData(nameof(Seeds))]
    public void DarkWeld_Misaligned_IsDetected_AndDistanceAccurate(int seed)
    {
        var (result, image) = Inspect(seed, seed * 3 + 1, WeldDefect.Misaligned, 0.62, 0.20, forceBright: false);
        using (image)
        {
            Assert.False(result.IsOk);
            Assert.Contains(WeldDefect.Misaligned, result.Defects);
            Assert.True(Math.Abs(result.OffsetDistanceMm - Math.Sqrt(0.62 * 0.62 + 0.20 * 0.20)) < 0.10,
                $"焊偏距离测量误差过大: 实测 {result.OffsetDistanceMm:F3}mm");
        }
    }

    [Theory]
    [MemberData(nameof(Seeds))]
    public void BrightWeld_Misaligned_IsDetected(int seed)
    {
        var (result, image) = Inspect(seed, seed * 9 + 5, WeldDefect.Misaligned, -0.55, 0.25, forceBright: true);
        using (image)
        {
            Assert.False(result.IsOk);
            Assert.Contains(WeldDefect.Misaligned, result.Defects);
        }
    }

    /// <summary>
    /// 可测区间上沿的回归护栏：0.70mm 偏移（对孔半径 ≈0.7×，接近模型极限）。
    ///
    /// 这个用例盯的是卡尺搜索内边界 `PinSearchInnerRatioOfFamily`。盘沿最近处在 q − D；
    /// 搜索窗若从盘外开始（rStart &gt; q − D），那些射线整条落在熔核纹理上，
    /// 拟合会收敛到一个**自洽但错误**的圆，把 0.7~0.8mm 的焊偏测成 0.1mm 并**误判合格**。
    /// 该缺陷已修（基准由焊环半径改为灰圆族值半径后取 0.09 ≈ 2.0px），本用例防止它复发——
    /// 所以断言既查"判 NG"也查"量得准"，
    /// 只查前者是拦不住误判合格的。
    /// </summary>
    [Theory]
    [MemberData(nameof(Seeds))]
    public void Misaligned_NearMeasurableEnvelope_IsMeasuredAccurately(int seed)
    {
        var bright = seed % 2 == 0;
        var (result, image) = Inspect(seed, seed * 11 + 13, WeldDefect.Misaligned, 0.70, 0, forceBright: bright);
        using (image)
        {
            Assert.False(result.IsOk);
            Assert.Contains(WeldDefect.Misaligned, result.Defects);
            Assert.True(Math.Abs(result.OffsetDistanceMm - 0.70) < 0.10,
                $"近可测上限处偏移测偏: 真值 0.70, 实测 {result.OffsetDistanceMm:F3}mm");
        }
    }

    [Theory]
    [MemberData(nameof(Seeds))]
    public void MissingWeld_IsDetected_WithoutMisaligned(int seed)
    {
        var (result, image) = Inspect(seed, seed * 4 + 7, WeldDefect.MissingWeld, 0.10, 0.10);
        using (image)
        {
            Assert.False(result.IsOk);
            Assert.Contains(WeldDefect.MissingWeld, result.Defects);
            Assert.DoesNotContain(WeldDefect.Misaligned, result.Defects);
            Assert.DoesNotContain(WeldDefect.Spatter, result.Defects);
        }
    }

    [Theory]
    [MemberData(nameof(Seeds))]
    public void Blowout_IsDetected_BothPolarities(int seed)
    {
        // 奇数种子暗焊缝,偶数种子亮焊缝
        var bright = seed % 2 == 0;
        var (result, image) = Inspect(seed, seed * 6 + 2, WeldDefect.Blowout, 0.20, 0.10, forceBright: bright);
        using (image)
        {
            Assert.False(result.IsOk);
            Assert.Contains(WeldDefect.Blowout, result.Defects);
        }
    }

    [Theory]
    [MemberData(nameof(Seeds))]
    public void Spatter_IsDetected_BothPolarities(int seed)
    {
        var bright = seed % 2 == 0;
        var (result, image) = Inspect(seed, seed * 8 + 4, WeldDefect.Spatter, 0.20, 0.10, forceBright: bright);
        using (image)
        {
            Assert.False(result.IsOk);
            Assert.Contains(WeldDefect.Spatter, result.Defects);
        }
    }

    [Fact]
    public void SameScene_DifferentIllumination_GivesConsistentResult()
    {
        // 同一场景（几何/极性/缺陷完全一致），仅光照不同：
        // 判定结论必须一致，偏移测量差异应远小于公差
        foreach (var sceneSeed in new[] { 11, 12, 13, 14 })
        {
            var (r1, img1) = Inspect(sceneSeed, 100, defect: null, 0.20, -0.10);
            var (r2, img2) = Inspect(sceneSeed, 200, defect: null, 0.20, -0.10);
            using (img1)
            using (img2)
            {
                Assert.True(r1.IsOk, $"光照 A 下 OK 点误判: {r1.DefectText}");
                Assert.True(r2.IsOk, $"光照 B 下 OK 点误判: {r2.DefectText}");
                Assert.True(Math.Abs(r1.OffsetXmm - r2.OffsetXmm) < 0.05,
                    $"X 偏移随光照漂移: {r1.OffsetXmm:F3} vs {r2.OffsetXmm:F3}");
                Assert.True(Math.Abs(r1.OffsetYmm - r2.OffsetYmm) < 0.05,
                    $"Y 偏移随光照漂移: {r1.OffsetYmm:F3} vs {r2.OffsetYmm:F3}");
            }
        }
    }

    private (PointInspectionResult Result, Mat Image) Inspect(int sceneSeed, int illumSeed,
        WeldDefect? defect, double offsetXMm, double offsetYMm, bool? forceBright = null)
    {
        var scenario = new PointScenario
        {
            Point = new WeldPoint { Index = 0, Name = "R" },
            Defect = defect,
            TrueOffsetXmm = offsetXMm,
            TrueOffsetYmm = offsetYMm
        };
        var image = _generator.Render(scenario, sceneSeed, illumSeed, forceBright);
        var result = _algorithm.Inspect(image, scenario.Point);
        return (result, image);
    }
}
