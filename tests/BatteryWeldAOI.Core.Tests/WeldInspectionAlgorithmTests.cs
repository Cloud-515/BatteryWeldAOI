using BatteryWeldAOI.Core.Models;
using BatteryWeldAOI.Core.Vision;
using OpenCvSharp;
using Xunit;

namespace BatteryWeldAOI.Core.Tests;

/// <summary>
/// 算法精度与缺陷判定的闭环测试：
/// 用 ImageGenerator 按已知真值渲染图像，再由 WeldInspectionAlgorithm 测量，
/// 验证「测量值 ≈ 真值」以及四类缺陷的检出。
/// </summary>
public class WeldInspectionAlgorithmTests : IDisposable
{
    // 公差取 0.5mm 而非默认的 1.0mm：正弦拟合模型只在"熔核中心落在孔洞内"时成立，
    // 而真机孔洞半径 ≈ 0.26×熔核半径 ≈ 1.0mm，即算法的**可测上限恰好等于默认公差**。
    // 用默认公差就构造不出"既可测、又超差"的样本——这个巧合本身就是标定风险，
    // 详见 RealisticRobustnessTests 的类注释。
    // 显式关闭工作分辨率归一化，让算法在配置的 1280×1024 上原生工作——本套合成夹具就是
    // 在那个尺度下标定的（形态学核被绝对上限冻结成 15/21/15/9px）。理由与取舍
    // 见 RealisticRobustnessTests 的同名注释。
    private readonly InspectionConfig _config = new()
    {
        OffsetToleranceMm = 0.5,
        MaxWorkingBase = 0,
        // 同 RealisticRobustnessTests：支撑弧段门限随成像条件标定，本夹具按自己的历史值 0.50。
        MinRingArcSupport = 0.50,
        // 本类（及 RealisticRobustnessTests）用**合成夹具**，须关掉"孔锚精修"
        // （InspectionConfig.WeldRingRefineEdgeFraction）：夹具渲染的熔核半径是 0.18~0.22×短边，
        // 本来就落在**本产品实测族值带（0.142~0.147）之外**，而精修的搜索带上界锚在
        // 1.15×族值中位——夹具上的真边界因此落在带外，每条射线都顶在带的边缘，
        // 精修会把圆心拉回孔心，把夹具的"真实偏移"抹平成 0（实测 seed 1/3 的 OK 点偏移
        // 由 0.20mm 变成 -0.019mm，Misaligned 也因此检不出来）。
        // 夹具也**不模拟**精修要治的现象（镜面冲白把熔核一侧的纹理抹掉、以及熔核外侧
        // 那条 1.23× 半径的凹陷细暗圈），故在这套语料上关掉它既正确也不损失覆盖——
        // 精修本身由 tests/pp 真实帧的测试覆盖（PpSampleTests / RingCredibilityGateTests）。
        WeldRingRefineEdgeFraction = 0,
        // 同上：卡尺兜底的**有效性判据**也是随成像条件/产品标定的配方参数。
        // 本夹具的孔位端面是画出来的灰盘，`HoleLocator` 的三材质分层在这里本来就给不出候选
        // （一直走卡尺），而卡尺在本夹具上是准的——判据按现场 20MP 良品标定（见
        // InspectionConfig.CaliperMaxResidualRatio 的实测表），在夹具上会把正常的卡尺结果
        // 误判为"不可信"（实测 25 例里 7 例由"测得出"变成 HasError）。故本语料显式关闭，
        // 由 tests/pp 真实帧的用例覆盖该判据（HoleTruthTests / PpSampleTests）。
        CaliperMaxResidualRatio = double.MaxValue,
        CaliperMaxRadiusRatioOfFamily = double.MaxValue,
    };
    private readonly NinePointCalibration _calibration;
    private readonly ImageGenerator _generator;
    private readonly WeldInspectionAlgorithm _algorithm;

    public WeldInspectionAlgorithmTests()
    {
        // 纯缩放标定，方便数值断言
        var pairs = new List<(Point2d Pixel, Point2d Mm)>
        {
            (new Point2d(0, 0), new Point2d(0, 0)),
            (new Point2d(1000, 0), new Point2d(25, 0)),
            (new Point2d(0, 1000), new Point2d(0, 25)),
        };
        _calibration = new NinePointCalibration(pairs);
        _generator = new ImageGenerator(_config, _calibration);
        _algorithm = new WeldInspectionAlgorithm(_config, _calibration);
    }

    public static TheoryData<int> Seeds => new() { 1, 2, 3, 4, 5 };

    [Theory]
    [MemberData(nameof(Seeds))]
    public void OkPoint_MeasuresTrueOffset_Within0p05mm(int seed)
    {
        var (result, _) = Inspect(seed, defect: null, offsetXMm: 0.20, offsetYMm: -0.15);

        Assert.True(result.IsOk);
        // 实测误差 ≈1px(0.025mm)，容限 ±0.05mm（xUnit Equal(precision) 是舍入后比较，
        // 真值 -0.15 恰在舍入边界上会产生假失败，故用显式误差断言）
        Assert.True(Math.Abs(result.OffsetXmm - 0.20) < 0.05,
            $"X 偏移测量误差过大: 真值 0.20, 实测 {result.OffsetXmm:F3}");
        Assert.True(Math.Abs(result.OffsetYmm - (-0.15)) < 0.05,
            $"Y 偏移测量误差过大: 真值 -0.15, 实测 {result.OffsetYmm:F3}");
        Assert.True(result.OffsetDistanceMm < _config.OffsetToleranceMm);
    }

    [Theory]
    [MemberData(nameof(Seeds))]
    public void ZeroOffsetPoint_IsOk(int seed)
    {
        var (result, _) = Inspect(seed, defect: null, offsetXMm: 0, offsetYMm: 0);

        Assert.True(result.IsOk);
        // 零偏移点测量误差 ≤4px（公差 1.0mm 的 10%）
        Assert.True(result.OffsetDistanceMm < 0.10, $"零偏移点测得 {result.OffsetDistanceMm:F3}mm");
    }

    [Theory]
    [MemberData(nameof(Seeds))]
    public void MisalignedPoint_IsDetected_AndDistanceAccurate(int seed)
    {
        // 偏移量必须**同时**满足：大于公差（否则判不出焊偏）、小于可测上限（否则正弦模型失效）。
        // 0.70mm 对 0.5mm 公差是 1.4 倍余量；对可测上限（≈0.8×孔半径 ≈0.8mm）也留有余量。
        // 这个区间很窄，是真实的物理限制而非测试妥协——见 InspectionConfig.OffsetToleranceMm。
        var (result, _) = Inspect(seed, WeldDefect.Misaligned, offsetXMm: 0.70, offsetYMm: 0);

        Assert.False(result.IsOk);
        Assert.Contains(WeldDefect.Misaligned, result.Defects);
        // 偏移量越大，正弦模型漏掉的二次谐波残差 D²/(4q) 越大，故容限取 0.05mm（≈2px）
        Assert.True(Math.Abs(result.OffsetDistanceMm - 0.70) < 0.05,
            $"焊偏量测量误差过大: 真值 0.70, 实测 {result.OffsetDistanceMm:F3}");
    }

    [Theory]
    [MemberData(nameof(Seeds))]
    public void MissingWeld_IsDetected(int seed)
    {
        var (result, _) = Inspect(seed, WeldDefect.MissingWeld, offsetXMm: 0.1, offsetYMm: 0.1);

        Assert.False(result.IsOk);
        Assert.Contains(WeldDefect.MissingWeld, result.Defects);
        Assert.DoesNotContain(WeldDefect.Misaligned, result.Defects);
    }

    [Theory]
    [MemberData(nameof(Seeds))]
    public void Blowout_IsDetected(int seed)
    {
        var (result, _) = Inspect(seed, WeldDefect.Blowout, offsetXMm: 0.2, offsetYMm: 0.1);

        Assert.False(result.IsOk);
        Assert.Contains(WeldDefect.Blowout, result.Defects);
    }

    [Theory]
    [MemberData(nameof(Seeds))]
    public void Spatter_IsDetected(int seed)
    {
        var (result, _) = Inspect(seed, WeldDefect.Spatter, offsetXMm: 0.2, offsetYMm: 0.1);

        Assert.False(result.IsOk);
        Assert.Contains(WeldDefect.Spatter, result.Defects);
    }

    [Fact]
    public void Annotate_ReturnsAnnotatedImage()
    {
        var (result, image) = Inspect(99, defect: null, offsetXMm: 0.1, offsetYMm: 0);
        using (image)
        using (var annotated = _algorithm.Annotate(image, result))
        {
            Assert.Equal(_config.ImageWidth, annotated.Width);
            Assert.Equal(_config.ImageHeight, annotated.Height);
        }
    }

    private (PointInspectionResult Result, Mat Image) Inspect(int seed, WeldDefect? defect,
        double offsetXMm, double offsetYMm)
    {
        var scenario = new PointScenario
        {
            Point = new WeldPoint { Index = 0, Name = "TEST" },
            Defect = defect,
            TrueOffsetXmm = offsetXMm,
            TrueOffsetYmm = offsetYMm
        };

        var image = _generator.Render(scenario, new Random(seed));
        var result = _algorithm.Inspect(image, scenario.Point);
        return (result, image);
    }

    public void Dispose()
    {
        // Mat 资源在测试内释放；无需额外清理
    }
}
