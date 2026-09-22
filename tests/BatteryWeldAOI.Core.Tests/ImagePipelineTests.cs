using BatteryWeldAOI.Core.Models;
using BatteryWeldAOI.Core.Pipeline;
using BatteryWeldAOI.Core.Planning;
using BatteryWeldAOI.Core.Vision;
using OpenCvSharp;
using Xunit;

namespace BatteryWeldAOI.Core.Tests;

/// <summary>
/// 流水线健壮性测试：单点算法异常必须被隔离为错误结果，
/// 不得击穿 worker 或影响其他点位的正常检测。
/// </summary>
public class ImagePipelineTests
{
    private static (WeldInspectionAlgorithm Algorithm, ImageGenerator Generator,
        NinePointCalibration Calibration) Build()
    {
        // 本类用**合成夹具**（ImageGenerator），与 WeldInspectionAlgorithmTests 同口径：
        // 卡尺兜底的有效性判据是按现场 20MP 良品标定的配方参数（见 InspectionConfig.CaliperMaxResidualRatio），
        // 在合成夹具上孔位本来就走卡尺且卡尺是准的 → 判据会把正常点位误判为 HasError，
        // 而本类断言的是"正常点位 IsOk"，故显式关闭。该判据由 tests/pp 的真实帧用例覆盖
        // （HoleTruthTests / PpSampleTests）。
        var config = new InspectionConfig
        {
            CaliperMaxResidualRatio = double.MaxValue,
            CaliperMaxRadiusRatioOfFamily = double.MaxValue,
        };
        var calibration = DemoPlanBuilder.BuildCalibration(config);
        return (new WeldInspectionAlgorithm(config, calibration),
            new ImageGenerator(config, calibration), calibration);
    }

    [Fact]
    public async Task FaultyImage_IsIsolatedAsErrorResult_AndWorkerSurvives()
    {
        var (algorithm, generator, _) = Build();
        using var pipeline = new ImagePipeline(algorithm, workerCount: 1);

        // 剧本只需要配置来定"焊偏点的真值距离"（公差 × 1.3，见 DemoPlanBuilder.BuildPlan），
        // 与 Build() 里那份配置无关，故此处按默认值另建一份。
        var (points, scenarios) = DemoPlanBuilder.BuildPlan(2, seed: 42, new InspectionConfig());

        var results = new List<PointInspectionResult>();
        var sync = new TaskCompletionSource();
        var received = 0;
        pipeline.ResultProcessed += (result, annotated) =>
        {
            results.Add(result);
            annotated.Dispose(); // 模拟订阅方处理
            if (Interlocked.Increment(ref received) == 2)
                sync.TrySetResult();
        };

        // 点位0：已释放的 Mat（模拟取图/数据损坏）→ 算法必然抛异常
        var dead = new Mat(64, 64, MatType.CV_8UC3, Scalar.Black);
        dead.Dispose();
        pipeline.Enqueue(points[0], dead);

        // 点位1：正常合成图像 → 应正常检测
        var good = generator.Render(scenarios[1], new Random(123));
        pipeline.Enqueue(points[1], good);

        pipeline.CompleteAdding();
        using var timeoutCts = new CancellationTokenSource(TimeSpan.FromSeconds(30));
        try
        {
            await sync.Task.WaitAsync(timeoutCts.Token);
        }
        catch (TimeoutException)
        {
            Assert.Fail("流水线未在超时内产出两个结果, worker 可能已被单点异常击穿");
        }
        await pipeline.Completion;

        var errorResult = results.Single(r => r.Point.Index == 0);
        Assert.True(errorResult.HasError, "坏图点位应标记 HasError");
        Assert.False(errorResult.IsOk);
        Assert.NotNull(errorResult.ErrorMessage);

        var normalResult = results.Single(r => r.Point.Index == 1);
        Assert.False(normalResult.HasError, "正常点位不应受前一个坏图影响");
        Assert.True(normalResult.IsOk);
    }
}
