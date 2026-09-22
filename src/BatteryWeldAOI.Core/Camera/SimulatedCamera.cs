namespace BatteryWeldAOI.Core.Camera;

using BatteryWeldAOI.Core.Models;
using BatteryWeldAOI.Core.Vision;
using OpenCvSharp;

/// <summary>
/// 模拟相机：根据点位剧本用 <see cref="ImageGenerator"/> 渲染图像，
/// 并模拟曝光耗时。每个点位使用独立随机种子，保证同一批次可复现。
/// </summary>
public sealed class SimulatedCamera : ICameraGrabber
{
    private readonly ImageGenerator _generator;
    private readonly IReadOnlyList<PointScenario> _scenarios;
    private readonly int _baseSeed;
    private readonly int _exposureMs;

    public SimulatedCamera(ImageGenerator generator, IReadOnlyList<PointScenario> scenarios,
        int baseSeed = 42, int exposureMs = 8)
    {
        _generator = generator;
        _scenarios = scenarios;
        _baseSeed = baseSeed;
        _exposureMs = exposureMs;
    }

    public string Name => "SimulatedCamera(500W@GlobalShutter)";

    public Mat TriggerGrab(int pointIndex)
    {
        if (pointIndex < 0 || pointIndex >= _scenarios.Count)
            throw new ArgumentOutOfRangeException(nameof(pointIndex), "点位序号超出剧本范围");

        // 模拟曝光与传输耗时
        Thread.Sleep(_exposureMs);

        var scenario = _scenarios[pointIndex];
        // 确定性种子：同批次可复现（HashCode.Combine 每进程随机，不利于回归）
        var rng = new Random(unchecked(_baseSeed * 397 + pointIndex * 1009 + 1));
        return _generator.Render(scenario, rng);
    }

    public void Dispose()
    {
    }
}
