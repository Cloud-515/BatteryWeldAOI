namespace MakeDemoVideo;

using BatteryWeldAOI.Core.Configuration;
using BatteryWeldAOI.Core.Models;
using BatteryWeldAOI.Core.Planning;
using BatteryWeldAOI.Core.Vision;
using OpenCvSharp;

/// <summary>
/// 演示视频生成工具：用 ImageGenerator 按剧本合成焊点图像并编码为视频文件，
/// 供"视频文件"检测模式离线验证（无需相机与 PLC）。
/// 用法: MakeDemoVideo [输出路径=output\demo_video.avi] [帧数=24] [种子=42]
/// </summary>
internal static class Program
{
    private static int Main(string[] args)
    {
        var outPath = args.Length > 0 ? args[0]
            : Path.Combine(AppContext.BaseDirectory, "output", "demo_video.avi");
        var frames = args.Length > 1 && int.TryParse(args[1], out var n) ? n : 24;
        var seed = args.Length > 2 && int.TryParse(args[2], out var s) ? s : 42;

        var config = new AppConfig();
        var (points, scenarios) = DemoPlanBuilder.BuildPlan(frames, seed, config.Inspection);
        var calibration = DemoPlanBuilder.BuildCalibration(config.Inspection);
        var generator = new ImageGenerator(config.Inspection, calibration);

        var dir = Path.GetDirectoryName(Path.GetFullPath(outPath));
        Directory.CreateDirectory(dir!);

        using var writer = new VideoWriter(outPath, FourCC.FromString("mp4v"), 10,
            new Size(config.Inspection.ImageWidth, config.Inspection.ImageHeight), isColor: true);
        if (!writer.IsOpened())
        {
            Console.Error.WriteLine("[错误] 视频编码器初始化失败（mp4v）");
            return 1;
        }

        for (var i = 0; i < frames; i++)
        {
            var rng = new Random(unchecked(seed * 397 + i * 1009 + 1));
            using var frame = generator.Render(scenarios[i], rng);
            writer.Write(frame);
        }

        Console.WriteLine($"已生成演示视频: {Path.GetFullPath(outPath)}");
        Console.WriteLine($"帧数: {frames}, 分辨率: {config.Inspection.ImageWidth}x{config.Inspection.ImageHeight}, " +
                          "缺陷剧本与仿真模式一致（3=焊偏 7=漏焊 11=炸焊 15=飞溅）");
        return 0;
    }
}
