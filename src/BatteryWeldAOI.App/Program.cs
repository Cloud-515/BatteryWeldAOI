namespace BatteryWeldAOI.App;

using BatteryWeldAOI.Core.Camera;
using BatteryWeldAOI.Core.Communication;
using BatteryWeldAOI.Core.Configuration;
using BatteryWeldAOI.Core.Models;
using BatteryWeldAOI.Core.Orchestration;
using BatteryWeldAOI.Core.Pipeline;
using BatteryWeldAOI.Core.Planning;
using BatteryWeldAOI.Core.Reporting;
using BatteryWeldAOI.Core.StateMachine;
using BatteryWeldAOI.Core.Vision;
using OpenCvSharp;
using System.Text;

/// <summary>
/// 电池包焊后 AOI 检测系统 —— 控制台入口。
/// 完整跑通「上位机 <-> TCP PLC 模拟器 <-> 模拟相机 <-> 视觉流水线 <-> CSV 报表」链路。
/// 用法: BatteryWeldAOI.App [点位数=config] [随机种子=config]
/// 配置文件: 程序目录下 config.json（不存在时使用默认值并自动生成）。
/// </summary>
internal static class Program
{
    private static readonly object ConsoleLock = new();

    private static async Task<int> Main(string[] args)
    {
        Console.OutputEncoding = Encoding.UTF8;
        var configPath = Path.Combine(AppContext.BaseDirectory, "config.json");
        var config = AppConfig.LoadOrDefault(configPath);

        // 命令行参数覆盖配置文件
        if (args.Length > 0 && int.TryParse(args[0], out var n)) config.PointCount = n;
        if (args.Length > 1 && int.TryParse(args[1], out var s)) config.RandomSeed = s;
        config.Save(configPath); // 首次运行落盘默认配置，便于用户修改

        Console.WriteLine("=====================================================");
        Console.WriteLine("  BatteryWeldAOI - 电池包焊后视觉检测系统 (仿真演示)");
        Console.WriteLine("  Stop-and-Go | 状态机 | TCP-PLC | 生产者-消费者流水线");
        Console.WriteLine("=====================================================");
        Console.WriteLine($"点位数: {config.PointCount}, 随机种子: {config.RandomSeed}, " +
                          $"配置文件: {configPath}");
        Console.WriteLine();

        var inspection = config.Inspection;
        var (points, scenarios) = DemoPlanBuilder.BuildPlan(config.PointCount, config.RandomSeed, inspection);
        var calibration = DemoPlanBuilder.BuildCalibration(inspection);

        // ---- 硬件层：PLC 模拟器（下位机）+ TCP 客户端 + 模拟相机 ----
        using var plcSimulator = new PlcSimulator(port: 0, moveTimeMs: inspection.PlcMoveTimeMs);
        plcSimulator.Log += msg => Log(msg);
        plcSimulator.Start();

        using var plc = new TcpPlcClient("127.0.0.1", plcSimulator.Port,
            inspection.ArrivalPollIntervalMs, inspection.ArrivalTimeoutMs, inspection.PlcResponseTimeoutMs);
        var generator = new ImageGenerator(inspection, calibration);
        using var camera = new SimulatedCamera(generator, scenarios, baseSeed: config.RandomSeed);

        // ---- 视觉层：算法 + 流水线 ----
        var algorithm = new WeldInspectionAlgorithm(inspection, calibration);
        using var pipeline = new ImagePipeline(algorithm, workerCount: config.WorkerCount);

        var outputDir = Path.Combine(AppContext.BaseDirectory, config.OutputDirectory);
        var imageDir = Path.Combine(outputDir, "images");
        Directory.CreateDirectory(imageDir);

        pipeline.ResultProcessed += (result, annotated) =>
        {
            lock (ConsoleLock)
            {
                Console.WriteLine($"  [视觉] {result}");
            }
            if (config.SaveAnnotatedImages)
            {
                var imgPath = Path.Combine(imageDir,
                    $"point_{result.Point.Index:D3}_{(result.IsOk ? "ok" : "ng")}.png");
                Cv2.ImWrite(imgPath, annotated);
            }
        };

        // ---- 流程层：状态机 + 控制器 ----
        var stateMachine = new MachineStateMachine();
        stateMachine.StateChanged += (from, to) => Log($"  [状态机] {from} -> {to}");

        var controller = new InspectionController(plc, camera, pipeline, stateMachine, points);
        controller.Log += msg => Log(msg);

        Console.WriteLine($"共 {points.Count} 个焊点, 公差 ±{inspection.OffsetToleranceMm}mm, " +
                          $"像素当量 {inspection.MmPerPixel}mm/px");
        Console.WriteLine("-----------------------------------------------------");

        try
        {
            var report = await controller.RunAsync(CancellationToken.None);

            Console.WriteLine("-----------------------------------------------------");
            Console.WriteLine(report.WasCancelled ? "检测被中止, 部分结果:" : "整包检测完成:");
            Console.WriteLine($"  总点位: {report.TotalCount} | OK: {report.OkCount} | NG: {report.NgCount}" +
                              (report.ErrorCount > 0 ? $" | 异常: {report.ErrorCount}" : string.Empty));
            if (report.UploadFailedCount > 0)
                Console.WriteLine($"  [警告] {report.UploadFailedCount} 个点位结果上传 PLC 失败");
            if (report.DefectSummary.Count > 0)
            {
                var detail = string.Join(", ",
                    report.DefectSummary.Select(kv => $"{WeldDefectText.Of(kv.Key)} x{kv.Value}"));
                Console.WriteLine($"  缺陷统计: {detail}");
            }
            if (report.TotalCount > 0)
                Console.WriteLine($"  整包节拍: {report.CycleTimeSec:F2}s " +
                                  $"({report.CycleTimeSec * 1000 / Math.Max(1, report.TotalCount):F0}ms/点)");
            if (report.OkCount > 0)
                Console.WriteLine($"  合格点最大偏移: {report.Results.Where(r => r.IsOk).Max(r => r.OffsetDistanceMm):F3}mm");

            var reportPath = Path.Combine(outputDir, $"report_{DateTime.Now:yyyyMMdd_HHmmss}.csv");
            CsvReportWriter.Write(report, reportPath);
            Console.WriteLine();
            Console.WriteLine($"报表已导出: {reportPath}");
            if (config.SaveAnnotatedImages)
                Console.WriteLine($"标注图像目录: {imageDir}");
            if (report.WasCancelled)
                return 1; // 中止
            return report.NgCount == 0 && report.ErrorCount == 0 ? 0 : 2; // 0=全OK, 2=存在NG/异常
        }
        catch (Exception ex)
        {
            Console.WriteLine();
            Console.WriteLine($"[异常] 检测中止: {ex.Message}");
            Console.WriteLine($"状态机当前状态: {stateMachine.Current}");
            return 1;
        }
    }

    private static void Log(string msg)
    {
        lock (ConsoleLock)
        {
            Console.WriteLine(msg);
        }
    }
}
