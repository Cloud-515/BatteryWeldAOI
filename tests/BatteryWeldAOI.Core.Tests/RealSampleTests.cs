using BatteryWeldAOI.Core.Camera;
using BatteryWeldAOI.Core.Models;
using BatteryWeldAOI.Core.Vision;
using OpenCvSharp;
using Xunit;
using Xunit.Abstractions;

namespace BatteryWeldAOI.Core.Tests;

/// <summary>
/// 真机样本回归测试：把 tests/tp 下的 28 张现场取图逐张跑一遍完整算法。
///
/// 这是本项目最有价值的一组测试——旧版算法正是在这批图上全军覆没
/// （28 个点位全部误判 NG）。此后任何改动只要让真机样本回归，这里就会失败。
///
/// 样本集可能不随源码分发（体积原因）；找不到目录时本测试跳过而非失败。
/// </summary>
public class RealSampleTests
{
    private readonly ITestOutputHelper _output;
    private readonly InspectionConfig _config = new()
    {
        // tests/tp 是**旧相机**的小图（182~508px），与现场 20MP 图（5472×3648）不是同一套成像：
        // 其熔核纹理更细碎、边界支撑弧段天生偏低（实测本套 28 张里有 6 张低于生产默认门限 0.82），
        // 孔/环比也更大（实测 0.18~0.34，而现场 20MP 图是 0.214~0.300）。
        // 两个门限都是在现场 20MP 图上标定的（见 InspectionConfig.MinRingArcSupport /
        // MaxPinRadiusRatio），故本套按自己的历史值标定。
        MinRingArcSupport = 0.50,
        MaxPinRadiusRatio = 0.45,
        // 同理：卡尺兜底的有效性判据是在现场 20MP 图上标定的（InspectionConfig.CaliperMaxResidualRatio）。
        // 本套旧相机小图的孔/环比族值本就不同（0.18~0.34），该判据尚未在本语料上标定，
        // 故先显式关闭（**待办**：在 tests/tp 上标定后启用，见 FIXPLAN 第 5 节残留清单）。
        CaliperMaxResidualRatio = double.MaxValue,
        CaliperMaxRadiusRatioOfFamily = double.MaxValue,
    };
    private readonly WeldInspectionAlgorithm _algorithm;

    public RealSampleTests(ITestOutputHelper output)
    {
        _output = output;
        // 纯缩放标定（0.025 mm/px），与 RealisticRobustnessTests 保持一致
        var calibration = new NinePointCalibration(new List<(Point2d Pixel, Point2d Mm)>
        {
            (new Point2d(0, 0), new Point2d(0, 0)),
            (new Point2d(1000, 0), new Point2d(25, 0)),
            (new Point2d(0, 1000), new Point2d(0, 25)),
        });
        _algorithm = new WeldInspectionAlgorithm(_config, calibration);
    }

    [Fact]
    public void RealSamples_AllMeasuredOk()
    {
        var directory = FindSampleDirectory();
        if (directory is null)
        {
            _output.WriteLine("未找到 tests/tp 样本目录，跳过。");
            return;
        }

        var files = Directory.GetFiles(directory, "*.png")
            .OrderBy(f => int.TryParse(Path.GetFileNameWithoutExtension(f), out var n) ? n : int.MaxValue)
            .ToList();
        Assert.NotEmpty(files);

        var failures = new List<string>();
        var pinRatios = new List<double>();
        var ringRatios = new List<double>();
        var offsets = new List<double>();

        foreach (var file in files)
        {
            var name = Path.GetFileName(file);
            using var image = Cv2.ImRead(file, ImreadModes.Color);
            if (image.Empty())
            {
                failures.Add($"{name}: 读取失败");
                continue;
            }

            var point = new WeldPoint { Index = 0, Name = name };
            var result = _algorithm.Inspect(image, point);

            if (result.HasError)
            {
                failures.Add($"{name}: 测量失败 — {result.ErrorMessage}");
                continue;
            }
            if (!result.IsOk)
            {
                failures.Add($"{name}: 误判 NG — {result.DefectText}, D={result.OffsetDistanceMm:F3}mm");
                continue;
            }

            var ringRatio = result.WeldRadiusPx / Math.Min(image.Height, image.Width);
            var pinRatio = result.PoleRadiusPx / result.WeldRadiusPx;
            ringRatios.Add(ringRatio);
            pinRatios.Add(pinRatio);
            offsets.Add(result.OffsetDistanceMm);

            _output.WriteLine($"{name,-8} R/短边={ringRatio:F4} 孔/环={pinRatio:F3} " +
                              $"D={result.OffsetDistanceMm:F3}mm ({result.ProcessingMs:F0}ms)");

            // 孔洞半径 / 焊环半径：**本区间随 2026-09-17 的孔位基准修正重新标定**。
            //
            // 原来是 [0.17, 0.38]，依据"实测 0.255±0.036"——但那批数字来自**梯度卡尺**：
            // 卡尺的射线原点是焊环圆心，孔心偏离它时量到的是熔核内沿，半径约为真值的两倍。
            // 孔位改由 `HoleLocator` 的三材质分层给出后（同批图里 8.png 即走此路径），
            // 本批 28 张实测 0.118~0.341（中位 0.248；走孔位路径的帧约 0.12~0.18，
            // 仍走卡尺的帧约 0.26~0.34）。区间按实测放宽到 [0.11, 0.36]，两侧各留约 7% 余量。
            //
            // 注意这条断言抓的是"焊环半径被撑大/低估"这个**比例关系**，不是孔位的绝对可信度；
            // 孔位基准是否退化要看它处在 0.12~0.18 还是 0.26~0.38 这两簇中的哪一簇，
            // 那件事由 PpSampleTests 的孔/环比断言（同一 SN 内必须稳定在同一簇）来盯。
            // 也别指望上界能抓住 27.png 那类事故：焊环半径被右侧粘连凸起撑大时这个比值是被
            // **压低**的，看起来仍在正常区间里——那条由下一条断言的环/短边上界来拦。
            Assert.InRange(pinRatio, 0.11, 0.36);
            // 焊环半径 / 短边：同一视野下应为常数。实测 0.130~0.177。
            //
            // 上界 0.19 是特意收紧的，不要放宽：27.png（Img026）曾因熔核与右侧一片粗糙区
            // 被闭运算桥接成"圆盘 + 凸起"，最小二乘圆拟合被凸起撑到 0.198×短边、圆心偏移
            // 19.4px——而 1.0mm 公差在这张 438px 宽的图上只折算到 13.7px，拟合误差比整个
            // 公差还大。当时的上界 0.26 完全拦不住它。改用径向中位数圆拟合后回到 0.170，
            // 本上界即为该回归的哨兵。同理下限 0.12 拦"半径被低估到接近漏焊阈值"的退化。
            Assert.InRange(ringRatio, 0.12, 0.19);
        }

        Assert.True(failures.Count == 0,
            $"{failures.Count}/{files.Count} 个真机样本未通过:\n  " + string.Join("\n  ", failures));

        _output.WriteLine($"\n全部 {files.Count} 个真机样本判定合格；" +
                          $"孔/环={Average(pinRatios):F3} 环/短边={Average(ringRatios):F4} " +
                          $"平均偏移={Average(offsets):F3}mm");
    }

    /// <summary>
    /// 与 <see cref="RealSamples_AllMeasuredOk"/> 同样跑这批真机图，但**走应用真正使用的取图路径**
    /// （<see cref="ImageFolderGrabber"/> → 算法），而不是直接把文件读成 Mat。
    ///
    /// 这个区别曾经是致命的：取图器把每张图 Cv2.Resize 到配置分辨率 1280×1024，
    /// 而现场图只有 182×184 ~ 490×508，被拉伸 5 倍多（横纵还是不同比例）。
    /// 重采样破坏了"物理特征尺度 / 像素栅格"的比例关系，纹理能量与飞溅颗粒计数整体漂移，
    /// 实测 15/28 张良品被误判为飞溅 NG——而上面那条直接读文件的测试却 28/28 通过。
    /// "测试全绿、现场全 NG"的缝就在这两个入口之间，所以这一条必须独立存在。
    /// </summary>
    [Fact]
    public void RealSamples_ThroughActualGrabberPath_AllMeasuredOk()
    {
        var directory = FindSampleDirectory();
        if (directory is null)
        {
            _output.WriteLine("未找到 tests/tp 样本目录，跳过。");
            return;
        }

        using var grabber = new ImageFolderGrabber(directory,
            _config.ImageWidth, _config.ImageHeight);

        var failures = new List<string>();
        for (var i = 0; i < grabber.ImageCount; i++)
        {
            var name = Path.GetFileName(grabber.Files[i]);
            using var image = grabber.TriggerGrab(i);
            var result = _algorithm.Inspect(image, new WeldPoint { Index = i, Name = name });

            if (result.HasError)
                failures.Add($"{name}: 测量失败 — {result.ErrorMessage}");
            else if (!result.IsOk)
                failures.Add($"{name}: 误判 NG — {result.DefectText}, D={result.OffsetDistanceMm:F3}mm");
            else
                _output.WriteLine($"{name,-8} [{image.Width}×{image.Height}] " +
                                  $"D={result.OffsetDistanceMm:F3}mm ({result.ProcessingMs:F0}ms)");
        }

        Assert.True(failures.Count == 0,
            $"取图器路径下 {failures.Count}/{grabber.ImageCount} 个真机样本未通过:\n  "
            + string.Join("\n  ", failures));
    }

    private static double Average(List<double> values) =>
        values.Count == 0 ? 0 : values.Sum() / values.Count;    /// <summary>从测试输出目录向上查找仓库内的 tests/tp 样本目录。</summary>
    private static string? FindSampleDirectory()
    {
        var dir = new DirectoryInfo(AppContext.BaseDirectory);
        while (dir is not null)
        {
            var candidate = Path.Combine(dir.FullName, "tests", "tp");
            if (Directory.Exists(candidate))
                return candidate;
            dir = dir.Parent;
        }
        return null;
    }
}
