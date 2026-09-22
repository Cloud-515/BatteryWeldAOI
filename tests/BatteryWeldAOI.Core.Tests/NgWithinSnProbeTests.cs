using BatteryWeldAOI.Core.Models;
using BatteryWeldAOI.Core.Vision;
using OpenCvSharp;
using Xunit;
using Xunit.Abstractions;

namespace BatteryWeldAOI.Core.Tests;

/// <summary>
/// 临时探针（调查用，非回归断言）：把**每个含 NG 帧的 SN** 的全部帧都跑一遍，
/// 在 SN 内部对照 NG 帧与它自己的 OK 兄弟帧。
///
/// 为什么必须在 SN 内部比：不同 SN 的工作距离/倍率不同，环半径比的"家族值"本身就不同，
/// 跨 SN 汇总会把倍率差异混进来（NgProbeTests 目前就是跨 SN 汇总，存在这个混淆）。
/// 同一 SN 的 24 帧是**同一个焊点连拍**（实测焊心在画幅内仅抖动约 300px/5472），
/// 因此 SN 内部对照能直接回答：NG 帧在几何量上是不是离群点。
/// </summary>
public class NgWithinSnProbeTests
{
    private readonly ITestOutputHelper _output;
    private readonly InspectionConfig _config = new();
    private readonly WeldInspectionAlgorithm _algorithm;

    public NgWithinSnProbeTests(ITestOutputHelper output)
    {
        _output = output;
        var calibration = new NinePointCalibration(new List<(Point2d Pixel, Point2d Mm)>
        {
            (new Point2d(0, 0), new Point2d(0, 0)),
            (new Point2d(1000, 0), new Point2d(25, 0)),
            (new Point2d(0, 1000), new Point2d(0, 25)),
        });
        _algorithm = new WeldInspectionAlgorithm(_config, calibration);
    }

    private sealed record Frame(string Name, bool IsNg, string Verdict, double D,
        double RingRatio, double PinOverRing, double RingCx, double RingCy);

    [Fact]
    public void DumpWithinSn()
    {
        var root = FindPpRoot();
        if (root is null) { _output.WriteLine("skip"); return; }

        var ngDirs = Directory.GetFiles(root, "*_NG_*.jpg", SearchOption.AllDirectories)
            .Select(f => Path.GetDirectoryName(f)!)
            .Distinct()
            .OrderBy(d => d, StringComparer.Ordinal)
            .ToList();

        var csv = new List<string> { "sn,frame,isNg,verdict,D_mm,ringRatio,pinOverRing,ringCx,ringCy" };

        foreach (var dir in ngDirs)
        {
            var files = Directory.GetFiles(dir, "*.jpg")
                .OrderBy(f => Path.GetFileName(f), StringComparer.Ordinal).ToList();
            var sn = Path.GetFileName(dir);
            var frames = new List<Frame>();

            foreach (var file in files)
            {
                var name = Path.GetFileName(file);
                var tag = name[..Math.Max(0, name.IndexOf("_Scene", StringComparison.Ordinal))];
                using var image = Cv2.ImRead(file, ImreadModes.Color);
                if (image.Empty()) continue;

                var r = _algorithm.Inspect(image, new WeldPoint { Index = 0, Name = tag });
                var pb = Math.Max(1, Math.Min(r.ProcessedSizePx.Height, r.ProcessedSizePx.Width));
                var f = new Frame(tag, name.Contains("_NG_"),
                    r.HasError ? "ERR" : r.IsOk ? "OK" : "NG",
                    r.OffsetDistanceMm, r.WeldRadiusPx / pb,
                    r.WeldRadiusPx > 0 ? r.PoleRadiusPx / r.WeldRadiusPx : 0,
                    r.WeldCenterPx.X, r.WeldCenterPx.Y);
                frames.Add(f);
                csv.Add(string.Join(',', sn, f.Name, f.IsNg ? 1 : 0, f.Verdict,
                    f.D.ToString("F4"), f.RingRatio.ToString("F4"),
                    f.PinOverRing.ToString("F4"), f.RingCx.ToString("F1"), f.RingCy.ToString("F1")));
            }

            var ng = frames.Where(f => f.IsNg).ToList();
            var ok = frames.Where(f => !f.IsNg && f.Verdict == "OK").ToList();
            if (ng.Count == 0 || ok.Count == 0) continue;

            static string Band(IEnumerable<double> v)
            {
                var a = v.OrderBy(x => x).ToList();
                return a.Count == 0 ? "-" : $"{a[0]:F3}~{a[^1]:F3} 中位{a[a.Count / 2]:F3}";
            }
            // NG 帧在兄弟帧分布里的分位（0=最小，1=最大）
            static double Pct(double v, List<double> pool) =>
                pool.Count == 0 ? -1 : pool.Count(x => x < v) / (double)pool.Count;

            var okD = ok.Select(f => f.D).ToList();
            var okR = ok.Select(f => f.RingRatio).ToList();

            _output.WriteLine($"=== {sn[..Math.Min(sn.Length, 28)]}  帧{frames.Count} (OK判定{ok.Count})");
            _output.WriteLine($"    兄弟OK帧: D {Band(okD)} | 环r/短边 {Band(okR)}");
            foreach (var f in ng)
                _output.WriteLine(
                    $"    NG帧 {f.Name}: 判定={f.Verdict} D={f.D:F3}(分位{Pct(f.D, okD):P0}) " +
                    $"环r/短边={f.RingRatio:F4}(分位{Pct(f.RingRatio, okR):P0})");
        }

        var outPath = Path.Combine(Path.GetTempPath(), "ng_within_sn.csv");
        File.WriteAllText(outPath, string.Join("\n", csv), new System.Text.UTF8Encoding(true));
        _output.WriteLine($"\nCSV: {outPath}  行数={csv.Count - 1}");
    }

    private static string? FindPpRoot()
    {
        var dir = new DirectoryInfo(AppContext.BaseDirectory);
        while (dir is not null)
        {
            var c = Path.Combine(dir.FullName, "tests", "pp");
            if (Directory.Exists(c)) return c;
            dir = dir.Parent;
        }
        return null;
    }
}
