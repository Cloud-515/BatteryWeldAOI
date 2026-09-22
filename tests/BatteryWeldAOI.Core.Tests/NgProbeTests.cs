using BatteryWeldAOI.Core.Models;
using BatteryWeldAOI.Core.Vision;
using OpenCvSharp;
using Xunit;
using Xunit.Abstractions;

namespace BatteryWeldAOI.Core.Tests;

/// <summary>
/// 探针：把现场 OK / NG 两组的几何量 dump 成 CSV，用于判定"厂商 NG 的定义"。
/// </summary>
public class NgProbeTests
{
    private readonly ITestOutputHelper _output;
    private readonly InspectionConfig _config = new();
    private readonly WeldInspectionAlgorithm _algorithm;

    public NgProbeTests(ITestOutputHelper output)
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

    [Fact]
    public void DumpGeometry()
    {
        var root = FindPpRoot();
        if (root is null)
        {
            _output.WriteLine("skip");
            return;
        }

        var all = Directory.GetFiles(root, "*.jpg", SearchOption.AllDirectories);
        var ng = all.Where(f => Path.GetFileName(f).Contains("_NG_")).ToList();
        // 取 OK 样本：每个 SN 目录抽 6 张，控制总耗时
        var ok = all.Where(f => Path.GetFileName(f).Contains("_OK_"))
            .GroupBy(f => Path.GetDirectoryName(f)!)
            .SelectMany(g => g.OrderBy(x => x, StringComparer.Ordinal).Take(6))
            .ToList();

        var boxW = _config.MaxWorkingBase;
        var rows = new List<string> { "group,sn,scene,verdict,Dmm,dx_mm,dy_mm,weldCx_norm,weldCy_norm,poleCx_norm,poleCy_norm,Dpx,q_px,residual_px,q_over_R,rawWeldCx,rawWeldCy" };

        void Emit(string file, string group)
        {
            var name = Path.GetFileName(file);
            var scene = name[..Math.Max(0, name.IndexOf("_Scene", StringComparison.Ordinal))];
            var sn = name.Split("SN_").Length > 1 ? "SN_" + name.Split("SN_")[1].Split('_')[0] : "?";
            using var image = Cv2.ImRead(file, ImreadModes.Color);
            if (image.Empty())
                return;
            var r = _algorithm.Inspect(image, new WeldPoint { Index = 0, Name = scene });
            var pw = Math.Max(1, r.ProcessedSizePx.Width);
            var ph = Math.Max(1, r.ProcessedSizePx.Height);
            var verdict = r.HasError ? "ERR" : r.IsOk ? "OK" : "NG";
            var qOverR = r.WeldRadiusPx > 0 ? r.PoleRadiusPx / r.WeldRadiusPx : 0;
            rows.Add(string.Join(',',
                group, sn, scene.Replace(',', '_'), verdict,
                r.OffsetDistanceMm.ToString("F4"), r.OffsetXmm.ToString("F4"), r.OffsetYmm.ToString("F4"),
                (r.WeldCenterPx.X / pw).ToString("F5"), (r.WeldCenterPx.Y / ph).ToString("F5"),
                (r.PoleCenterPx.X / pw).ToString("F5"), (r.PoleCenterPx.Y / ph).ToString("F5"),
                (Math.Sqrt(r.OffsetXmm * r.OffsetXmm + r.OffsetYmm * r.OffsetYmm) / _config.MmPerPixel).ToString("F2"),
                r.PoleRadiusPx.ToString("F2"), "0", qOverR.ToString("F4"),
                r.WeldCenterPx.X.ToString("F1"), r.WeldCenterPx.Y.ToString("F1")));
        }

        foreach (var f in ng) Emit(f, "NG");
        foreach (var f in ok) Emit(f, "OK");

        var outPath = Path.Combine(Path.GetTempPath(), "ng_probe_geometry.csv");
        File.WriteAllText(outPath, string.Join("\n", rows), new System.Text.UTF8Encoding(true));
        _output.WriteLine($"CSV: {outPath}  行数={rows.Count - 1}  NG={ng.Count} OK={ok.Count}  workBase={boxW}");

        // 分组统计：焊环中心相对画幅中心的位移（若厂商按"名义位置"判焊偏，这一项应能分开两组）
        static (double Mean, double Min, double Max) Stat(List<double> v) =>
            v.Count == 0 ? (0, 0, 0) : (v.Average(), v.Min(), v.Max());

        foreach (var g in new[] { "OK", "NG" })
        {
            var sel = rows.Skip(1).Select(x => x.Split(',')).Where(c => c[0] == g).ToList();
            var d = sel.Select(c => double.Parse(c[4], System.Globalization.CultureInfo.InvariantCulture)).ToList();
            var frameOff = sel.Select(c => Math.Sqrt(
                Math.Pow(double.Parse(c[7], System.Globalization.CultureInfo.InvariantCulture) - 0.5, 2) +
                Math.Pow(double.Parse(c[8], System.Globalization.CultureInfo.InvariantCulture) - 0.5, 2))).ToList();
            var poleOff = sel.Select(c => Math.Sqrt(
                Math.Pow(double.Parse(c[9], System.Globalization.CultureInfo.InvariantCulture) - 0.5, 2) +
                Math.Pow(double.Parse(c[10], System.Globalization.CultureInfo.InvariantCulture) - 0.5, 2))).ToList();
            var qr = sel.Select(c => double.Parse(c[14], System.Globalization.CultureInfo.InvariantCulture)).ToList();
            var ds = Stat(d); var fs = Stat(frameOff); var ps = Stat(poleOff); var qs = Stat(qr);
            _output.WriteLine(
                $"[{g}] n={sel.Count}  " +
                $"焊环-孔洞 D(mm): 均值={ds.Mean:F3} [{ds.Min:F3}~{ds.Max:F3}]  " +
                $"焊环离画幅中心: 均值={fs.Mean:F4} [{fs.Min:F4}~{fs.Max:F4}]  " +
                $"孔洞离画幅中心: 均值={ps.Mean:F4} [{ps.Min:F4}~{ps.Max:F4}]  " +
                $"孔/环: 均值={qs.Mean:F3} [{qs.Min:F3}~{qs.Max:F3}]");
        }
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
