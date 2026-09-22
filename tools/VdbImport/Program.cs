using System.Globalization;
using System.Text;
using BatteryWeldAOI.Core.Diagnostics;
using BatteryWeldAOI.Core.Models;
using BatteryWeldAOI.Core.Vision;
using OpenCvSharp;

// 现场 VDB 数据导入 / 横向体检工具（开发期使用）。
//
// 现场样本目录（tests/pp）里，Orignal_pic 下是厂商 GUI 渲染出的 jpg，
// VdbGui_pic 下是同名的 .vdb —— 后者是厂商软件的私有格式，但可以读出它自己的定位结果
// （圆心 X/Y + 半径），见 VendorVdbReader 的格式说明。这给了本项目一份**零人工标注**的
// 参考基准：20955 帧覆盖的焊环定位圆心。
//
// 两个子命令：
//   export <pp根目录> <输出CSV>
//       把语料里所有 .vdb 的厂商测量值导成 CSV（纯解析，不解图，几千个文件几秒完成）。
//       产物用途：给"焊环定位"这一步当参考基准、做困难帧挖掘、以及作为后续弱监督的索引。
//   agree <pp根目录> <输出CSV> [SN目录数上限]
//       横向体检：对每个 Pos0 帧，把本项目算法测出的焊环中心与厂商圆心对齐比较，
//       输出逐帧偏差与最差帧清单。这是把"定位精度"从定性断言（不误判）升级为
//       可量化指标的那一步。
//
//       CSV 里同时给出双方半径，由此可以导出一个很有用的**并块指标**：
//       OurRingRadius_px / VendorRadius_px。厂商半径很稳定（约 0.132×短边），
//       本项目半径一旦明显偏大就说明掩码把相邻结构并进来了——这正是
//       InspectionConfig.PreOpenKernelRatio 记录的失败模式，它既会伪造漏焊 NG，
//       也会在还没到 NG 的中间态里把圆心悄悄带偏（实测最差帧半径虚增 35%、圆心偏 202px）。
//       按这个比值排序即可捞出全部并块帧，比只看 NG 清单更早发现读数被污染的点位。
//
// 注意 agree 会逐帧解码 20MP 图并跑完整算法，单帧约 0.3~0.5s：
// 全量 20943 帧约 2~3 小时，故支持用 SN 目录数上限做抽样（工具会先报出帧数与预估耗时）。
//
// 另一个必须知道的背景：这批 jpg 是 JPEG 质量 1（0.105 bit/px）的重压缩渲染图，
// 不是原始相机帧，且厂商自己的判定是在原始帧上做的。所以本工具量出的是
// "在厂商渲染图上，我们的定位与厂商定位的一致性"，不是绝对精度。

if (args.Length < 3)
{
    Console.WriteLine("用法:");
    Console.WriteLine("  export <pp根目录> <输出CSV>");
    Console.WriteLine("  agree  <pp根目录> <输出CSV> [SN目录数上限]");
    Console.WriteLine();
    Console.WriteLine("示例:");
    Console.WriteLine(@"  dotnet run --project tools\VdbImport -- export tests\pp .scratch\vendor_vdb.csv");
    Console.WriteLine(@"  dotnet run --project tools\VdbImport -- agree  tests\pp .scratch\locator_agreement.csv 20");
    return 1;
}

var command = args[0];
var root = args[1];
var outCsv = args[2];

if (!Directory.Exists(root))
{
    Console.WriteLine($"目录不存在: {Path.GetFullPath(root)}");
    return 1;
}

return command switch
{
    "export" => Export(root, outCsv),
    "agree" => Agree(root, outCsv, args.Length > 3 ? int.Parse(args[3], CultureInfo.InvariantCulture) : 0),
    _ => Unknown(command),
};

static int Unknown(string command)
{
    Console.WriteLine($"未知子命令: {command}（可用: export / agree）");
    return 1;
}

// ---------------------------------------------------------------------------------------
//  export：纯解析，不解图
// ---------------------------------------------------------------------------------------
static int Export(string root, string outCsv)
{
    var files = EnumerateVdb(root).ToList();
    Console.WriteLine($"发现 {files.Count} 个 .vdb，开始解析...");

    var inv = CultureInfo.InvariantCulture;
    var sb = new StringBuilder();
    sb.AppendLine("File,Serial,Position,Timestamp,VendorLabel,LocateCenterX_px,LocateCenterY_px," +
                  "LocateRadius_px,ResultD,HeaderLength");

    int ok = 0, failed = 0, noRadius = 0, ng = 0;
    var errors = new List<string>();

    foreach (var file in files)
    {
        if (!VendorVdbReader.TryRead(file, out var record, out var error))
        {
            failed++;
            if (errors.Count < 20)
                errors.Add($"{Path.GetFileName(file)}: {error}");
            continue;
        }

        ok++;
        if (record!.IsVendorNg == true)
            ng++;
        if (!record.HasLocateRadius)
            noRadius++;

        sb.Append(Escape(record.FileName)).Append(',')
          .Append(record.Serial).Append(',')
          .Append(record.Position).Append(',')
          .Append(Escape(record.Timestamp)).Append(',')
          .Append(record.IsVendorNg is null ? "" : record.IsVendorNg.Value ? "NG" : "OK").Append(',')
          .Append(record.LocateCenterX.ToString("F3", inv)).Append(',')
          .Append(record.LocateCenterY.ToString("F3", inv)).Append(',')
          .Append(record.HasLocateRadius ? record.LocateRadiusPx.ToString("F3", inv) : "").Append(',')
          .Append(record.ResultD.ToString("F3", inv)).Append(',')
          .Append(record.HeaderLength.ToString(inv)).AppendLine();
    }

    WriteCsv(outCsv, sb);
    Console.WriteLine($"解析成功 {ok}，失败 {failed}；厂商判 NG {ng}（{Ratio(ng, ok):P2}）；半径未取到 {noRadius}");
    if (errors.Count > 0)
    {
        Console.WriteLine("失败清单（最多列 20 条）:");
        foreach (var e in errors)
            Console.WriteLine("  " + e);
    }
    Console.WriteLine($"已写出: {Path.GetFullPath(outCsv)}");
    return failed == 0 ? 0 : 2;
}

// ---------------------------------------------------------------------------------------
//  agree：本项目算法 vs 厂商圆心的横向体检
// ---------------------------------------------------------------------------------------
static int Agree(string root, string outCsv, int snLimit)
{
    var pairs = EnumeratePos0JpgPairs(root)
        .GroupBy(p => Path.GetDirectoryName(p.Vdb)!, StringComparer.Ordinal)
        .OrderBy(g => g.Key, StringComparer.Ordinal)
        .ToList();

    var totalPairs = pairs.Sum(g => g.Count());
    if (snLimit > 0)
        pairs = pairs.Take(snLimit).ToList();
    var target = pairs.Sum(g => g.Count());

    Console.WriteLine($"SN 目录 {pairs.Count} 个 / 待比帧 {target} 帧" +
                      (snLimit > 0 ? $"（全量为 {totalPairs} 帧，本次抽样）" : "（全量）"));
    Console.WriteLine($"预估耗时约 {target * 0.4 / 60.0:F1} 分钟（单帧 20MP 解码 + 完整算法约 0.4s）");

    // 现场数据的像素当量一律取自配置（InspectionConfig.MmPerPixel），**不再硬编码 0.025**：
    // 2026-09-17 现场实测把当量改成 0.1003 时，这里写死的 0.025 会让同一批图在界面上与本工具里
    // 报出差 4 倍的毫米数（详见 NinePointCalibration.PureScale 的说明）。
    var config = new InspectionConfig();
    var algorithm = new WeldInspectionAlgorithm(config, NinePointCalibration.PureScale(config.MmPerPixel));

    var inv = CultureInfo.InvariantCulture;
    var deltas = new List<double>();
    var ourRadii = new List<double>();
    var vendorRadii = new List<double>();
    // 只保留最差若干帧用于摘要：全量两万行若攒在内存里，中途中断就什么都留不下，
    // 因此逐行流式落盘（见下方 StreamWriter），摘要侧只留 Top-N。
    var worst = new List<(double Delta, string Name, string Ours, string Vendor)>();
    const int worstKeep = 50;
    // 定位失败（未找到焊环）= 无法参与偏差统计，但本身是要报出来的指标
    int locateFailed = 0, compared = 0, parsed = 0, parseFailed = 0, missingJpg = 0;
    var failures = new List<string>();
    var swAll = System.Diagnostics.Stopwatch.StartNew();

    var fullOut = Path.GetFullPath(outCsv);
    Directory.CreateDirectory(Path.GetDirectoryName(fullOut)!);
    // UTF-8 BOM：与 CsvReportWriter 同口径，Excel 可直接打开
    using var writer = new StreamWriter(fullOut, append: false, new UTF8Encoding(true));
    writer.WriteLine("Source,Serial,Timestamp,VendorLabel,Status,OurDefects,OurRingCx_px,OurRingCy_px," +
                     "VendorCx_px,VendorCy_px,Delta_px,OurRingRadius_px,OurRingInnerRadius_px,OurRingWidth_px," +
                     "VendorRadius_px,OurPinRadius_px,OurPoleCx_px,OurPoleCy_px," +
                     "OurOffset_mm,Processing_ms");

    foreach (var (vdb, jpg) in pairs.SelectMany(g => g))
    {
        if (!VendorVdbReader.TryRead(vdb, out var vendor, out var parseError))
        {
            parseFailed++;
            if (failures.Count < 20)
                failures.Add($"{Path.GetFileName(vdb)}: vdb 解析失败 — {parseError}");
            continue;
        }
        parsed++;

        if (!File.Exists(jpg))
        {
            missingJpg++;
            if (failures.Count < 20)
                failures.Add($"{Path.GetFileName(vdb)}: 找不到同名 jpg");
            continue;
        }

        using var image = Cv2.ImRead(jpg, ImreadModes.Color);
        if (image.Empty())
        {
            missingJpg++;
            failures.Add($"{Path.GetFileName(jpg)}: 图像读取失败");
            continue;
        }

        var result = algorithm.Inspect(image, new WeldPoint { Index = 0, Name = Path.GetFileNameWithoutExtension(jpg) });

        // 我们的几何量在工作分辨率坐标系里，由 Core 统一换算回原图幅（工具与测试必须同口径）
        var agreement = VendorLocatorAgreement.Compare(result, vendor!);
        if (agreement is { } a)
        {
            compared++;
            deltas.Add(a.DeltaPx);
            ourRadii.Add(a.OurRadiusPx);
            if (vendor!.HasLocateRadius)
                vendorRadii.Add(vendor.LocateRadiusPx);
        }
        else
        {
            locateFailed++;
            if (failures.Count < 20)
                failures.Add($"{Path.GetFileName(jpg)}: 未定位到焊环 — {result.ErrorMessage ?? result.DefectText}");
        }

        var status = result.HasError ? "ERROR" : result.IsOk ? "OK" : "NG";
        var vendorLabel = vendor!.IsVendorNg is null ? "" : vendor.IsVendorNg.Value ? "NG" : "OK";
        writer.WriteLine(string.Join(',',
            Escape(Path.GetFileName(jpg)),
            vendor.Serial,
            Escape(vendor.Timestamp),
            vendorLabel,
            status,
            Escape(result.DefectText),
            F1(agreement?.OurCenterX, inv),
            F1(agreement?.OurCenterY, inv),
            vendor.LocateCenterX.ToString("F1", inv),
            vendor.LocateCenterY.ToString("F1", inv),
            F1(agreement?.DeltaPx, inv),
            F1(agreement?.OurRadiusPx, inv),
            result.WeldInnerRadiusSourcePx > 0 ? result.WeldInnerRadiusSourcePx.ToString("F1", inv) : "",
            result.WeldRingWidthSourcePx > 0 ? result.WeldRingWidthSourcePx.ToString("F1", inv) : "",
            vendor.HasLocateRadius ? vendor.LocateRadiusPx.ToString("F1", inv) : "",
            result.PoleRadiusSourcePx > 0 ? result.PoleRadiusSourcePx.ToString("F1", inv) : "",
            result.PoleRadiusSourcePx > 0 ? result.PoleCenterSourcePx.X.ToString("F1", inv) : "",
            result.PoleRadiusSourcePx > 0 ? result.PoleCenterSourcePx.Y.ToString("F1", inv) : "",
            result.HasValidOffset ? result.OffsetDistanceMm.ToString("F4", inv) : "",
            result.ProcessingMs.ToString("F0", inv)));

        if (agreement is { } ag)
        {
            worst.Add((ag.DeltaPx, Path.GetFileName(jpg), status, vendorLabel));
            if (worst.Count > worstKeep * 2)
            {
                worst.Sort((x, y) => y.Delta.CompareTo(x.Delta));
                worst.RemoveRange(worstKeep, worst.Count - worstKeep);
            }
        }

        // 每 200 帧 flush 一次：全量约一小时，中途断电/中止也要留下已完成部分
        if (compared % 200 == 0 && compared > 0)
        {
            writer.Flush();
            var elapsed = swAll.Elapsed.TotalMinutes;
            var eta = elapsed / compared * (target - compared);
            Console.WriteLine($"  已比 {compared}/{target}  中位偏差 {PercentileOf(deltas, 50):F1}px  " +
                              $"已用 {elapsed:F1}min  预计剩余 {eta:F1}min");
        }
    }

    writer.Flush();

    Console.WriteLine();
    Console.WriteLine($"解析 {parsed}（失败 {parseFailed}），缺图/读图失败 {missingJpg}");
    Console.WriteLine($"参与比较 {compared}，其中我们**未定位到焊环** {locateFailed}（{Ratio(locateFailed, compared + locateFailed):P2}）");

    if (deltas.Count > 0)
    {
        Console.WriteLine();
        Console.WriteLine("=== 本项目焊环中心 vs 厂商定位圆心（原图像素）===");
        Console.WriteLine($"  中位 {PercentileOf(deltas, 50):F1}   均值 {deltas.Average():F1}   " +
                          $"p90 {PercentileOf(deltas, 90):F1}   p99 {PercentileOf(deltas, 99):F1}   最大 {deltas.Max():F1}");
        if (ourRadii.Count > 0 && vendorRadii.Count > 0)
        {
            var ourRadius = PercentileOf(ourRadii, 50);
            var vendorRadius = PercentileOf(vendorRadii, 50);
            Console.WriteLine($"  参考：本算法焊环半径中位 {ourRadius:F0}px、厂商定位半径中位 {vendorRadius:F1}px，"
                              + $"故中位偏差约 {PercentileOf(deltas, 50) / ourRadius:P1} 本算法半径");
            Console.WriteLine("        （两个半径取的「环边界」不同，不可互相替代，见 VendorVdbRecord 注释）");
        }

        Console.WriteLine();
        Console.WriteLine("最差 15 帧（按偏差降序，完整清单见 CSV 的 Delta_px 列）:");
        foreach (var (delta, name, ours, vendorLabel) in worst.OrderByDescending(w => w.Delta).Take(15))
            Console.WriteLine($"  {delta,7:F1}px  {name}  我们的判定={ours}  厂商={vendorLabel}");
    }

    var vendorSpread = VendorSelfSpread(root, out var spreadCount);
    if (vendorSpread is not null)
    {
        Console.WriteLine();
        Console.WriteLine("=== 参考基准：厂商自己在同一帧三个位置标记上的圆心互差 ===");
        Console.WriteLine($"  （{spreadCount} 帧；这是同一张图上换配置得到的差异，量的是厂商自己的重复性，非物理真值）");
        Console.WriteLine($"  中位 {vendorSpread.Value.Median:F1}   均值 {vendorSpread.Value.Mean:F1}   p90 {vendorSpread.Value.P90:F1}   最大 {vendorSpread.Value.Max:F1}");
        Console.WriteLine($"  → 一致性目标不应比这个数更严：贴合厂商到 {vendorSpread.Value.Median:F0}px 以内已经超出了厂商自身的复现能力");
    }

    if (failures.Count > 0)
    {
        Console.WriteLine();
        Console.WriteLine("异常/失败样例（最多 20 条）:");
        foreach (var f in failures)
            Console.WriteLine("  " + f);
    }

    Console.WriteLine();
    Console.WriteLine($"已写出: {Path.GetFullPath(outCsv)}");
    return compared > 0 ? 0 : 2;
}

// ---------------------------------------------------------------------------------------
//  公共小工具
// ---------------------------------------------------------------------------------------

/// <summary>语料里所有 .vdb（仅厂商 GUI 目录，避免把别处的同名文件算进来）。</summary>
static IEnumerable<string> EnumerateVdb(string root) =>
    Directory.EnumerateFiles(root, "*.vdb", SearchOption.AllDirectories)
        .Where(f => f.Contains("VdbGui_pic", StringComparison.OrdinalIgnoreCase))
        .OrderBy(f => f, StringComparer.Ordinal);

/// <summary>
/// Pos0 的 vdb 与其同名 jpg 的配对。只有 Pos0 存在导出的 jpg
/// （Pos1/Pos2 的 vdb 内嵌的是同一张图，不提供额外图像数据）。
/// </summary>
static IEnumerable<(string Vdb, string Jpg)> EnumeratePos0JpgPairs(string root)
{
    foreach (var vdb in EnumerateVdb(root))
    {
        if (!Path.GetFileName(vdb).Contains("Pos0", StringComparison.Ordinal))
            continue;
        var jpg = Path.ChangeExtension(
            vdb.Replace("VdbGui_pic", "Orignal_pic", StringComparison.OrdinalIgnoreCase), ".jpg");
        yield return (vdb, jpg);
    }
}

/// <summary>同一帧内厂商三个位置标记的圆心最大互差（= 厂商自身重复性的参考）。</summary>
static (double Median, double Mean, double P90, double Max)? VendorSelfSpread(string root, out int frameCount)
{
    var byFrame = new Dictionary<(string Serial, string Timestamp), List<(double X, double Y)>>();
    foreach (var f in EnumerateVdb(root))
    {
        if (!VendorVdbReader.TryRead(f, out var r, out _))
            continue;
        var key = (r!.Serial, r.Timestamp);
        if (!byFrame.TryGetValue(key, out var list))
            byFrame[key] = list = new List<(double, double)>();
        list.Add((r.LocateCenterX, r.LocateCenterY));
    }

    var spreads = new List<double>();
    foreach (var list in byFrame.Values)
    {
        if (list.Count < 2)
            continue;
        var max = 0.0;
        for (var i = 0; i < list.Count; i++)
            for (var j = i + 1; j < list.Count; j++)
                max = Math.Max(max, Math.Sqrt(Math.Pow(list[i].X - list[j].X, 2) + Math.Pow(list[i].Y - list[j].Y, 2)));
        spreads.Add(max);
    }

    frameCount = spreads.Count;
    if (spreads.Count == 0)
        return null;
    spreads.Sort();
    return (Median(spreads), spreads.Average(), Percentile(spreads, 90), spreads[^1]);
}

static double Median(List<double> sorted) => Percentile(sorted, 50);

/// <summary>对任意序列取分位数（内部排序副本，避免调用方必须先排好序）。</summary>
static double PercentileOf(IEnumerable<double> values, double pct) =>
    Percentile(values.OrderBy(v => v).ToList(), pct);

/// <summary>线性插值分位数（输入必须已升序），与 Core 的 Geo.Percentile 同一口径。</summary>
static double Percentile(List<double> sorted, double pct)
{
    if (sorted.Count == 0)
        return 0;
    if (sorted.Count == 1)
        return sorted[0];
    var pos = pct / 100.0 * (sorted.Count - 1);
    var lo = (int)Math.Floor(pos);
    var hi = Math.Min(lo + 1, sorted.Count - 1);
    var t = pos - lo;
    return sorted[lo] * (1 - t) + sorted[hi] * t;
}

static double Ratio(int part, int whole) => whole == 0 ? 0 : part / (double)whole;

/// <summary>可空数值的固定小数格式化，null 输出空字段（CSV 里表示"不适用"）。</summary>
static string F1(double? value, CultureInfo inv) => value is { } v ? v.ToString("F1", inv) : string.Empty;

/// <summary>CSV 字段转义：含逗号/引号/换行时加引号包裹（与 CsvReportWriter 同口径）。</summary>
static string Escape(string? value)
{
    if (string.IsNullOrEmpty(value))
        return string.Empty;
    if (value.IndexOfAny(new[] { ',', '"', '\n', '\r' }) < 0)
        return value;
    return "\"" + value.Replace("\"", "\"\"") + "\"";
}

static void WriteCsv(string path, StringBuilder content)
{
    var full = Path.GetFullPath(path);
    Directory.CreateDirectory(Path.GetDirectoryName(full)!);
    File.WriteAllText(full, content.ToString(), new UTF8Encoding(true));
}
