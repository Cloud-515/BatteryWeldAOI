using BatteryWeldAOI.Core.Diagnostics;
using BatteryWeldAOI.Core.Models;
using BatteryWeldAOI.Core.Vision;
using OpenCvSharp;

// 开发期诊断工具。两种模式：
//
//   dotnet run -- file <图片路径|目录> [imageWidth] [imageHeight] [mmPerPixel]
//       对单张图 / 整个目录跑**真实算法**，打印逐帧判定与判定依据。
//       用来回答"这张图被判成了什么、依据是什么"。
//
//   dotnet run -- ring <图片路径> [imageWidth] [imageHeight] [mmPerPixel]
//       对单张图打印焊环定位的**每一条路径**各自的中间量（主路径 / 核宽阶梯 / 双尺度 /
//       贴边清除第三路径），以及原始掩码的连通域统计。
//       用来回答"为什么这一帧测不出来"——公开 API 只给最终结论，看不出是哪条路径、
//       被哪道闸门拦下的。
//
//   dotnet run -- corpus <SN目录的根> [SN个数上限] [--no-fallback] [--no-roughness] [--with-rho] [--csv <输出csv>]
//       批量跑一个语料目录（其下每个子目录是一个 SN、各含若干帧），汇总判定分布、
//       偏移量中位数，并列出所有非 OK 帧及原因；有同名 .vdb 时同时给出与厂商圆的偏差。
//       `--no-fallback` 关掉第四路径、`--no-roughness` 关掉粗糙度闸门、`--with-rho` 打开 ρ 一致性闸门（A/B 对照用），
//       `--csv` 落逐帧明细供两次运行对差。
//
// 需要访问 Core 的内部类型（WeldRingLocator / Geo），故 Core 上有 InternalsVisibleTo("Diag")。

if (args.Length == 0)
{
    Console.WriteLine("用法:");
    Console.WriteLine("  dotnet run -- file   <图片路径|目录> [imageWidth] [imageHeight] [mmPerPixel]");
    Console.WriteLine("  dotnet run -- ring   <图片路径> [imageWidth] [imageHeight] [mmPerPixel]");
    Console.WriteLine("  dotnet run -- vdb    <vdb路径>");
    Console.WriteLine("  dotnet run -- corpus <SN目录的根> [SN个数上限] [--no-fallback] [--no-roughness] [--with-rho] [--csv <输出csv>]");
    Console.WriteLine("  dotnet run -- probe  <SN目录的根> [SN个数上限] [--csv <输出csv>]");
    Console.WriteLine("  dotnet run -- seed   <图片路径> <圆心x> <圆心y> <半径>   # 原图像素；从指定种子跑精修");
    return;
}

var mode = args[0];
if (mode is not ("file" or "ring" or "vdb" or "corpus" or "probe" or "seed"))
{
    Console.WriteLine($"未知模式 '{mode}'，只支持 file / ring / vdb / corpus / probe / seed。");
    return;
}

if (args.Length < 2)
{
    Console.WriteLine("缺少图片路径。");
    return;
}

// 缺省值取 InspectionConfig 的默认值，不在这里重复写字面量——否则标定一改默认值，
// 诊断工具仍按旧值跑，输出的毫米数与算法实际判定用的口径不一致（这类"两处各写一份"
// 的重复正是本项目反复踩过的坑，见 InspectionConfig.MmPerPixel 的实测记录）。
var defaults = new InspectionConfig();
var width = args.Length > 2 && int.TryParse(args[2], out var w) ? w : defaults.ImageWidth;
var height = args.Length > 3 && int.TryParse(args[3], out var h) ? h : defaults.ImageHeight;
var mmPerPixel = args.Length > 4 && double.TryParse(args[4], out var m) ? m : defaults.MmPerPixel;

var cfg = new InspectionConfig { ImageWidth = width, ImageHeight = height, MmPerPixel = mmPerPixel };
var calib = new NinePointCalibration(new List<(Point2d Pixel, Point2d Mm)>
{
    (new(0, 0), new(0, 0)),
    (new(width, 0), new(width * mmPerPixel, 0)),
    (new(0, height), new(0, height * mmPerPixel)),
});

if (mode == "file")
    RunFileMode(args[1], cfg, calib, width, height, mmPerPixel);
else if (mode == "vdb")
    RunVdbMode(args[1]);
else if (mode == "corpus")
    RunCorpusMode(args);
else if (mode == "probe")
    RunProbeMode(args);
else if (mode == "seed")
    RunSeedMode(args, cfg);
else
    RunRingProbe(args[1], cfg);

/// <summary>
/// 从**指定种子**跑一遍边界精修，回答"给定这个圆心/半径，我们的精修量得出什么"。
///
/// 用途：当算法自己给不出结果（种子不可信）时，用外部基准（例如厂商 `.vdb` 的圆）
/// 当种子，就能把**"种子不对"与"精修量不出来"这两件事分开**——
/// 前者是取候选的策略问题（可修），后者是成像/机理问题（要换方法）。
///
/// 用法: dotnet run -- seed &lt;图片路径&gt; &lt;圆心x&gt; &lt;圆心y&gt; &lt;半径&gt;
/// 圆心与半径都是**原图像素**（与 `.vdb` 同口径），内部按工作分辨率换算。
/// </summary>
static void RunSeedMode(string[] args, InspectionConfig cfg)
{
    if (args.Length < 5
        || !double.TryParse(args[2], out var cx) || !double.TryParse(args[3], out var cy)
        || !double.TryParse(args[4], out var rSrc))
    {
        Console.WriteLine("用法: dotnet run -- seed <图片路径> <圆心x> <圆心y> <半径>  （原图像素）");
        return;
    }

    using var bgr = Cv2.ImRead(args[1], ImreadModes.Color);
    if (bgr.Empty())
    {
        Console.WriteLine($"读取失败: {args[1]}");
        return;
    }

    // 与 WeldInspectionAlgorithm.Inspect 完全相同的预处理
    using var grayFull = new Mat();
    Cv2.CvtColor(bgr, grayFull, ColorConversionCodes.BGR2GRAY);
    using var gray = new Mat();
    var work = grayFull;
    var srcBase = Math.Min(grayFull.Height, grayFull.Width);
    if (cfg.MaxWorkingBase > 0 && srcBase > cfg.MaxWorkingBase)
    {
        var scale = cfg.MaxWorkingBase / (double)srcBase;
        Cv2.Resize(grayFull, gray, new Size(), scale, scale, InterpolationFlags.Area);
        work = gray;
    }
    Cv2.MedianBlur(work, work, 3);

    var workBase = Math.Min(work.Height, work.Width);
    var toWork = workBase / (double)srcBase;
    var seed = new Point2d(cx * toWork, cy * toWork);
    var seedR = rSrc * toWork;

    Console.WriteLine($"文件: {Path.GetFileName(args[1])}");
    Console.WriteLine($"原图种子 ({cx:F1},{cy:F1}) r={rSrc:F1} → 工作分辨率 "
                      + $"({seed.X:F1},{seed.Y:F1}) r={seedR:F1}（base={workBase}）");
    Console.WriteLine($"配置: 粗糙度闸门={cfg.WeldRingRefineMinRoughnessRatio}"
                      + $" ρ闸门={cfg.WeldRingRefineMinRhoRatio}"
                      + $" 精修弧段门限={cfg.WeldRingRefineMinArcSupport:P0}"
                      + $" 半径修正上限={cfg.WeldRingRefineMaxRadiusCorrection:P0}");

    var r = WeldRingLocator.RefineBoundary(work, seed, seedR, cfg, altCenter: null,
        msg => Console.WriteLine(msg));
    if (r is not { } v)
    {
        Console.WriteLine("→ 精修**没有**给出结果（返回 null）");
        Console.WriteLine("   含义：不是「种子不对」，而是这个成像条件下 |∇I| 上量不出自洽的熔核外沿。");
        return;
    }

    var back = 1.0 / toWork;
    Console.WriteLine($"→ 精修给出 圆心=({v.Circle.Center.X * back:F1},{v.Circle.Center.Y * back:F1}) "
                      + $"r={v.Circle.Radius * back:F1}（原图像素）"
                      + $" 半径比={v.Circle.Radius / workBase:F4}"
                      + $" 支撑弧段={v.ArcSupport:P1} 截断比={v.ClippedRatio:F2}"
                      + $" 残差={v.ResidualScatter:F4} ρ={v.EdgeRadius * back:F1}");
    Console.WriteLine($"   与种子的位移 = {Geo.Dist(v.Circle.Center, seed) * back:F1} 原图像素"
                      + $"（种子半径的 {Geo.Dist(v.Circle.Center, seed) / seedR:P0}）");
}

/// <summary>
/// **离线判别研究**：对语料逐帧导出"精修两个起点各自单独跑一遍"的结果 + 厂商圆基准，
/// 用来回答"哪个起点更该被采纳、用什么量能判出来"（FIXPLAN 第 10 节）。
///
/// 与 `corpus` 的区别：`corpus` 只给**最终采纳**的那个圆，看不出另一个起点是什么样；
/// 而仲裁规则要改进，缺的正是"两个都看到"。探针挂在真实算法上（`WeldInspectionAlgorithm.RefineProbe`），
/// 因此种子与孔心都是算法实际用的那些——**不能**用 Python 复刻或硬编码种子代替
/// （第 9.6② 节记录过这个坑：硬编码种子曾让探针结论与算法相反）。
///
/// 用法: dotnet run -- probe &lt;SN目录的根&gt; [SN个数上限] [--csv &lt;输出&gt;]
/// </summary>
static void RunProbeMode(string[] args)
{
    var root = args[1];
    var limit = 0;
    string? csvPath = null;
    for (var i = 2; i < args.Length; i++)
    {
        if (args[i] == "--csv" && i + 1 < args.Length)
            csvPath = args[++i];
        else if (int.TryParse(args[i], out var n))
            limit = n;
    }

    var cfg = new InspectionConfig();
    var algo = new WeldInspectionAlgorithm(cfg, NinePointCalibration.PureScale(cfg.MmPerPixel));

    var dirs = Directory.GetDirectories(root)
        .OrderBy(d => Path.GetFileName(d), StringComparer.Ordinal)
        .ToList();
    if (limit > 0)
        dirs = dirs.Take(limit).ToList();

    var vdbRoot = Path.Combine(Path.GetDirectoryName(root.TrimEnd('\\', '/'))!, "VdbGui_pic");
    Console.WriteLine($"探针研究 语料 {root}：{dirs.Count} 个 SN 目录，厂商基准="
                      + $"{(Directory.Exists(vdbRoot) ? "有" : "无")}");

    var rows = new List<string>
    {
        "SN,帧标,厂商x,厂商y,厂商r,种子x,种子y,种子r,孔心x,孔心y,"
        + "主x,主y,主r,主弧段,主截断,主残差,主rho,"
        + "次x,次y,次r,次弧段,次截断,次残差,次rho,"
        + "采纳x,采纳y,采纳r,采纳弧段,采纳截断,采纳残差,采纳rho,"
        + "判定,依据"
    };
    var probed = 0;
    var frames = 0;

    foreach (var dir in dirs)
    {
        var sn = Path.GetFileName(dir);
        var snTag = sn.Length >= 32 ? sn.Substring(28, 4) : sn;
        foreach (var file in Directory.GetFiles(dir, "*.jpg")
                     .OrderBy(f => Path.GetFileName(f), StringComparer.Ordinal))
        {
            using var image = Cv2.ImRead(file, ImreadModes.Color);
            if (image.Empty())
                continue;
            frames++;
            var name = Path.GetFileName(file);
            var sceneAt = name.IndexOf("_Scene", StringComparison.Ordinal);
            var tag = sceneAt > 0 ? name[..sceneAt] : name;

            // 厂商基准
            var vx = double.NaN;
            var vy = double.NaN;
            var vr = double.NaN;
            var vdb = Path.Combine(vdbRoot, sn, Path.GetFileNameWithoutExtension(name) + ".vdb");
            if (File.Exists(vdb) && VendorVdbReader.TryRead(vdb, out var record, out _)
                && record is { HasLocateRadius: true })
            {
                vx = record.LocateCenterX;
                vy = record.LocateCenterY;
                vr = record.LocateRadiusPx;
            }

            // 探针快照直接存在 Core 的类型里，不再自建记录类型——顶级语句文件里
            // 类型声明必须放在全部顶级语句之后，少一个类型就少一处这种坑。
            WeldInspectionAlgorithm.RefineProbeSnapshot? row = null;
            algo.RefineProbe = p => row = p;
            var result = algo.Inspect(image, new WeldPoint { Index = 0, Name = tag });
            algo.RefineProbe = null;

            if (row is null)
                continue;   // 没走到精修（焊环或孔位没测出来）——研究集只收有决策点的帧
            probed++;
            var r0 = row.Value;
            rows.Add(string.Join(',', snTag, tag,
                Fmt(vx), Fmt(vy), Fmt(vr),
                Fmt(r0.SeedCenter.X), Fmt(r0.SeedCenter.Y), Fmt(r0.SeedRadius),
                Fmt(r0.PinCenter.X), Fmt(r0.PinCenter.Y),
                S(r0.Primary), S(r0.Secondary), S(r0.Adopted),
                result.HasError ? "异常" : result.IsOk ? "OK" : "NG",
                (result.ErrorMessage ?? string.Empty).Replace(',', ';')));
        }
    }

    if (csvPath is not null)
    {
        File.WriteAllLines(csvPath, rows, new System.Text.UTF8Encoding(true));
        Console.WriteLine($"已写入 {csvPath}");
    }
    Console.WriteLine($"帧数 {frames}，进入精修决策点 {probed} 帧");

    static string S(RingRefinement? r) => r is { } v
        ? string.Join(',', Fmt(v.Circle.Center.X), Fmt(v.Circle.Center.Y), Fmt(v.Circle.Radius),
            Fmt(v.ArcSupport), Fmt(v.ClippedRatio), Fmt(v.ResidualScatter), Fmt(v.EdgeRadius))
        : ",,,,,,";

    static string Fmt(double v) => double.IsNaN(v) ? "" : v.ToString("F4",
        System.Globalization.CultureInfo.InvariantCulture);
}

/// <summary>
/// 语料批量回归：对根目录下每个 SN 子目录跑完整算法，汇总判定分布、偏移量中位数，
/// 并列出所有非 OK 帧及原因。A/B 对照用 <c>--no-fallback</c> 关掉第四路径
/// （<see cref="InspectionConfig.MinRingFallbackArcSupport"/> = 0）。
/// </summary>
static void RunCorpusMode(string[] args)
{
    var root = args[1];
    var limit = 0;
    var useFallback = true;
    var useRoughness = true;
    // ρ 闸门默认关闭（它在 960 帧上更强，但唯一变差的那帧有独立物理证据说明它判错了，
    // 见 InspectionConfig.WeldRingRefineMinRhoRatio）。`--with-rho` 用于复跑那次实验。
    var useRho = false;
    string? csvPath = null;
    for (var i = 2; i < args.Length; i++)
    {
        if (args[i] == "--no-fallback")
            useFallback = false;
        else if (args[i] == "--no-roughness")
            useRoughness = false;
        else if (args[i] == "--with-rho")
            useRho = true;
        else if (args[i] == "--csv" && i + 1 < args.Length)
            csvPath = args[++i];
        else if (int.TryParse(args[i], out var n))
            limit = n;
    }

    var cfg = new InspectionConfig
    {
        MinRingFallbackArcSupport = useFallback ? new InspectionConfig().MinRingFallbackArcSupport : 0,
        WeldRingRefineMinRoughnessRatio = useRoughness
            ? new InspectionConfig().WeldRingRefineMinRoughnessRatio
            : 0,
        WeldRingRefineMinRhoRatio = useRho ? 0.98 : 0
    };
    var algo = new WeldInspectionAlgorithm(cfg, NinePointCalibration.PureScale(cfg.MmPerPixel));

    var dirs = Directory.GetDirectories(root)
        .OrderBy(d => Path.GetFileName(d), StringComparer.Ordinal)
        .ToList();
    if (limit > 0)
        dirs = dirs.Take(limit).ToList();

    // 厂商 .vdb 在语料根的同级 VdbGui_pic/<SN>/<同名>.vdb —— 拿它当独立的定位参考基准。
    var vdbRoot = Path.Combine(Path.GetDirectoryName(root.TrimEnd('\\', '/'))!, "VdbGui_pic");
    var hasVendor = Directory.Exists(vdbRoot);

    Console.WriteLine($"语料 {root}：{dirs.Count} 个 SN 目录，第四路径={(useFallback ? "开" : "关")}"
                      + $"（MinRingFallbackArcSupport={cfg.MinRingFallbackArcSupport}）"
                      + $"，粗糙度闸门={(useRoughness ? "开" : "关")}"
                      + $"（{cfg.WeldRingRefineMinRoughnessRatio}）"
                      + $"，ρ闸门={(useRho ? "开" : "关")}"
                      + $"（{cfg.WeldRingRefineMinRhoRatio}）"
                      + $"，厂商基准={(hasVendor ? "有" : "无")}");

    int frames = 0, ok = 0, defects = 0, errors = 0;
    var offsets = new List<double>();
    var problems = new List<string>();
    var deviations = new List<double>();
    var radiusRatios = new List<double>();
    var rows = new List<string> { "SN,帧标,判定,环半径比,支撑弧段,孔环比,内径比,偏移mm,环心x,环心y,"
                                  + "厂商环心x,厂商环心y,厂商半径,圆心偏差px,半径比_我们_厂商,耗时ms,依据" };

    foreach (var dir in dirs)
    {
        var files = Directory.GetFiles(dir, "*.jpg")
            .OrderBy(f => Path.GetFileName(f), StringComparer.Ordinal)
            .ToList();
        foreach (var file in files)
        {
            using var image = Cv2.ImRead(file, ImreadModes.Color);
            if (image.Empty())
                continue;
            var name = Path.GetFileName(file);
            var sceneAt = name.IndexOf("_Scene", StringComparison.Ordinal);
            var tag = sceneAt > 0 ? name[..sceneAt] : name;
            var sn = Path.GetFileName(dir);
            // SN 目录名形如 SN_CCSAGZZG01A253692509090000XX025080700034——各 SN 只有中间两位不同，
            // 直接截尾部会把所有帧都标成同一串数字。
            var snTag = sn.Length >= 32 ? sn.Substring(28, 4) : sn;
            frames++;
            var result = algo.Inspect(image, new WeldPoint { Index = 0, Name = tag });
            if (result.HasValidOffset)
                offsets.Add(result.OffsetDistanceMm);

            var verdict = result.HasError ? "异常" : result.IsOk ? "OK" : "NG";
            if (result.IsOk)
                ok++;
            else if (result.HasError)
            {
                errors++;
                problems.Add($"{snTag} / {tag}: 异常 — {result.ErrorMessage}");
            }
            else
            {
                defects++;
                problems.Add($"{snTag} / {tag}: NG — {result.DefectTextZh} ({result.ErrorMessage})");
            }

            var vendorCx = double.NaN;
            var vendorCy = double.NaN;
            var vendorR = double.NaN;
            var dev = double.NaN;
            var radiusRatio = double.NaN;
            if (hasVendor)
            {
                var vdb = Path.Combine(vdbRoot, sn, Path.GetFileNameWithoutExtension(name) + ".vdb");
                if (File.Exists(vdb)
                    && VendorVdbReader.TryRead(vdb, out var record, out _)
                    && record is { HasLocateRadius: true } && result.WeldRadiusSourcePx > 0)
                {
                    vendorCx = record.LocateCenterX;
                    vendorCy = record.LocateCenterY;
                    vendorR = record.LocateRadiusPx;
                    dev = Math.Sqrt(Math.Pow(result.WeldCenterSourcePx.X - vendorCx, 2)
                                    + Math.Pow(result.WeldCenterSourcePx.Y - vendorCy, 2));
                    radiusRatio = result.WeldRadiusSourcePx / vendorR;
                    deviations.Add(dev);
                    radiusRatios.Add(radiusRatio);
                }
            }

            var processedBase = Math.Max(1, Math.Min(result.ProcessedSizePx.Height, result.ProcessedSizePx.Width));
            rows.Add(string.Join(',', snTag, tag, verdict,
                Fmt(result.WeldRadiusPx > 0 ? result.WeldRadiusPx / processedBase : 0),
                Fmt(result.WeldArcSupport),
                Fmt(result.WeldRadiusPx > 0 ? result.PoleRadiusPx / result.WeldRadiusPx : 0),
                // 内径比 = 熔核内沿半径 ÷ 外半径；0 = 本次未取到内沿
                Fmt(result.WeldRadiusPx > 0 && result.WeldInnerRadiusPx > 0
                    ? result.WeldInnerRadiusPx / result.WeldRadiusPx : 0),
                Fmt(result.HasValidOffset ? result.OffsetDistanceMm : double.NaN),
                Fmt(result.WeldCenterSourcePx.X), Fmt(result.WeldCenterSourcePx.Y),
                Fmt(vendorCx), Fmt(vendorCy), Fmt(vendorR), Fmt(dev), Fmt(radiusRatio),
                Fmt(result.ProcessingMs),
                (result.ErrorMessage ?? string.Empty).Replace(',', ';')));
        }
    }

    if (csvPath is not null)
    {
        File.WriteAllLines(csvPath, rows, new System.Text.UTF8Encoding(true));
        Console.WriteLine($"逐帧明细已写入 {csvPath}");
    }

    offsets.Sort();
    var median = offsets.Count > 0 ? offsets[offsets.Count / 2] : 0;
    var p90 = offsets.Count > 0 ? offsets[(int)(offsets.Count * 0.9)] : 0;
    Console.WriteLine($"帧数 {frames}：合格 {ok}、NG {defects}、异常 {errors}；"
                      + $"偏移量 中位 {median:F3}mm、p90 {p90:F3}mm、max "
                      + $"{(offsets.Count > 0 ? offsets[^1] : 0):F3}mm");
    if (deviations.Count > 0)
    {
        deviations.Sort();
        radiusRatios.Sort();
        Console.WriteLine($"与厂商圆对照（{deviations.Count} 帧）：圆心偏差 中位 "
                          + $"{deviations[deviations.Count / 2]:F1}px、p90 "
                          + $"{deviations[(int)(deviations.Count * 0.9)]:F1}px、max {deviations[^1]:F1}px；"
                          + $"半径比 中位 {radiusRatios[radiusRatios.Count / 2]:F3}、"
                          + $"落在 [0.92,1.10] 外 {(radiusRatios.Count(r => r < 0.92 || r > 1.10))} 帧");
    }
    Console.WriteLine($"非 OK 清单（{problems.Count}）:");
    foreach (var p in problems)
        Console.WriteLine("  " + p);

    static string Fmt(double v) => double.IsNaN(v) ? "" : v.ToString("F4",
        System.Globalization.CultureInfo.InvariantCulture);
}

/// <summary>打印厂商 .vdb 里的定位结果（圆心/半径），作为独立参考基准。</summary>
static void RunVdbMode(string path)
{
    var record = VendorVdbReader.Read(path);
    Console.WriteLine($"文件: {Path.GetFileName(record.FileName)}");
    Console.WriteLine($"SN={record.Serial} Pos={record.Position} 时刻={record.Timestamp} "
                      + $"厂商NG={record.IsVendorNg?.ToString() ?? "未知"}");
    Console.WriteLine($"厂商圆: 圆心=({record.LocateCenterX:F1},{record.LocateCenterY:F1}) "
                      + (record.HasLocateRadius ? $"半径={record.LocateRadiusPx:F1}" : "半径=未记录")
                      + $" 焊偏距离={record.ResultD:F3}");
}

/// <summary>
/// 现场实拍图诊断：对单张图或整个目录逐张跑**真实算法**（不注入任何合成缺陷），
/// 打印判定、判定依据与关键几何量。用来回答"这张图为什么被判 NG/异常"。
/// </summary>
static void RunFileMode(string target, InspectionConfig cfg, NinePointCalibration calib,
    int width, int height, double mmPerPixel)
{
    var algo = new WeldInspectionAlgorithm(cfg, calib);

    var extensions = new[] { ".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff" };
    var files = Directory.Exists(target)
        ? Directory.GetFiles(target)
            .Where(f => extensions.Contains(Path.GetExtension(f).ToLowerInvariant()))
            .OrderBy(f => Path.GetFileName(f), StringComparer.Ordinal)
            .ToList()
        : new List<string> { target };

    Console.WriteLine($"配置 {width}×{height} @ {mmPerPixel}mm/px，工作分辨率上限 {cfg.MaxWorkingBase}px（0=不归一化）");
    Console.WriteLine("判定    帧标                     环r/短边  支撑弧段 孔/环   偏移        焊环圆心(原图)        极柱圆心(原图)        耗时    判定依据");

    var notOk = 0;
    foreach (var file in files)
    {
        using var image = Cv2.ImRead(file, ImreadModes.Color);
        if (image.Empty())
        {
            Console.WriteLine($"读取失败: {file}");
            continue;
        }

        var name = Path.GetFileName(file);
        var sceneAt = name.IndexOf("_Scene", StringComparison.Ordinal);
        var tag = sceneAt > 0 ? name[..sceneAt] : name;
        var point = new WeldPoint { Index = 0, Name = tag, SourceName = name };

        var result = algo.Inspect(image, point);
        if (!result.IsOk)
            notOk++;

        var verdict = result.HasError ? "异常" : result.IsOk ? "OK" : "NG";
        var processedBase = Math.Max(1, Math.Min(result.ProcessedSizePx.Height, result.ProcessedSizePx.Width));
        var ringRatio = result.WeldRadiusPx > 0 ? result.WeldRadiusPx / processedBase : 0;
        var pinRatio = result.WeldRadiusPx > 0 ? result.PoleRadiusPx / result.WeldRadiusPx : 0;
        var offset = result.HasValidOffset ? $"D={result.OffsetDistanceMm:F3}mm" : "D=不适用";
        var reason = string.IsNullOrWhiteSpace(result.ErrorMessage) ? string.Empty : result.ErrorMessage;

        // 圆心必须一起打印：只给半径/偏移量时，无法判断"这一帧是不是偏了"，
        // 也无法与厂商 .vdb 逐帧对照（本轮诊断就是靠这一列才看出黑环帧的偏差最大）。
        // 极柱圆心也要打：同一焊点/同一相机下，"焊环相对极柱的偏移向量"应当恒定，
        // 有了这两列才能把"测量退化"与"零件真的在动"分开。
        var center = result.WeldRadiusPx > 0
            ? $"({result.WeldCenterSourcePx.X:F1},{result.WeldCenterSourcePx.Y:F1})"
            : "-";
        var pin = result.PoleRadiusPx > 0
            ? $"({result.PoleCenterSourcePx.X:F1},{result.PoleCenterSourcePx.Y:F1})"
            : "-";
        Console.WriteLine($"{verdict,-6} {tag,-24} {ringRatio,8:F4} {result.WeldArcSupport,7:P0} "
                          + $"{pinRatio,7:F3} {offset,-12} {center,-21} {pin,-21} "
                          + $"{result.ProcessingMs,6:F0}ms  {reason}");
    }

    Console.WriteLine($"共 {files.Count} 张，非 OK {notOk} 张");
}

/// <summary>
/// 单帧焊环定位的**逐路径**探针：把 <see cref="WeldInspectionAlgorithm"/> 里那几条路径
/// 各自跑一遍并打印结果，用于判断"测不出来"卡在哪一步。
/// </summary>
static void RunRingProbe(string file, InspectionConfig cfg)
{
    using var bgr = Cv2.ImRead(file, ImreadModes.Color);
    if (bgr.Empty())
    {
        Console.WriteLine($"读取失败: {file}");
        return;
    }

    // 与 WeldInspectionAlgorithm.Inspect 完全相同的预处理（原生转灰度 → 归一化 → 3×3 中值）。
    using var grayFull = new Mat();
    Cv2.CvtColor(bgr, grayFull, ColorConversionCodes.BGR2GRAY);
    using var gray = new Mat();
    var work = grayFull;
    var normalized = false;
    var srcBase = Math.Min(grayFull.Height, grayFull.Width);
    if (cfg.MaxWorkingBase > 0 && srcBase > cfg.MaxWorkingBase)
    {
        var scale = cfg.MaxWorkingBase / (double)srcBase;
        Cv2.Resize(grayFull, gray, new Size(), scale, scale, InterpolationFlags.Area);
        work = gray;
        normalized = true;
    }
    Cv2.MedianBlur(work, work, 3);

    var baseSize = Math.Min(work.Height, work.Width);
    var margin = (int)(baseSize * cfg.BorderMarginRatio);
    Console.WriteLine($"文件: {Path.GetFileName(file)}");
    Console.WriteLine($"原图 {grayFull.Width}×{grayFull.Height} → 工作图 {work.Width}×{work.Height}"
                      + $"，base={baseSize}，归一化={normalized}，边框禁区 {margin}px");
    Console.WriteLine($"门限: 支撑弧段≥{cfg.MinRingArcSupport:P0}  族值带 [{cfg.MinRingRadiusRatio:F3},"
                      + $" {cfg.RingRadiusFamilyMaxRatio:F3}]  有效性上限 {cfg.MaxRingRadiusRatio:F3}");

    DumpMaskStages(work, cfg, baseSize, margin, out var dumpDir);

    Console.WriteLine();
    Console.WriteLine("---- 主路径（不做薄结构删除，不清贴边）----");
    using (var main = WeldRingLocator.Detect(work, cfg, normalized ? cfg.PreOpenKernelRatio : 0))
        Print(main, cfg);

    Console.WriteLine();
    Console.WriteLine("---- 核宽阶梯（激进薄结构删除）----");
    foreach (var k in cfg.FallbackPreOpenKernelRatios)
    {
        using var c = WeldRingLocator.Detect(work, cfg, k);
        Console.Write($"  preOpen={k:F3}: ");
        Print(c, cfg, inline: true);
    }

    Console.WriteLine();
    Console.WriteLine("---- 双尺度门控 ----");
    using (var dual = WeldRingLocator.Detect(work, cfg, 0, dualScaleGate: true))
        Print(dual, cfg);

    Console.WriteLine();
    Console.WriteLine("---- 第三路径（形态学之前先清贴边连通域）----");
    using (var cleaned = WeldRingLocator.Detect(work, cfg, 0, dualScaleGate: false, preCleanBorder: true))
    {
        Print(cleaned, cfg);
        var accepted = cleaned.Ok
                       && cleaned.RadiusRatio >= cfg.MinRingRadiusRatio
                       && cleaned.RadiusRatio <= cfg.MaxRingRadiusRatio;
        Console.WriteLine("  第三路径是否被采纳（要求半径 ∈ "
                          + $"[{cfg.MinRingRadiusRatio:F3}, {cfg.MaxRingRadiusRatio:F3}]）: "
                          + (accepted
                              ? "是"
                              : cleaned.Ok
                                  ? $"否——半径比 {cleaned.RadiusRatio:F4} 越界"
                                  : "否——它自己就没给出可信结果"));
    }

    // ---- 边界精修：两个起点各自的结果与仲裁量 ----
    // 用户报的"黑环导致焊环识别偏移"就发生在这一步：两个起点各给一个圆，由仲裁规则二选一。
    // 只打印最终圆心看不出"是谁赢的、凭什么赢"，必须把 arc / 残差 / 截断比一起打出来。
    //
    // 种子的来源要分清：算法在"主路径失败"时会走核宽阶梯**按支撑弧段择优**采纳某一档，
    // 随后**双尺度门控只要"半径落族值带 + 弧段达标"就会覆盖阶梯结果**（见
    // WeldInspectionAlgorithm 的重试块），所以精修的种子常常是这几条路径之一。
    // **必须把它们全部遍历**：各档种子只差零点几像素，却可能翻转边缘决策——
    // 本工具曾经同时漏掉"全部阶梯档"与"双尺度"两个来源，于是探针的结论与算法不一致。
    Console.WriteLine();
    Console.WriteLine("---- 边界精修 ----");
    var refineSeeds = cfg.FallbackPreOpenKernelRatios
        .Select(k => ($"阶梯 {k:F3}", (double?)k))
        .Append(("双尺度门控（弧段达标时会覆盖阶梯）", (double?)1.5))
        .Append(("主路径（不清贴边）", (double?)null))
        .Append(("第三路径（形态学前先清贴边）", (double?)double.NaN))
        .ToList();
    foreach (var (tag, kernel) in refineSeeds)
    {
        using var seed = kernel is null
            ? WeldRingLocator.Detect(work, cfg, 0)
            : double.IsNaN(kernel.Value)
                ? WeldRingLocator.Detect(work, cfg, 0, dualScaleGate: false, preCleanBorder: true)
                : Math.Abs(kernel.Value - 1.5) < 1e-9
                    ? WeldRingLocator.Detect(work, cfg, 0, dualScaleGate: true)
                    : WeldRingLocator.Detect(work, cfg, kernel.Value);
        Console.WriteLine($"  [{tag}]");
        if (!seed.Ok)
        {
            Console.WriteLine("    未给出候选");
            continue;
        }
        Console.WriteLine($"    种子 ({seed.Center.X:F1},{seed.Center.Y:F1}) r={seed.Radius:F1}"
                          + $" 半径比={seed.RadiusRatio:F4} 支撑弧段={seed.ArcSupport:P1}");
        var hole = HoleLocator.Locate(work, seed.Texture!, seed.Threshold, seed.Center,
            seed.Radius, cfg);
        var pinCenter = hole?.Center ?? PinLocator.Locate(work, seed.Center, seed.BaseSize, cfg)?.Center;
        Console.WriteLine($"    孔心来源: {(hole is not null ? "HoleLocator 三材质分层" : "PinLocator 卡尺兜底")}"
                          + (pinCenter is { } pc ? $"  孔心=({pc.X:F1},{pc.Y:F1})" : "  未定位到孔位"));
        var arbitrated = WeldRingLocator.RefineBoundary(work, seed.Center, seed.Radius, cfg,
            pinCenter, msg => Console.WriteLine(msg));
        if (arbitrated is { } av)
        {
            var b0 = Math.Min(work.Height, work.Width);
            Console.WriteLine($"    → 采用 ({av.Circle.Center.X:F1},{av.Circle.Center.Y:F1})"
                              + $" r={av.Circle.Radius:F1} 半径比={av.Circle.Radius / b0:F4}"
                              + $" 支撑弧段={av.ArcSupport:P1} 残差={av.ResidualScatter:F4}");
        }
    }
}


/// <summary>
/// 打印 <see cref="WeldRingLocator.Detect"/> 形态学**之前**的原始掩码连通域统计，以及
/// "先清贴边"之后剩下的连通域——这两组数决定第三路径到底看见了什么。
/// 这是对 Detect 前段的复刻（照抄参数，不改语义）。
/// 同时把工作灰度图 / 纹理能量图 / 原始掩码 / 清贴边后掩码写到 <c>.scratch/ringprobe/</c>，
/// 供人眼核对"掩码里到底有什么"。
/// </summary>
static void DumpMaskStages(Mat gray8, InspectionConfig cfg, int baseSize, int margin,
    out string dumpDir)
{
    dumpDir = Path.Combine(FindRepoRoot(), ".scratch", "ringprobe");
    Directory.CreateDirectory(dumpDir);

    var win = Geo.Odd(baseSize * cfg.TextureWindowRatio, 5, 15);
    using var energy = TextureEnergy(gray8, win);
    ZeroBorder(energy, margin);
    var step = Math.Max(1, baseSize / 256);
    var median = Geo.Percentile32F(energy, 50, step);
    var p99 = Geo.Percentile32F(energy, 99, step);
    var threshold = median + cfg.TextureThresholdK * (p99 - median);

    using var mask = new Mat();
    Cv2.Compare(energy, new Scalar(threshold), mask, CmpType.GT);

    Console.WriteLine();
    Console.WriteLine($"---- 原始掩码（阈值 {threshold:F2} = 中位 {median:F2} + "
                      + $"{cfg.TextureThresholdK:F2}×(p99 {p99:F2} − 中位)），形态学之前 ----");
    DumpComponents(mask, baseSize, margin, "原始");

    using var cleared = mask.Clone();
    RemoveBorderComponents(cleared, margin);
    Console.WriteLine("   把贴边连通域清掉之后:");
    DumpComponents(cleared, baseSize, margin, "清贴边后");

    Cv2.ImWrite(Path.Combine(dumpDir, "01_gray.png"), gray8);
    using (var energyView = new Mat())
    {
        Cv2.Normalize(energy, energyView, 0, 255, NormTypes.MinMax, MatType.CV_8U);
        Cv2.ImWrite(Path.Combine(dumpDir, "02_energy.png"), energyView);
    }
    Cv2.ImWrite(Path.Combine(dumpDir, "03_mask_raw.png"), mask);
    Cv2.ImWrite(Path.Combine(dumpDir, "04_mask_bordercleaned.png"), cleared);

    // 形态学各阶段的掩码：复刻 Detect 的 `preOpen? → close×2 → open`，
    // 用来回答"激进阶梯做完之后到底剩了什么"。
    foreach (var k in new[] { 0.0 }.Concat(cfg.FallbackPreOpenKernelRatios))
    {
        using var staged = mask.Clone();
        if (k > 0)
        {
            var maxPreOpenK = Math.Max(3, (int)Math.Round(baseSize * 0.11) | 1);
            var preOpenK = Geo.Odd(baseSize * k, 3, maxPreOpenK);
            using var se = Cv2.GetStructuringElement(MorphShapes.Ellipse, new Size(preOpenK, preOpenK));
            Cv2.MorphologyEx(staged, staged, MorphTypes.Open, se);
        }
        var closeK = Geo.Odd(baseSize * cfg.CloseKernelRatio, 3, 21);
        using (var se = Cv2.GetStructuringElement(MorphShapes.Ellipse, new Size(closeK, closeK)))
            Cv2.MorphologyEx(staged, staged, MorphTypes.Close, se, iterations: 2);
        var openK = Geo.Odd(baseSize * cfg.OpenKernelRatio, 3, 15);
        using (var se = Cv2.GetStructuringElement(MorphShapes.Ellipse, new Size(openK, openK)))
            Cv2.MorphologyEx(staged, staged, MorphTypes.Open, se);

        var tag = k > 0 ? $"preOpen{k:F3}" : "preOpen0";
        Cv2.ImWrite(Path.Combine(dumpDir, $"06_morph_{tag}.png"), staged);
        Console.WriteLine($"   形态学阶段 preOpen={k:F3} → 剩余像素 {Cv2.CountNonZero(staged)}，"
                          + $"连通域（面积≥{baseSize * baseSize * 0.002:F0}）: {DescribeComponents(staged, baseSize, margin)}");
        TraceCandidate(gray8, staged, cfg, baseSize, margin, tag, dumpDir);
    }

    // 连通域着色图：把每个面积达标的连通域涂成不同的灰度，用来判断"焊环到底跟谁连在一起"。
    // 纯灰度分不出"哪个白块属于哪个域"，而这正是本帧失败的关键事实。
    using (var labels = new Mat())
    using (var stats = new Mat())
    using (var centroids = new Mat())
    {
        var count = Cv2.ConnectedComponentsWithStats(mask, labels, stats, centroids,
            PixelConnectivity.Connectivity8);
        using var colored = new Mat(mask.Size(), MatType.CV_8UC1, Scalar.Black);
        var shade = 255;
        for (var i = 1; i < count; i++)
        {
            var area = stats.At<int>(i, (int)ConnectedComponentsTypes.Area);
            if (area < baseSize * baseSize * 0.002)
                continue;
            using var comp = new Mat();
            Cv2.Compare(labels, new Scalar(i), comp, CmpType.EQ);
            colored.SetTo(Scalar.All(shade), comp);
            var bbox = new Rect(
                stats.At<int>(i, (int)ConnectedComponentsTypes.Left),
                stats.At<int>(i, (int)ConnectedComponentsTypes.Top),
                stats.At<int>(i, (int)ConnectedComponentsTypes.Width),
                stats.At<int>(i, (int)ConnectedComponentsTypes.Height));
            var touches = bbox.Left <= margin || bbox.Top <= margin
                          || bbox.Right >= mask.Width - margin || bbox.Bottom >= mask.Height - margin;
            Console.WriteLine($"   着色 {shade,3}: area={area,7} bbox=({bbox.X},{bbox.Y},{bbox.Width},{bbox.Height})"
                              + (touches ? " 贴边" : ""));
            shade -= 60;
            if (shade < 60) shade = 255;
        }
        Cv2.ImWrite(Path.Combine(dumpDir, "05_mask_components.png"), colored);
    }

    Console.WriteLine($"   中间图已写入 {dumpDir}");
}

static string FindRepoRoot()
{
    var dir = new DirectoryInfo(AppContext.BaseDirectory);
    while (dir is not null)
    {
        if (File.Exists(Path.Combine(dir.FullName, "BatteryWeldAOI.sln")))
            return dir.FullName;
        dir = dir.Parent;
    }
    return AppContext.BaseDirectory;
}

/// <summary>一行摘要：面积达标且**不贴边**的连通域（贴边的那种会被"贴边=背景"规则剔掉）。</summary>
static string DescribeComponents(Mat mask, int baseSize, int margin)
{
    using var labels = new Mat();
    using var stats = new Mat();
    using var centroids = new Mat();
    var count = Cv2.ConnectedComponentsWithStats(mask, labels, stats, centroids,
        PixelConnectivity.Connectivity8);
    var minArea = baseSize * baseSize * 0.002;
    var kept = new List<string>();
    var dropped = 0;
    for (var i = 1; i < count; i++)
    {
        var area = stats.At<int>(i, (int)ConnectedComponentsTypes.Area);
        if (area < minArea)
            continue;
        var bbox = new Rect(
            stats.At<int>(i, (int)ConnectedComponentsTypes.Left),
            stats.At<int>(i, (int)ConnectedComponentsTypes.Top),
            stats.At<int>(i, (int)ConnectedComponentsTypes.Width),
            stats.At<int>(i, (int)ConnectedComponentsTypes.Height));
        var touches = bbox.Left <= margin || bbox.Top <= margin
                      || bbox.Right >= mask.Width - margin || bbox.Bottom >= mask.Height - margin;
        if (touches)
        {
            dropped++;
            continue;
        }
        kept.Add($"area={area} bbox=({bbox.X},{bbox.Y},{bbox.Width},{bbox.Height})");
    }

    return kept.Count == 0
        ? $"{dropped} 个全部贴边 → 候选数 0"
        : string.Join(" | ", kept) + (dropped > 0 ? $"（另有 {dropped} 个贴边已剔）" : "");
}

/// <summary>
/// 复刻 <see cref="WeldRingLocator.Detect"/> 的**候选选择 + 圆拟合**尾部，把选中的连通域轮廓
/// 与拟合圆叠在工作灰度图上导出，用来回答"支撑弧段为什么这么低"。
/// </summary>
static void TraceCandidate(Mat gray8, Mat mask, InspectionConfig cfg, int baseSize, int margin,
    string tag, string dumpDir)
{
    using var labels = new Mat();
    using var stats = new Mat();
    using var centroids = new Mat();
    var count = Cv2.ConnectedComponentsWithStats(mask, labels, stats, centroids,
        PixelConnectivity.Connectivity8);

    Rect best = default;
    double bestFilled = 0;
    double bestRoundness = 0;
    for (var i = 1; i < count; i++)
    {
        var area = stats.At<int>(i, (int)ConnectedComponentsTypes.Area);
        if (area < baseSize * baseSize * cfg.MinWeldBlobRatio)
            continue;
        var bbox = new Rect(
            stats.At<int>(i, (int)ConnectedComponentsTypes.Left),
            stats.At<int>(i, (int)ConnectedComponentsTypes.Top),
            stats.At<int>(i, (int)ConnectedComponentsTypes.Width),
            stats.At<int>(i, (int)ConnectedComponentsTypes.Height));
        if (bbox.Left <= margin || bbox.Top <= margin
            || bbox.Right >= mask.Width - margin || bbox.Bottom >= mask.Height - margin)
            continue;

        using var sub = new Mat(labels, bbox);
        using var comp = new Mat();
        Cv2.Compare(sub, new Scalar(i), comp, CmpType.EQ);
        using var filled = FillHoles(comp);
        var filledArea = Cv2.CountNonZero(filled);
        using var dt = new Mat();
        Cv2.DistanceTransform(filled, dt, DistanceTypes.L2, DistanceTransformMasks.Mask5);
        Cv2.MinMaxLoc(dt, out _, out var dtMax, out _, out _);
        var roundness = dtMax / Math.Max(Math.Sqrt(filledArea / Math.PI), 1e-6);
        if (roundness < cfg.MinWeldRoundness || roundness > cfg.MaxWeldRoundness)
            continue;
        if (filledArea <= bestFilled)
            continue;
        best = bbox;
        bestFilled = filledArea;
        bestRoundness = roundness;
    }

    if (bestFilled <= 0)
    {
        Console.WriteLine($"     [{tag}] 候选选择：无（未过圆度/面积/贴边筛选）");
        return;
    }

    using var sub2 = new Mat(labels, best);
    using var comp2 = new Mat();
    var label = (int)sub2.At<int>(0, 0);
    // 取包围盒内属于该连通域的像素质心对应的标签（At(0,0) 可能落在域外，改用众数）
    var counts = new Dictionary<int, int>();
    for (var y = 0; y < sub2.Rows; y++)
        for (var x = 0; x < sub2.Cols; x++)
        {
            var v = sub2.At<int>(y, x);
            if (v == 0) continue;
            counts[v] = counts.GetValueOrDefault(v) + 1;
        }
    label = counts.OrderByDescending(kv => kv.Value).First().Key;
    Cv2.Compare(sub2, new Scalar(label), comp2, CmpType.EQ);
    using var filledBest = FillHoles(comp2);

    var solid = new Mat(new Size(mask.Width, mask.Height), MatType.CV_8UC1, Scalar.Black);
    using (var roi = new Mat(solid, best))
        filledBest.CopyTo(roi);
    using var forContours = solid.Clone();
    Cv2.FindContours(forContours, out var contours, out _,
        RetrievalModes.External, ContourApproximationModes.ApproxNone);
    if (contours.Length == 0)
    {
        solid.Dispose();
        Console.WriteLine($"     [{tag}] 候选选择：轮廓提取失败");
        return;
    }
    var biggest = contours.MaxBy(c => Cv2.ContourArea(c))!;
    var pts = biggest.Select(p => new Point2d(p.X, p.Y)).ToList();
    using var dtBest = new Mat();
    Cv2.DistanceTransform(filledBest, dtBest, DistanceTypes.L2, DistanceTransformMasks.Mask5);
    Cv2.MinMaxLoc(dtBest, out _, out _, out _, out var peak);
    var seed = new Point2d(best.X + peak.X, best.Y + peak.Y);

    var fit = Geo.MedianRadiusCircle(pts, seed);
    Console.WriteLine($"     [{tag}] 候选 bbox=({best.X},{best.Y},{best.Width},{best.Height}) "
                      + $"填洞面积={bestFilled:F0} 圆度={bestRoundness:F2} → 拟合"
                      + (fit is { } f
                          ? $" 圆心=({f.Circle.Center.X:F1},{f.Circle.Center.Y:F1}) 半径={f.Circle.Radius:F1}"
                            + $" 半径比={f.Circle.Radius / baseSize:F4} 支撑弧段={f.ArcSupport:P0}"
                          : " 失败"));

    using var view = new Mat();
    Cv2.CvtColor(gray8, view, ColorConversionCodes.GRAY2BGR);
    Cv2.DrawContours(view, contours, -1, new Scalar(0, 0, 255), 1);
    if (fit is { } ff)
        Cv2.Circle(view, (Point)ff.Circle.Center, (int)Math.Round(ff.Circle.Radius), new Scalar(0, 255, 0), 1);
    Cv2.ImWrite(Path.Combine(dumpDir, $"07_candidate_{tag}.png"), view);
    solid.Dispose();
}

static void DumpComponents(Mat mask, int baseSize, int margin, string label)
{
    using var labels = new Mat();
    using var stats = new Mat();
    using var centroids = new Mat();
    var count = Cv2.ConnectedComponentsWithStats(mask, labels, stats, centroids,
        PixelConnectivity.Connectivity8);
    var minArea = baseSize * baseSize * 0.002;
    var rows = new List<(int Area, Rect BBox, double Ratio, double Inscribed)>();

    for (var i = 1; i < count; i++)
    {
        var area = stats.At<int>(i, (int)ConnectedComponentsTypes.Area);
        if (area < minArea)
            continue;
        var bbox = new Rect(
            stats.At<int>(i, (int)ConnectedComponentsTypes.Left),
            stats.At<int>(i, (int)ConnectedComponentsTypes.Top),
            stats.At<int>(i, (int)ConnectedComponentsTypes.Width),
            stats.At<int>(i, (int)ConnectedComponentsTypes.Height));
        using var sub = new Mat(labels, bbox);
        using var comp = new Mat();
        Cv2.Compare(sub, new Scalar(i), comp, CmpType.EQ);
        using var filled = FillHoles(comp);
        var filledArea = Cv2.CountNonZero(filled);
        using var dt = new Mat();
        Cv2.DistanceTransform(filled, dt, DistanceTypes.L2, DistanceTransformMasks.Mask5);
        Cv2.MinMaxLoc(dt, out _, out var dtMax, out _, out _);
        var ratio = Math.Sqrt(filledArea / Math.PI) / baseSize;
        rows.Add((area, bbox, ratio, dtMax));
    }

    Console.WriteLine($"   [{label}] 面积≥{minArea:F0} 的连通域 {rows.Count} 个（贴边禁区 {margin}px）:");
    foreach (var r in rows.OrderByDescending(r => r.Area).Take(8))
    {
        var touches = r.BBox.Left <= margin || r.BBox.Top <= margin
                      || r.BBox.Right >= mask.Width - margin || r.BBox.Bottom >= mask.Height - margin;
        Console.WriteLine($"     area={r.Area,7} bbox=({r.BBox.X},{r.BBox.Y},{r.BBox.Width},{r.BBox.Height})"
                          + $" 等效半径比={r.Ratio:F4} 内切半径={r.Inscribed:F1}px"
                          + (touches ? "  **贴边**" : ""));
    }
}

static void Print(RingDetection d, InspectionConfig cfg, bool inline = false)
{
    var prefix = inline ? "" : "  ";
    if (!d.Ok)
    {
        Console.WriteLine($"{prefix}失败({d.FailureKind}): {d.Failure}");
        return;
    }

    Console.WriteLine($"{prefix}OK 圆心=({d.Center.X:F1},{d.Center.Y:F1}) 半径={d.Radius:F1}"
                      + $" 半径比={d.RadiusRatio:F4} 支撑弧段={d.ArcSupport:P1} 圆度={d.Roundness:F2}"
                      + (d.RadiusRatio > cfg.RingRadiusFamilyMaxRatio ? "  ← 在族值带外" : ""));
}

static Mat TextureEnergy(Mat gray8, int win)
{
    using var f = new Mat();
    gray8.ConvertTo(f, MatType.CV_32F);
    using var f2 = new Mat();
    Cv2.Multiply(f, f, f2);
    using var mean = new Mat();
    using var meanSq = new Mat();
    Cv2.BoxFilter(f, mean, MatType.CV_32F, new Size(win, win));
    Cv2.BoxFilter(f2, meanSq, MatType.CV_32F, new Size(win, win));
    using var meanSquared = new Mat();
    Cv2.Multiply(mean, mean, meanSquared);
    var energy = new Mat();
    Cv2.Subtract(meanSq, meanSquared, energy);
    Cv2.Threshold(energy, energy, 0, 0, ThresholdTypes.Tozero);
    Cv2.Sqrt(energy, energy);
    return energy;
}

static void ZeroBorder(Mat mat, int margin)
{
    if (margin <= 0)
        return;
    var w = mat.Width;
    var h = mat.Height;
    using (var r = new Mat(mat, new Rect(0, 0, w, margin))) r.SetTo(Scalar.All(0));
    using (var r = new Mat(mat, new Rect(0, h - margin, w, margin))) r.SetTo(Scalar.All(0));
    using (var r = new Mat(mat, new Rect(0, 0, margin, h))) r.SetTo(Scalar.All(0));
    using (var r = new Mat(mat, new Rect(w - margin, 0, margin, h))) r.SetTo(Scalar.All(0));
}

static void RemoveBorderComponents(Mat mask, int margin)
{
    if (margin <= 0)
        return;
    var w = mask.Width;
    var h = mask.Height;
    using var labels = new Mat();
    using var stats = new Mat();
    using var centroids = new Mat();
    var count = Cv2.ConnectedComponentsWithStats(mask, labels, stats, centroids,
        PixelConnectivity.Connectivity8);
    for (var i = 1; i < count; i++)
    {
        var left = stats.At<int>(i, (int)ConnectedComponentsTypes.Left);
        var top = stats.At<int>(i, (int)ConnectedComponentsTypes.Top);
        var width = stats.At<int>(i, (int)ConnectedComponentsTypes.Width);
        var height = stats.At<int>(i, (int)ConnectedComponentsTypes.Height);
        if (left > margin && top > margin && left + width < w - margin && top + height < h - margin)
            continue;
        using var comp = new Mat();
        Cv2.Compare(labels, new Scalar(i), comp, CmpType.EQ);
        mask.SetTo(Scalar.All(0), comp);
    }
}

static Mat FillHoles(Mat bin)
{
    using var padded = new Mat();
    Cv2.CopyMakeBorder(bin, padded, 1, 1, 1, 1, BorderTypes.Constant, Scalar.Black);
    using var flooded = padded.Clone();
    Cv2.FloodFill(flooded, new Point(0, 0), Scalar.White);
    using var holes = new Mat();
    Cv2.BitwiseNot(flooded, holes);
    using var inner = new Mat(holes, new Rect(1, 1, bin.Width, bin.Height));
    var result = new Mat();
    Cv2.BitwiseOr(bin, inner, result);
    Cv2.Threshold(result, result, 0, 255, ThresholdTypes.Binary);
    return result;
}
