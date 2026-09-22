using BatteryWeldAOI.Core.Diagnostics;
using OpenCvSharp;
using Xunit;
using Xunit.Abstractions;

namespace BatteryWeldAOI.Core.Tests;

/// <summary>
/// 厂商 VDB 读取器测试。
///
/// 现场语料里的 <c>VdbGui_pic/**/*.vdb</c> 是厂商软件的私有格式（魔数 <c>BDV.</c>），
/// 此前项目只识别到魔数就当"不可读"（见 PpSampleTests 类注释）。
/// 现在能读出厂商自己的圆定位结果，它给了"焊环定位"这一步一份 20955 帧覆盖、
/// 零人工标注的参考基准——所以解析正确性必须被测试守住：
/// 一旦厂商换模板、头部偏移漂移，这里会失败而不是静默给出错值。
///
/// 样本集可能不随源码分发；找不到目录的测试跳过而非失败（与 RealSampleTests 同口径）。
/// 不依赖样本的用例（文件名解析、非 VDB 文件拒绝）始终运行。
/// </summary>
public class VendorVdbReaderTests
{
    private readonly ITestOutputHelper _output;

    public VendorVdbReaderTests(ITestOutputHelper output) => _output = output;

    // -----------------------------------------------------------------------------------
    //  纯逻辑：不依赖现场样本
    // -----------------------------------------------------------------------------------

    [Fact]
    public void ParseFileName_SplitsAllConventionFields()
    {
        const string name =
            "19-22-06 113_Scene1Pos0_index1_SN_CCSAGZZG01A2536925090900002025080700034_OK_255.vdb";

        var parts = VendorVdbReader.ParseFileName(name);

        Assert.Equal("SN_CCSAGZZG01A2536925090900002025080700034", parts.Serial);
        Assert.Equal("Pos0", parts.Position);
        Assert.Equal("19-22-06 113", parts.Timestamp);
        Assert.False(parts.IsVendorNg);

        var ng = VendorVdbReader.ParseFileName(name.Replace("_OK_", "_NG_"));
        Assert.True(ng.IsVendorNg);

        // 文件名没有判定标记时必须返回 null，而不是默认成 OK——
        // 现场语料里判定结论**只**存在于文件名，把"没标记"读成"合格"会凭空造出一个良品
        var unlabeled = VendorVdbReader.ParseFileName("whatever_SN_ABC.vdb");
        Assert.Null(unlabeled.IsVendorNg);
    }

    [Fact]
    public void NonVdbFile_IsRejectedWithReason()
    {
        var path = Path.Combine(Path.GetTempPath(), $"bwaoi_not_vdb_{Guid.NewGuid():N}.vdb");
        File.WriteAllBytes(path, System.Text.Encoding.ASCII.GetBytes("not a vdb file at all, wrong magic"));
        try
        {
            Assert.False(VendorVdbReader.TryRead(path, out var record, out var error));
            Assert.Null(record);
            Assert.Contains("魔数", error);
        }
        finally
        {
            File.Delete(path);
        }
    }

    // -----------------------------------------------------------------------------------
    //  真实样本：字段正确性与"原图即渲染图"这一关键结论
    // -----------------------------------------------------------------------------------

    [Fact]
    public void RealCorpus_ParsesLocateResultForEveryFile()
    {
        var files = FindSampleVdbs(limit: 60);
        if (files.Count == 0)
        {
            _output.WriteLine("未找到 tests/pp 下的 .vdb 样本，跳过。");
            return;
        }

        var parsed = 0;
        foreach (var file in files)
        {
            Assert.True(VendorVdbReader.TryRead(file, out var record, out var error),
                $"{Path.GetFileName(file)}: 解析失败 — {error}");

            // 定位圆心必须落在图像范围内。头部里的 double 偏移一旦漂移，
            // 最容易出现的症状就是读到一个"合法但荒谬"的坐标（例如把指针当坐标），本断言即拦它。
            Assert.InRange(record!.LocateCenterX, 0, 6000);
            Assert.InRange(record.LocateCenterY, 0, 6000);
            Assert.False(string.IsNullOrEmpty(record.Serial));
            Assert.False(string.IsNullOrEmpty(record.Timestamp));
            Assert.Matches(@"^Pos\d$", record.Position);

            // 定位半径是模板相关的尽力而为字段；现场语料实测中位约 481px（约 0.13×短边）
            Assert.True(record.HasLocateRadius, $"{Path.GetFileName(file)}: 未取到定位半径");
            Assert.InRange(record.LocateRadiusPx, 200, 900);

            // 结果字符串的第三个值被厂商钳在 ±7.425（可为负），语义未定，只保证解析正确
            Assert.InRange(record.ResultD, -7.5, 7.5);

            parsed++;
        }

        _output.WriteLine($"解析 {parsed} 个 vdb 全部通过字段合理性检查");
    }

    /// <summary>
    /// vdb 内嵌的 JPEG 与同名 Orignal_pic jpg **逐像素完全相同**。
    ///
    /// 这条结论改变了这批图像的定位：所谓"原图"其实是厂商 VDB GUI 的渲染输出，
    /// 而不是原始相机帧——它带着软件画上去的叠加物件，且是 JPEG 质量 1（0.105 bit/px）
    /// 的重压缩结果。这正是"用这批图训练/做亚像素测量"时必须先知道的域信息，
    /// 所以用测试把它钉住：哪天现场换成导出原始帧，本测试会失败并提醒重新评估。
    /// </summary>
    [Fact]
    public void EmbeddedJpeg_IsPixelIdenticalToSiblingOriginalJpg()
    {
        var vdb = FindSampleVdbs(limit: 200)
            .FirstOrDefault(f => FindSiblingJpg(f) is not null);
        if (vdb is null)
        {
            _output.WriteLine("未找到可配对的 vdb/jpg 样本，跳过。");
            return;
        }

        var jpg = FindSiblingJpg(vdb)!;
        Assert.True(VendorVdbReader.TryReadEmbeddedImage(vdb, out var embedded, out var error), error);
        using var jpeg = embedded!;
        using var original = Cv2.ImRead(jpg, ImreadModes.Grayscale);

        Assert.False(original.Empty());
        Assert.Equal(original.Width, jpeg.Width);
        Assert.Equal(original.Height, jpeg.Height);

        using var grayEmbedded = new Mat();
        Cv2.CvtColor(jpeg, grayEmbedded, ColorConversionCodes.BGR2GRAY);
        using var diff = new Mat();
        Cv2.Absdiff(grayEmbedded, original, diff);
        Cv2.MinMaxLoc(diff, out _, out double max);

        // 允许 JPEG 色度转换带来的极小往返误差，但不允许"有叠加/无叠加"这种量级的差别
        Assert.True(max <= 2, $"内嵌图与 {Path.GetFileName(jpg)} 不一致：最大差值 {max}");
        _output.WriteLine($"{Path.GetFileName(vdb)} 内嵌图与同名 jpg 一致（{jpeg.Width}×{jpeg.Height}，最大差 {max}）");
    }

    // -----------------------------------------------------------------------------------

    /// <summary>现场 vdb 样本（优先取 Pos0，便于与 jpg 配对）；找不到目录时返回空。</summary>
    private static List<string> FindSampleVdbs(int limit)
    {
        var root = FindPpRoot();
        if (root is null)
            return new List<string>();

        return Directory.EnumerateFiles(root, "*.vdb", SearchOption.AllDirectories)
            .Where(f => f.Contains("VdbGui_pic", StringComparison.OrdinalIgnoreCase))
            .Where(f => Path.GetFileName(f).Contains("Pos0", StringComparison.Ordinal))
            .OrderBy(f => f, StringComparer.Ordinal)
            .Take(limit)
            .ToList();
    }

    private static string? FindSiblingJpg(string vdb)
    {
        var jpg = Path.ChangeExtension(
            vdb.Replace("VdbGui_pic", "Orignal_pic", StringComparison.OrdinalIgnoreCase), ".jpg");
        return File.Exists(jpg) ? jpg : null;
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
