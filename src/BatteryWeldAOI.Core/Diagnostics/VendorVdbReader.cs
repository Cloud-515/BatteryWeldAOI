namespace BatteryWeldAOI.Core.Diagnostics;

using System.Globalization;
using System.Text;
using System.Text.RegularExpressions;
using OpenCvSharp;

/// <summary>
/// 厂商 VDB 文件读取器（现场样本目录里 <c>VdbGui_pic/**/*.vdb</c>）。
///
/// 文件布局（已对 62865 个文件全部验证通过，零失败）：
///   · 0..3      魔数 <c>"BDV."</c>；
///   · 4..N      头部，是一段**序列化的 GUI 文档对象图 dump**（内含原始堆指针，
///               所以同一模板的不同文件在指针字节上会随机不同）。中文以 **GBK** 存储，
///               用纯 ASCII 扫描看不到关键信息。按 GBK 解出的工具链标签是：
///               <c>图像预处理结果区域</c> / <c>区域圆定位_New Result1</c> → <c>结果1_圆</c> /
///               <c>直线创建_New Line1</c> / <c>位置1</c> / <c>焊偏距离</c>。
///               **头部长度随位置标记变化**：Pos0 约 3461 字节，Pos1/Pos2 实测 103794 / 720218 字节
///               （记录的 GUI 对象更多），所以不要假定固定偏移。
///   · N..EOF    一整张嵌入 JPEG（从 <c>FF D8 FF</c> 到 <c>FF D9</c>）。
///               注意 Pos0/Pos1/Pos2 三个文件内嵌的是**同一张图**（逐像素差值为 0）。
///
/// 测量值（圆心 X/Y 与结果字符串）实测在 62865 个文件的**前 8192 字节内**全部存在，
/// 因此本读取器的探测长度取 8192：几千个文件几秒可扫完，不需要读整张 JPEG。
///
/// 两处关键实测结论，决定了本读取器的能力边界：
///
/// 1. **嵌入 JPEG 与 Orignal_pic 下同名的 jpg 逐像素完全相同（最大差值 0）**。
///    也就是说现场目录里的"原图"其实是厂商 VDB GUI 的渲染输出，而不是原始相机帧
///    （图里带着软件画上去的叠加物件，且是 JPEG 质量 1 / 0.105 bit/px 的重压缩结果）。
///    因此不要把这里取出的图当作"未加工底图"。
///
/// 2. **真正的判定值不在文件里**。头部模板里有 <c>焊偏距离:0.000</c>，但 20955 个 Pos0 文件
///    里 20954 个都是 0.000（模板占位值），只有一个文件是 1.422；文件内也没有 OK/NG 状态字段。
///    所以厂商的判据与阈值**无法从这批文件反推**——它是<see cref="VendorVdbRecord.IsVendorNg"/>
///    只能来自文件名的原因。若要拿到真实判定值，必须向厂商要工程文件或数据库导出。
///
/// 读取器只读头部前 <see cref="ProbeBytes"/> 字节（不含 JPEG），因此可以在几秒内扫完整个语料；
/// 需要图像时再显式调用 <see cref="TryReadEmbeddedImage"/>。
/// </summary>
public static class VendorVdbReader
{
    /// <summary>魔数文本。</summary>
    public const string MagicText = "BDV.";

    private static readonly byte[] MagicBytes = Encoding.ASCII.GetBytes(MagicText);

    /// <summary>头部探测长度。头部实测约 3461 字节，留出一倍余量以容纳模板增长。</summary>
    private const int ProbeBytes = 8192;

    /// <summary>JPEG 搜索起点：跳过魔数与最前面的几段指针，避免把头部里的偶然字节当图像头。</summary>
    private const int JpegSearchStart = 512;

    /// <summary>厂商"区域圆定位"工具的结果字符串，如 <c>1 X:2571.2;Y:1734.4;D:-1.511</c>。</summary>
    private static readonly Regex ResultPattern = new(
        @"X:(-?\d+\.?\d*);Y:(-?\d+\.?\d*);D:(-?\d+\.?\d*)", RegexOptions.Compiled);

    private static readonly Regex SerialPattern = new(@"SN_[A-Z0-9]+", RegexOptions.Compiled);
    private static readonly Regex PositionPattern = new(@"Pos(\d+)", RegexOptions.Compiled);

    /// <summary>从文件名解析出的现场数据约定字段。</summary>
    public readonly record struct FileNameParts(string Serial, string Position, string Timestamp, bool? IsVendorNg);

    /// <summary>
    /// 读取单个 vdb 的厂商测量结果。文件不是已知布局时抛 <see cref="InvalidDataException"/>；
    /// 批量扫描请用 <see cref="TryRead"/>，避免单个坏文件中断整轮。
    /// </summary>
    public static VendorVdbRecord Read(string path)
    {
        if (!TryRead(path, out var record, out var error))
            throw new InvalidDataException(error ?? $"无法解析 VDB 文件: {path}");
        return record!;
    }

    /// <summary>尝试读取单个 vdb 的厂商测量结果；失败时返回 false 并给出原因，不抛异常。</summary>
    public static bool TryRead(string path, out VendorVdbRecord? record, out string? error)
    {
        record = null;
        error = null;

        byte[] header;
        try
        {
            using var stream = File.OpenRead(path);
            header = new byte[ProbeBytes];
            var read = 0;
            while (read < header.Length)
            {
                var n = stream.Read(header, read, header.Length - read);
                if (n <= 0)
                    break;
                read += n;
            }
            if (read < header.Length)
                Array.Resize(ref header, read);
        }
        catch (Exception ex)
        {
            error = $"读取失败: {ex.Message}";
            return false;
        }

        if (header.Length < MagicBytes.Length || !header.AsSpan(0, MagicBytes.Length).SequenceEqual(MagicBytes))
        {
            error = $"魔数不是 \"{MagicText}\"（前 4 字节: {Describe(header)}）";
            return false;
        }

        var match = ResultPattern.Match(Encoding.ASCII.GetString(header));
        if (!match.Success)
        {
            error = "头部未找到圆定位结果字符串（模板可能已变更）";
            return false;
        }

        var x = double.Parse(match.Groups[1].Value, CultureInfo.InvariantCulture);
        var y = double.Parse(match.Groups[2].Value, CultureInfo.InvariantCulture);
        var d = double.Parse(match.Groups[3].Value, CultureInfo.InvariantCulture);

        // 嵌入 JPEG 的位置是**可选**信息：Pos0 头部约 3461 字节，而 Pos1/Pos2 的头部可达
        // 数百 KB（实测 103794 / 720218 字节），超出探测长度。找不到就置 0，
        // 不影响测量值——要取图像请走 TryReadEmbeddedImage（它会读整个文件）。
        var jpegStart = FindJpegStart(header);

        var fileName = Path.GetFileName(path);
        var parts = ParseFileName(fileName);

        record = new VendorVdbRecord
        {
            FilePath = Path.GetFullPath(path),
            FileName = fileName,
            Serial = parts.Serial,
            Position = parts.Position,
            Timestamp = parts.Timestamp,
            IsVendorNg = parts.IsVendorNg,
            LocateCenterX = x,
            LocateCenterY = y,
            // 半径是模板相关字段，取不到就留 NaN，不影响圆心可用性
            LocateRadiusPx = FindLocateRadius(header, x, y),
            ResultD = d,
            HeaderLength = jpegStart < 0 ? 0 : jpegStart,
        };
        return true;
    }

    /// <summary>
    /// 解析文件名里的现场数据约定字段，形如
    /// <c>19-22-06 113_Scene1Pos0_index1_SN_CCSAGZZG01A2...34_OK_255.vdb</c>。
    /// 判定标记（<c>_OK_</c>/<c>_NG_</c>）是现场语料里唯一的判定来源（见类型注释）。
    /// </summary>
    public static FileNameParts ParseFileName(string fileName)
    {
        var stem = Path.GetFileNameWithoutExtension(fileName);

        var sceneAt = stem.IndexOf("_Scene", StringComparison.Ordinal);
        var timestamp = sceneAt > 0 ? stem[..sceneAt] : string.Empty;

        var serialMatch = SerialPattern.Match(stem);
        var positionMatch = PositionPattern.Match(stem);

        bool? isNg = stem.Contains("_NG_", StringComparison.Ordinal) ? true
            : stem.Contains("_OK_", StringComparison.Ordinal) ? false
            : null;

        return new FileNameParts(
            serialMatch.Success ? serialMatch.Value : string.Empty,
            positionMatch.Success ? "Pos" + positionMatch.Groups[1].Value : string.Empty,
            timestamp,
            isNg);
    }

    /// <summary>
    /// 取出嵌入的 JPEG 并解码成 <see cref="Mat"/>（调用方负责释放）。
    /// 返回 false 时 <paramref name="image"/> 为 null 并给出原因。
    ///
    /// 注意：这是**厂商 GUI 的渲染输出**，与同名 Orignal_pic jpg 逐像素相同，
    /// 已带叠加标注且是 JPEG 质量 1 的重压缩结果——不要当作原始相机帧使用。
    /// </summary>
    public static bool TryReadEmbeddedImage(string path, out Mat? image, out string? error)
    {
        image = null;
        error = null;

        byte[] bytes;
        try
        {
            bytes = File.ReadAllBytes(path);
        }
        catch (Exception ex)
        {
            error = $"读取失败: {ex.Message}";
            return false;
        }

        if (!TrySliceEmbeddedJpeg(bytes, out var start, out var end))
        {
            error = "未找到完整的嵌入 JPEG（缺少 FFD8FF 或 FFD9）";
            return false;
        }

        try
        {
            image = Cv2.ImDecode(bytes[start..end], ImreadModes.Color);
        }
        catch (Exception ex)
        {
            error = $"解码失败: {ex.Message}";
            return false;
        }

        if (image is null || image.Empty())
        {
            image?.Dispose();
            image = null;
            error = "嵌入 JPEG 解码结果为空";
            return false;
        }
        return true;
    }

    /// <summary>取出嵌入 JPEG 的原始字节（不解码）。</summary>
    public static bool TryReadEmbeddedJpegBytes(string path, out byte[]? jpeg, out string? error)
    {
        jpeg = null;
        error = null;
        try
        {
            var bytes = File.ReadAllBytes(path);
            if (!TrySliceEmbeddedJpeg(bytes, out var start, out var end))
            {
                error = "未找到完整的嵌入 JPEG（缺少 FFD8FF 或 FFD9）";
                return false;
            }
            jpeg = bytes[start..end];
            return true;
        }
        catch (Exception ex)
        {
            error = $"读取失败: {ex.Message}";
            return false;
        }
    }

    /// <summary>
    /// 定位嵌入 JPEG 的 <c>[start, end)</c> 区间。
    /// 结束标记取起始之后的**第一个** <c>FF D9</c>：JPEG 熵编码段里 <c>FF</c> 必须转义成 <c>FF 00</c>，
    /// 所以 EOI 之外的 <c>FF D9</c> 不会出现。
    /// </summary>
    private static bool TrySliceEmbeddedJpeg(byte[] bytes, out int start, out int end)
    {
        start = FindJpegStart(bytes);
        end = 0;
        if (start < 0)
            return false;
        var eoi = FindMarker(bytes, 0xFF, 0xD9, start + 2);
        if (eoi < 0)
            return false;
        end = eoi + 2;
        return true;
    }

    private static int FindJpegStart(byte[] bytes)
    {
        for (var i = JpegSearchStart; i <= bytes.Length - 3; i++)
            if (bytes[i] == 0xFF && bytes[i + 1] == 0xD8 && bytes[i + 2] == 0xFF)
                return i;
        return -1;
    }

    private static int FindMarker(byte[] bytes, byte first, byte second, int from)
    {
        for (var i = Math.Max(from, 0); i < bytes.Length - 1; i++)
            if (bytes[i] == first && bytes[i + 1] == second)
                return i;
        return -1;
    }

    /// <summary>
    /// 找厂商定位半径：头部里存在一段连续三个 double 即 <c>(圆心X, 圆心Y, 半径)</c>，
    /// 用字符串里已知的 X/Y 去匹配这段三元组（而不是写死偏移），厂商换模板后仍能命中，
    /// 命不中则返回 <see cref="double.NaN"/>。
    ///
    /// 容差取 0.06 与 1e-4 相对值的较大者：字符串里的 X/Y 只保留一位小数（如 2571.2），
    /// 而 double 是 2571.1936，所以必须留出舍入余量；同时两位数千的坐标上这个容差
    /// 远小于任何偶然匹配的量级，不会误命中。
    /// </summary>
    private static double FindLocateRadius(byte[] header, double centerX, double centerY)
    {
        for (var offset = 0; offset + 24 <= header.Length; offset++)
        {
            if (!MatchesDouble(header, offset, centerX) || !MatchesDouble(header, offset + 8, centerY))
                continue;

            var radius = BitConverter.ToDouble(header, offset + 16);
            if (double.IsFinite(radius) && radius > 0)
                return radius;
        }
        return double.NaN;
    }

    private static bool MatchesDouble(byte[] header, int offset, double expected)
    {
        var value = BitConverter.ToDouble(header, offset);
        if (!double.IsFinite(value))
            return false;
        var tolerance = Math.Max(0.06, Math.Abs(expected) * 1e-4);
        return Math.Abs(value - expected) <= tolerance;
    }

    private static string Describe(byte[] header)
    {
        if (header.Length == 0)
            return "空文件";
        var n = Math.Min(header.Length, 4);
        var sb = new StringBuilder();
        for (var i = 0; i < n; i++)
            sb.Append(header[i].ToString("X2", CultureInfo.InvariantCulture)).Append(' ');
        return sb.ToString().TrimEnd();
    }
}
