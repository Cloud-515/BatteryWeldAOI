namespace BatteryWeldAOI.Core.Camera;

using OpenCvSharp;

/// <summary>
/// 图片文件夹取图器：把文件夹内的图片按文件名自然顺序当作"逐点位实拍图"。
/// 每次 TriggerGrab 顺序读取下一张，**按原始像素尺寸原样交给算法，不做任何缩放**。
/// 自然顺序排序保证 point_2.jpg 排在 point_10.jpg 之前（字典序会颠倒两者）。
/// 注意：TriggerGrab 必须按点位序号顺序调用（与检测流程一致）。
///
/// **为什么不缩放到配置分辨率**（这里曾有一个 Cv2.Resize，是真实故障源）：
/// 检测算法的一切参数都是尺度相对量（分母 base = min(高,宽) 或焊环半径），
/// 因此**天生与分辨率无关**，不需要固定尺寸；而重采样会破坏"物理特征尺度 / 像素栅格"
/// 这一比例关系，使纹理能量、飞溅颗粒计数等统计量整体漂移。
/// 实测（tests/tp 的 28 张现场良品图，配置 1280×1024）：
///   · 原样送入 → 28/28 判 OK；
///   · 先拉伸到 1280×1024 再送入 → 仅 13/28 判 OK，其余 15 张被误判为飞溅 NG
///     （251×236 的图被拉成 1280×1024，横向 5.10×、纵向 4.34×，既是 17.5% 的非等比畸变，
///      也让鱼鳞斑/噪点尺度与 TextureWindowRatio、MinSpatterBlobRatio 失配）。
/// 应用界面里"图片文件夹模式 28 个良品几乎全 NG"的现象即由此而来。
///
/// 而且受损的不只是判定，还有**测量值本身**：同一张 1.png，拉伸路径测出 D=0.492mm，
/// 原样路径测出 D=0.083mm——相差 6 倍，纯粹由重采样造成。所以"先缩放到配置分辨率"
/// 这件事既不能保证判对，也不能保证测对。
///
/// 毫米换算由 <see cref="Vision.WeldInspectionAlgorithm"/> 按"配置分辨率 ÷ 实际图幅"
/// 分轴折算，因此这里不做缩放**不会**让偏移量的物理含义变差；相反，它把折算放在原始
/// 像素上进行，避免了先把图拉变形、再按变形后的像素测量中心这一双重误差。
/// （折算模型本身的适用性见 <see cref="Models.InspectionConfig.ImageWidth"/> 的说明。）
/// </summary>
public sealed class ImageFolderGrabber : ICameraGrabber
{
    private static readonly string[] ImageExtensions = { ".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff" };

    private readonly IReadOnlyList<string> _files;

    public string Name { get; }

    /// <summary>文件夹内待检图片总数。</summary>
    public int ImageCount => _files.Count;

    /// <summary>已按自然顺序排序的图片路径（供界面预览与日志使用）。</summary>
    public IReadOnlyList<string> Files => _files;

    /// <summary>
    /// 调用方（界面）配置的分辨率。仅用于日志/核对——取图器不会把图片缩放到该尺寸，
    /// 若与实际图幅不一致，应在配置里把 ImageWidth/ImageHeight/MmPerPixel 改成相机真实值。
    /// </summary>
    public Size ConfiguredSize { get; }

    public ImageFolderGrabber(string folderPath, int outputWidth, int outputHeight)
    {
        if (!Directory.Exists(folderPath))
            throw new DirectoryNotFoundException($"图片文件夹不存在: {folderPath}");

        _files = Directory.EnumerateFiles(folderPath)
            .Where(f => ImageExtensions.Contains(Path.GetExtension(f).ToLowerInvariant()))
            .OrderBy(f => Path.GetFileName(f), NaturalFileNameComparer.Instance)
            .ToList();
        if (_files.Count == 0)
            throw new IOException($"文件夹内没有可识别的图片({string.Join("/", ImageExtensions)}): {folderPath}");

        ConfiguredSize = new Size(outputWidth, outputHeight);
        Name = $"ImageFolder({Path.GetFileName(Path.TrimEndingDirectorySeparator(folderPath))}, {_files.Count} 张)";
    }

    /// <summary>读取第 pointIndex 张图片（原始尺寸）。返回的 Mat 由调用方负责释放。</summary>
    public Mat TriggerGrab(int pointIndex)
    {
        if (pointIndex < 0 || pointIndex >= _files.Count)
            throw new IOException($"图片索引 {pointIndex} 超出范围（共 {_files.Count} 张）");

        var image = Cv2.ImRead(_files[pointIndex], ImreadModes.Color);
        if (image.Empty())
        {
            image.Dispose();
            throw new IOException($"图片读取失败（格式不受支持或文件损坏）: {_files[pointIndex]}");
        }

        return image;
    }

    public void Dispose()
    {
    }
}

/// <summary>
/// 文件名自然顺序比较器：把文件名拆成"非数字段 + 数字段"逐段比较，
/// 数字段按数值比较（"9" &lt; "10"），非数字段按字典序。
/// </summary>
public sealed class NaturalFileNameComparer : IComparer<string>
{
    public static readonly NaturalFileNameComparer Instance = new();

    public int Compare(string? x, string? y)
    {
        if (ReferenceEquals(x, y)) return 0;
        if (x is null) return -1;
        if (y is null) return 1;

        int ix = 0, iy = 0;
        while (ix < x.Length && iy < y.Length)
        {
            var cx = x[ix];
            var cy = y[iy];
            if (char.IsDigit(cx) && char.IsDigit(cy))
            {
                // 提取完整数字段比较：去前导零后先比长度（即数值位数）再逐字符比较，
                // 避免"数值解析"对超长数字串（如时间戳基文件名）抛溢出异常
                var sx = ix;
                while (ix < x.Length && char.IsDigit(x[ix])) ix++;
                var sy = iy;
                while (iy < y.Length && char.IsDigit(y[iy])) iy++;
                var dx = x[sx..ix].TrimStart('0');
                var dy = y[sy..iy].TrimStart('0');
                if (dx.Length != dy.Length) return dx.Length.CompareTo(dy.Length);
                var order = string.CompareOrdinal(dx, dy);
                if (order != 0) return order;
            }
            else
            {
                var order = string.Compare(x[ix..(ix + 1)], y[iy..(iy + 1)],
                    StringComparison.OrdinalIgnoreCase);
                if (order != 0) return order;
                ix++;
                iy++;
            }
        }
        return (x.Length - ix).CompareTo(y.Length - iy);
    }
}
