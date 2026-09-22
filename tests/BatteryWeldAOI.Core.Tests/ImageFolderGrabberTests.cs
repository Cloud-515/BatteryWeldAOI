using BatteryWeldAOI.Core.Camera;
using OpenCvSharp;
using Xunit;

namespace BatteryWeldAOI.Core.Tests;

/// <summary>
/// 图片文件夹取图器的测试：
/// 用合成 PNG 验证数量统计、自然顺序排序、**原始尺寸原样传递**与像素内容完整性，
/// 以及文件夹不存在 / 无图片 / 非图片文件过滤等边界条件。
/// </summary>
public class ImageFolderGrabberTests : IDisposable
{
    private readonly string _dir = Path.Combine(Path.GetTempPath(), $"bwaoi_images_{Guid.NewGuid():N}");

    /// <summary>写入一张纯色底 + 圆点的测试图片，返回文件名。</summary>
    private string WriteTestImage(string fileName, int width, int height, int dotX, Scalar background)
    {
        Directory.CreateDirectory(_dir);
        using var mat = new Mat(height, width, MatType.CV_8UC3, background);
        Cv2.Circle(mat, new Point(dotX, height / 2), 15, Scalar.White, -1);
        var path = Path.Combine(_dir, fileName);
        Assert.True(Cv2.ImWrite(path, mat), $"测试图片写入失败: {fileName}");
        return path;
    }

    /// <summary>
    /// 图幅必须**原样**返回。这里刻意让图片尺寸(640×480)与"配置分辨率"(320×240)不同，
    /// 断言取图器不做任何缩放——算法是尺度相对量，重采样只会破坏纹理统计
    /// （见 ImageFolderGrabber 类注释里的实测数据：拉伸后 28 张良品有 15 张被误判）。
    /// </summary>
    [Fact]
    public void ImageFolder_ReadBack_Sorted_NativeSizeAndIntact()
    {
        // 故意乱序写入：字典序会得到 point_1, point_10, point_2...，自然顺序应为 1,2,10
        const int width = 640, height = 480;
        WriteTestImage("point_10.png", width, height, 230, new Scalar(60, 120, 200));
        WriteTestImage("point_2.png", width, height, 70, new Scalar(30, 90, 160));
        WriteTestImage("point_1.png", width, height, 50, new Scalar(20, 80, 150));

        using var grabber = new ImageFolderGrabber(_dir, outputWidth: 320, outputHeight: 240);
        Assert.Equal(3, grabber.ImageCount);
        Assert.Equal(new[] { "point_1.png", "point_2.png", "point_10.png" },
            grabber.Files.Select(Path.GetFileName));

        var expectedDotX = new[] { 50, 70, 230 };
        var expectedBlue = new byte[] { 20, 30, 60 };
        for (var i = 0; i < grabber.ImageCount; i++)
        {
            using var mat = grabber.TriggerGrab(i);
            Assert.Equal(width, mat.Width);
            Assert.Equal(height, mat.Height);
            Assert.Equal(3, mat.Channels());
            // 背景色应随图片不同（验证内容对应到正确文件，而非顺序错乱）
            var bg = mat.At<Vec3b>(10, 10);
            Assert.Equal(expectedBlue[i], bg.Item0);
            // 白色圆点应随图片平移（原图坐标，不经过任何缩放）
            var dot = mat.At<Vec3b>(height / 2, expectedDotX[i]);
            Assert.True(dot.Item0 > 200 && dot.Item1 > 200 && dot.Item2 > 200,
                $"第 {i} 张图片圆点内容缺失 (B={dot.Item0},G={dot.Item1},R={dot.Item2})");
        }
    }

    [Fact]
    public void ImageFolder_NonImageFiles_FilteredOut()
    {
        WriteTestImage("a.png", 100, 100, 50, Scalar.Gray);
        File.WriteAllText(Path.Combine(_dir, "notes.txt"), "不是图片");
        File.WriteAllText(Path.Combine(_dir, "report.csv"), "1,2,3");
        Directory.CreateDirectory(Path.Combine(_dir, "sub.png")); // 同名目录也应被排除

        using var grabber = new ImageFolderGrabber(_dir, 320, 240);
        Assert.Equal(1, grabber.ImageCount);
        Assert.Equal("a.png", Path.GetFileName(grabber.Files[0]));
    }

    [Fact]
    public void ImageFolder_MissingDirectory_Throws()
    {
        Assert.Throws<DirectoryNotFoundException>(
            () => new ImageFolderGrabber(@"Z:\definitely\not\exist", 1280, 1024));
    }

    [Fact]
    public void ImageFolder_EmptyDirectory_Throws()
    {
        Directory.CreateDirectory(_dir);
        Assert.Throws<IOException>(
            () => new ImageFolderGrabber(_dir, 1280, 1024));
    }

    [Fact]
    public void ImageFolder_IndexOutOfRange_Throws()
    {
        WriteTestImage("only.png", 100, 100, 50, Scalar.Gray);
        using var grabber = new ImageFolderGrabber(_dir, 320, 240);
        Assert.Throws<IOException>(() => grabber.TriggerGrab(1));
        Assert.Throws<IOException>(() => grabber.TriggerGrab(-1));
    }

    [Fact]
    public void NaturalFileNameComparer_DigitSegments_CompareNumerically()
    {
        var cmp = NaturalFileNameComparer.Instance;
        Assert.True(cmp.Compare("point_2.png", "point_10.png") < 0);
        Assert.True(cmp.Compare("point_10.png", "point_2.png") > 0);
        Assert.Equal(0, cmp.Compare("point_2.png", "point_2.png"));
        Assert.True(cmp.Compare("a2b10", "a2b9") > 0);       // 前缀数字段相同, 后段按数值
        Assert.True(cmp.Compare("a02", "a2") == 0);           // 前导零不改变顺序
        Assert.True(cmp.Compare("IMG_9", "IMG_10.png") < 0);  // 数字段之后允许长度不同
    }

    public void Dispose()
    {
        try { Directory.Delete(_dir, recursive: true); } catch (IOException) { }
    }
}
