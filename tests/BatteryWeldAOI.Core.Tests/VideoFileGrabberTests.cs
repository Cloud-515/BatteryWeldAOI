using BatteryWeldAOI.Core.Camera;
using BatteryWeldAOI.Core.Communication;
using BatteryWeldAOI.Core.Models;
using OpenCvSharp;
using Xunit;

namespace BatteryWeldAOI.Core.Tests;

/// <summary>
/// 视频文件取图器与无 PLC 模式的测试：
/// 用 VideoWriter 合成一段已知内容的视频，再经 VideoFileGrabber 回读，
/// 验证帧数、缩放与像素内容完整传递。
/// </summary>
public class VideoFileGrabberTests : IDisposable
{
    private readonly string _path = Path.Combine(Path.GetTempPath(),
        $"bwaoi_video_{Guid.NewGuid():N}.avi");

    /// <summary>合成 6 帧视频：每帧为纯色底 + 帧号圆点。</summary>
    private void WriteTestVideo(int frames, int width, int height)
    {
        using var writer = new VideoWriter(_path, FourCC.FromString("mp4v"), 30,
            new Size(width, height), isColor: true);
        Assert.True(writer.IsOpened(), "测试视频编码器不可用（mp4v）");
        for (var i = 0; i < frames; i++)
        {
            using var frame = new Mat(height, width, MatType.CV_8UC3,
                new Scalar(40 * i % 255, 120, 200));
            Cv2.Circle(frame, new Point(50 + 20 * i, height / 2), 15, Scalar.White, -1);
            writer.Write(frame);
        }
    }

    [Fact]
    public void VideoFile_ReadBack_FramesResizedAndIntact()
    {
        const int frames = 6;
        WriteTestVideo(frames, 640, 480);

        using var grabber = new VideoFileGrabber(_path, outputWidth: 320, outputHeight: 240);
        Assert.Equal(frames, grabber.FrameCount);

        for (var i = 0; i < frames; i++)
        {
            using var mat = grabber.TriggerGrab(i);
            Assert.Equal(320, mat.Width);
            Assert.Equal(240, mat.Height);
            Assert.Equal(3, mat.Channels());
            // 白色圆点应随帧号平移：检查圆心像素亮度
            var circleX = (50 + 20 * i) * 320 / 640;
            var circleY = 240 / 2;
            var bgr = mat.At<Vec3b>(circleY, circleX);
            Assert.True(bgr.Item0 > 200 && bgr.Item1 > 200 && bgr.Item2 > 200,
                $"第 {i} 帧圆点内容缺失 (B={bgr.Item0},G={bgr.Item1},R={bgr.Item2})");
        }
    }

    [Fact]
    public void VideoFile_MissingFile_Throws()
    {
        Assert.Throws<FileNotFoundException>(
            () => new VideoFileGrabber(@"Z:\definitely\not\exist.mp4", 1280, 1024));
    }

    [Fact]
    public void NullPlc_AllOperations_CompleteImmediately()
    {
        using var plc = new NullPlcClient();
        Assert.True(plc.IsConnected);

        // 无 PLC 模式下"到位"恒为真——MoveTo 后 WaitForArrival 不应等待
        Assert.True(plc.ConnectAsync(CancellationToken.None).IsCompletedSuccessfully);
        Assert.True(plc.MoveToAsync(0, CancellationToken.None).IsCompletedSuccessfully);
        Assert.True(plc.WaitForArrivalAsync(0, CancellationToken.None).IsCompletedSuccessfully);

        var result = new PointInspectionResult
        {
            Point = new WeldPoint { Index = 0, Name = "Frame000" }
        };
        Assert.True(plc.SendResultAsync(result, CancellationToken.None).IsCompletedSuccessfully);
        Assert.True(plc.FinishAsync(CancellationToken.None).IsCompletedSuccessfully);
    }

    public void Dispose()
    {
        try { File.Delete(_path); } catch (IOException) { }
    }
}
