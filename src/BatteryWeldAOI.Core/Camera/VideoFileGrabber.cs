namespace BatteryWeldAOI.Core.Camera;

using OpenCvSharp;

/// <summary>
/// 视频文件取图器：把视频逐帧当作"逐点位实拍图"。
/// 每次 TriggerGrab 顺序读取下一帧并缩放到配置分辨率
/// （算法与九点标定按固定分辨率建立，分辨率不一致将导致测量失真）。
/// 注意：TriggerGrab 必须按点位序号顺序调用（与检测流程一致）。
/// </summary>
public sealed class VideoFileGrabber : ICameraGrabber
{
    private readonly VideoCapture _capture;
    private readonly int _width;
    private readonly int _height;

    public string Name { get; }

    /// <summary>视频总帧数（个别封装可能报 0，此时以实际读取结束为准）。</summary>
    public int FrameCount { get; }

    public VideoFileGrabber(string path, int outputWidth, int outputHeight)
    {
        if (!File.Exists(path))
            throw new FileNotFoundException("视频文件不存在", path);

        _capture = new VideoCapture(path);
        if (!_capture.IsOpened())
        {
            _capture.Dispose();
            throw new IOException($"无法打开视频（编码不受支持或文件损坏）: {path}");
        }

        _width = outputWidth;
        _height = outputHeight;
        FrameCount = (int)_capture.Get(VideoCaptureProperties.FrameCount);
        Name = $"VideoFile({Path.GetFileName(path)}, {FrameCount} 帧)";
    }

    public Mat TriggerGrab(int pointIndex)
    {
        using var frame = new Mat();
        if (!_capture.Read(frame) || frame.Empty())
            throw new IOException($"视频第 {pointIndex} 帧读取失败（文件结束或读取错误）");

        var resized = new Mat();
        Cv2.Resize(frame, resized, new Size(_width, _height), interpolation: InterpolationFlags.Linear);
        return resized;
    }

    public void Dispose() => _capture.Dispose();
}
