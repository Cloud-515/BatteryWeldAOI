namespace BatteryWeldAOI.Core.Camera;

using OpenCvSharp;

/// <summary>
/// 真实摄像头取图器（Windows DirectShow 后端）。
/// 打开后自动丢弃前几帧等待自动曝光稳定，按配置分辨率缩放输出。
/// 真机产线请用海康/Basler SDK 实现替换；本类覆盖普通 UVC 相机的
/// 视频评审 / 台架调试场景。
/// </summary>
public sealed class CameraGrabber : ICameraGrabber
{
    private const int WarmupFrames = 5;

    private readonly VideoCapture _capture;
    private readonly int _width;
    private readonly int _height;

    public string Name { get; }

    public CameraGrabber(int deviceIndex, int outputWidth, int outputHeight)
    {
        _capture = new VideoCapture(deviceIndex, VideoCaptureAPIs.DSHOW);
        if (!_capture.IsOpened())
        {
            _capture.Dispose();
            throw new IOException($"无法打开摄像头 {deviceIndex}（设备不存在或被占用）");
        }

        // 请求采集分辨率（相机支持则生效，否则保持默认）
        _capture.Set(VideoCaptureProperties.FrameWidth, outputWidth);
        _capture.Set(VideoCaptureProperties.FrameHeight, outputHeight);

        _width = outputWidth;
        _height = outputHeight;

        // 自动曝光/白平衡稳定前的帧偏黑偏色，丢弃
        using var warmup = new Mat();
        for (var i = 0; i < WarmupFrames; i++)
            _capture.Read(warmup);

        var actualW = (int)_capture.Get(VideoCaptureProperties.FrameWidth);
        var actualH = (int)_capture.Get(VideoCaptureProperties.FrameHeight);
        Name = $"Camera#{deviceIndex}({actualW}x{actualH}采集)";
    }

    public Mat TriggerGrab(int pointIndex)
    {
        using var frame = new Mat();
        if (!_capture.Read(frame) || frame.Empty())
            throw new IOException($"摄像头 {pointIndex} 号取帧失败（设备断开或超时）");

        var resized = new Mat();
        Cv2.Resize(frame, resized, new Size(_width, _height), interpolation: InterpolationFlags.Linear);
        return resized;
    }

    public void Dispose() => _capture.Dispose();
}
