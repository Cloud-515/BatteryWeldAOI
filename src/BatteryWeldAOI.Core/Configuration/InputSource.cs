namespace BatteryWeldAOI.Core.Configuration;

/// <summary>图像输入源类型。</summary>
public enum InputSource
{
    /// <summary>仿真演示：合成图像（内置剧本，可复现）。</summary>
    Simulation,

    /// <summary>视频文件回放：逐帧作为待检图。</summary>
    VideoFile,

    /// <summary>图片文件夹批量识别：每张图片作为一个焊点（按文件名自然顺序）。</summary>
    ImageFolder,

    /// <summary>真实摄像头（DirectShow）：实时取帧检测。</summary>
    Camera
}
