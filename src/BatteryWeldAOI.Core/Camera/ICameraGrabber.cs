namespace BatteryWeldAOI.Core.Camera;

using OpenCvSharp;

/// <summary>
/// 相机取图抽象（软触发模式）。真实项目替换为海康/Basler SDK 封装：
/// 到位信号到达后调用 TriggerGrab，SDK 回调线程内完成取图并返回。
/// </summary>
public interface ICameraGrabber : IDisposable
{
    string Name { get; }

    /// <summary>软触发拍照并阻塞至帧返回（异常时抛出异常，不返回 null）。</summary>
    Mat TriggerGrab(int pointIndex);
}
