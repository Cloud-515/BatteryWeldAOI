namespace BatteryWeldAOI.Wpf.Services;

using System.Runtime.InteropServices;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using OpenCvSharp;

/// <summary>
/// OpenCV Mat 到 WPF BitmapSource 的转换。
/// 像素数据被完整复制并 Freeze，可在任意线程创建、任意线程渲染。
/// 不拥有也不释放传入的 Mat（调用方持有生命周期）。
/// </summary>
public static class MatConverter
{
    /// <summary>转换 8 位灰度/BGR/BGRA 图像；空图返回 null。</summary>
    public static BitmapSource? ToBitmapSource(Mat? mat)
    {
        if (mat is null || mat.Empty())
            return null;

        // 仅在不连续时克隆（克隆件归本方法释放，原 Mat 不得动）
        Mat? clone = null;
        try
        {
            var src = mat.IsContinuous() ? mat : (clone = mat.Clone());
            var format = src.Channels() switch
            {
                1 => PixelFormats.Gray8,
                4 => PixelFormats.Bgra32,
                _ => PixelFormats.Bgr24
            };

            var stride = (int)src.Step();
            var data = new byte[stride * src.Height];
            Marshal.Copy(src.Data, data, 0, data.Length);

            var bitmap = BitmapSource.Create(src.Width, src.Height, 96, 96, format, null, data, stride);
            bitmap.Freeze();
            return bitmap;
        }
        finally
        {
            clone?.Dispose();
        }
    }
}
