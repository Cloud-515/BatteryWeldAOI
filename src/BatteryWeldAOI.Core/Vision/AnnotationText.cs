namespace BatteryWeldAOI.Core.Vision;

using System.Drawing.Drawing2D;
using System.Drawing.Imaging;
using System.Drawing.Text;
using System.Runtime.InteropServices;
using System.Text;
using OpenCvSharp;
using GdiBitmap = System.Drawing.Bitmap;
using GdiBrush = System.Drawing.SolidBrush;
using GdiColor = System.Drawing.Color;
using GdiFont = System.Drawing.Font;
using GdiFontFamily = System.Drawing.FontFamily;
using GdiFontStyle = System.Drawing.FontStyle;
using GdiGraphics = System.Drawing.Graphics;
using GdiGraphicsUnit = System.Drawing.GraphicsUnit;
using GdiPointF = System.Drawing.PointF;
using GdiRectangle = System.Drawing.Rectangle;
using GdiStringFormat = System.Drawing.StringFormat;
using GdiStringFormatFlags = System.Drawing.StringFormatFlags;

// 本文件是全项目唯一使用 GDI+（System.Drawing.Common，Windows 专有 API）的地方。
// 每个 GDI+ 调用点都在 OperatingSystem.IsWindows() 判断之后（见 CjkCapable），
// 非 Windows 一律走 Hershey 降级路径，因此这里显式关闭平台兼容性告警——
// 不是掩盖问题，而是该限制已被运行时判断覆盖。
#pragma warning disable CA1416

/// <summary>
/// 标注文字绘制：把结论文字画到 OpenCV Mat 上，**支持中文**。
///
/// 为什么不能用 <c>Cv2.PutText</c>：OpenCV 的 Hershey 字体只覆盖 ASCII，遇到非 ASCII 字节
/// 一律画成一个 "?"。而本项目的结论文字里带中文（测量失败原因、缺陷中文名），于是
/// **恰好只有异常/不良的那几张标注图**整行变成 <c>ERROR: ????????????</c>——详情窗口里写着原因、
/// 图上却全是问号，操作员根本不知道这个点为什么被判异常。
/// 这就是"异常图显示成问号"故障的根因（2026-09-14 用 tests/pp 现场图复现）。
///
/// 实现：Windows 下用 GDI+ 把文字渲染到<b>文字所在的那一小块</b>位图上（GDI+ 自带抗锯齿与
/// 系统字体回退，中文可见），再写回 Mat，不做整图转换——20MP 图上每次标注的额外开销可忽略。
/// 非 Windows 退回 Hershey 字体，并把文字里画不出的部分显式降级（见
/// <see cref="SanitizeForHershey"/>），绝不把一串 "?" 画到图上冒充内容。
/// </summary>
public static class AnnotationText
{
    /// <summary>Hershey 字号与像素高的换算系数（两者视觉高度经验比）。</summary>
    private const double HersheyScaleToPixel = 30.0;

    private static readonly Lazy<GdiFontFamily?> CjkFamily = new(ResolveCjkFamily);

    /// <summary>当前环境能否用 GDI+ 画中文（Windows 且系统装有中文字体）。</summary>
    public static bool CjkCapable => OperatingSystem.IsWindows() && ResolveCjkFamily() is not null;

    /// <summary>两行文字之间的基线间距（像素）。</summary>
    public static double LineHeight(double fontPx) => fontPx * 1.35;

    /// <summary>测量文字占用的像素尺寸（按 <paramref name="fontPx"/> 字号）。</summary>
    public static OpenCvSharp.Size Measure(string text, double fontPx)
    {
        if (string.IsNullOrEmpty(text))
            return new OpenCvSharp.Size(0, 0);

        if (CjkCapable)
        {
            try
            {
                using var font = CreateFont(fontPx, 1);
                using var fmt = CreateFormat();
                using var probe = new GdiBitmap(1, 1);
                using var g = GdiGraphics.FromImage(probe);
                var size = g.MeasureString(text, font, GdiPointF.Empty, fmt);
                return new OpenCvSharp.Size(
                    (int)Math.Ceiling(size.Width), (int)Math.Ceiling(size.Height));
            }
            catch (Exception)
            {
                // 落到下面的 Hershey 估算
            }
        }

        var plain = SanitizeForHershey(text);
        var textSize = Cv2.GetTextSize(plain, HersheyFonts.HersheySimplex, HersheyScale(fontPx), 1,
            out var baseLine);
        return new OpenCvSharp.Size(textSize.Width, textSize.Height + baseLine);
    }

    /// <summary>按可用宽度截断文字（超出部分以 "…" 结尾）。</summary>
    public static string Ellipsize(string text, int availablePx, double fontPx)
    {
        if (string.IsNullOrEmpty(text) || availablePx <= 0)
            return string.Empty;
        if (Measure(text, fontPx).Width <= availablePx)
            return text;

        for (var len = text.Length - 1; len > 0; len--)
        {
            var candidate = text[..len] + "…";
            if (Measure(candidate, fontPx).Width <= availablePx)
                return candidate;
        }
        return "…";
    }

    /// <summary>
    /// 在图上绘制一行文字。<paramref name="baselineLeft"/> 与 <c>Cv2.PutText</c> 一致，
    /// 为<b>基线左端</b>；<paramref name="bgrColor"/> 为 BGR 顺序（同 OpenCvSharp 惯例）。
    /// </summary>
    /// <param name="thickness">线宽：≥2 时以粗体渲染（GDI+ 没有独立线宽概念）。</param>
    public static void Draw(Mat image, string text, OpenCvSharp.Point baselineLeft, double fontPx,
        Scalar bgrColor, int thickness = 1)
    {
        if (image is null || image.Empty() || string.IsNullOrWhiteSpace(text))
            return;

        if (TryDrawWithGdi(image, text, baselineLeft, fontPx, bgrColor, thickness))
            return;

        // 降级：Hershey 只能画 ASCII，先把画不出的部分净化掉
        var plain = SanitizeForHershey(text);
        if (plain.Length == 0)
            return;
        Cv2.PutText(image, plain, baselineLeft, HersheyFonts.HersheySimplex, HersheyScale(fontPx),
            bgrColor, thickness, LineTypes.AntiAlias);
    }

    /// <summary>
    /// Hershey 降级路径的文本净化：Hershey 只认 ASCII，与其让中文变成一串无从辨认的 "?"，
    /// 不如把每段非 ASCII 折叠成一个占位符，让操作员一眼看出"这里有内容没画出来"。
    /// </summary>
    public static string SanitizeForHershey(string text)
    {
        var sb = new StringBuilder(text.Length);
        var insideNonAscii = false;
        foreach (var ch in text)
        {
            if (ch < 0x80)
            {
                sb.Append(ch);
                insideNonAscii = false;
            }
            else if (!insideNonAscii)
            {
                sb.Append('*');
                insideNonAscii = true;
            }
        }
        return sb.ToString().Trim();
    }

    // ---- GDI+ 路径 ----

    private static bool TryDrawWithGdi(Mat image, string text, OpenCvSharp.Point baselineLeft,
        double fontPx, Scalar color, int thickness)
    {
        if (!CjkCapable)
            return false;
        try
        {
            DrawWithGdi(image, text, baselineLeft, fontPx, color, thickness);
            return true;
        }
        catch (Exception)
        {
            // GDI+ 不可用（会话/字体异常）时退回 Hershey，标注失败不能连累检测流程
            return false;
        }
    }

    private static void DrawWithGdi(Mat image, string text, OpenCvSharp.Point baselineLeft,
        double fontPx, Scalar color, int thickness)
    {
        var family = CjkFamily.Value!;
        var px = (float)Math.Clamp(fontPx, 6.0, 4000.0);
        var style = thickness >= 2 ? GdiFontStyle.Bold : GdiFontStyle.Regular;
        using var font = new GdiFont(family, px, style, GdiGraphicsUnit.Pixel);
        using var fmt = CreateFormat();
        using var brush = new GdiBrush(ToGdiColor(color));

        float textWidth, textHeight;
        using (var probe = new GdiBitmap(1, 1))
        using (var g = GdiGraphics.FromImage(probe))
        {
            var size = g.MeasureString(text, font, GdiPointF.Empty, fmt);
            textWidth = size.Width;
            textHeight = size.Height;
        }

        // PutText 与 DrawString 的锚点不同：前者是基线左端，后者是版面框左上角，
        // 这里按字体 ascent 把基线换算成版面框顶（含少量留白供描边）。
        var pad = (int)Math.Ceiling(px * 0.35) + 2;
        var ascent = px * family.GetCellAscent(style) / family.GetEmHeight(style);
        var left = baselineLeft.X - pad;
        var top = (int)Math.Round(baselineLeft.Y - ascent) - pad;
        var width = (int)Math.Ceiling(textWidth) + pad * 2;
        var height = (int)Math.Ceiling(textHeight) + pad * 2;
        if (width <= 0 || height <= 0)
            return;

        var x0 = Math.Max(0, left);
        var y0 = Math.Max(0, top);
        var x1 = Math.Min(image.Width, left + width);
        var y1 = Math.Min(image.Height, top + height);
        if (x1 <= x0 || y1 <= y0)
            return;

        using var patch = new Mat(image, new Rect(x0, y0, x1 - x0, y1 - y0));
        Mat? temp = null;
        try
        {
            var target = patch.Channels() switch
            {
                1 => temp = Convert(patch, ColorConversionCodes.GRAY2BGR),
                4 => temp = Convert(patch, ColorConversionCodes.BGRA2BGR),
                _ => patch
            };

            using (var bitmap = ToBitmap(target))
            {
                using (var g = GdiGraphics.FromImage(bitmap))
                {
                    g.TextRenderingHint = TextRenderingHint.AntiAliasGridFit;
                    g.SmoothingMode = SmoothingMode.AntiAlias;
                    DrawHaloed(g, text, font, fmt, brush, pad - (x0 - left), pad - (y0 - top), px);
                }
                FromBitmap(bitmap, target);
            }

            if (!ReferenceEquals(target, patch))
            {
                var back = patch.Channels() == 1
                    ? ColorConversionCodes.BGR2GRAY
                    : ColorConversionCodes.BGR2BGRA;
                Cv2.CvtColor(target, patch, back);
            }
        }
        finally
        {
            temp?.Dispose();
        }
    }

    /// <summary>先描一圈半透明深色描边再画彩色文字：亮金属表面上的绿/红细字原本几乎看不清。</summary>
    private static void DrawHaloed(GdiGraphics g, string text, GdiFont font, GdiStringFormat fmt,
        GdiBrush brush, float x, float y, float px)
    {
        var offset = Math.Max(1f, px / 22f);
        using var halo = new GdiBrush(GdiColor.FromArgb(200, 0, 0, 0));
        for (var dy = -1; dy <= 1; dy++)
        {
            for (var dx = -1; dx <= 1; dx++)
            {
                if (dx == 0 && dy == 0)
                    continue;
                g.DrawString(text, font, halo, x + dx * offset, y + dy * offset, fmt);
            }
        }
        g.DrawString(text, font, brush, x, y, fmt);
    }

    private static Mat Convert(Mat src, ColorConversionCodes code)
    {
        var dst = new Mat();
        Cv2.CvtColor(src, dst, code);
        return dst;
    }

    private static GdiFont CreateFont(double fontPx, int thickness) =>
        new(CjkFamily.Value ?? GdiFontFamily.GenericSansSerif,
            (float)Math.Clamp(fontPx, 6.0, 4000.0),
            thickness >= 2 ? GdiFontStyle.Bold : GdiFontStyle.Regular,
            GdiGraphicsUnit.Pixel);

    private static GdiStringFormat CreateFormat()
    {
        var fmt = (GdiStringFormat)GdiStringFormat.GenericTypographic.Clone();
        fmt.FormatFlags |= GdiStringFormatFlags.NoWrap | GdiStringFormatFlags.MeasureTrailingSpaces;
        return fmt;
    }

    private static GdiColor ToGdiColor(Scalar bgr) =>
        GdiColor.FromArgb(Clamp(bgr[2]), Clamp(bgr[1]), Clamp(bgr[0]));

    private static int Clamp(double v) => (int)Math.Clamp(Math.Round(v), 0, 255);

    private static GdiBitmap ToBitmap(Mat bgr)
    {
        var width = bgr.Width;
        var height = bgr.Height;
        var bitmap = new GdiBitmap(width, height, PixelFormat.Format24bppRgb);
        var data = bitmap.LockBits(new GdiRectangle(0, 0, width, height),
            ImageLockMode.WriteOnly, PixelFormat.Format24bppRgb);
        try
        {
            var row = new byte[width * 3];
            for (var y = 0; y < height; y++)
            {
                Marshal.Copy(IntPtr.Add(bgr.Data, (int)(y * bgr.Step())), row, 0, row.Length);
                Marshal.Copy(row, 0, IntPtr.Add(data.Scan0, y * data.Stride), row.Length);
            }
        }
        finally
        {
            bitmap.UnlockBits(data);
        }
        return bitmap;
    }

    private static void FromBitmap(GdiBitmap bitmap, Mat bgr)
    {
        var width = bgr.Width;
        var height = bgr.Height;
        var data = bitmap.LockBits(new GdiRectangle(0, 0, width, height),
            ImageLockMode.ReadOnly, PixelFormat.Format24bppRgb);
        try
        {
            var row = new byte[width * 3];
            for (var y = 0; y < height; y++)
            {
                Marshal.Copy(IntPtr.Add(data.Scan0, y * data.Stride), row, 0, row.Length);
                Marshal.Copy(row, 0, IntPtr.Add(bgr.Data, (int)(y * bgr.Step())), row.Length);
            }
        }
        finally
        {
            bitmap.UnlockBits(data);
        }
    }

    private static GdiFontFamily? ResolveCjkFamily()
    {
        if (!OperatingSystem.IsWindows())
            return null;

        string[] candidates =
        {
            "Microsoft YaHei UI", "Microsoft YaHei", "SimHei", "SimSun", "NSimSun",
            "Noto Sans CJK SC", "Source Han Sans SC", "Arial Unicode MS"
        };
        try
        {
            using var installed = new InstalledFontCollection();
            var available = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            foreach (var family in installed.Families)
                available.Add(family.Name);
            foreach (var name in candidates)
            {
                if (available.Contains(name))
                    return new GdiFontFamily(name);
            }
        }
        catch (Exception)
        {
            // 字体枚举失败 → 视为无中文字体，走降级路径
        }
        return null;
    }

    private static double HersheyScale(double fontPx) => Math.Max(0.25, fontPx / HersheyScaleToPixel);
}

#pragma warning restore CA1416
