using System.Text;
using BatteryWeldAOI.Core.Models;
using BatteryWeldAOI.Core.Reporting;
using BatteryWeldAOI.Core.Vision;
using OpenCvSharp;
using Xunit;

namespace BatteryWeldAOI.Core.Tests;

/// <summary>
/// 标注图文字与异常显示的回归测试。
///
/// 背景（现场故障）：标注图上的结论文字原先由 <c>Cv2.PutText</c> 绘制，而 OpenCV 的 Hershey
/// 字体只覆盖 ASCII，遇到中文**每个字节**都画成一个 "?"。结论文字里只有异常分支带中文
/// （测量失败原因），于是恰好只有异常那几张图变成 <c>ERROR: ????????????</c>——
/// 详情窗口里写着原因、图上却全是问号，操作员看图和看图上的字都得不到信息。
///
/// 本测试把这条路锁死：中文必须渲染成汉字，而不是等长的问号串。
/// 另附"偏移量不适用"与报表源文件列的断言（异常点的可读性与可追溯性）。
/// </summary>
public class AnnotationTextTests
{
    private const string ChineseReason = "焊环边界支撑弧段不足";

    [Fact]
    public void OnWindows_ChineseRendering_IsAvailable()
    {
        if (!OperatingSystem.IsWindows())
            return;                        // 非 Windows 走 Hershey 降级路径，见下一个测试

        Assert.True(AnnotationText.CjkCapable,
            "Windows 上应能找到中文字体（Microsoft YaHei / SimHei 等），否则标注图上的中文会退化");
    }

    [Fact]
    public void ChineseText_IsNotRenderedAsQuestionMarks()
    {
        // 旧实现的故障特征：每个非 ASCII **字节**都被画成一个 "?"，
        // 于是"中文串"与"等字节数的问号串"逐像素完全相同。修复后两者必须明显不同。
        var questionMarks = new string('?', Encoding.UTF8.GetByteCount(ChineseReason));

        using var chinese = RenderLabel(ChineseReason);
        using var questions = RenderLabel(questionMarks);

        var diff = DiffRatio(chinese, questions);
        Assert.True(diff > 0.01,
            $"中文与等长问号串的渲染结果几乎一致（差异 {diff:P2}）——中文又被画成问号了");
    }

    [Fact]
    public void Draw_ChineseText_LeavesInkOnImage()
    {
        using var image = new Mat(200, 1600, MatType.CV_8UC3, new Scalar(30, 30, 30));
        using var before = image.Clone();

        AnnotationText.Draw(image, ChineseReason, new Point(20, 120), 64, new Scalar(0, 0, 255));

        Assert.True(DiffRatio(before, image) > 0.001, "中文一个字都没画出来");
    }

    [Fact]
    public void Annotate_ErrorResult_DrawsReasonInsteadOfQuestionMarks()
    {
        // 端到端复现现场故障：用真实 Annotate 画一张"测量失败"的标注图
        var algorithm = new WeldInspectionAlgorithm(new InspectionConfig(), FlatCalibration());
        var result = new PointInspectionResult
        {
            Point = new WeldPoint { Index = 1, Name = "Img001", SourceName = "19-22-07 729_x.jpg" },
            HasError = true,
            ErrorMessage = "焊环边界支撑弧段不足（仅覆盖 32% 圆周，下限 50%）",
            ProcessedSizePx = new Size(1280, 1024)
        };

        using var image = new Mat(600, 800, MatType.CV_8UC3, new Scalar(120, 120, 120));
        using var annotated = algorithm.Annotate(image, result);

        // 标注图必须真的写上了字，且与"整段画成问号"的旧行为不同
        Assert.True(DiffRatio(image, annotated) > 0.001, "异常标注图上一个字都没有");

        using var legacyStyle = image.Clone();
        Cv2.PutText(legacyStyle,
            $"ERROR: {result.ErrorMessage}",
            new Point(16, 26), HersheyFonts.HersheySimplex, 0.8, new Scalar(0, 165, 255), 1,
            LineTypes.AntiAlias);
        Assert.True(DiffRatio(annotated, legacyStyle) > 0.005,
            "异常标注图与旧的 PutText 画法一致——中文仍会被画成问号");
    }

    [Fact]
    public void Measure_And_Ellipsize_RespectAvailableWidth()
    {
        var size = AnnotationText.Measure(ChineseReason, 40);
        Assert.True(size.Width > 40, $"中文测量宽度异常: {size.Width}");

        var ellipsized = AnnotationText.Ellipsize(ChineseReason + ChineseReason, size.Width, 40);
        Assert.EndsWith("…", ellipsized);
        Assert.True(AnnotationText.Measure(ellipsized, 40).Width <= size.Width,
            "截断后的文字仍超出可用宽度");
    }

    [Fact]
    public void HersheyFallbackText_NeverContainsQuestionMark()
    {
        // 降级路径（无中文字体）宁可少画，也不能把中文画成一串"?"冒充内容
        var sanitized = AnnotationText.SanitizeForHershey($"异常：{ChineseReason}（仅覆盖 32% 圆周）");

        Assert.DoesNotContain('?', sanitized);
        Assert.Contains("32%", sanitized);
    }

    [Fact]
    public void ShortReason_KeepsTheCause_AndDropsTheDetail()
    {
        Assert.Equal("焊环边界支撑弧段不足",
            PointInspectionResult.ShortReason("焊环边界支撑弧段不足（仅覆盖 32% 圆周，下限 50%）"));
        Assert.Equal("未找到孔洞边缘",
            PointInspectionResult.ShortReason("未找到孔洞边缘，无法判定焊偏"));
        Assert.Equal("测量失败", PointInspectionResult.ShortReason(null));
    }

    [Fact]
    public void MissingWeld_HasNoValidOffset_AndIsReportedAsNotApplicable()
    {
        // 未检出焊环时偏移量恒为 0.000mm：必须显示"不适用"，
        // 否则 0.000mm 会被操作员读成"焊得很正"——最该复核的点位读反了。
        var config = new InspectionConfig { OffsetToleranceMm = 0.5, MaxWorkingBase = 0 };
        var calibration = FlatCalibration();
        var algorithm = new WeldInspectionAlgorithm(config, calibration);
        var generator = new ImageGenerator(config, calibration);
        var point = new WeldPoint { Index = 0, Name = "Img010" };

        using var image = generator.Render(new PointScenario
        {
            Point = point,
            Defect = WeldDefect.MissingWeld,
            TrueOffsetXmm = 0.1,
            TrueOffsetYmm = 0.1
        }, new Random(3));
        var result = algorithm.Inspect(image, point);

        Assert.Contains(WeldDefect.MissingWeld, result.Defects);
        Assert.False(result.HasValidOffset);
        Assert.Equal(0.0, result.OffsetDistanceMm);
        // 判定依据必须随结果带出：只报"漏焊"操作员无法复核
        Assert.StartsWith("漏焊：", result.DefectTextZh);
        Assert.Contains("未找到", result.DefectTextZh);
        Assert.Contains("偏移量不适用", result.ToString());
    }

    [Fact]
    public void DefectNames_AreSharedChineseText()
    {
        Assert.Equal("焊偏", WeldDefectText.Of(WeldDefect.Misaligned));
        Assert.Equal("漏焊、焊渣飞溅",
            WeldDefectText.Of(new[] { WeldDefect.MissingWeld, WeldDefect.Spatter }));
        Assert.Equal("-", WeldDefectText.Of(Array.Empty<WeldDefect>()));
    }

    [Fact]
    public void CsvReport_KeepsSourceFileName_WithEscaping()
    {
        var report = new InspectionReport
        {
            StartedAtUtc = DateTime.UtcNow,
            FinishedAtUtc = DateTime.UtcNow,
            Results = new[]
            {
                new PointInspectionResult
                {
                    Point = new WeldPoint
                    {
                        Index = 0,
                        Name = "Img000",
                        SourceName = "19-22-07 729_a,b\"c.jpg"
                    },
                    Defects = new[] { WeldDefect.MissingWeld }
                }
            }
        };

        var path = Path.Combine(Path.GetTempPath(), $"bwaoi_report_{Guid.NewGuid():N}.csv");
        try
        {
            CsvReportWriter.Write(report, path);
            var lines = File.ReadAllLines(path);

            Assert.StartsWith("Index,Point,Source,Result", lines[0]);
            Assert.Contains("\"19-22-07 729_a,b\"\"c.jpg\"", lines[1]);
            Assert.Contains("MissingWeld", lines[1]);
        }
        finally
        {
            File.Delete(path);
        }
    }

    // ---- 辅助 ----

    private static NinePointCalibration FlatCalibration() => new(new List<(Point2d Pixel, Point2d Mm)>
    {
        (new Point2d(0, 0), new Point2d(0, 0)),
        (new Point2d(1000, 0), new Point2d(25, 0)),
        (new Point2d(0, 1000), new Point2d(0, 25)),
    });

    private static Mat RenderLabel(string text)
    {
        var image = new Mat(200, 1600, MatType.CV_8UC3, new Scalar(30, 30, 30));
        AnnotationText.Draw(image, text, new Point(20, 120), 64, new Scalar(0, 0, 255));
        return image;
    }

    /// <summary>两图不同像素的占比（仅比较同一尺寸的前三通道）。</summary>
    private static double DiffRatio(Mat a, Mat b)
    {
        Assert.Equal(a.Size(), b.Size());
        using var diff = new Mat();
        Cv2.Absdiff(a, b, diff);
        using var gray = new Mat();
        Cv2.CvtColor(diff, gray, ColorConversionCodes.BGR2GRAY);
        using var mask = new Mat();
        Cv2.Threshold(gray, mask, 8, 255, ThresholdTypes.Binary);
        var changed = Cv2.CountNonZero(mask);
        return changed / (double)(a.Width * a.Height);
    }
}
