namespace BatteryWeldAOI.Core.Reporting;

using System.Globalization;
using System.Text;
using BatteryWeldAOI.Core.Models;

/// <summary>
/// CSV 报表导出（UTF-8 BOM，Excel 可直接打开）。
/// </summary>
public static class CsvReportWriter
{
    public static void Write(InspectionReport report, string filePath)
    {
        var sb = new StringBuilder();
        // Source 列 = 该点位对应的原始图像（图片文件夹/视频模式），
        // 没有它就无法把报表里的 Img007 对应回具体文件，异常点没法复查。
        // Reason 列 = 判定依据（漏焊的实测覆盖率/填充率与下限等）；测量失败的原因已并入 Defects 列。
        // ArcSupport 列 = 焊环边界的支撑弧段比例（0~1）。它是判断"这个焊环圆心可不可信"的唯一直接量
        // （见 InspectionConfig.MinRingArcSupport）：良品 0.93~1.00，被细圈污染的帧只有 0.64~0.94。
        // 复查某一帧时，先看这一列就知道该不该信那个圆。
        sb.AppendLine("Index,Point,Source,Result,OffsetX_mm,OffsetY_mm,Distance_mm,ArcSupport,Defects,Reason,Processing_ms");

        var inv = CultureInfo.InvariantCulture;
        foreach (var r in report.Results)
        {
            sb.Append(r.Point.Index).Append(',')
              .Append(r.Point.Name).Append(',')
              .Append(Escape(r.Point.SourceName)).Append(',')
              .Append(r.HasError ? "ERROR" : r.IsOk ? "OK" : "NG").Append(',')
              .Append(r.OffsetXmm.ToString("F4", inv)).Append(',')
              .Append(r.OffsetYmm.ToString("F4", inv)).Append(',')
              .Append(r.OffsetDistanceMm.ToString("F4", inv)).Append(',')
              .Append(r.WeldArcSupport.ToString("F2", inv)).Append(',')
              .Append(Escape(r.DefectText)).Append(',')
              .Append(Escape(r.HasError ? null : r.ErrorMessage)).Append(',')
              .Append(r.ProcessingMs.ToString("F1", inv)).AppendLine();
        }

        sb.AppendLine();
        sb.Append("Summary,,Total,").Append(report.TotalCount).AppendLine();
        sb.Append(",,OK,").Append(report.OkCount).AppendLine();
        sb.Append(",,NG,").Append(report.NgCount).AppendLine();
        sb.Append(",,Error,").Append(report.ErrorCount).AppendLine();
        sb.Append(",,UploadFailed,").Append(report.UploadFailedCount).AppendLine();
        sb.Append(",,Cancelled,").Append(report.WasCancelled ? "Yes" : "No").AppendLine();
        sb.Append(",,CycleTime_s,").Append(report.CycleTimeSec.ToString("F2", inv)).AppendLine();

        Directory.CreateDirectory(Path.GetDirectoryName(Path.GetFullPath(filePath))!);
        File.WriteAllText(filePath, sb.ToString(), new UTF8Encoding(true));
    }

    /// <summary>CSV 字段转义：含逗号/引号/换行（文件名与诊断信息都可能带）时加引号包裹。</summary>
    private static string Escape(string? value)
    {
        if (string.IsNullOrEmpty(value))
            return string.Empty;
        if (value.IndexOfAny(new[] { ',', '"', '\n', '\r' }) < 0)
            return value;
        return "\"" + value.Replace("\"", "\"\"") + "\"";
    }
}
