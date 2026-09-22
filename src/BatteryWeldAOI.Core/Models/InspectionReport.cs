namespace BatteryWeldAOI.Core.Models;

/// <summary>
/// 整包检测报表：汇总所有点位结果与节拍信息。
/// </summary>
public sealed class InspectionReport
{
    public required DateTime StartedAtUtc { get; init; }

    public required DateTime FinishedAtUtc { get; init; }

    public required IReadOnlyList<PointInspectionResult> Results { get; init; }

    public int TotalCount => Results.Count;

    public int OkCount => Results.Count(r => r.IsOk);

    /// <summary>缺陷 NG 数（不含处理异常点）。</summary>
    public int NgCount => Results.Count(r => !r.IsOk && !r.HasError);

    /// <summary>处理异常点数（取图/算法故障，需人工复检）。</summary>
    public int ErrorCount => Results.Count(r => r.HasError);

    /// <summary>流程被取消（如操作员中止）；报表为已完成点位的部分结果。</summary>
    public bool WasCancelled { get; init; }

    /// <summary>结果上传 PLC 最终失败（重试耗尽）的点数。</summary>
    public int UploadFailedCount { get; init; }

    /// <summary>整包节拍（秒）。</summary>
    public double CycleTimeSec => (FinishedAtUtc - StartedAtUtc).TotalSeconds;

    /// <summary>按缺陷类型统计数量。</summary>
    public IReadOnlyDictionary<WeldDefect, int> DefectSummary =>
        Results.SelectMany(r => r.Defects)
               .GroupBy(d => d)
               .ToDictionary(g => g.Key, g => g.Count());
}
