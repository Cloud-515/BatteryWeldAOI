namespace BatteryWeldAOI.Wpf.ViewModels;

using System.Windows.Media;
using BatteryWeldAOI.Core.Models;

/// <summary>点位矩阵中单个点位的显示状态。</summary>
public enum PointStatus
{
    Pending,
    Inspecting,
    Ok,
    Ng,
    Error
}

/// <summary>
/// 点位矩阵单元格视图模型。检测完成后挂载完整结果与标注图路径，
/// 供详情窗口（点击格子弹出）读取。
/// </summary>
public sealed class PointItemViewModel : ObservableObject
{
    private PointStatus _status = PointStatus.Pending;
    private string _tooltip = string.Empty;

    public int Index { get; }
    public string Name { get; }

    /// <summary>检测结果（完成后有效）；null 表示尚未完成。</summary>
    public PointInspectionResult? Result { get; private set; }

    /// <summary>标注图 PNG 路径（保存开关开启时有效）。</summary>
    public string? ImagePath { get; private set; }

    public PointItemViewModel(int index, string name)
    {
        Index = index;
        Name = name;
        _tooltip = $"{name}（待检）";
    }

    public PointStatus Status
    {
        get => _status;
        set
        {
            if (SetProperty(ref _status, value))
                OnPropertyChanged(nameof(Brush));
        }
    }

    public string Tooltip
    {
        get => _tooltip;
        private set => SetProperty(ref _tooltip, value);
    }

    public string IndexText => $"{Index:D2}";

    public Brush Brush => Status switch
    {
        PointStatus.Ok => new SolidColorBrush(Color.FromRgb(0x2E, 0x7D, 0x32)),
        PointStatus.Ng => new SolidColorBrush(Color.FromRgb(0xC6, 0x28, 0x28)),
        PointStatus.Error => new SolidColorBrush(Color.FromRgb(0xEF, 0x6C, 0x00)),
        PointStatus.Inspecting => new SolidColorBrush(Color.FromRgb(0x15, 0x65, 0xC0)),
        _ => new SolidColorBrush(Color.FromRgb(0x3C, 0x3C, 0x3C))
    };

    /// <summary>是否可打开详情（已完成检测：OK/NG/异常）。</summary>
    public bool HasDetail => Status is PointStatus.Ok or PointStatus.Ng or PointStatus.Error;

    /// <summary>检测结果到达：挂载结果、图像路径与提示文本。</summary>
    public void SetResult(PointInspectionResult result, string? imagePath)
    {
        Result = result;
        ImagePath = imagePath;

        var source = string.IsNullOrWhiteSpace(result.Point.SourceName)
            ? string.Empty
            : $"\n源文件: {result.Point.SourceName}";

        // 偏移量无物理意义（测量失败 / 未检出焊环）时显示"不适用"，
        // 不显示 0.000mm——那会被读成"焊得很正"。
        var offset = result.HasValidOffset
            ? $", D={result.OffsetDistanceMm:F3}mm, dX={result.OffsetXmm:F3} dY={result.OffsetYmm:F3}"
            : "，偏移量不适用（未检出焊环）";

        Tooltip = result.HasError
            ? $"{Name}: 异常 - {result.ErrorMessage}{source}"
            : $"{Name}: {result.DefectTextZh}{offset} ({result.ProcessingMs:F0}ms){source}";
    }
}
