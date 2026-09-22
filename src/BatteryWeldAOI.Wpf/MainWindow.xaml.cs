namespace BatteryWeldAOI.Wpf;

using System.Collections.Specialized;
using System.ComponentModel;
using System.Windows;
using System.Windows.Threading;
using BatteryWeldAOI.Wpf.ViewModels;

/// <summary>
/// 主窗口：负责视图装配、日志自动滚动与点位详情窗口的生命周期管理，
/// 业务全部在 MainViewModel。
/// </summary>
public partial class MainWindow : Window
{
    private readonly MainViewModel _viewModel = new();
    private PointDetailWindow? _detailWindow;

    public MainWindow()
    {
        InitializeComponent();
        DataContext = _viewModel;
        _viewModel.Logs.CollectionChanged += LogsCollectionChanged;
        _viewModel.PropertyChanged += OnViewModelPropertyChanged;
    }

    /// <summary>日志追加时自动滚动到底部。必须在 Background 优先级延迟执行——
    /// 在 CollectionChanged 内同步 ScrollIntoView 会打断项生成器，
    /// 批量追加时触发「ItemsControl 与项源不一致」崩溃。</summary>
    private void LogsCollectionChanged(object? sender, NotifyCollectionChangedEventArgs e)
    {
        if (e.Action == NotifyCollectionChangedAction.Add && LogList.Items.Count > 0)
            Dispatcher.BeginInvoke(
                () => LogList.ScrollIntoView(LogList.Items[^1]),
                DispatcherPriority.Background);
    }

    /// <summary>打开或切换点位详情窗口（非模态）。
    /// WPF Window 关闭后不可复用（Close 后再 Show 会抛 InvalidOperationException），
    /// 因此窗口任何方式关闭时即清空引用，下次点击重建。</summary>
    private void ShowPointDetail(PointItemViewModel point)
    {
        if (_detailWindow is null)
        {
            _detailWindow = new PointDetailWindow { Owner = this };
            _detailWindow.Closed += (_, _) => _detailWindow = null;
        }
        _detailWindow.ShowPoint(point, _viewModel.Points.ToList(),
            _viewModel.Config.Inspection.OffsetToleranceMm);
    }

    /// <summary>点位矩阵格子点击：已完成的点位弹出详情窗口。</summary>
    private void OnPointCellClick(object sender, RoutedEventArgs e)
    {
        if ((sender as FrameworkElement)?.DataContext is PointItemViewModel { HasDetail: true } item)
            ShowPointDetail(item);
    }

    /// <summary>新一轮检测开始时关闭详情窗口（旧点位列表已失效）。</summary>
    private void OnViewModelPropertyChanged(object? sender, PropertyChangedEventArgs e)
    {
        if (e.PropertyName == nameof(MainViewModel.IsRunning) && _viewModel.IsRunning)
            _detailWindow?.Close();
    }

    protected override void OnClosed(EventArgs e)
    {
        _detailWindow?.Close();
        _viewModel.Shutdown();
        base.OnClosed(e);
    }
}
