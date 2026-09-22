namespace BatteryWeldAOI.Wpf;

using System.IO;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using BatteryWeldAOI.Core.Models;
using BatteryWeldAOI.Wpf.ViewModels;

/// <summary>
/// 点位检测详情窗口（非模态）：显示单个点位的标注大图与全部检测指标。
/// 主窗口点击其他格子弹出/更新本窗口；◀▶ 或方向键在已完成点位间导航。
/// </summary>
public partial class PointDetailWindow : Window
{
    private IReadOnlyList<PointItemViewModel> _points = Array.Empty<PointItemViewModel>();
    private int _index = -1;
    private double _toleranceMm;

    public PointDetailWindow()
    {
        InitializeComponent();
        KeyDown += OnKeyDown;
    }

    /// <summary>显示指定点位；窗口已打开时仅切换内容。</summary>
    public void ShowPoint(PointItemViewModel point, IReadOnlyList<PointItemViewModel> allPoints, double toleranceMm)
    {
        _points = allPoints;
        _toleranceMm = toleranceMm;
        _index = -1;
        for (var i = 0; i < allPoints.Count; i++)
        {
            if (ReferenceEquals(allPoints[i], point))
            {
                _index = i;
                break;
            }
        }
        Update();

        if (!IsVisible)
            Show();
        Activate();
    }

    private void Update()
    {
        if (_index < 0 || _index >= _points.Count)
            return;

        var point = _points[_index];
        var result = point.Result;
        if (result is null)
            return;

        // ---- 头部 ----
        Title = $"点位详情 — {point.Name}";
        PointNameText.Text = $"{point.Name}（#{point.Index:D2}）";

        var (statusLabel, badgeColor, labelColor) = result.HasError
            ? ("异常", Color.FromRgb(0xEF, 0x6C, 0x00), Colors.White)
            : result.IsOk
                ? ("OK", Color.FromRgb(0x2E, 0x7D, 0x32), Colors.White)
                : ("NG", Color.FromRgb(0xC6, 0x28, 0x28), Colors.White);
        StatusBadge.Background = new SolidColorBrush(badgeColor);
        StatusText.Text = statusLabel;
        StatusText.Foreground = new SolidColorBrush(labelColor);

        // ---- 指标 ----
        ResultText.Text = result.HasError ? $"异常：{result.ErrorMessage}" : result.IsOk ? "OK（合格）" : "NG（不合格）";
        ResultText.Foreground = new SolidColorBrush(result.HasError
            ? Color.FromRgb(0xEF, 0x6C, 0x00)
            : result.IsOk ? Color.FromRgb(0x66, 0xBB, 0x6A) : Color.FromRgb(0xEF, 0x53, 0x50));

        DefectText.Text = result.HasError || result.IsOk
            ? "-"
            : result.DefectTextZh;

        // 偏移量无物理意义（测量失败 / 未检出焊环）时一律显示"--"：
        // 给 0.000mm 会被读成"焊得很正"，正好把该人工复核的点位读反。
        OffsetXText.Text = result.HasValidOffset ? $"{result.OffsetXmm:+0.000;-0.000} mm" : "--";
        OffsetYText.Text = result.HasValidOffset ? $"{result.OffsetYmm:+0.000;-0.000} mm" : "--";

        var misaligned = result.Defects.Contains(WeldDefect.Misaligned);
        if (!result.HasValidOffset)
        {
            DistanceText.Text = "—（偏移量不适用）";
            DistanceText.Foreground = new SolidColorBrush(Color.FromRgb(0x9E, 0x9E, 0x9E));
        }
        else
        {
            // 公差结论只在"偏移超差"和"合格"两种情形下成立：
            // 漏焊/炸焊/飞溅的点位偏移可能完全在公差内，写"（公差内）"会和 NG 徽标自相矛盾。
            var tolerance = misaligned ? "（超差）" : result.IsOk ? "（公差内）" : "（偏移未超差）";
            DistanceText.Text = $"{result.OffsetDistanceMm:F3} mm{tolerance}";
            DistanceText.Foreground = new SolidColorBrush(
                misaligned ? Color.FromRgb(0xEF, 0x53, 0x50) : Colors.White);
        }

        ToleranceText.Text = $"±{_toleranceMm:F3} mm";
        ProcessingText.Text = result.HasError ? "--" : $"{result.ProcessingMs:F1} ms";

        // 坐标与半径一律给**原图**坐标系（标签已注明"（原图）"，图幅见「图像尺寸」行）。
        // 算法在输入图较大时会先缩到工作分辨率再检测，直接呈现内部坐标会让操作员
        // 拿它去图上核对时量不到（20MP 现场图上与显示值差约 8 倍），于是"标注圈是不是偏了"
        // 这种问题无法自查——这正是这套坐标被改成原图口径的原因。
        //
        // 半径也要给：只给中心，操作员无法判断标注圈的大小对不对
        // （"圈画得比焊环大一圈"这类疑问都出在半径上）。
        PoleCenterText.Text = result.PoleRadiusPx > 0
            ? $"({result.PoleCenterSourcePx.X:F1}, {result.PoleCenterSourcePx.Y:F1})  r={result.PoleRadiusSourcePx:F1}"
            : "（未检出孔洞）";
        // 边界支撑弧段一并给出：它是判断"这个焊环圆心可不可信"的唯一直接量（见
        // InspectionConfig.MinRingArcSupport）。只给圆心/半径，操作员没法区分
        // "这个圈是量出来的"和"这个圈被细圈污染带偏了"——实测污染帧的支撑弧段只有
        // 0.64~0.94，而良品是 0.93~1.00。
        WeldCenterText.Text = result.HasError
            ? $"--（支撑弧段 {result.WeldArcSupport:P0}）"
            : result.WeldRadiusPx > 0
                ? $"({result.WeldCenterSourcePx.X:F1}, {result.WeldCenterSourcePx.Y:F1})  " +
                  $"r={result.WeldRadiusSourcePx:F1}  支撑弧段 {result.WeldArcSupport:P0}"
                : "（未检出焊缝）";

        // 熔核是"有厚度的环"：把内沿半径与环宽一并给出（内沿取不到时说明原因，
        // 而不是显示 0——0 会被读成"环宽为零"，与"未取到"是两件事）。
        RingWidthText.Text = result.WeldRadiusSourcePx <= 0
            ? "—"
            : result.WeldInnerRadiusPx > 0
                ? $"外半径 {result.WeldRadiusSourcePx:F1} ｜ 内半径 {result.WeldInnerRadiusSourcePx:F1} " +
                  $"｜ 环宽 {result.WeldRingWidthSourcePx:F1} px"
                : "—（本次未取到熔核内沿）";

        // 原图坐标要配图幅才可解释；同时点出工作分辨率，避免把"坐标与图对不上"误当成算法错位
        var sourceSize = result.SourceSizePx.Width > 0 ? result.SourceSizePx : result.ProcessedSizePx;
        var workSize = result.ProcessedSizePx;
        ImageSizeText.Text = sourceSize.Width <= 0
            ? "—"
            : sourceSize == workSize
                ? $"{sourceSize.Width}×{sourceSize.Height}（未缩放）"
                : $"{sourceSize.Width}×{sourceSize.Height}，检测工作图 {workSize.Width}×{workSize.Height}" +
                  $"（缩放比 1:{result.SourceScale:F2}）";

        SourceText.Text = string.IsNullOrWhiteSpace(result.Point.SourceName)
            ? "—（仿真/相机模式无源文件）"
            : result.Point.SourceName;
        SourceText.ToolTip = result.Point.SourceName;

        // ---- 图像（懒加载，检测时不常驻内存） ----
        LoadImage(point.ImagePath);

        // ---- 导航可用性 ----
        PrevButton.IsEnabled = FindNeighbor(-1) >= 0;
        NextButton.IsEnabled = FindNeighbor(1) >= 0;
    }

    private void LoadImage(string? path)
    {
        if (path is not null && File.Exists(path))
        {
            try
            {
                var bitmap = new BitmapImage();
                bitmap.BeginInit();
                bitmap.CacheOption = BitmapCacheOption.OnLoad;
                bitmap.UriSource = new Uri(Path.GetFullPath(path));
                bitmap.EndInit();
                bitmap.Freeze();
                DetailImage.Source = bitmap;
                NoImageText.Visibility = Visibility.Collapsed;
                ImagePathText.Text = path;
                ImagePathText.ToolTip = path;
                return;
            }
            catch (IOException)
            {
                // 文件读取失败按无图处理
            }
        }

        DetailImage.Source = null;
        NoImageText.Visibility = Visibility.Visible;
        ImagePathText.Text = path is null ? "未保存" : "文件缺失";
        ImagePathText.ToolTip = null;
    }

    /// <summary>找相邻的已完成点位（跳过未检/检测中）。</summary>
    private int FindNeighbor(int direction)
    {
        var i = _index;
        while (true)
        {
            i += direction;
            if (i < 0 || i >= _points.Count)
                return -1;
            if (_points[i].HasDetail)
                return i;
        }
    }

    private void Navigate(int direction)
    {
        var target = FindNeighbor(direction);
        if (target >= 0)
        {
            _index = target;
            Update();
        }
    }

    private void OnPrev(object sender, RoutedEventArgs e) => Navigate(-1);

    private void OnNext(object sender, RoutedEventArgs e) => Navigate(1);

    private void OnKeyDown(object sender, KeyEventArgs e)
    {
        switch (e.Key)
        {
            case Key.Left:
                Navigate(-1);
                e.Handled = true;
                break;
            case Key.Right:
                Navigate(1);
                e.Handled = true;
                break;
            case Key.Escape:
                Close();
                e.Handled = true;
                break;
        }
    }

    private void OnExport(object sender, RoutedEventArgs e)
    {
        var point = _index >= 0 && _index < _points.Count ? _points[_index] : null;
        if (point?.ImagePath is not { } source || !File.Exists(source))
        {
            MessageBox.Show(this, "该点位没有已保存的标注图，无法导出。\n请开启「保存逐点标注图」后重新检测。",
                "点位详情", MessageBoxButton.OK, MessageBoxImage.Information);
            return;
        }

        var dialog = new Microsoft.Win32.SaveFileDialog
        {
            Title = "导出标注图",
            FileName = $"{point.Name}_{(point.Result?.IsOk == true ? "OK" : "NG")}.png",
            Filter = "PNG 图像|*.png"
        };
        if (dialog.ShowDialog(this) == true)
        {
            try
            {
                File.Copy(source, dialog.FileName, overwrite: true);
            }
            catch (Exception ex)
            {
                MessageBox.Show(this, $"导出失败: {ex.Message}", "点位详情",
                    MessageBoxButton.OK, MessageBoxImage.Warning);
            }
        }
    }

    private void OnClose(object sender, RoutedEventArgs e) => Close();
}
