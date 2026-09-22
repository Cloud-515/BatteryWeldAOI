namespace BatteryWeldAOI.Wpf.ViewModels;

using System.Collections.ObjectModel;
using System.Diagnostics;
using System.IO;
using System.Windows.Input;
using System.Windows.Media.Imaging;
using System.Windows.Threading;
using BatteryWeldAOI.Core.Camera;
using BatteryWeldAOI.Core.Communication;
using BatteryWeldAOI.Core.Configuration;
using BatteryWeldAOI.Core.Models;
using BatteryWeldAOI.Core.Orchestration;
using BatteryWeldAOI.Core.Pipeline;
using BatteryWeldAOI.Core.Planning;
using BatteryWeldAOI.Core.Reporting;
using BatteryWeldAOI.Core.StateMachine;
using BatteryWeldAOI.Core.Vision;
using BatteryWeldAOI.Wpf.Services;
using OpenCvSharp;

/// <summary>
/// 主窗口视图模型：支持三种输入源（仿真 / 视频文件 / 摄像头），
/// 负责整包检测的启动/停止、实时结果展示、参数配置持久化与报表导出。
/// 所有设备事件统一封送回 UI 线程。
/// </summary>
public sealed class MainViewModel : ObservableObject
{
    private const int MaxVideoFrames = 10_000;
    private const int MaxLogLines = 800;

    private readonly string _configPath;
    private readonly SynchronizationContext _ui;
    private readonly Stopwatch _stopwatch = new();
    private readonly DispatcherTimer _timer;
    private readonly DispatcherTimer _previewTimer;
    private readonly object _resultsLock = new();
    private readonly List<PointInspectionResult> _results = new();
    private DateTime _runStartedUtc;

    private CancellationTokenSource? _cts;
    private PlcSimulator? _simulator;
    private IPlcClient? _plc;
    private ICameraGrabber? _camera;
    private ImagePipeline? _pipeline;
    private VideoCapture? _previewCapture;
    private bool _isPreviewing;

    private bool _isRunning;
    private BitmapSource? _currentImage;
    private string _imageCaption = string.Empty;
    private string _machineStateText = "待机";
    private string _elapsedText = "00:00.0";
    private string _cycleTimeText = "--";
    private int _processedCount;
    private int _okCount;
    private int _ngCount;
    private int _errorCount;

    public MainViewModel()
    {
        _ui = SynchronizationContext.Current
              ?? throw new InvalidOperationException("MainViewModel 必须在 UI 线程创建");

        _configPath = Path.Combine(AppContext.BaseDirectory, "config.json");
        Config = AppConfig.LoadOrDefault(_configPath);
        RebuildPoints();

        Logs = new ObservableCollection<string>();
        Logs.Add($"配置文件: {_configPath}");
        Logs.Add($"当前配方: 输入源 {TranslateSource(Config.InputSource)}, 点位数 {Config.PointCount}, " +
                 $"公差 ±{Config.Inspection.OffsetToleranceMm}mm, 流水线 {Config.WorkerCount} 线程");
        // 把**生效的标定**一并打出来：配置文件里若冻结了旧的 MmPerPixel（本机曾把 0.025 落盘，
        // 而现场实测应为 0.1003），所有毫米读数会静默偏小 4 倍且界面上看不出来——2026-09-17 实测踩到过。
        Logs.Add($"[标定] 生效当量 {Config.Inspection.MmPerPixel}mm/标定像素 ⇒ " +
                 $"视野 {Config.Inspection.ImageWidth * Config.Inspection.MmPerPixel:F1}×" +
                 $"{Config.Inspection.ImageHeight * Config.Inspection.MmPerPixel:F1}mm" +
                 $"（现场实测应为 0.0235mm/原图像素；若此数不是 0.1003，请检查配置文件是否冻结了旧值）");

        _timer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(100) };
        _timer.Tick += (_, _) => ElapsedText = _stopwatch.Elapsed.ToString(@"mm\:ss\.f");

        _previewTimer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(66) };
        _previewTimer.Tick += (_, _) => PreviewTick();

        StartCommand = new RelayCommand(_ => _ = RunAsync(), _ => !IsRunning);
        StopCommand = new RelayCommand(_ => Stop(), _ => IsRunning);
        ResetCommand = new RelayCommand(_ => Reset(), _ => !IsRunning);
        ExportCommand = new RelayCommand(_ => ExportReport(), _ => !IsRunning && _results.Count > 0);
        SaveConfigCommand = new RelayCommand(_ => SaveConfig(), _ => !IsRunning);
        BrowseVideoCommand = new RelayCommand(_ => BrowseVideo(), _ => !IsRunning);
        BrowseImageFolderCommand = new RelayCommand(_ => BrowseImageFolder(), _ => !IsRunning);
        TogglePreviewCommand = new RelayCommand(_ => TogglePreview(),
            _ => !IsRunning && IsCameraSource);
    }

    // ---- 绑定属性 ----

    public AppConfig Config { get; }

    public ObservableCollection<PointItemViewModel> Points { get; } = new();

    public ObservableCollection<string> Logs { get; }

    public ICommand StartCommand { get; }
    public ICommand StopCommand { get; }
    public ICommand ResetCommand { get; }
    public ICommand ExportCommand { get; }
    public ICommand SaveConfigCommand { get; }
    public ICommand BrowseVideoCommand { get; }
    public ICommand BrowseImageFolderCommand { get; }
    public ICommand TogglePreviewCommand { get; }

    // ---- 输入源 ----

    public bool IsSimulationSource
    {
        get => Config.InputSource == InputSource.Simulation;
        set { if (value) SetSource(InputSource.Simulation); }
    }

    public bool IsVideoFileSource
    {
        get => Config.InputSource == InputSource.VideoFile;
        set { if (value) SetSource(InputSource.VideoFile); }
    }

    public bool IsImageFolderSource
    {
        get => Config.InputSource == InputSource.ImageFolder;
        set { if (value) SetSource(InputSource.ImageFolder); }
    }

    public bool IsCameraSource
    {
        get => Config.InputSource == InputSource.Camera;
        set { if (value) SetSource(InputSource.Camera); }
    }

    public string VideoFilePath => Config.VideoFilePath;

    public string ImageFolderPath => Config.ImageFolderPath;

    public bool IsPreviewing
    {
        get => _isPreviewing;
        private set
        {
            if (SetProperty(ref _isPreviewing, value))
                OnPropertyChanged(nameof(TogglePreviewText));
        }
    }

    public string TogglePreviewText => IsPreviewing ? "关闭预览" : "打开预览";

    private void SetSource(InputSource source)
    {
        if (Config.InputSource == source)
            return;
        Config.InputSource = source;
        if (source != InputSource.Camera && IsPreviewing)
            StopPreview();
        OnPropertyChanged(nameof(IsSimulationSource));
        OnPropertyChanged(nameof(IsVideoFileSource));
        OnPropertyChanged(nameof(IsImageFolderSource));
        OnPropertyChanged(nameof(IsCameraSource));
        CommandManager.InvalidateRequerySuggested();
        Log($"输入源切换为: {TranslateSource(source)}");
    }

    private void BrowseImageFolder()
    {
        var dialog = new System.Windows.Forms.FolderBrowserDialog
        {
            Description = "选择包含待检图片的文件夹",
            ShowNewFolderButton = false,
            UseDescriptionForTitle = true
        };
        if (dialog.ShowDialog() != System.Windows.Forms.DialogResult.OK)
            return;

        Config.ImageFolderPath = dialog.SelectedPath;
        OnPropertyChanged(nameof(ImageFolderPath));

        // 预读首张图片验证文件夹内容，并作为缩略图
        try
        {
            using var probe = new ImageFolderGrabber(dialog.SelectedPath,
                Config.Inspection.ImageWidth, Config.Inspection.ImageHeight);
            using var frame = probe.TriggerGrab(0);
            CurrentImage = MatConverter.ToBitmapSource(frame);
            ImageCaption = $"{Path.GetFileName(dialog.SelectedPath)} — 共 {probe.ImageCount} 张图片（首张预览: {Path.GetFileName(probe.Files[0])}）";
            Log($"[输入源] 已选择图片文件夹: {dialog.SelectedPath} ({probe.ImageCount} 张)");

            // 图幅与配置分辨率不一致不会影响检测判定（算法参数均为尺度相对量），
            // 但毫米换算按"配置视野"折算，故必须让操作员看见这个差异并核对标定。
            // 宽高比也不一致时（说明配置里那组像素尺寸不是该相机的真实图幅）额外点出来。
            // 注意措辞：**毫米读数的尺度现在已经可信**——MmPerPixel 由现场实测重标过
            // （2026-09-17：凹陷圆环 30mm ↔ 1278px ⇒ 0.0235mm/原图像素），且折算系数是各向同性的；
            // 宽高比差异只说明"配置的像素尺寸是标定坐标系的，不是相机的"，不再意味着尺度是猜的。
            if (frame.Width != probe.ConfiguredSize.Width || frame.Height != probe.ConfiguredSize.Height)
            {
                var aspectDeviation = Config.Inspection.FovAspectDeviation(frame.Width, frame.Height);
                var aspectNote = aspectDeviation > 0.01
                    ? $"，且两者宽高比相差 {aspectDeviation:P0}（配置里的像素尺寸不是该相机的真实图幅）"
                    : string.Empty;
                Log($"[标定] 提示: 图片实际尺寸 {frame.Width}×{frame.Height} 与配置坐标系 " +
                    $"{probe.ConfiguredSize.Width}×{probe.ConfiguredSize.Height} 不一致{aspectNote}。" +
                    $"毫米偏移按配置视野（{Config.Inspection.ImageWidth * Config.Inspection.MmPerPixel:F1}mm 宽）等比折算，" +
                    $"当前当量 {Config.Inspection.MmPerPixel}mm/标定像素 由现场实测得到；" +
                    $"要做正式九点标定时，把 ImageWidth/ImageHeight 改成相机真实图幅、MmPerPixel 按标定板结果填。");
            }
        }
        catch (Exception ex)
        {
            Log($"[输入源] 图片文件夹不可用: {ex.Message}");
        }
    }

    private void BrowseVideo()
    {
        var dialog = new Microsoft.Win32.OpenFileDialog
        {
            Title = "选择检测视频",
            Filter = "视频文件|*.avi;*.mp4;*.mkv;*.mov;*.wmv|所有文件|*.*"
        };
        if (dialog.ShowDialog() != true)
            return;

        Config.VideoFilePath = dialog.FileName;
        OnPropertyChanged(nameof(VideoFilePath));

        // 预读首帧验证可解码性，并作为缩略图
        try
        {
            using var probe = new VideoFileGrabber(dialog.FileName,
                Config.Inspection.ImageWidth, Config.Inspection.ImageHeight);
            using var frame = probe.TriggerGrab(0);
            CurrentImage = MatConverter.ToBitmapSource(frame);
            ImageCaption = $"{Path.GetFileName(dialog.FileName)} — 共 {probe.FrameCount} 帧（首帧预览）";
            Log($"[输入源] 已选择视频: {dialog.FileName} ({probe.FrameCount} 帧)");
        }
        catch (Exception ex)
        {
            Log($"[输入源] 视频无法读取: {ex.Message}");
        }
    }

    // ---- 摄像头预览 ----

    private void TogglePreview()
    {
        if (IsPreviewing)
        {
            StopPreview();
            return;
        }

        try
        {
            var capture = new VideoCapture(Config.CameraDeviceIndex, VideoCaptureAPIs.DSHOW);
            if (!capture.IsOpened())
            {
                capture.Dispose();
                Log($"[预览] 无法打开摄像头 {Config.CameraDeviceIndex}（设备不存在或被占用）");
                return;
            }

            _previewCapture = capture;
            IsPreviewing = true;
            ImageCaption = $"相机预览 #{Config.CameraDeviceIndex}（点击「开始检测」后将逐帧检测）";
            Log($"[预览] 摄像头 {Config.CameraDeviceIndex} 已打开");
            _previewTimer.Start();
        }
        catch (Exception ex)
        {
            Log($"[预览] 打开失败: {ex.Message}");
        }
    }

    private void StopPreview()
    {
        _previewTimer.Stop();
        _previewCapture?.Dispose();
        _previewCapture = null;
        IsPreviewing = false;
        Log("[预览] 已关闭");
    }

    private void PreviewTick()
    {
        if (_previewCapture is null)
            return;
        using var frame = new Mat();
        if (!_previewCapture.Read(frame) || frame.Empty())
            return;
        var bitmap = MatConverter.ToBitmapSource(frame);
        if (bitmap is not null)
            CurrentImage = bitmap;
    }

    // ---- 运行状态 ----

    public bool IsRunning
    {
        get => _isRunning;
        private set
        {
            if (SetProperty(ref _isRunning, value))
            {
                OnPropertyChanged(nameof(CanEditParams));
                CommandManager.InvalidateRequerySuggested();
            }
        }
    }

    public bool CanEditParams => !IsRunning;

    public BitmapSource? CurrentImage
    {
        get => _currentImage;
        private set => SetProperty(ref _currentImage, value);
    }

    public string ImageCaption
    {
        get => _imageCaption;
        private set => SetProperty(ref _imageCaption, value);
    }

    public string MachineStateText
    {
        get => _machineStateText;
        private set => SetProperty(ref _machineStateText, value);
    }

    public string ElapsedText
    {
        get => _elapsedText;
        private set => SetProperty(ref _elapsedText, value);
    }

    public string CycleTimeText
    {
        get => _cycleTimeText;
        private set => SetProperty(ref _cycleTimeText, value);
    }

    public int TotalCount => Points.Count;

    public int ProcessedCount
    {
        get => _processedCount;
        private set => SetProperty(ref _processedCount, value);
    }

    public int OkCount
    {
        get => _okCount;
        private set => SetProperty(ref _okCount, value);
    }

    public int NgCount
    {
        get => _ngCount;
        private set => SetProperty(ref _ngCount, value);
    }

    public int ErrorCount
    {
        get => _errorCount;
        private set => SetProperty(ref _errorCount, value);
    }

    public double ProgressPercent => TotalCount == 0 ? 0 : (double)ProcessedCount / TotalCount * 100;

    public string OutputDir => Path.Combine(AppContext.BaseDirectory, Config.OutputDirectory);

    // ---- 命令实现 ----

    private async Task RunAsync()
    {
        if (IsRunning)
            return;

        var config = Config;
        var inspection = config.Inspection;

        if (config.InputSource == InputSource.VideoFile &&
            (string.IsNullOrWhiteSpace(config.VideoFilePath) || !File.Exists(config.VideoFilePath)))
        {
            Log("[错误] 请先选择有效的视频文件");
            return;
        }

        if (config.InputSource == InputSource.ImageFolder &&
            (string.IsNullOrWhiteSpace(config.ImageFolderPath) || !Directory.Exists(config.ImageFolderPath)))
        {
            Log("[错误] 请先选择有效的图片文件夹");
            return;
        }

        if (IsPreviewing)
            StopPreview(); // 释放设备给检测用

        SaveConfig(); // 以界面当前参数为准落盘
        ResetRunState();
        IsRunning = true;
        Log($"开始检测: 输入源 {TranslateSource(config.InputSource)}, 公差 ±{inspection.OffsetToleranceMm}mm");

        _cts = new CancellationTokenSource();
        _runStartedUtc = DateTime.UtcNow;
        _stopwatch.Restart();
        _timer.Start();

        MachineStateMachine stateMachine = new();
        stateMachine.StateChanged += (from, to) =>
            _ui.Post(_ => MachineStateText = TranslateState(to), null);

        try
        {
            IReadOnlyList<WeldPoint> points;
            var calibration = DemoPlanBuilder.BuildCalibration(inspection);

            switch (config.InputSource)
            {
                case InputSource.VideoFile:
                {
                    var grabber = new VideoFileGrabber(config.VideoFilePath,
                        inspection.ImageWidth, inspection.ImageHeight);
                    _camera = grabber;
                    _plc = new NullPlcClient();
                    var videoName = Path.GetFileName(config.VideoFilePath);
                    var frameCount = grabber.FrameCount > 0
                        ? Math.Min(grabber.FrameCount, MaxVideoFrames)
                        : config.PointCount;
                    points = BuildSequentialPoints(frameCount, "Frame", i => $"{videoName} #{i}");
                    if (grabber.FrameCount > MaxVideoFrames)
                        Log($"[输入源] 视频共 {grabber.FrameCount} 帧, 超过上限 {MaxVideoFrames}, 仅处理前 {MaxVideoFrames} 帧");
                    if (grabber.FrameCount <= 0)
                        Log($"[输入源] 视频帧数未知, 按 {frameCount} 帧尝试读取");
                    Log($"[输入源] 视频模式: {Path.GetFileName(config.VideoFilePath)} → {points.Count} 点");
                    break;
                }

                case InputSource.ImageFolder:
                {
                    var grabber = new ImageFolderGrabber(config.ImageFolderPath,
                        inspection.ImageWidth, inspection.ImageHeight);
                    _camera = grabber;
                    _plc = new NullPlcClient();
                    var imageCount = Math.Min(grabber.ImageCount, MaxVideoFrames);
                    // 点位带上源文件名：报表/提示/标注图都要能把异常点对应回具体文件
                    points = BuildSequentialPoints(imageCount, "Img",
                        i => Path.GetFileName(grabber.Files[i]));
                    if (grabber.ImageCount > MaxVideoFrames)
                        Log($"[输入源] 文件夹共 {grabber.ImageCount} 张图片, 超过上限 {MaxVideoFrames}, 仅处理前 {MaxVideoFrames} 张（按文件名自然顺序）");
                    Log($"[输入源] 图片文件夹模式: {Path.GetFileName(config.ImageFolderPath)} → {points.Count} 点");
                    break;
                }

                case InputSource.Camera:
                {
                    var grabber = new CameraGrabber(config.CameraDeviceIndex,
                        inspection.ImageWidth, inspection.ImageHeight);
                    _camera = grabber;
                    _plc = new NullPlcClient();
                    points = BuildSequentialPoints(config.PointCount, "Cam");
                    Log($"[输入源] 摄像头模式: {grabber.Name} → {points.Count} 点");
                    break;
                }

                default:
                {
                    var (planPoints, scenarios) = DemoPlanBuilder.BuildPlan(config.PointCount, config.RandomSeed, inspection);
                    points = planPoints;
                    var simulator = new PlcSimulator(port: 0, moveTimeMs: inspection.PlcMoveTimeMs);
                    simulator.Log += msg => Log(msg);
                    simulator.Start();
                    _simulator = simulator;
                    _plc = new TcpPlcClient("127.0.0.1", simulator.Port,
                        inspection.ArrivalPollIntervalMs, inspection.ArrivalTimeoutMs, inspection.PlcResponseTimeoutMs);
                    _camera = new SimulatedCamera(
                        new ImageGenerator(inspection, calibration), scenarios, baseSeed: config.RandomSeed);
                    Log($"[输入源] 仿真模式: {points.Count} 点, 种子 {config.RandomSeed}");
                    break;
                }
            }

            RebuildPoints(points);

            // ---- 视觉层：算法 + 流水线 ----
            var algorithm = new WeldInspectionAlgorithm(inspection, calibration);
            _pipeline = new ImagePipeline(algorithm, workerCount: config.WorkerCount);
            _pipeline.ResultProcessed += OnResultProcessed;

            var controller = new InspectionController(_plc, _camera, _pipeline, stateMachine, points);
            controller.Log += msg => Log(msg);
            controller.PointGrabbed += point =>
                _ui.Post(_ =>
                {
                    var item = Points.FirstOrDefault(p => p.Index == point.Index);
                    if (item is { Status: PointStatus.Pending })
                        item.Status = PointStatus.Inspecting;
                }, null);

            // 后台线程执行检测编排，UI 保持响应
            var report = await Task.Run(() => controller.RunAsync(_cts.Token), _cts.Token);

            _stopwatch.Stop();
            CycleTimeText = report.TotalCount > 0
                ? $"{report.CycleTimeSec:F2}s ({report.CycleTimeSec * 1000 / report.TotalCount:F0}ms/点)"
                : "--";
            Log(report.WasCancelled
                ? $"检测被中止: 已检 {report.TotalCount}, OK {report.OkCount}, NG {report.NgCount}, 异常 {report.ErrorCount}"
                : $"检测完成: OK {report.OkCount}, NG {report.NgCount}, 异常 {report.ErrorCount}, 节拍 {CycleTimeText}");
            if (report.UploadFailedCount > 0)
                Log($"[警告] {report.UploadFailedCount} 个点位结果上传 PLC 失败");

            var path = WriteReport(report.WasCancelled ? "stopped" : "finished");
            if (path is not null)
                Log($"报表已导出: {path}");
        }
        catch (OperationCanceledException)
        {
            _stopwatch.Stop();
            Log("检测已取消");
        }
        catch (Exception ex)
        {
            _stopwatch.Stop();
            Log($"[异常] 检测中止: {ex.Message}");
            var path = WriteReport("aborted");
            if (path is not null)
                Log($"部分结果报表已导出: {path}");
        }
        finally
        {
            _timer.Stop();
            IsRunning = false;
            CleanupHardware();
        }
    }

    private void Stop()
    {
        if (!IsRunning)
            return;
        Log("操作员请求停止, 等待安全停机...");
        _cts?.Cancel();
    }

    private void Reset()
    {
        if (IsRunning)
            return;
        if (IsPreviewing)
            StopPreview();
        ResetRunState();
        CurrentImage = null;
        ImageCaption = string.Empty;
        MachineStateText = "待机";
        CycleTimeText = "--";
        Log("已复位");
    }

    private void SaveConfig()
    {
        try
        {
            Config.Save(_configPath);
            Log($"配置已保存: {_configPath}");
        }
        catch (Exception ex)
        {
            Log($"[警告] 配置保存失败: {ex.Message}");
        }
    }

    private void ExportReport()
    {
        var path = WriteReport("manual");
        if (path is not null)
            Log($"报表已导出: {path}");
        else
            Log("暂无检测结果可导出");
    }

    // ---- 事件处理 ----

    /// <summary>流水线 worker 线程回调：先复制图像数据，再封送到 UI 线程。</summary>
    private void OnResultProcessed(PointInspectionResult result, Mat annotated)
    {
        BitmapSource? bitmap = null;
        string? imagePath = null;
        try
        {
            bitmap = MatConverter.ToBitmapSource(annotated);
            if (Config.SaveAnnotatedImages)
            {
                var imageDir = Path.Combine(OutputDir, "images");
                Directory.CreateDirectory(imageDir);
                imagePath = Path.Combine(imageDir,
                    $"point_{result.Point.Index:D3}_{(result.IsOk ? "ok" : "ng")}.png");
                Cv2.ImWrite(imagePath, annotated);
            }
        }
        catch (Exception ex)
        {
            Log($"[视觉] 保存标注图失败({result.Point}): {ex.Message}");
        }

        lock (_resultsLock)
        {
            _results.Add(result);
        }

        var snapshot = bitmap;
        var savedPath = imagePath;
        _ui.Post(_ => ApplyResult(result, snapshot, savedPath), null);
    }

    private void ApplyResult(PointInspectionResult result, BitmapSource? bitmap, string? imagePath)
    {
        ProcessedCount++;

        if (result.HasError)
        {
            ErrorCount++;
            Log($"[视觉] {result.Point} 处理异常: {result.ErrorMessage}");
        }
        else if (result.IsOk)
        {
            OkCount++;
            Log($"[视觉] {result}");
        }
        else
        {
            NgCount++;
            Log($"[视觉] {result}");
        }

        var item = Points.FirstOrDefault(p => p.Index == result.Point.Index);
        if (item is not null)
        {
            item.Status = result.HasError ? PointStatus.Error : result.IsOk ? PointStatus.Ok : PointStatus.Ng;
            item.SetResult(result, imagePath);
        }

        if (bitmap is not null)
        {
            CurrentImage = bitmap;
            // 偏移量无物理意义时（测量失败 / 未检出焊环）不报 0.000mm，改报"不适用"，
            // 否则最该人工复核的点位反而显示成"焊得很正"
            var offset = result.HasValidOffset ? $"  D={result.OffsetDistanceMm:F3}mm" : string.Empty;
            ImageCaption = $"{result.Point.Name} — {result.DefectTextZh}{offset}";
        }

        OnPropertyChanged(nameof(ProgressPercent));
    }

    // ---- 辅助 ----

    private static IReadOnlyList<WeldPoint> BuildSequentialPoints(int count, string prefix,
        Func<int, string?>? sourceName = null) =>
        Enumerable.Range(0, count)
            .Select(i => new WeldPoint
            {
                Index = i,
                Name = $"{prefix}{i:D3}",
                SourceName = sourceName?.Invoke(i)
            })
            .ToList();

    private void RebuildPoints()
    {
        var points = Config.InputSource is InputSource.VideoFile or InputSource.ImageFolder
            ? null
            : BuildSequentialPoints(Config.PointCount, "Cam");
        if (points is null)
        {
            // 视频/图片文件夹模式：数量在打开文件后确定，先按占位显示
            var plan = DemoPlanBuilder.BuildPlan(Config.PointCount, Config.RandomSeed, Config.Inspection);
            RebuildPoints(plan.Points);
        }
        else
        {
            RebuildPoints(points);
        }
    }

    private void RebuildPoints(IReadOnlyList<WeldPoint> points)
    {
        Points.Clear();
        foreach (var p in points)
            Points.Add(new PointItemViewModel(p.Index, p.Name));
        OnPropertyChanged(nameof(TotalCount));
        OnPropertyChanged(nameof(ProgressPercent));
    }

    private void ResetRunState()
    {
        lock (_resultsLock)
        {
            _results.Clear();
        }
        ProcessedCount = OkCount = NgCount = ErrorCount = 0;
        _stopwatch.Reset();
        ElapsedText = "00:00.0";
        RebuildPoints();
        OnPropertyChanged(nameof(ProgressPercent));
    }

    private string? WriteReport(string suffix)
    {
        List<PointInspectionResult> snapshot;
        lock (_resultsLock)
        {
            snapshot = _results.OrderBy(r => r.Point.Index).ToList();
        }
        if (snapshot.Count == 0)
            return null;

        var report = new InspectionReport
        {
            StartedAtUtc = _runStartedUtc,
            FinishedAtUtc = DateTime.UtcNow,
            Results = snapshot
        };
        var path = Path.Combine(OutputDir, $"report_{DateTime.Now:yyyyMMdd_HHmmss}_{suffix}.csv");
        CsvReportWriter.Write(report, path);
        return path;
    }

    private void CleanupHardware()
    {
        _pipeline?.Dispose();
        _pipeline = null;
        _camera?.Dispose();
        _camera = null;
        _plc?.Dispose();
        _plc = null;
        _simulator?.Dispose();
        _simulator = null;
        _cts?.Dispose();
        _cts = null;
    }

    /// <summary>窗口关闭时调用：取消运行中的检测并释放资源。</summary>
    public void Shutdown()
    {
        try
        {
            _cts?.Cancel();
        }
        catch (ObjectDisposedException)
        {
        }
        StopPreview();
        CleanupHardware();
    }

    private void Log(string msg)
    {
        var line = $"{DateTime.Now:HH:mm:ss.fff}  {msg}";
        _ui.Post(_ =>
        {
            Logs.Add(line);
            while (Logs.Count > MaxLogLines)
                Logs.RemoveAt(0);
        }, null);
    }

    private static string TranslateState(MachineState state) => state switch
    {
        MachineState.Idle => "待机",
        MachineState.Moving => "轴运动中",
        MachineState.Inspecting => "检测中",
        MachineState.Finished => "整包完成",
        MachineState.Error => "错误",
        MachineState.Stopped => "已停机",
        _ => state.ToString()
    };

    private static string TranslateSource(InputSource source) => source switch
    {
        InputSource.Simulation => "仿真演示",
        InputSource.VideoFile => "视频文件",
        InputSource.ImageFolder => "图片文件夹",
        InputSource.Camera => "摄像头",
        _ => source.ToString()
    };
}
