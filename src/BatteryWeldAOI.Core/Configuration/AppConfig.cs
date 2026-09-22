namespace BatteryWeldAOI.Core.Configuration;

using System.Text.Json;
using System.Text.Json.Serialization;
using BatteryWeldAOI.Core.Models;

/// <summary>
/// 应用级配置（配方）：点位规划、流水线并发、输出与检测参数。
/// 从 JSON 文件加载（不存在时使用默认值），界面/控制台共用。
/// </summary>
public sealed class AppConfig
{
    private static readonly JsonSerializerOptions JsonOptions = new()
    {
        WriteIndented = true,
        DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull
    };

    /// <summary>
    /// 配置文件结构版本。**改动 <see cref="InspectionConfig"/> 的默认值时必须 +1。**
    ///
    /// 为什么需要它（这是一次真实事故的修复）：<see cref="Save"/> 会把**整份**
    /// <see cref="InspectionConfig"/> 写进 JSON，而 System.Text.Json 反序列化时"JSON 里有的键
    /// 一律覆盖、没有的键保持默认"。于是程序第一次运行就会把当时的所有视觉参数冻结进文件，
    /// 此后**改代码里的默认值对这个文件完全无效**——现场就发生过：
    /// `MinRingArcSupport` 在代码里由 0.50 提到 0.82（那一轮修复的核心改动），
    /// 而现场 `config.json` 里仍写着 0.50，程序照旧执行 0.50，
    /// 用户看到的现象是"改了代码却一点没变"，排查方向被彻底带偏（单元测试全绿，
    /// 因为它们用的是 `new InspectionConfig()` 而不是现场那份 JSON）。
    ///
    /// 现在的口径：加载时若文件版本 &lt; 本值，则**丢弃文件里的 Inspection 整块**、
    /// 改用当前代码默认值（点位数、路径等操作员字段仍然保留），并在下次保存时写入新版本。
    /// 这样"默认值变更"与"操作员调参"两件事不再互相掩盖。
    /// </summary>
    public int ConfigVersion { get; set; }

    /// <summary>
    /// 当前配置结构版本，见 <see cref="ConfigVersion"/>。
    ///
    /// 历史：1 = 引入版本闸门那一版；**2 = 焊环精修的射线原点由"孔心"改为"熔核自身圆心"**
    /// （`WeldRingRefine*` 一组参数整体重写：删掉了 `MaxSeedRatio` 与四个 `RatioOfBlob/Family`，
    /// 新增 `ProfileBandRatio`/`Iterations`/`ConvergePx`/`MaxRadiusCorrection`，并改了
    /// `BandRatio` 与 `InlierTolerance` 的默认值）；
    /// **3 = 卡尺兜底结果的有效性判据**（新增 `CaliperMaxResidualRatio` 与
    /// `CaliperMaxRadiusRatioOfFamily`；`HoleLocator` 的分层判据同时改为"两种分法 × 两种形态学
    /// 出候选、打分择优"——冲白工况下原本会返回 null 而静默退到卡尺，详见
    /// `Vision.HoleLocator` 与 `REPORT_hole_near_ring.md`）；
    /// **4 = 焊环精修的边界粗糙度闸门**（新增 `WeldRingRefineMinRoughnessRatio`，
    /// 修"黑环（与孔同轴的机加工阴影边）把孔锚起点吸走"造成的圆心偏移，见 FIXPLAN 第 9 节）；
    /// **5 = 焊环精修的 ρ 一致性闸门**（新增 `WeldRingRefineMinRhoRatio`，量"两个起点看到的是不是
    /// 同一条边界"——离线判别研究显示它是唯一能分开"该选哪个起点"的量，见 FIXPLAN 第 10 节；
    /// **默认 0 = 关闭**：它唯一变差的那帧有独立物理证据说明它判错了）；
    /// **6 = 第五路径（多起点重试）**（新增 `MultiSeedRetryStepRatio` / `MultiSeedRetryRings`，
    /// 修"熔核外沿可测、但掩码算出的候选圆心偏了"造成的"测不出来"，见 FIXPLAN 第 11 节）。
    /// </summary>
    public const int CurrentConfigVersion = 6;

    /// <summary>图像输入源：仿真 / 视频文件 / 摄像头。</summary>
    public InputSource InputSource { get; set; } = InputSource.Simulation;

    /// <summary>视频文件路径（InputSource=VideoFile 时有效）。</summary>
    public string VideoFilePath { get; set; } = string.Empty;

    /// <summary>图片文件夹路径（InputSource=ImageFolder 时有效，文件夹内图片按文件名自然顺序逐张检测）。</summary>
    public string ImageFolderPath { get; set; } = string.Empty;

    /// <summary>摄像头设备索引（InputSource=Camera 时有效，0 = 第一台）。</summary>
    public int CameraDeviceIndex { get; set; } = 0;

    /// <summary>检测的点位数（仿真与摄像头模式使用；视频模式按视频帧数）。</summary>
    public int PointCount { get; set; } = 24;

    /// <summary>随机种子（决定模拟图像与缺陷剧本，同种子可复现）。</summary>
    public int RandomSeed { get; set; } = 42;

    /// <summary>视觉流水线并行 worker 数。</summary>
    public int WorkerCount { get; set; } = 2;

    /// <summary>是否保存逐点标注图 PNG。</summary>
    public bool SaveAnnotatedImages { get; set; } = true;

    /// <summary>报表与图像输出目录（相对路径基于程序目录）。</summary>
    public string OutputDirectory { get; set; } = "output";

    /// <summary>检测算法与设备参数。</summary>
    public InspectionConfig Inspection { get; set; } = new();

    /// <summary>
    /// 加载配置；文件不存在或损坏时返回默认配置（不抛异常，保证设备可启动）。
    ///
    /// **版本闸门**：文件里的 <see cref="ConfigVersion"/> 低于当前值时，视觉参数一律取代码默认值，
    /// 不取文件里的历史值——否则"代码默认值改了但程序行为不变"这类静默失效会再次发生
    /// （详见 <see cref="ConfigVersion"/>）。操作员字段（路径、点位数、并发数……）不受影响。
    /// </summary>
    public static AppConfig LoadOrDefault(string path)
    {
        try
        {
            if (!File.Exists(path))
                return Fresh();

            var config = JsonSerializer.Deserialize<AppConfig>(File.ReadAllText(path), JsonOptions);
            if (config is null)
                return Fresh();

            if (config.ConfigVersion < CurrentConfigVersion)
            {
                // 旧版文件：保留它记住的输入源/路径等操作员设置，视觉参数回到代码默认值。
                config.Inspection = new InspectionConfig();
                config.ConfigVersion = CurrentConfigVersion;
            }
            return config;
        }
        catch (Exception ex) when (ex is IOException or JsonException or UnauthorizedAccessException)
        {
            return Fresh();
        }
    }

    /// <summary>新建一份"当前版本"的默认配置（保存时会把版本写进文件）。</summary>
    private static AppConfig Fresh() => new() { ConfigVersion = CurrentConfigVersion };

    public void Save(string path)
    {
        ConfigVersion = CurrentConfigVersion;
        var dir = Path.GetDirectoryName(Path.GetFullPath(path));
        if (!string.IsNullOrEmpty(dir))
            Directory.CreateDirectory(dir);
        File.WriteAllText(path, JsonSerializer.Serialize(this, JsonOptions));
    }
}
