using BatteryWeldAOI.Core.Configuration;
using BatteryWeldAOI.Core.Models;
using Xunit;

namespace BatteryWeldAOI.Core.Tests;

/// <summary>JSON 配置（配方）加载与保存测试。</summary>
public class AppConfigTests : IDisposable
{
    private readonly string _path = Path.Combine(Path.GetTempPath(),
        $"bwaoi_config_{Guid.NewGuid():N}.json");

    [Fact]
    public void SaveThenLoad_RoundTripsAllFields()
    {
        var config = new AppConfig
        {
            PointCount = 96,
            RandomSeed = 7,
            WorkerCount = 4,
            SaveAnnotatedImages = false,
            OutputDirectory = @"D:\aoi_out",
            Inspection = new InspectionConfig
            {
                OffsetToleranceMm = 0.8,
                MmPerPixel = 0.031,
                PlcMoveTimeMs = 120,
                ArrivalTimeoutMs = 3_000,
                PlcResponseTimeoutMs = 1_500,
                BlowoutRingRadiusRatio = 0.28,
                PinCaliperRays = 240,
                MaxSpatterCount = 25
            }
        };
        config.Save(_path);

        var loaded = AppConfig.LoadOrDefault(_path);
        Assert.Equal(96, loaded.PointCount);
        Assert.Equal(7, loaded.RandomSeed);
        Assert.Equal(4, loaded.WorkerCount);
        Assert.False(loaded.SaveAnnotatedImages);
        Assert.Equal(@"D:\aoi_out", loaded.OutputDirectory);
        Assert.Equal(0.8, loaded.Inspection.OffsetToleranceMm);
        Assert.Equal(0.031, loaded.Inspection.MmPerPixel);
        Assert.Equal(120, loaded.Inspection.PlcMoveTimeMs);
        Assert.Equal(3_000, loaded.Inspection.ArrivalTimeoutMs);
        Assert.Equal(1_500, loaded.Inspection.PlcResponseTimeoutMs);
        Assert.Equal(0.28, loaded.Inspection.BlowoutRingRadiusRatio);
        Assert.Equal(240, loaded.Inspection.PinCaliperRays);
        Assert.Equal(25, loaded.Inspection.MaxSpatterCount);
    }

    [Fact]
    public void MissingFile_ReturnsDefaults()
    {
        var config = AppConfig.LoadOrDefault(Path.Combine(Path.GetTempPath(), "definitely_missing.json"));
        Assert.Equal(24, config.PointCount);
        Assert.Equal(42, config.RandomSeed);
        // 默认公差 6.6mm 由现场 480 帧厂商良品推出（见 InspectionConfig.OffsetToleranceMm），
        // 并可在界面上改；这里钉的是"缺配置时拿到的是当前默认值"，不是那个数值本身的意义。
        Assert.Equal(6.6, config.Inspection.OffsetToleranceMm);
    }

    [Fact]
    public void CorruptFile_ReturnsDefaults_InsteadOfThrowing()
    {
        File.WriteAllText(_path, "{ not valid json !!!");
        var config = AppConfig.LoadOrDefault(_path);
        Assert.Equal(24, config.PointCount); // 默认值
    }

    /// <summary>
    /// 反射遍历 <see cref="InspectionConfig"/> 的每个可写属性，逐个改成非默认值后
    /// Save→Load，断言全部原样返回。
    ///
    /// 这一条盯的是一类**静默失效**：属性改名或加了 [JsonIgnore] 后，序列化器会跳过它，
    /// 而 System.Text.Json 默认**忽略 JSON 里的未知字段**——于是配置文件里写着
    /// `PoleMinRadiusPx: 140`，程序却当作没看见，操作员改了参数毫无效果且无任何报错。
    /// 本项目真实发生过：旧版 17 个 Inspection 键里 13 个已不存在，
    /// 而现场的调参建议正是照着这些死键给的。
    /// </summary>
    [Fact]
    public void EveryInspectionProperty_SurvivesSaveLoad()
    {
        var properties = typeof(InspectionConfig)
            .GetProperties(System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.Instance)
            .Where(p => p is { CanRead: true, CanWrite: true })
            .ToList();
        Assert.True(properties.Count > 20, "属性数量异常偏少，反射可能没拿到预期类型");

        var config = new AppConfig();
        var expected = new Dictionary<string, object?>();
        foreach (var p in properties)
        {
            var value = NonDefaultValue(p.PropertyType, p.GetValue(config.Inspection));
            p.SetValue(config.Inspection, value);
            expected[p.Name] = value;
        }
        config.Save(_path);

        var loaded = AppConfig.LoadOrDefault(_path).Inspection;
        var broken = properties
            .Where(p => !ValuesEqual(p.GetValue(loaded), expected[p.Name]))
            .Select(p => $"{p.Name}: 写入 {Describe(expected[p.Name])} → 读回 {Describe(p.GetValue(loaded))}")
            .ToList();

        Assert.True(broken.Count == 0,
            "以下视觉参数无法经 JSON 往返（改名/加了 JsonIgnore？操作员将改了不生效且无报错）:\n  "
            + string.Join("\n  ", broken));
    }

    /// <summary>
    /// 值等价比较。数组必须按**序列**比——用 <see cref="object.Equals(object?, object?)"/>
    /// 比数组是引用相等，JSON 往返后必然不等，会把"往返其实成功"误报成失败。
    /// </summary>
    private static bool ValuesEqual(object? a, object? b) => (a, b) switch
    {
        (double[] x, double[] y) => x.SequenceEqual(y),
        _ => Equals(a, b),
    };

    private static string Describe(object? value) =>
        value is double[] arr ? "[" + string.Join(", ", arr) + "]" : value?.ToString() ?? "null";

    /// <summary>
    /// 构造一个与当前值不同的合法值，避免与默认值相等导致断言失去意义。
    ///
    /// 这个 switch 是**故意抛异常**的：InspectionConfig 一旦新增了这里没覆盖的类型，
    /// 本测试立刻失败并提示补构造，而不是静默地"往返测试通过但那个属性没被真正验证"。
    /// （2026-09-16 新增 double[] 时即按此提示补充。）
    /// </summary>
    private static object NonDefaultValue(Type type, object? current) => type switch
    {
        _ when type == typeof(int) => (int)current! + 7,
        _ when type == typeof(double) => Math.Round((double)current! + 0.037, 4),
        _ when type == typeof(bool) => !(bool)current!,
        _ when type == typeof(string) => "x" + Guid.NewGuid().ToString("N")[..6],
        // 数组：整体缩放后取整，保证每个元素都变且仍合法（倍数类核宽参数要求正值）
        _ when type == typeof(double[]) => ((double[])current!)
            .Select(v => Math.Round(v * 1.5 + 0.013, 4)).ToArray(),
        _ => throw new NotSupportedException(
            $"InspectionConfig 新增了 {type.Name} 类型的属性，请在此补充测试值构造")
    };

    public void Dispose()
    {
        try { File.Delete(_path); } catch (IOException) { }
    }
}
