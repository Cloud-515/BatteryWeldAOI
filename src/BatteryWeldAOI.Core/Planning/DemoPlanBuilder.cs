namespace BatteryWeldAOI.Core.Planning;

using BatteryWeldAOI.Core.Models;
using BatteryWeldAOI.Core.Vision;
using OpenCvSharp;

/// <summary>
/// 演示用的点位规划与标定构建器。
/// 真实设备中点位表来自电池包型号配方、九点标定由标定板流程采样生成，
/// 此处以确定性算法生成，供控制台程序与 WPF 界面共用。
/// </summary>
public static class DemoPlanBuilder
{
    /// <summary>
    /// 生成点位规划与模拟剧本（固定缺陷位 3=焊偏 / 7=漏焊 / 11=炸焊 / 15=飞溅，各命中一次）。
    ///
    /// **焊偏点的真值距离由公差推出（公差 × 1.3），不再写死 1.2~1.7mm**（2026-09-17 改）：
    /// 写死时它与 <see cref="InspectionConfig.OffsetToleranceMm"/> 脱钩——公差一改，这个"超差点"
    /// 要么不再超差（演示里焊偏消失）、要么相对夹具量程离谱（演示里量程外压缩，看着像算法漏检）。
    /// 只有 ≥1.2mm 的下限保留，是为了在公差很小时仍能演示出"偏移量明显但判定合格"的正常点对照。
    /// </summary>
    /// <param name="config">检测配置：公差决定焊偏点的真值，像素当量与几何常数决定夹具量程上限。</param>
    public static (IReadOnlyList<WeldPoint> Points, IReadOnlyList<PointScenario> Scenarios) BuildPlan(
        int count, int seed, InspectionConfig config)
    {
        var rng = new Random(seed);
        var defectSchedule = new Dictionary<int, WeldDefect>
        {
            [3] = WeldDefect.Misaligned,
            [7] = WeldDefect.MissingWeld,
            [11] = WeldDefect.Blowout,
            [15] = WeldDefect.Spatter
        };

        // 合成夹具能可靠复现的偏移上限（mm）。夹具里"孔位搜索区"只有 0.45×焊环半径
        // （InspectionConfig.HoleSearchOuterRatio），孔心再往外偏移就会跑到搜索区之外，
        // 孔位定位退化、测出的偏移量被压缩——那会让演示看起来像"算法漏检"，
        // 实际是夹具几何撑不住这么大的偏移。真机上不受此限（孔心偏移是待测量、不是夹具参数）。
        var fixtureBasePx = Math.Min(config.ImageWidth, config.ImageHeight);
        var fixtureRingPx = fixtureBasePx * WeldSceneRenderer.NominalRingRadiusRatio;
        var fixtureHolePx = fixtureRingPx * WeldSceneRenderer.PinRadiusRatio;
        var fixtureMaxOffsetMm = Math.Max(0.1,
            (config.HoleSearchOuterRatio * fixtureRingPx - fixtureHolePx) * config.MmPerPixel);

        var misalignedMm = Math.Min(Math.Max(1.2, config.OffsetToleranceMm * 1.3), fixtureMaxOffsetMm);

        var points = new List<WeldPoint>(count);
        var scenarios = new List<PointScenario>(count);

        for (var i = 0; i < count; i++)
        {
            points.Add(new WeldPoint
            {
                Index = i,
                Name = $"Cell{i / 4 + 1:D2}-{(i % 4) switch { 0 => "POS", 1 => "NEG", _ => "BUS" }}",
                StageX = 90.0 * (i % 8),
                StageY = 45.0 * (i / 8)
            });

            // 注意：不能用 GetValueOrDefault —— 未命中 key 会返回 default(WeldDefect)=Misaligned(0)
            var defect = defectSchedule.TryGetValue(i, out var d) ? d : (WeldDefect?)null;
            double dx, dy;
            if (defect == WeldDefect.Misaligned)
            {
                var angle = rng.NextDouble() * 2 * Math.PI;
                var dist = misalignedMm;      // 由公差推出并夹到夹具量程，见方法说明
                dx = Math.Cos(angle) * dist;
                dy = Math.Sin(angle) * dist;
            }
            else
            {
                var angle = rng.NextDouble() * 2 * Math.PI;
                var dist = rng.NextDouble() * 0.45; // 合格范围内的正常偏移
                dx = Math.Cos(angle) * dist;
                dy = Math.Sin(angle) * dist;
            }

            scenarios.Add(new PointScenario
            {
                Point = points[i],
                Defect = defect,
                TrueOffsetXmm = dx,
                TrueOffsetYmm = dy
            });
        }

        return (points, scenarios);
    }

    /// <summary>
    /// 构建九点标定：真实设备中由"运动平台带标定板走九点"生成样本，
    /// 这里用已知的地面真值仿射（缩放 mm/px + 0.6° 安装偏角）生成样本对。
    /// </summary>
    public static NinePointCalibration BuildCalibration(InspectionConfig config)
    {
        var scale = config.MmPerPixel;
        var theta = 0.6 * Math.PI / 180; // 相机安装偏角 ~0.6°
        var cos = Math.Cos(theta);
        var sin = Math.Sin(theta);

        var pairs = new List<(Point2d Pixel, Point2d Mm)>();
        foreach (var py in new[] { 256, 512, 768 })
            foreach (var px in new[] { 320, 640, 960 })
                pairs.Add((new Point2d(px, py),
                    new Point2d(
                        scale * (cos * px - sin * py),
                        scale * (sin * px + cos * py))));

        return new NinePointCalibration(pairs);
    }
}
