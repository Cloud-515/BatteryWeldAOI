using BatteryWeldAOI.Core.Vision;
using OpenCvSharp;
using Xunit;

namespace BatteryWeldAOI.Core.Tests;

public class NinePointCalibrationTests
{
    [Fact]
    public void PureScale_Is_Recovered_Exactly()
    {
        // 9 点网格，纯缩放 0.025 mm/px
        var pairs = BuildGrid((x, y) => new Point2d(x * 0.025, y * 0.025));
        var calib = new NinePointCalibration(pairs);

        var mm = calib.Transform(new Point2d(1000, 500));
        Assert.Equal(25.0, mm.X, 9);
        Assert.Equal(12.5, mm.Y, 9);
    }

    [Fact]
    public void RotationAndScale_Is_Recovered()
    {
        const double scale = 0.025;
        const double theta = 1.5 * Math.PI / 180; // 1.5° 安装偏角
        var cos = Math.Cos(theta);
        var sin = Math.Sin(theta);

        var pairs = BuildGrid((x, y) => new Point2d(
            scale * (cos * x - sin * y),
            scale * (sin * x + cos * y)));
        var calib = new NinePointCalibration(pairs);

        var probe = new Point2d(873, 291);
        var mm = calib.Transform(probe);
        var expected = new Point2d(
            scale * (cos * probe.X - sin * probe.Y),
            scale * (sin * probe.X + cos * probe.Y));
        Assert.Equal(expected.X, mm.X, 6);
        Assert.Equal(expected.Y, mm.Y, 6);
    }

    [Fact]
    public void Inverse_RoundTrips()
    {
        const double scale = 0.03;
        const double theta = -0.8 * Math.PI / 180;
        var cos = Math.Cos(theta);
        var sin = Math.Sin(theta);
        var pairs = BuildGrid((x, y) => new Point2d(
            scale * (cos * x - sin * y) + 5.0,
            scale * (sin * x + cos * y) - 3.0));

        var calib = new NinePointCalibration(pairs);
        var inv = calib.Invert();

        var probe = new Point2d(512, 384);
        var roundTrip = inv.Transform(calib.Transform(probe));
        Assert.Equal(probe.X, roundTrip.X, 6);
        Assert.Equal(probe.Y, roundTrip.Y, 6);
    }

    [Fact]
    public void Collinear_Samples_Throw()
    {
        var pairs = new List<(Point2d, Point2d)>
        {
            (new Point2d(0, 0), new Point2d(0, 0)),
            (new Point2d(100, 100), new Point2d(2.5, 2.5)),
            (new Point2d(200, 200), new Point2d(5, 5)),
        };
        Assert.ThrowsAny<Exception>(() => new NinePointCalibration(pairs));
    }

    private static List<(Point2d Pixel, Point2d Mm)> BuildGrid(Func<double, double, Point2d> toMm)
    {
        var pairs = new List<(Point2d, Point2d)>();
        foreach (var y in new[] { 100.0, 500.0, 900.0 })
            foreach (var x in new[] { 100.0, 600.0, 1100.0 })
                pairs.Add((new Point2d(x, y), toMm(x, y)));
        return pairs;
    }
}
