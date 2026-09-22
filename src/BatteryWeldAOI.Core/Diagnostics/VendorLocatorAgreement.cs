namespace BatteryWeldAOI.Core.Diagnostics;

using BatteryWeldAOI.Core.Models;

/// <summary>本项目焊环定位与厂商定位的一致性测量结果（都在原图像素坐标系里）。</summary>
/// <param name="OurCenterX">本项目焊环中心 X（原图坐标）。</param>
/// <param name="OurCenterY">本项目焊环中心 Y（原图坐标）。</param>
/// <param name="OurRadiusPx">本项目焊环半径（原图像素）。</param>
/// <param name="DeltaPx">与厂商定位圆心的距离（像素）。</param>
public readonly record struct LocatorAgreement(
    double OurCenterX, double OurCenterY, double OurRadiusPx, double DeltaPx);

/// <summary>
/// 本项目焊环定位 vs 厂商 VDB 定位的横向一致性比较。
///
/// 换算一律走 <see cref="PointInspectionResult"/> 的原图坐标属性
/// （<see cref="PointInspectionResult.WeldCenterSourcePx"/> 等），不再自己乘缩放比：
/// 算法在输入图短边超过 <see cref="InspectionConfig.MaxWorkingBase"/> 时会先等比缩小再检测，
/// 因此 <see cref="PointInspectionResult.WeldCenterPx"/> 是**工作分辨率**坐标，
/// 而厂商的圆定位结果是**原图**坐标，直接相减会差出一整个缩放比
/// （20MP 现场图上约 8 倍），看上去像"偏差几百像素"的严重问题，实际是没换算。
/// 这个换算只应有一处实现，否则工具、测试与界面迟早会各写一份并写歪。
///
/// 未定位到焊环（<see cref="PointInspectionResult.WeldRadiusPx"/> 为 0）时返回 null：
/// 此时圆心没有意义（是默认值 0,0），把它当成一个"偏差 3000px 的坏样本"会污染统计——
/// 这类帧属于"定位失败"，应当单独计数，而不是混进偏差分布。
/// </summary>
public static class VendorLocatorAgreement
{
    public static LocatorAgreement? Compare(PointInspectionResult result, VendorVdbRecord vendor)
    {
        if (result.WeldRadiusPx <= 0)
            return null;

        var center = result.WeldCenterSourcePx;
        var dx = center.X - vendor.LocateCenterX;
        var dy = center.Y - vendor.LocateCenterY;

        return new LocatorAgreement(
            center.X, center.Y, result.WeldRadiusSourcePx, Math.Sqrt(dx * dx + dy * dy));
    }
}
