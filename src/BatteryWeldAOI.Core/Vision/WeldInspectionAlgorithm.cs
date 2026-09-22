namespace BatteryWeldAOI.Core.Vision;

using System.Diagnostics;
using BatteryWeldAOI.Core.Models;
using OpenCvSharp;

/// <summary>
/// 焊点检测算法：判定「焊环圆心是否与孔洞同心」，并识别漏焊 / 炸焊 / 焊偏 / 飞溅。
///
/// 检测对象（现场成像）：一块光滑金属端面上有一个孔洞（极柱），孔洞周围是激光焊形成的
/// 粗糙环形熔核（焊环）。焊接合格时熔核与孔洞同心；焊偏时熔核整体偏离孔洞。
///
/// 四步：
///   1. 找焊环——纹理能量分割 + 圆度筛选 + 径向中位数圆拟合（<see cref="WeldRingLocator"/>）；
///   2. 找孔洞——逐射线自适应梯度卡尺 + 正弦拟合（<see cref="PinLocator"/>），
///      正弦拟合的一次项系数 (b, c) 直接给出孔心相对焊环中心的偏移向量；
///   3. 量质量——环弧覆盖率、填充率、环内暗区、环外飞溅（<see cref="WeldMetrics"/>）；
///   4. 判缺陷——漏焊 / 炸焊 / 焊偏 / 飞溅，并把像素偏移经九点标定换算为毫米。
///
/// 设计要点：**所有参数都是相对量**（以 base = min(高,宽) 或焊环半径为分母），
/// 不含任何绝对像素阈值或绝对灰度阈值。旧版算法把"极柱半径 140~230px""焊缝最小对比度 25 灰阶"
/// 这类常数写死，只在合成图上成立，一上真机 28 个点位全部误判 NG——这是本次重写的直接原因。
/// 相对量参数对分辨率、镜头、曝光、增益都不敏感，换相机不需要重新标定参数。
///
/// 测量失败（未找到孔洞边缘、孔洞/焊环尺寸异常、拟合残差过大）不判缺陷，而是标记
/// <see cref="PointInspectionResult.HasError"/>：此时偏移量无效，必须人工复核。
/// 把"测不出来"报成"合格"或"不合格"都是错的。
/// </summary>
public sealed class WeldInspectionAlgorithm
{
    private readonly InspectionConfig _cfg;
    private readonly NinePointCalibration _calibration;

    public WeldInspectionAlgorithm(InspectionConfig config, NinePointCalibration calibration)
    {
        _cfg = config;
        _calibration = calibration;
    }

    /// <summary>
    /// **诊断探针**（默认 null = 零行为影响）。
    ///
    /// 非 null 时，<see cref="Inspect"/> 会在正常流程**之外**把两个精修起点**各自单独**再跑一遍
    /// （`altCenter: null`，即退化为单起点行为），把两个结果与"实际采纳的那个"一起回调出去。
    ///
    /// 为什么需要它：`RefineBoundary` 的仲裁是在内部完成的，外部只看得到赢家——
    /// 而"哪个起点更该被采纳"这件事恰恰需要**两个都看到**，并且必须是在**算法真实使用的种子**
    /// 上看到的（`tools/Diag` 曾因硬编码种子而得出与算法相反的结论，见 FIXPLAN 第 9.6② 节）。
    ///
    /// 它只多算两次、**不改变任何判定**：`Adopted` 与不带探针时逐字段相同。
    /// 仅供 `tools/Diag` 的 `probe` 模式做离线判别研究（FIXPLAN 第 10 节）。
    /// </summary>
    internal Action<RefineProbeSnapshot>? RefineProbe { get; set; }

    /// <summary>精修两个起点的诊断快照，见 <see cref="RefineProbe"/>。</summary>
    /// <param name="SeedCenter">算法实际使用的种子圆心（工作分辨率）。</param>
    /// <param name="SeedRadius">种子半径（两个起点共用的尺度基准）。</param>
    /// <param name="PinCenter">孔位圆心（第二起点）。</param>
    /// <param name="Primary">以种子圆心为起点单独跑一遍的结果；null = 该起点没给出结果。</param>
    /// <param name="Secondary">以孔心为起点单独跑一遍的结果；null = 该起点没给出结果。</param>
    /// <param name="Adopted">算法**实际采纳**的精修结果（= 带仲裁与闸门的那一次）。</param>
    internal readonly record struct RefineProbeSnapshot(
        Point2d SeedCenter, double SeedRadius, Point2d PinCenter,
        RingRefinement? Primary, RingRefinement? Secondary, RingRefinement? Adopted);

    public PointInspectionResult Inspect(Mat bgr, WeldPoint point)
    {
        var sw = Stopwatch.StartNew();

        // 先在原生分辨率上转灰度。这一步兼作对输入 Mat 的原生校验：取图损坏或已被释放的
        // Mat 会在这里抛出**可捕获**的异常，流水线据此把坏图隔离成错误结果。
        // （不能改成先读 bgr.Height 再决定是否缩放——已释放的 Mat 读尺寸会
        //   AccessViolation 直接打死进程，且该异常无法捕获。）
        using var grayFull = new Mat();
        Cv2.CvtColor(bgr, grayFull, ColorConversionCodes.BGR2GRAY);

        // ---- 工作分辨率归一化 ----
        // 参数虽是相对量，但形态学核带绝对上限，base 超过 MaxWorkingBase 后相对尺度失真，
        // 整条链路会失效（实测 20MP 现场图 24/24 误判）。详见 InspectionConfig.MaxWorkingBase。
        using var gray = new Mat();
        var work = grayFull;
        var normalized = false;
        var srcBase = Math.Min(grayFull.Height, grayFull.Width);
        if (_cfg.MaxWorkingBase > 0 && srcBase > _cfg.MaxWorkingBase)
        {
            var scale = _cfg.MaxWorkingBase / (double)srcBase;
            Cv2.Resize(grayFull, gray, new Size(), scale, scale, InterpolationFlags.Area);
            work = gray;
            normalized = true;
        }

        Cv2.MedianBlur(work, work, 3);

        // 原图尺寸从 grayFull 取（不读 bgr）：上面那次 CvtColor 已兼作原生校验，
        // 在此之后读尺寸才是安全的。
        return InspectWorking(work, grayFull.Size(), point, sw, normalized);
    }

    /// <summary>
    /// 在工作分辨率灰度图上执行检测。返回的所有像素几何量都在 <paramref name="work"/> 的坐标系里，
    /// 并把该尺寸随结果带出（<see cref="PointInspectionResult.ProcessedSizePx"/>），
    /// 供 <see cref="Annotate"/> 在原始图上按比例还原；
    /// <paramref name="sourceSize"/> 是原始输入图尺寸，用于让界面/报表拿到**原图坐标**
    /// （见 <see cref="PointInspectionResult.SourceSizePx"/>）。
    /// 毫米折算按工作图幅进行，因此归一化不改变偏移量的物理含义。
    /// </summary>
    /// <param name="normalized">本图是否经过工作分辨率归一化（决定是否启用薄结构删除）。</param>
    private PointInspectionResult InspectWorking(Mat work, Size sourceSize, WeldPoint point, Stopwatch sw, bool normalized)
    {
        // ---- Step 1: 焊环 ----
        // 薄结构删除在主路径上仅在归一化生效时可用（原生分辨率下同样核宽会把稀疏掩码的熔核删掉，
        // 见 InspectionConfig.PreOpenKernelRatio 的实测记录）。
        var ring = WeldRingLocator.Detect(work, _cfg,
            normalized ? _cfg.PreOpenKernelRatio : 0);

        // 兜底：主路径（保守：不做薄结构删除）结果不可信时，再用"先删薄结构"的激进路径试一次。
        // 两条路径在现场数据上互补：
        //   · 环形光源反射在工件上留下一条约 20px 厚的细圈。它有两种破坏形态：
        //     紧贴熔核外沿时闭运算会把两者桥接成**一整块贴边的巨大连通域**，被
        //     "贴边连通域 = 台面/背景"这条剔除规则连熔核一起扔掉 → 主路径无候选 →
        //     良品被伪造成漏焊 NG（2026-09-14 用户报的 Img010 即此）；
        //     与熔核有间隔时细圈自成一环，其内部被 FillHoles 填成实心盘，**按填洞面积排序时
        //     压过真正的熔核**（实测伪候选半径 0.195~0.200×base vs 良品族值 0.142~0.146）。
        //   · 激进路径先把这个细圈开运算掉，闭运算就桥接不起来，熔核重新成为独立连通域。
        //
        // 触发条件本轮有变（这是关键）：旧条件是"主路径**完全找不到**焊环"，而上面第二种形态下
        // 主路径**明明能给出一个候选**（只是半径落在族值带外），所以旧条件永远不会命中它——
        // Img000 那一帧就是这么把圆心拖偏 202.6px 的。现在改为"没找到 **或** 半径落在实测族值带外"。
        // 之所以不再担心该项文档里记录的代价（激进路径会把稀疏掩码点位的熔核一起删掉）：
        // **只采纳落回族值带的重试结果**，而稀疏掩码点位的半径本来就在族值带内，根本不会走到这里。
        // 激进路径是否"找到了候选但不可信"（供下面把"确定没焊"降级为"测不出来"）。
        var aggressiveSawCandidate = false;

        // 要不要再花代价重试一次 = "没找到" **或** "半径不像本产品的正常焊环"。
        //
        // 族值带（虽宽松）只能用来**触发**，不能用来**否决**——2026-09-17 实测：
        // 厂商判 NG 的那 12 帧半径比 0.196~0.199、支撑弧段 100%，即"熔核整体明显变大"正是
        // 厂商的 NG 形态；而反射细圈被填洞成实心盘的伪候选实测 0.193~0.200，
        // **两者在半径、圆度、面积密度上完全重叠**。用带否决就会把前者一起废掉
        // （实测收紧到 0.135~0.152 后 `NgSamples_AreNotJudgedOk` 立刻报 12/12 失败）。
        // 能不能信由 `InspectionConfig.MinRingArcSupport`（边界支撑弧段）决定，
        // 而且那道闸门就在 WeldRingLocator 内部——低支撑角的检测结果根本不会以 Ok 返回。
        var needsRetry = !ring.Ok || !IsInRingRadiusFamily(ring.RadiusRatio);

        // 重试路径的候选（核宽阶梯 / 双尺度 / 贴边清除兜底共用），null = 前两条路径都没给出可用结果。
        RingDetection? best = null;

        if (normalized && _cfg.PreOpenKernelRatio <= 0 && needsRetry)
        {
            // 核宽阶梯：逐级加大薄结构删除力度。**走完全部级别**（不提前退出），
            // 在"半径落回族值带"的候选里按 <see cref="IsBetterRingCandidate"/> 择优：
            // 核宽不够时细圈有残留 → 半径偏大；核宽过度时熔核外缘被削 → 半径偏小；
            // 两者都偏离族值，取最接近族值的就是残留最小的那一级。
            // （实测 Img000：只跑第 1 级会停在 0.173×短边、圆心仍偏 116.9px；
            //   走完阶梯取最接近族值的一级才能把残留清干净。）
            var familyCenter = 0.5 * (_cfg.MinRingRadiusRatio + _cfg.RingRadiusFamilyMaxRatio);

            foreach (var kernelRatio in _cfg.FallbackPreOpenKernelRatios)
            {
                var candidate = WeldRingLocator.Detect(work, _cfg, kernelRatio);
                if (!candidate.Ok)
                {
                    // 激进路径"找到了但不可信"这件事本身就是证据：视野里并非空无一物。
                    // 记下来，供下面把主路径的"确定没焊"降级为"测不出来"——否则会凭空造出漏焊缺陷。
                    aggressiveSawCandidate = true;
                    candidate.Dispose();
                    continue;
                }

                aggressiveSawCandidate = true;
                if (!IsInRingRadiusFamily(candidate.RadiusRatio))
                {
                    candidate.Dispose();
                    continue;
                }

                if (IsBetterRingCandidate(candidate, best, familyCenter))
                {
                    best?.Dispose();
                    best = candidate;
                }
                else
                {
                    candidate.Dispose();
                }
            }

            // 双尺度重试：熔核外面常有一圈**细密的灰斑噪声带**（表面污渍 / 环形光散斑），
            // 它在小窗能量图上同样"粗糙"因而被并进掩码，使红圈半径偏大 17~20%、圆心偏到 3.7mm。
            // 判据是纹理尺度：鱼鳞斑在小窗与大窗上都高，细散斑在大窗上被平均掉。
            // 实测（与厂商 .vdb 对照）：两个坏帧的圆心差由 156px / 88px 降到 27px / 21px。
            //
            // **优先于核宽阶梯的结果**，理由：本路径不做形态学预开，半径不被侵蚀，
            // 而阶梯是"削掉外圈材料"来逼近族值——削本身会把圆心推向剩余形状的质心
            // （实测该帧阶梯给 (2670,1735) r=561，双尺度给更贴合纹理的 r=580、
            //   圆心离厂商只差 27px，阶梯则差 62px）。
            // 只在同时满足"族值带 + 支撑弧段门限"时才覆盖阶梯结果，否则维持阶梯/主路径不变。
            if (_cfg.DualScaleWindowRatio > 0)
            {
                var dual = WeldRingLocator.Detect(work, _cfg, 0, dualScaleGate: true);
                if (dual.Ok)
                {
                    aggressiveSawCandidate = true;
                    if (IsInRingRadiusFamily(dual.RadiusRatio)
                        && dual.ArcSupport >= _cfg.MinRingArcSupport)
                    {
                        best?.Dispose();
                        best = dual;
                    }
                    else
                    {
                        dual.Dispose();
                    }
                }
                else
                {
                    aggressiveSawCandidate = true;
                    dual.Dispose();
                }
            }

            if (best is not null)
            {
                ring.Dispose();     // 被替换的结果仍持有纹理能量图，必须显式释放
                ring = best;
            }
        }

        // ---- 第三路径：先删贴边连通域，再走一遍主路径 ----
        //
        // 只在前两条路径**都没有给出可用结果**时启用（`best` 为 null 且主路径未成功）。
        // 它修的是"熔核被闭运算桥接进工件外框、于是被'贴边=背景'规则连熔核一起扔掉"这一类：
        // 实测 `SN_...0007` 第 10 帧（用户报的 Img010）主路径零候选、核宽阶梯给不出可信候选，
        // 整帧报"未找到纹理显著区域（无焊点）"——而画面里焊环清晰可见。
        // 该帧的本路径结果为 R=716.6（ratio 0.196），随后由 <see cref="WeldRingLocator.RefineBoundary"/>
        // 精修到 485.7（厂商 .vdb 481.6，差 1.8px）。
        //
        // **采纳条件是"半径落在一个焊环可能有的尺寸区间内"**（下界 = 熔核未成形的下限，
        // 上界 = 有效性上限），两头都必要：
        //   · 上界拦"整幅台面被阈值选中"的退化候选（见上面那段说明）；
        //   · 下界拦"只捞到一块小碎斑"的候选。缺了下界会**把结果从'测不出来'改成'漏焊 NG'**——
        //     实测 20 SN / 480 帧厂商良品里有 3 帧（sn03/sn18/sn19 的 index 10）因此由
        //     `异常（支撑弧段不足）` 变成 `NG（漏焊）`，而它们的真实问题是"这一帧测不准"，
        //     不是"没焊上"。本路径是兜底，只应在它**真给出一个像焊环的结果**时才接管，
        //     否则保持主路径的结论。
        if (!ring.Ok && best is null && _cfg.EnableRingBorderCleanupFallback)
        {
            var cleaned = WeldRingLocator.Detect(work, _cfg, 0,
                dualScaleGate: false, preCleanBorder: true);
            if (cleaned.Ok
                && cleaned.RadiusRatio >= _cfg.MinRingRadiusRatio
                && cleaned.RadiusRatio <= _cfg.MaxRingRadiusRatio)
            {
                ring.Dispose();
                ring = cleaned;
            }
            else
            {
                cleaned.Dispose();
            }
        }

        // ---- 第四路径：掩码边界"不像一整圈"、但尺度可信的候选，交边界精修去定性 ----
        //
        // 只在前三条路径都没给出可用结果时启用，且**必须被精修转正**才允许进入判定
        // （转正在下面"第四路径的候选：必须转正"那一段，位置在孔位之前）。
        //
        // 修的是哪一类（现场 `SN_...0004` 第 10 帧 / OK_471，2026-09-19 用户报的那张）：
        // 熔核的鱼鳞纹理在**同一个 SN 的 23 帧里都是致密的**，只有这一帧偏稀疏——纹理掩码在
        // 鱼鳞斑之间断裂，于是激进薄结构删除之后剩下的是**边缘破碎**的掩码。它的外接圆其实很准
        // （k=0.060 一级：圆心与厂商 `.vdb` 只差 1.2px、半径比 0.1376 vs 真值 0.1317），
        // 但"掩码轮廓落在中位半径 ±10% 的角向扇区占比"只有 60%，够不着 0.82 那道门限。
        // 那道门限是拿**主路径的实心掩码**标定的，用它否决"掩码稀疏"的候选是口径错配。
        //
        // 凭什么放宽是安全的：本路径上的结果一律打上 MaskSupportRelaxed，
        // 唯一能"转正"它的是 RefineBoundary——它在 |∇I| 上逐射线量熔核外沿，
        // 与掩码连不连续无关，并给出自己的支撑弧段。转正失败的帧仍然报"测不出来"。
        //
        // **触发条件里那条 `kind == Unmeasurable || aggressiveSawCandidate` 是安全性的关键**：
        // 它保证本路径只可能改写"本来就会报**测不出来**"的帧（那种帧的结论是"交人工复核"），
        // 绝不会碰到"主路径明确判了漏焊"的帧（NoWeld 且没有任何路径找到过候选）。
        // 这条约束把漏焊的召回率**原样保住**——本路径最坏的情况只是把"复核"变成另一个"复核"，
        // 不会把"确定的缺陷"变成合格。
        var skipFinalRefine = false;
        var wouldReportUnmeasurable = ring.FailureKind == RingFailure.Unmeasurable
                                      || aggressiveSawCandidate;
        if (!ring.Ok && best is null && wouldReportUnmeasurable && normalized
            && _cfg.PreOpenKernelRatio <= 0
            && _cfg.MinRingFallbackArcSupport > 0
            && _cfg.WeldRingRefineProfileOuterRatio > 0 && _cfg.WeldRingRefineEdgeFraction > 0)
        {
            // 这一档的支撑弧段**全都低于主门限**（否则前两条路径早就采纳了），拿它排序等于拿噪声
            // 排序，故改用"半径最接近族值中位"。实测本帧三级 0.1688 / 0.1376 / 0.1277 里
            // 只有中间那一级的圆心对得上厂商（1.2px），另外两级分别差 8.4px / 7.5px 且半径偏 28% / 3%。
            var familyCenter = 0.5 * (_cfg.MinRingRadiusRatio + _cfg.RingRadiusFamilyMaxRatio);
            foreach (var kernelRatio in _cfg.FallbackPreOpenKernelRatios)
            {
                var candidate = WeldRingLocator.Detect(work, _cfg, kernelRatio,
                    minArcSupport: _cfg.MinRingFallbackArcSupport);
                if (!candidate.Ok)
                {
                    aggressiveSawCandidate = true;
                    candidate.Dispose();
                    continue;
                }

                aggressiveSawCandidate = true;
                if (!IsInRingRadiusFamily(candidate.RadiusRatio))
                {
                    candidate.Dispose();
                    continue;
                }

                if (best is null
                    || Math.Abs(candidate.RadiusRatio - familyCenter)
                       < Math.Abs(best.RadiusRatio - familyCenter))
                {
                    best?.Dispose();
                    best = candidate;
                }
                else
                {
                    candidate.Dispose();
                }
            }

            if (best is not null)
            {
                ring.Dispose();
                ring = best;
            }
        }

        // 本方法负责释放最终采用的环检测结果（原 `using var ring`，因中途可能换用兜底结果而改为显式接管）
        using var ringScope = ring;

        // "掩码边界不成圈、未经精修认证"这个状态**随结果一起传递**（RingDetection.MaskSupportRelaxed），
        // 而不是靠一个本地标志记住"我跑过第四路径"——后者一旦有人在中间插入一次 return 就会失配
        // （本轮真踩过：认证写在孔位之后，孔位提前 return 就把未认证的圆放进了报文）。
        var provisional = ring.MaskSupportRelaxed;

        // 焊环定位失败要分两种报法，报错方向正好相反：
        //   NoWeld       → 确定没焊上，是可判定的缺陷；
        //   Unmeasurable → 测不出来（图太小、形状不支持），偏移量无效，必须人工复核。
        // 把后者报成漏焊 NG 会凭空制造一个不存在的缺陷，报成合格则是漏检。
        if (!ring.Ok)
        {
            var kind = ring.FailureKind;
            var reason = ring.Failure ?? "未找到焊环";

            // 激进路径找到过候选却不可信（半径不在族值带内）→ 视野里并非空无一物，
            // "确定没焊"这个结论就不再成立。降级为"测不出来"交人工复核，
            // 而不是把一个良品报成漏焊 NG（本仓库一贯取舍：宁可复核，不可伪造缺陷）。
            if (kind == RingFailure.NoWeld && aggressiveSawCandidate)
            {
                kind = RingFailure.Unmeasurable;
                reason += "；激进薄结构删除路径虽找到候选，但其半径不在实测族值带内";
            }

            return kind == RingFailure.Unmeasurable
                ? Fail(point, sw, reason, work.Size(), sourceSize)
                : MissingWeld(point, sw, reason, work.Size(), sourceSize);
        }

        // 找到的圆盘比任何真实焊点都小（例如只剩孔口边缘）→ 熔核未成形，仍是漏焊。
        // 这与"半径过大"要区别对待：偏小意味着焊点不存在，是缺陷；
        // 偏大意味着锁定了错误目标，是测量失败（见下）。
        if (ring.RadiusRatio < _cfg.MinRingRadiusRatio)
            return MissingWeld(point, sw,
                $"未找到焊环（最大圆形纹理区仅 {ring.RadiusRatio:P1}×短边）", work.Size(), sourceSize);

        // 半径超过**有效性上限** = 两条路径都没给出可信的焊环。这不是"没焊上"，而是"测不出来"：
        // 报漏焊会凭空造一个不存在的缺陷，报合格是漏检——两种都错，正确答案是交人工复核。
        // 注意本项（0.32）高于炸焊阈值（0.26）：炸焊读数（0.26~0.32）在上面已被判为可信、
        // 走炸焊判定，只有超过 0.32 才落到这里。族值带那一层的把关在 IsTrustworthyRing。
        if (ring.RadiusRatio > _cfg.MaxRingRadiusRatio)
            return Fail(point, sw,
                $"焊环半径超出有效性上限（{ring.RadiusRatio:P1}×短边，上限 {_cfg.MaxRingRadiusRatio:P0}）；" +
                "两条检测路径均未给出可信焊环",
                work.Size(), sourceSize, ring);

        // ---- 第四路径的候选：**必须**在任何下游使用之前完成"转正"----
        //
        // 位置很关键：孔位（Step 2）拿焊环的圆心/半径当射线原点与搜索尺度，孔位校验不过就**提前返回**，
        // 于是"转正"如果放在 Step 2.5 就会被这一条提前返回绕过——实测 `SN_...0502` 第 10 帧
        // （`21-39-16 665`）正是如此：候选圆偏厂商 144px，孔位校验先失败退出，
        // 一个没被认证过的圆就这样进了报文。所以转正必须紧跟焊环定位、在孔位之前。
        //
        // 代价是这里拿不到 `pin.Center` 当第二起点，只能用单起点。这个取舍是有意的：
        // 未认证的圆**绝不能**参与孔位测量（否则孔位也跟着错，而孔位校验又可能恰好通过）。
        if (provisional)
        {
            var cert = WeldRingLocator.RefineBoundary(work, ring.Center, ring.Radius, _cfg,
                altCenter: null);

            // 三条认证判据，与下面报错文字一一对应；全过才算转正。
            var certFail = cert is not { } c
                ? "精修在 |∇I| 上找不到自洽的熔核外沿"
                : c.ArcSupport < _cfg.MinRingArcSupport
                    ? $"激进薄结构删除给出的候选，其掩码边界不成圈；精修后边界仅覆盖 " +
                      $"{c.ArcSupport:P0} 圆周，下限 {_cfg.MinRingArcSupport:P0}"
                    : !IsInRingRadiusFamily(c.Circle.Radius / ring.BaseSize)
                        ? $"精修后半径 {c.Circle.Radius / ring.BaseSize:P1}×短边 不在族值带 " +
                          $"[{_cfg.MinRingRadiusRatio:P1}, {_cfg.RingRadiusFamilyMaxRatio:P1}] 内"
                        : null;

            if (certFail is null)
            {
                var certified = cert!.Value;
                ring.ApplyRefinement(certified.Circle.Center, certified.Circle.Radius,
                    certified.ArcSupport);
                ring.MaskSupportRelaxed = false;   // 已认证，标记随之落下
                skipFinalRefine = true;            // 这个圆已经由精修定过，不再做第二遍（第二遍会拿孔心当第二起点）
            }
            else
            {
                // ---- 第五路径：多起点重试（2026-09-20，FIXPLAN 第 11 节）----
                //
                // **只在这里介入**——即"转正失败、本来就要报测不出来"的帧。曾经把它放在
                // 第四路径之后、转正之前，结果是它用网格结果**顶掉了 12 帧已经过认证的
                // 第四路径结果**（A/B 逐帧对差抓到：14 帧变化，而目标只有 2 帧）。
                // 用未验证的机制挤掉已验证的机制是错的，所以触发面必须窄到"转正失败"。
                var rescued = TryMultiSeedRetry(work, ring);
                if (rescued is { } w)
                {
                    ring.Dispose();
                    // MaskSupportRelaxed 保持 false —— 这个圆**已经过精修认证**（弧段达标），
                    // 所以下面那道"必须转正"的检查不再需要跑。
                    ring = w;
                    skipFinalRefine = true;
                }
                else
                {
                    return Fail(point, sw, $"焊环候选未被边界精修确认（{certFail}）",
                        work.Size(), sourceSize);
                }
            }
        }

        // ---- Step 2: 孔位（焊偏判定的基准） ----
        // 先用**三材质分层**定位孔位：焊环内部只有暗缝隙 / 孔位端面 / 端面金属三层，
        // 取中间那一层即孔位端面（见 HoleLocator，不含任何绝对灰度阈值）。
        // 这是唯一不受"焊环圆心离孔心很远"影响的取法——实测两者相距可达 97px（比孔半径 96px 还大），
        // 沿用它当射线原点会让射线在**孔沿被反光冲掉**的一侧越过孔位、落到熔核内沿上：
        // 实测把孔位半径量成 199px（真值约 96px）、圆心偏 100px+，并因此把偏移量**系统性压低**
        // （Img023 报 0.152mm 而真值约 0.56mm，偏小的方向正是"该判 NG 却判合格"）。
        // 分层失败时退回梯度卡尺，不为此引入新的测量失败。
        // 分层失败时退回梯度卡尺——但**兜底结果必须自带有效性判据**（2026-09-18）：
        // 卡尺的模型只在 D < 0.8×孔半径 时成立，而它的射线原点是焊环圆心；孔心一旦离得远，
        // 它会把"半圈孔沿 + 半圈熔核内沿"平均成一个**自洽但无意义**的圆——实测由此静默给出
        // 偏低 2.5 倍的偏移量并判 OK（Img000：1.761mm vs 真值 ≈4.45mm，圆心偏 6.21mm）。
        // 详见 InspectionConfig.CaliperMaxResidualRatio 的实测记录。
        var hole = HoleLocator.Locate(work, ring.Texture!, ring.Threshold, ring.Center, ring.Radius, _cfg);
        PinFit? pin;
        if (hole is { } holeFit)
        {
            pin = new PinFit(holeFit.Center, holeFit.Radius, Geo.Dist(holeFit.Center, ring.Center),
                holeFit.ResidualPx, UsedRays: 0, TotalRays: 0);
        }
        else
        {
            pin = PinLocator.Locate(work, ring.Center, ring.BaseSize, _cfg);
            if (pin is { } caliper)
            {
                var familyRadius = ring.BaseSize * _cfg.PinRadiusFamilyRatio;
                var residualRatio = caliper.Radius > 0 ? caliper.ResidualPx / caliper.Radius : double.MaxValue;
                if (residualRatio > _cfg.CaliperMaxResidualRatio)
                    return Fail(point, sw,
                        $"孔位不可信：分层未给出候选，卡尺兜底的解与孔沿不自洽" +
                        $"（逐射线残差 {residualRatio:P1}×半径，上限 {_cfg.CaliperMaxResidualRatio:P0}）",
                        work.Size(), sourceSize, ring, caliper);
                if (caliper.Radius > _cfg.CaliperMaxRadiusRatioOfFamily * familyRadius)
                    return Fail(point, sw,
                        $"孔位不可信：分层未给出候选，卡尺兜底的解半径饱和在搜索窗附近" +
                        $"（{caliper.Radius / familyRadius:P0}×孔位族值半径，上限 {_cfg.CaliperMaxRadiusRatioOfFamily:P0}）",
                        work.Size(), sourceSize, ring, caliper);
            }
        }
        if (pin is null)
            return Fail(point, sw, "未定位到孔位，无法判定焊偏", work.Size(), sourceSize, ring);

        // 孔位是在**精修前**的焊环圆上测量的（射线原点、搜索区都以它为准），而下面的闸门用的是
        // 精修后的焊环半径。两者口径不同，判定依据必须把两个数都写出来——否则操作员看到
        // "孔洞半径异常（43.5%）"无法判断到底是孔量大了还是环被搬小了（实测 Img009：
        // 孔位测量时环半径 530px，精修后 448px，同一个孔位读数 0.368 → 0.435 越界）。
        var ringRadiusAtPinMeasurement = ring.Radius;

        // ---- Step 2.5: 以**熔核自身圆心**为锚精修焊环圆 ----
        // 把红圈从"纹理掩码外接圆"升级为"用梯度幅值逐射线量出的熔核外沿"。
        // 这一步修的是用户报的"框选偏移"：掩码在镜面冲白的一侧缺失、在熔核外侧的细暗圈/散斑带
        // 上多余，两者都把整体圆拟合的圆心推走，而且是**跟着光照方向推**。
        //
        // **锚点必须是 ring.Center，不能是 pin.Center**（2026-09-18 修正）：以孔心为锚等于
        // 假设"熔核与极柱同心"，而"焊偏"正是要检验这个前提——实测那样会把偏移量抹平成接近零。
        // 孔心只作为**第二起点**传入（细圈帧上 Detect 的种子圆心本身偏 20~45px，两个起点
        // 互相校验），选择规则见 WeldRingLocator.RefineBoundary。
        // 精修失败时返回 null，此处保持原结果不变。
        //
        // 探针需要在精修**覆盖 ring 之前**拿到种子，故先快照（仅当探针被设置时才用到）。
        var probeSeedCenter = ring.Center;
        var probeSeedRadius = ring.Radius;
        if (!skipFinalRefine
            && _cfg.WeldRingRefineProfileOuterRatio > 0 && _cfg.WeldRingRefineEdgeFraction > 0)
        {
            var refined = WeldRingLocator.RefineBoundary(work, ring.Center, ring.Radius, _cfg,
                pin.Center);
            if (refined is { } rf)
            {
                var ratio = rf.Circle.Radius / ring.BaseSize;
                // 精修结果仍须落在半径**有效性**区间内，否则宁可保持旧结果：
                // 这一步只允许"把圆搬正"，不允许把可靠性判定也一起搬走。
                if (ratio >= _cfg.MinRingRadiusRatio && ratio <= _cfg.MaxRingRadiusRatio)
                    ring.ApplyRefinement(rf.Circle.Center, rf.Circle.Radius, rf.ArcSupport);
            }

            // 诊断探针：把两个起点**各自单独**再跑一遍（altCenter: null = 退化为单起点行为）。
            // 放在这里而不是更早，是因为"实际采纳的结果"要等 ApplyRefinement 之后才定下来；
            // 放在闸门（孔/环比、残差、拓扑）之前，是为了让"本来会被闸门拦下"的帧也进入研究集。
            if (RefineProbe is { } probe)
                probe(new RefineProbeSnapshot(probeSeedCenter, probeSeedRadius, pin.Center,
                    WeldRingLocator.RefineBoundary(work, probeSeedCenter, probeSeedRadius, _cfg,
                        altCenter: null),
                    WeldRingLocator.RefineBoundary(work, pin.Center, probeSeedRadius, _cfg,
                        altCenter: null),
                    refined));
        }

        // 校验一（尺寸族值）：两个地标各自的尺寸都必须在物理规格范围内。
        // 焊环半径已在上面的族值带校验过；这里查灰圆半径相对焊环半径的比例（实测 0.255±0.036）。
        var pinRatio = pin.Radius / ring.Radius;
        if (pinRatio < _cfg.MinPinRadiusRatio || pinRatio > _cfg.MaxPinRadiusRatio)
            return Fail(point, sw,
                $"孔洞半径异常（{pinRatio:P1}×焊环半径，期望 {_cfg.MinPinRadiusRatio:P0}~{_cfg.MaxPinRadiusRatio:P0}；" +
                $"孔位测量时焊环半径 {ringRadiusAtPinMeasurement:F0}px，判定时已精修为 {ring.Radius:F0}px，" +
                $"同一孔位读数按前者为 {pin.Radius / Math.Max(ringRadiusAtPinMeasurement, 1e-6):P1}）",
                work.Size(), sourceSize, ring, pin);

        // 校验二（拟合残差）：残差大说明射线没有一致地落在同一个圆上，圆心不可信。
        if (pin.ResidualPx > _cfg.MaxPinResidualRatio * ring.Radius)
            return Fail(point, sw,
                $"孔洞边缘拟合残差过大（{pin.ResidualPx / ring.Radius:P1}×焊环半径）",
                work.Size(), sourceSize, ring, pin);

        // 校验三（拓扑关系）：极柱必须落在焊环**内部**。
        // 判据 D + R_pin &lt; R_out（等价于 D &lt; R_out − R_pin）。不满足说明极柱已经探出焊环边界，
        // "焊环环绕极柱"这一基本前提不成立，此时两个圆心之间的偏移量不再有物理含义——
        // 报一个可能被读成"严重焊偏"的读数会误导处置，正确做法是判测量失败交人工复核。
        var centerGapPx = Geo.Dist(pin.Center, ring.Center);
        if (centerGapPx + pin.Radius > ring.Radius)
            return Fail(point, sw,
                $"极柱不在焊环内部（两心距 {centerGapPx:F0}px + 极柱半径 {pin.Radius:F0}px > 焊环半径 {ring.Radius:F0}px）",
                work.Size(), sourceSize, ring, pin);

        // ---- Step 3: 质量指标 ----
        // 注意顺序：指标必须在精修**之后**测——填充率/环弧覆盖率/暗区都以"圆内"为口径，
        // 先测再精修会让读数与界面上的圆对不上（而且填充率会凭空虚增，见 RingDetection.ApplyRefinement）。
        var metrics = WeldQualityMeter.Measure(work, ring.Solid!, ring.Texture!, ring.Threshold,
            ring.Center, ring.Radius, ring.BaseSize);

        var defects = new List<WeldDefect>();
        string? missingWeldReason = null;

        // 漏焊：环弧不连续（未成环 = 未焊透）或熔核面积明显不足（焊点残缺）。
        // 判定依据必须写进结果（PointInspectionResult.ErrorMessage 的约定）：报表与详情窗口据此
        // 区分"根本没焊"和"焊了但环不密实"，否则操作员只看到一个没有理由的"漏焊"。
        {
            var reasons = new List<string>();
            if (metrics.Coverage < _cfg.MinRingCoverage)
                reasons.Add($"环弧覆盖率 {metrics.Coverage:P0} < 下限 {_cfg.MinRingCoverage:P0}");
            if (metrics.Fill < _cfg.MinRingFill)
                reasons.Add($"熔核填充率 {metrics.Fill:P0} < 下限 {_cfg.MinRingFill:P0}");
            if (reasons.Count > 0)
            {
                defects.Add(WeldDefect.MissingWeld);
                missingWeldReason = string.Join("；", reasons);
            }
        }

        // 炸焊：熔核尺寸超限
        if (ring.RadiusRatio > _cfg.BlowoutRingRadiusRatio)
            defects.Add(WeldDefect.Blowout);

        // 飞溅：环外粗糙颗粒的总面积或数量超限
        if (metrics.SpatterAreaRatio > _cfg.MaxSpatterAreaRatio || metrics.SpatterCount > _cfg.MaxSpatterCount)
            defects.Add(WeldDefect.Spatter);

        // ---- Step 3.5: 熔核内沿（环形的内边界） ----
        // 熔核是"有厚度的环"，只给外边界表达不出环宽。内沿必须在**纹理能量图**上按
        // "粗糙起始半径"取：掩码里的内沿已被闭运算封死（实测后处理掩码是实心盘）。
        // 起点用灰圆半径跳过极柱边缘带——这正是本轮"灰圆独立测量"的成果。
        // 内外圆心偏差过大（含反射细圈残留）时不输出，避免给出一个假环宽；但不判测量失败，
        // 环宽本轮只作诊断量，不参与判定。
        var inner = WeldInnerLocator.Locate(ring.Texture!, ring.Threshold, ring.Center, ring.Radius,
            pin.Center, pin.Radius, _cfg, _cfg.PinCaliperRays);
        var innerRadius = 0.0;
        var innerCenter = default(Point2d);
        if (inner is { } innerFit
            && innerFit.ConcentricDeviationPx <= _cfg.WeldInnerMaxConcentricDeviationRatio * ring.Radius
            && innerFit.Radius < ring.Radius)
        {
            innerRadius = innerFit.Radius;
            innerCenter = innerFit.Center;
        }

        // ---- Step 4: 偏移量与公差 ----
        var (offsetX, offsetY) = ToMillimetres(pin.Center, ring.Center, work.Size());
        var distance = Math.Sqrt(offsetX * offsetX + offsetY * offsetY);
        if (distance > _cfg.OffsetToleranceMm)
            defects.Add(WeldDefect.Misaligned);

        sw.Stop();
        return new PointInspectionResult
        {
            Point = point,
            PoleCenterPx = pin.Center,
            PoleRadiusPx = pin.Radius,
            WeldCenterPx = ring.Center,
            WeldRadiusPx = ring.Radius,
            WeldArcSupport = ring.ArcSupport,
            WeldInnerRadiusPx = innerRadius,
            WeldInnerCenterPx = innerCenter,
            ProcessedSizePx = work.Size(),
            SourceSizePx = sourceSize,
            OffsetXmm = offsetX,
            OffsetYmm = offsetY,
            Defects = defects,
            ErrorMessage = missingWeldReason,
            ProcessingMs = sw.Elapsed.TotalMilliseconds
        };
    }

    /// <summary>
    /// **第五路径：多起点重试**（2026-09-20，FIXPLAN 第 11 节）。返回 null = 没找到可信结果。
    ///
    /// 调用点只有一个：第四路径的候选**转正失败**时（也就是"本来就要报测不出来"的帧）。
    /// 触发面必须窄到这一步——曾经放在转正之前，结果它用网格结果顶掉了 12 帧已经过认证的
    /// 第四路径结果（960 帧 A/B 逐帧对差抓到 14 帧变化，而目标只有 2 帧）。
    ///
    /// 它修的是哪一类：**熔核外沿在 |∇I| 上完全可测，但由纹理掩码算出的候选圆心偏了**，
    /// 而精修从偏掉的圆心出发会落进另一个"自洽但错误"的盆地。实测 `21-39-16 665`：
    /// 用厂商圆当种子 → 支撑弧段 99.2%、离厂商 18px；用掩码质心当种子 → r=535、弧段 70%；
    /// 两个种子**只差 2.5 工作像素**。这是**分叉**，换个"更好的估计器"没用，**必须搜索**。
    ///
    /// 为什么是"网格 + 弧段仲裁"：两帧的盆地实测显示掩码质心离真值 9 / 23 工作像素，
    /// 而正确盆地只覆盖约 3 工作像素宽，且**半径分不开两个盆地**（正确 486~489 vs
    /// 错误 450~539 重叠）——唯一能分开的是支撑弧段（99% vs 68~74%），而那正是既有的可信度量。
    /// 完整实测见 <see cref="InspectionConfig.MultiSeedRetryStepRatio"/>。
    /// </summary>
    private RingDetection? TryMultiSeedRetry(Mat work, RingDetection ring)
    {
        if (_cfg.MultiSeedRetryStepRatio <= 0)
            return null;
        // 搜索中心：第四路径的候选圆（此刻它就在 ring 里，且一定是 Ok 的）
        var hint = ring.BestCandidate ?? new CircleFit(ring.Center, ring.Radius);

        var familyRadius = 0.5 * (_cfg.MinRingRadiusRatio + _cfg.RingRadiusFamilyMaxRatio)
                           * ring.BaseSize;
        var step = hint.Radius * _cfg.MultiSeedRetryStepRatio;
        var rings = Math.Clamp(_cfg.MultiSeedRetryRings, 0, 3);
        RingDetection? winner = null;

        for (var gy = -rings; gy <= rings; gy++)
        for (var gx = -rings; gx <= rings; gx++)
        {
            var origin = new Point2d(hint.Center.X + gx * step, hint.Center.Y + gy * step);
            // 单起点跑（altCenter: null）：这一步的目的是**换种子重试**，
            // 而孔位（第二起点）本身依赖焊环圆心，此刻还拿不到。
            var refined = WeldRingLocator.RefineBoundary(work, origin, familyRadius, _cfg,
                altCenter: null);
            if (refined is not { } rf)
                continue;

            var ratio = rf.Circle.Radius / ring.BaseSize;
            if (rf.ArcSupport < _cfg.MinRingArcSupport
                || ratio < _cfg.MinRingRadiusRatio || ratio > _cfg.MaxRingRadiusRatio)
                continue;   // 错误盆地的弧段只有 68~74%，过不了这道门

            if (winner is null || rf.ArcSupport > winner.ArcSupport)
            {
                winner?.Dispose();
                // Texture/Solid 都取**克隆**：RingDetection.Dispose 会释放这两个 Mat，
                // 共享引用会导致双重释放（源 ring 在调用方也要被 Dispose）。
                winner = new RingDetection
                {
                    Ok = true,
                    Center = rf.Circle.Center,
                    Radius = rf.Circle.Radius,
                    RadiusRatio = ratio,
                    ArcSupport = rf.ArcSupport,
                    Solid = ring.Solid?.Clone(),
                    Texture = ring.Texture?.Clone(),
                    Threshold = ring.Threshold,
                    BaseSize = ring.BaseSize
                };
            }
        }

        if (winner is not null)
        {
            // 掩码裁到胜出的圆内（口径与 RingDetection.ApplyRefinement 一致）：
            // 质量指标全部以"圆内"为准，圆变小而掩码不变会让填充率凭空虚增。
            winner.ApplyRefinement(winner.Center, winner.Radius, winner.ArcSupport);
        }
        return winner;
    }

    /// <summary>
    /// 像素偏移 → 物理偏移。
    ///
    /// **两轴必须用同一个缩放系数**：这曾是本方法的一个真实缺陷——原实现对 X 用
    /// <c>ImageWidth ÷ 图宽</c>、对 Y 用 <c>ImageHeight ÷ 图高</c> 分轴折算，
    /// 于是输入图与配置视野的**宽高比不一致时，同一段物理距离在 X/Y 上会被折成不同的毫米数**。
    /// 实测（配置 1280×1024 跑 5472×3648 现场图，宽高比 1.25 vs 1.50）：
    /// X 系数 1.855、Y 系数 2.226，相差 20%——即偏移量的**方向**被歪曲了 20%，
    /// 而 1.0mm 公差的判定就建立在这个被歪曲的合成值上。
    /// 物理上"毫米/像素"是成像系统的属性，不应随轴向变化，故改为单一等比系数。
    ///
    /// 取宽边之比作为唯一系数（与 <see cref="InspectionConfig.ImageWidth"/> 原注释里
    /// X 轴的折算口径一致）。**这仍是近似模型**：输入图与配置视野宽高比不一致本身就说明
    /// 配置不是该相机的真实分辨率，任何单一系数都只是折中。换算的绝对正确性只能靠实测标定
    /// （用已知尺寸反推真实 mm/px）解决，诊断见
    /// <see cref="InspectionConfig.FovAspectDeviation"/>。
    ///
    /// 符号约定：offset = 焊环中心 − 孔洞中心。
    /// </summary>
    private (double X, double Y) ToMillimetres(Point2d pinPx, Point2d weldPx, Size imageSize)
    {
        var scale = _cfg.ImageWidth / (double)imageSize.Width;
        var pinMm = _calibration.Transform(new Point2d(pinPx.X * scale, pinPx.Y * scale));
        var weldMm = _calibration.Transform(new Point2d(weldPx.X * scale, weldPx.Y * scale));
        return (weldMm.X - pinMm.X, weldMm.Y - pinMm.Y);
    }

    /// <summary>
    /// 在核宽阶梯给出的多个"半径落回族值带"的候选之间择优。
    ///
    /// 判据顺序（**2026-09-17 试改过、又改回来了，记录为什么**）：
    ///  1. **边界支撑弧段**（<see cref="RingDetection.ArcSupport"/>）更高者优先——它直接度量
    ///     "这条边界是否像一整圈"，是本算法里唯一针对"圆心可信度"的质量量；
    ///  2. 并列时取半径**更接近族值中位**者。
    ///
    /// 曾经把顺序反过来（半径优先），以为"支撑弧段是自指的，污染越重块越大反而越高"。
    /// 实测证伪：Img010 上核宽 0.045 级（支撑 0.68、半径 0.1585）的圆心偏差 82px，
    /// **优于** 0.060 级（支撑 0.57、半径 0.1442 恰好等于族值）的 148px——半径对了、
    /// 圆心反而更偏，因为"削掉外圈材料"本身就会把圆心推向剩余形状的质心。
    /// 改成半径优先后，合成夹具的 OK 点出现 `Misaligned` 误判（D=0.6~0.8mm）：
    /// 夹具渲染的熔核（实测 0.182~0.217）本来就在族值带上界之外，阶梯产出的是它的"被削版本"，
    /// 按半径择优必然挑到削得最狠、圆心最偏的那一级。
    /// 支撑弧段优先则等价于"取削得最轻的那一级"，在两个数据集上都更好。
    ///
    /// 真正拦住污染候选的不是本方法，而是 `InspectionConfig.MinRingArcSupport` 那道闸门
    /// （在 <see cref="WeldRingLocator"/> 内部）：支撑弧段不足的候选根本不会以 Ok 返回，
    /// 所以本方法的入参已经都过了闸门。
    /// </summary>
    private static bool IsBetterRingCandidate(RingDetection candidate, RingDetection? current,
        double familyCenter)
    {
        if (current is null)
            return true;

        const double supportEpsilon = 0.02;   // 支撑弧段的并列容差
        if (candidate.ArcSupport > current.ArcSupport + supportEpsilon)
            return true;
        if (current.ArcSupport > candidate.ArcSupport + supportEpsilon)
            return false;

        return Math.Abs(candidate.RadiusRatio - familyCenter)
             < Math.Abs(current.RadiusRatio - familyCenter);
    }

    /// <summary>
    /// 拟合半径是否落在**实测族值带**内（**不是**有效性边界——两者分工见
    /// <see cref="InspectionConfig.RingRadiusFamilyMaxRatio"/> 的说明）。
    /// 带内 = 像本产品的正常焊环；带外 = 先当成"需要再试一次"的信号。
    ///
    /// **注意本判据只用于"触发重试"，不用于"否决结果"**：厂商 NG 的大熔核（实测 0.196~0.199）
    /// 与反射细圈的伪候选（0.193~0.200）在这个尺度上完全重叠，用它否决会同时废掉前者。
    /// 能不能信由 <see cref="IsTrustworthyRing"/>（边界支撑弧段）决定。
    /// </summary>
    private bool IsInRingRadiusFamily(double radiusRatio) =>
        radiusRatio >= _cfg.MinRingRadiusRatio && radiusRatio <= _cfg.RingRadiusFamilyMaxRatio;

    /// <summary>
    /// 构造"漏焊"结果：熔核不存在（未找到纹理区，或找到的圆形纹理区远小于任何真实熔核）。
    /// 与 <see cref="Fail"/> 的区别：这是**确定的缺陷**，不是测量失败——
    /// "视野里根本没有焊点"是可以判定的，不需要人工复核尺寸。
    /// </summary>
    private static PointInspectionResult MissingWeld(WeldPoint point, Stopwatch sw, string message,
        Size processedSize, Size sourceSize)
    {
        sw.Stop();
        return new PointInspectionResult
        {
            Point = point,
            ProcessedSizePx = processedSize,
            SourceSizePx = sourceSize,
            Defects = new[] { WeldDefect.MissingWeld },
            ErrorMessage = message,
            ProcessingMs = sw.Elapsed.TotalMilliseconds
        };
    }

    /// <summary>构造"测量失败"结果：不判缺陷，偏移量无效，交由人工复核。</summary>
    private static PointInspectionResult Fail(WeldPoint point, Stopwatch sw, string message,
        Size processedSize, Size sourceSize, RingDetection? ring = null, PinFit? pin = null)
    {
        sw.Stop();
        return new PointInspectionResult
        {
            Point = point,
            HasError = true,
            ErrorMessage = message,
            ProcessedSizePx = processedSize,
            SourceSizePx = sourceSize,
            PoleCenterPx = pin?.Center ?? default,
            PoleRadiusPx = pin?.Radius ?? 0,
            WeldCenterPx = ring?.Center ?? default,
            WeldRadiusPx = ring?.Radius ?? 0,
            WeldArcSupport = ring?.ArcSupport ?? 0,
            ProcessingMs = sw.Elapsed.TotalMilliseconds
        };
    }

    /// <summary>
    /// 在原图上标注检测结果（返回新 Mat，调用方负责释放）。
    ///
    /// 线宽、字号、十字臂长、文字位置一律按图像短边缩放：应用现在按**原始分辨率**保存标注图
    /// （图幅小到 182×184，见 ImageFolderGrabber），固定字号会把结论文字截断到画布之外——
    /// 而这行文字正是操作员看图时唯一要读的东西。
    ///
    /// 文字由 <see cref="AnnotationText"/> 绘制（GDI+ 中文渲染）：结论文字含中文，
    /// 早先用 <c>Cv2.PutText</c> 会把中文整段画成 "?"，异常图于是只剩
    /// <c>ERROR: ????????????</c>，操作员看不到任何原因。
    ///
    /// 几何量则按 <see cref="PointInspectionResult.ProcessedSizePx"/> 反算：输入图大于工作分辨率
    /// 时检测是在缩小图上做的，圆心/半径必须乘回原始图幅的比例，否则标注会缩在图的左上角。
    /// </summary>
    public Mat Annotate(Mat bgr, PointInspectionResult result)
    {
        var img = bgr.Clone();

        var baseSize = Math.Min(bgr.Width, bgr.Height);
        var thickness = Math.Max(1, (int)Math.Round(baseSize / 500.0));
        var crossArm = Math.Max(4, (int)Math.Round(baseSize * 0.012));
        var fontPx = Math.Max(13.0, baseSize * 0.030);
        var lineHeight = (int)Math.Round(AnnotationText.LineHeight(fontPx));
        var textX = Math.Max(4, (int)Math.Round(baseSize * 0.020));
        var textY = Math.Max((int)Math.Round(fontPx * 1.25), (int)Math.Round(baseSize * 0.043));
        var available = Math.Max(40, bgr.Width - textX * 2);

        var hasRing = result.WeldRadiusPx > 0;
        var hasPin = result.PoleRadiusPx > 0;

        // 检测可能是在等比缩小后的工作分辨率上做的，几何量要乘回原始图幅的比例
        var s = result.ProcessedSizePx.Width > 0
            ? bgr.Width / (double)result.ProcessedSizePx.Width
            : 1.0;
        var weldCenter = new Point2d(result.WeldCenterPx.X * s, result.WeldCenterPx.Y * s);
        var poleCenter = new Point2d(result.PoleCenterPx.X * s, result.PoleCenterPx.Y * s);

        // 测量失败时结果里的圆心/半径是**残值**（Fail 会把最后一次尝试的几何量带出来），
        // 用实线彩圈画出来等于告诉操作员"这是测出来的"——用户 2026-09-17 报的
        // "绿圈/红圈框选偏离"里有一部分正是这么被读成测量值的。
        // 所以失败帧改用**灰色虚线**：一眼能看出"这里没测准，去复核"。
        var failed = result.HasError;
        var ringColor = failed ? FailedGeometryColor : new Scalar(0, 0, 255);
        var innerColor = failed ? FailedGeometryColor : new Scalar(0, 165, 255);
        var pinColor = failed ? FailedGeometryColor : new Scalar(0, 255, 0);

        if (hasRing)
        {
            DrawCircle(img, weldCenter, (int)Math.Round(result.WeldRadiusPx * s), ringColor, thickness, failed);
            DrawCross(img, weldCenter, ringColor, crossArm, thickness);
        }
        // 熔核内沿：画出来操作员才看得到"环"的厚度（环宽 = 外半径 − 内半径）。
        // 用橙色细一档的线，与红色外圈区分开。
        //
        // **必须画在内沿**自己拟合出的圆心**上**（<see cref="PointInspectionResult.WeldInnerCenterPx"/>），
        // 不能画在焊环外边界圆心（weldCenter）上：用户 2026-09-17 指出"橙色圈要贴合焊环内边缘"，
        // 而实测内外圆心可以差到同心门限（0.05×焊环半径 ≈ 27 原始像素）——画在外圆心上，
        // 橙圈就会整体偏出去，看上去像"孔位把焊环中心带歪了"。内沿的圆心是本算法自己测出来的，
        // 结果里本来就带着，只是这里没用。
        if (result.WeldInnerRadiusPx > 0)
        {
            var innerCenter = new Point2d(result.WeldInnerCenterPx.X * s, result.WeldInnerCenterPx.Y * s);
            DrawCircle(img, innerCenter, (int)Math.Round(result.WeldInnerRadiusPx * s), innerColor,
                Math.Max(1, thickness - 2), failed);
            DrawCross(img, innerCenter, innerColor, Math.Max(4, crossArm / 2), Math.Max(1, thickness - 2));
        }
        if (hasPin)
        {
            DrawCircle(img, poleCenter, (int)Math.Round(result.PoleRadiusPx * s), pinColor, thickness, failed);
            DrawCross(img, poleCenter, pinColor, crossArm, thickness);
        }
        // 偏移向量只在**偏移量有效**时才画：测失败/未检出焊环时画这条黄线，
        // 等于在图上断言"这两个点之间就是这个偏移"。
        if (hasRing && hasPin && result.HasValidOffset)
            Cv2.Line(img, (Point)poleCenter, (Point)weldCenter,
                new Scalar(0, 255, 255), thickness);

        var (label, color) = BuildAnnotationLabel(result);
        AnnotationText.Draw(img, AnnotationText.Ellipsize(label, available, fontPx),
            new Point(textX, textY), fontPx, color, thickness);
        AnnotationText.Draw(img, AnnotationText.Ellipsize(result.Point.Name, available, fontPx),
            new Point(textX, textY + lineHeight), fontPx, color, thickness);

        // 批量识别（图片文件夹/视频）时，图上的文件名是"这个异常点对应哪张图"的唯一线索
        if (!string.IsNullOrWhiteSpace(result.Point.SourceName))
            AnnotationText.Draw(img, AnnotationText.Ellipsize($"源文件 {result.Point.SourceName}", available, fontPx),
                new Point(textX, textY + lineHeight * 2), fontPx, color, thickness);

        return img;
    }

    /// <summary>
    /// 标注图上的结论文字。异常只写"短原因"（完整诊断留给界面与报表）：
    /// 一行写不下的长句挤在图上必然被截断，反而不如告诉操作员"是什么问题"。
    /// </summary>
    private static (string Text, Scalar Color) BuildAnnotationLabel(PointInspectionResult result)
    {
        if (result.HasError)
            return ($"异常：{PointInspectionResult.ShortReason(result.ErrorMessage)}",
                new Scalar(0, 165, 255));
        if (result.IsOk)
            return ($"OK  D={result.OffsetDistanceMm:F3}mm", new Scalar(0, 200, 0));

        var offset = result.HasValidOffset
            ? $"D={result.OffsetDistanceMm:F3}mm"
            : "偏移量不适用（未检出焊环）";
        // 判定依据同样画到图上（短原因）：操作员看的就是这张图，
        // 只写"漏焊"而不写"覆盖率 62% < 75%"等于让他自己去猜。
        var reason = string.IsNullOrWhiteSpace(result.ErrorMessage)
            ? string.Empty
            : $"：{PointInspectionResult.ShortReason(result.ErrorMessage)}";
        return ($"NG（{WeldDefectText.Of(result.Defects)}{reason}）  {offset}", new Scalar(0, 0, 255));
    }

    private static void DrawCross(Mat img, Point2d center, Scalar color, int arm, int thickness)
    {
        var p = (Point)center;
        Cv2.Line(img, p with { X = p.X - arm }, p with { X = p.X + arm }, color, thickness, LineTypes.AntiAlias);
        Cv2.Line(img, p with { Y = p.Y - arm }, p with { Y = p.Y + arm }, color, thickness, LineTypes.AntiAlias);
    }

    /// <summary>
    /// 测量失败时几何量的画笔色（中灰）。与"红色焊环 / 橙色内沿 / 绿色极柱"形成明确区分：
    /// 灰虚线 = 残值、不可信；彩色实线 = 测出来的。
    /// </summary>
    private static readonly Scalar FailedGeometryColor = new(150, 150, 150);

    /// <summary>
    /// 画圆，可选虚线。OpenCV 没有虚线圆原语，用椭圆的圆弧分段拼（48 段、隔段画）。
    /// </summary>
    private static void DrawCircle(Mat img, Point2d center, int radius, Scalar color,
        int thickness, bool dashed)
    {
        if (radius <= 0)
            return;
        if (!dashed)
        {
            Cv2.Circle(img, (Point)center, radius, color, thickness);
            return;
        }

        const int segments = 48;
        var spanDeg = 360.0 / segments;
        for (var i = 0; i < segments; i += 2)
        {
            Cv2.Ellipse(img, (Point)center, new Size(radius, radius), 0,
                i * spanDeg, (i + 1) * spanDeg, color, thickness, LineTypes.AntiAlias);
        }
    }
}
