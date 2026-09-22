# tests 目录说明

## 语料与体积

| 目录 | 内容 | 体积 | 是否入库 |
|---|---|---|---|
| `pp/` | **现场厂商原始数据**：厂商导出的 jpg（5472×3648）+ 私有格式 `.vdb`（约 960 帧 × 40 SN 的样本 + `VdbGui_pic/`） | **约 30.5 GB** | **否**（见 `.gitignore`） |
| `tp/` | 早期旧相机小样本（182~508px） | 约 6.7 MB | 是 |
| `test/` | 其它小样本 | 约 4.6 MB | 是 |
| `BatteryWeldAOI.Core.Tests/` | xUnit 测试工程 | 源码入库（`bin/`、`obj/` 不入库） | 是 |

## `pp/` 缺失时的行为

**现场语料类测试在样本缺失时会自动跳过**（不是失败）。相关测试的构造方式是：

```csharp
var file = FindSample("22-59-49 478", "_OK_441", "SN_...0004...");
if (file is null)
{
    _output.WriteLine("未找到现场样本 tests/pp，跳过。");
    return;   // 跳过，不算失败
}
```

因此**克隆本仓库后直接 `dotnet test` 可以全绿**（除少数**合成夹具的既有失败**，见下），
只是少跑现场语料那一部分。受影响的测试类：

- `PpSampleTests` / `PpLocatorAgreementTests` / `PpBatchRegressionTests`（定位一致性与批量回归）
- `NgSampleTests`（厂商 NG 帧的行为契约）
- `RingCredibilityGateTests`（焊环可信度门限的现场回归）
- `WeldRingBoundaryTests` / `RingWidthTests` / `OffsetScaleTests`
- `HoleTruthTests`（用户手绘标注反解出的孔位真值）
- `RealSampleTests`（tests/tp）

## 需要这批数据时怎么放

把厂商导出的目录原样放到 `tests/pp/` 下即可，结构照现场导出：

```
tests/pp/
  2025-09-11/
    Orignal_pic/<SN>/<时刻>_Scene1Pos0_index1_<SN>_OK_xxx.jpg   # 厂商导出图（注：实为 GUI 渲染输出）
    VdbGui_pic/<SN>/<同名>.vdb                                  # 厂商私有格式，内含其圆定位结果
```

几个必须知道的坑（细节见 `README.md` 的"现场厂商数据"与 `FIXPLAN.md`）：

1. **`Orignal_pic` 里的 jpg 不是原图**——是厂商 GUI 的渲染输出，带叠加标注、JPEG 质量 1
   （0.105 bit/px）。拿它做亚像素测量或训练前必须先把这个域差算进去。
2. **厂商的判定值不在 `.vdb` 里**：`焊偏距离` 是 0.000 模板占位（20955 个 Pos0 文件中 20954 个如此），
   也没有 OK/NG 字段。判据与公差**无法从这批数据反推**。
3. **`一个 SN 目录 = 一条线上的 24 个零件**（不是同一零件的重复拍摄）**——
   所以同一 SN 内的偏移量跨度是**真实的零件间差异**，不能当测量误差用。
4. 厂商圆可用作**定位精度基准**，但要记住它自身复现性只有 **24.4px**（厂商在同一张图上
   换位置标记的圆心互差中位），而本项目与它的中位差是 **2.7px**——即我们已优于厂商自身的复现能力。
   **不要把它当成"逐帧真值"**（`FIXPLAN.md` 第 15 节有一次因此差点判错的记录）。
5. 现场语料测试可用环境变量放大范围：`BWAOI_PP_AGREE=<SN数>`（默认 8）、`BWAOI_PP_AGREE=0` 跳过。

## 已知的既有失败（与本仓库代码无关）

`WeldInspectionAlgorithmTests` 与 `RealisticRobustnessTests`（两套**合成夹具**）里有约 10 例
长期失败，`ImagePipelineTests` 有 1 例——合计 **11 例**。它们在 `FIXPLAN.md` 第 5.7 / 6.8 / 7.6 节
有逐条记录，成因是合成夹具的精度问题（多为"偏移量测偏"或"OK 点被误判 Misaligned"），
与现场算法链路无关。**基线是 11 例失败，不是 0。**
