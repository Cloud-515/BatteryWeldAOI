using System.Runtime.CompilerServices;

// 开发期诊断工具 tools/Diag 需要直接调用算法内部步骤（WeldRingLocator / Geo / HoleLocator 等），
// 以便在"某一帧为什么测不出来"时打印各条路径的中间量——只靠公开 API 只能拿到最终结论，
// 看不出是主路径零候选、核宽阶梯未落回族值带，还是第三路径被有效性上限拦下。
// 该授权只影响可见性，不改变任何运行时行为。
[assembly: InternalsVisibleTo("Diag")]
