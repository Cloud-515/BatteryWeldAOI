namespace BatteryWeldAOI.Core.StateMachine;

using System.Collections.Concurrent;
using BatteryWeldAOI.Core.Models;

/// <summary>
/// 设备主状态机：所有状态跳转必须经过本类校验，
/// 防止在错误状态下触发轴运动导致撞机。
/// </summary>
public sealed class MachineStateMachine
{
    private static readonly IReadOnlyDictionary<MachineState, MachineState[]> Transitions =
        new Dictionary<MachineState, MachineState[]>
        {
            [MachineState.Idle] = new[] { MachineState.Moving, MachineState.Error, MachineState.Stopped },
            [MachineState.Moving] = new[] { MachineState.Inspecting, MachineState.Error, MachineState.Stopped },
            [MachineState.Inspecting] = new[] { MachineState.Moving, MachineState.Finished, MachineState.Error, MachineState.Stopped },
            [MachineState.Error] = new[] { MachineState.Idle },
            [MachineState.Stopped] = new[] { MachineState.Idle },
            [MachineState.Finished] = new[] { MachineState.Idle }
        };

    private readonly object _lock = new();
    private readonly ConcurrentQueue<(MachineState From, MachineState To, DateTime Ts)> _history = new();

    public event Action<MachineState, MachineState>? StateChanged;

    public MachineState Current { get; private set; } = MachineState.Idle;

    public bool IsTerminal => Current is MachineState.Error or MachineState.Stopped;

    /// <summary>尝试跳转；非法跳转返回 false（不抛异常，由调用方决定是否升级为 Error）。</summary>
    public bool TryTransition(MachineState target)
    {
        lock (_lock)
        {
            if (!Transitions[Current].Contains(target))
                return false;

            var from = Current;
            Current = target;
            _history.Enqueue((from, target, DateTime.UtcNow));
            StateChanged?.Invoke(from, target);
            return true;
        }
    }

    /// <summary>急停：任意状态均可进入 Stopped。</summary>
    public void EmergencyStop()
    {
        lock (_lock)
        {
            if (Current == MachineState.Stopped)
                return;
            var from = Current;
            Current = MachineState.Stopped;
            _history.Enqueue((from, MachineState.Stopped, DateTime.UtcNow));
            StateChanged?.Invoke(from, MachineState.Stopped);
        }
    }

    /// <summary>复位：从 Error/Stopped/Finished 回到 Idle。</summary>
    public bool TryReset() => TryTransition(MachineState.Idle);

    public IReadOnlyList<(MachineState From, MachineState To, DateTime Ts)> GetHistory()
    {
        lock (_lock)
        {
            return _history.ToArray();
        }
    }

    /// <summary>判断状态跳转是否合法（供测试与外部诊断使用）。</summary>
    public static bool IsValidTransition(MachineState from, MachineState to) =>
        Transitions.TryGetValue(from, out var targets) && targets.Contains(to);
}
