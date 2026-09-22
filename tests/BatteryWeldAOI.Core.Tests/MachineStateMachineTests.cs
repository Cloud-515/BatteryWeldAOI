using BatteryWeldAOI.Core.Models;
using BatteryWeldAOI.Core.StateMachine;
using Xunit;

namespace BatteryWeldAOI.Core.Tests;

public class MachineStateMachineTests
{
    [Fact]
    public void Idle_To_Moving_Succeeds()
    {
        var sm = new MachineStateMachine();
        Assert.True(sm.TryTransition(MachineState.Moving));
        Assert.Equal(MachineState.Moving, sm.Current);
    }

    [Fact]
    public void Idle_To_Inspecting_IsRejected()
    {
        var sm = new MachineStateMachine();
        Assert.False(sm.TryTransition(MachineState.Inspecting));
        Assert.Equal(MachineState.Idle, sm.Current);
    }

    [Fact]
    public void Full_Inspection_Cycle_Transitions()
    {
        var sm = new MachineStateMachine();
        Assert.True(sm.TryTransition(MachineState.Moving));
        Assert.True(sm.TryTransition(MachineState.Inspecting));
        Assert.True(sm.TryTransition(MachineState.Moving));
        Assert.True(sm.TryTransition(MachineState.Inspecting));
        Assert.True(sm.TryTransition(MachineState.Finished));
        Assert.True(sm.TryReset());
        Assert.Equal(MachineState.Idle, sm.Current);
    }

    [Fact]
    public void Finished_To_Moving_IsRejected()
    {
        var sm = new MachineStateMachine();
        sm.TryTransition(MachineState.Moving);
        sm.TryTransition(MachineState.Inspecting);
        sm.TryTransition(MachineState.Finished);
        Assert.False(sm.TryTransition(MachineState.Moving));
    }

    [Fact]
    public void EmergencyStop_From_Inspecting_Enters_Stopped()
    {
        var sm = new MachineStateMachine();
        sm.TryTransition(MachineState.Moving);
        sm.TryTransition(MachineState.Inspecting);
        sm.EmergencyStop();
        Assert.Equal(MachineState.Stopped, sm.Current);
        Assert.True(sm.IsTerminal);
    }

    [Fact]
    public void Error_Requires_Reset_Before_Running()
    {
        var sm = new MachineStateMachine();
        sm.TryTransition(MachineState.Error);
        Assert.False(sm.TryTransition(MachineState.Moving), "Error 状态下不允许直接运动，必须先复位");
        Assert.True(sm.TryReset());
        Assert.True(sm.TryTransition(MachineState.Moving));
    }

    [Fact]
    public void StateChanged_Event_Fires()
    {
        var sm = new MachineStateMachine();
        MachineState? from = null, to = null;
        sm.StateChanged += (f, t) => { from = f; to = t; };
        sm.TryTransition(MachineState.Moving);
        Assert.Equal(MachineState.Idle, from);
        Assert.Equal(MachineState.Moving, to);
    }
}
