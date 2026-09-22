using System.Net;
using System.Net.Sockets;
using BatteryWeldAOI.Core.Communication;
using BatteryWeldAOI.Core.Models;
using Xunit;

namespace BatteryWeldAOI.Core.Tests;

/// <summary>
/// PLC 通信超时保护测试：轴卡死（到位超时）与链路假死（应答超时）
/// 都必须以明确的 TimeoutException 中止，而不是无限挂起。
/// </summary>
public class TcpPlcClientTimeoutTests
{
    [Fact]
    public async Task WaitForArrival_StuckAxis_ThrowsTimeout()
    {
        // 模拟轴卡死：运动耗时远大于到位等待超时
        using var simulator = new PlcSimulator(port: 0, moveTimeMs: 60_000);
        simulator.Start();

        using var client = new TcpPlcClient("127.0.0.1", simulator.Port,
            pollIntervalMs: 10, arrivalTimeoutMs: 200, responseTimeoutMs: 2_000);
        await client.ConnectAsync(CancellationToken.None);
        await client.MoveToAsync(0, CancellationToken.None);

        var ex = await Assert.ThrowsAsync<TimeoutException>(
            () => client.WaitForArrivalAsync(0, CancellationToken.None));
        Assert.Contains("到位", ex.Message);
    }

    [Fact]
    public async Task DeadLink_NoResponse_ThrowsTimeout()
    {
        // 模拟链路假死：对端接受连接但从不应答
        var listener = new TcpListener(IPAddress.Loopback, 0);
        listener.Start();
        using var silent = new TcpClient();
        _ = Task.Run(async () =>
        {
            using var accepted = await listener.AcceptTcpClientAsync();
            using var stream = accepted.GetStream();
            var buf = new byte[256];
            while (await stream.ReadAsync(buf) > 0) { /* 读而不答 */ }
        });
        await silent.ConnectAsync(IPAddress.Loopback, ((IPEndPoint)listener.LocalEndpoint).Port);

        using var client = new TcpPlcClient("127.0.0.1", ((IPEndPoint)listener.LocalEndpoint).Port,
            pollIntervalMs: 10, arrivalTimeoutMs: 5_000, responseTimeoutMs: 200);

        var ex = await Assert.ThrowsAsync<TimeoutException>(
            () => client.ConnectAsync(CancellationToken.None));
        Assert.Contains("应答超时", ex.Message);

        listener.Stop();
        silent.Dispose();
    }

    [Fact]
    public async Task SendResult_ErrorPoint_PayloadCarriesErrorText()
    {
        // 处理异常点位的结果上传：NG + 错误描述应完整送达模拟器
        using var simulator = new PlcSimulator(port: 0, moveTimeMs: 10);
        var received = new TaskCompletionSource<(int Index, string Payload)>();
        simulator.ResultReceived += (idx, ok, _, _, defects) =>
            received.TrySetResult((idx, defects));
        simulator.Start();

        using var client = new TcpPlcClient("127.0.0.1", simulator.Port,
            pollIntervalMs: 10, arrivalTimeoutMs: 2_000, responseTimeoutMs: 2_000);
        await client.ConnectAsync(CancellationToken.None);

        var result = new PointInspectionResult
        {
            Point = new WeldPoint { Index = 5, Name = "Cell02-NEG" },
            HasError = true,
            ErrorMessage = "图像损坏"
        };
        await client.SendResultAsync(result, CancellationToken.None);

        var (index, payload) = await received.Task.WaitAsync(TimeSpan.FromSeconds(5));
        Assert.Equal(5, index);
        Assert.Contains("ERROR", payload);
        Assert.Contains("图像损坏", payload);
    }
}
