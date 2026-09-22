namespace BatteryWeldAOI.Core.Communication;

using System.Net.Sockets;
using System.Text;
using BatteryWeldAOI.Core.Models;

/// <summary>
/// 基于 TCP 的 PLC 客户端实现，与 <see cref="PlcSimulator"/> 对接。
/// 所有交互均为异步，避免阻塞 UI / 流程线程。
/// 内置两类超时保护：单指令应答超时（检测断线/假死）与到位等待超时（检测轴卡死）。
/// </summary>
public sealed class TcpPlcClient : IPlcClient
{
    private readonly string _host;
    private readonly int _port;
    private readonly int _pollIntervalMs;
    private readonly int _arrivalTimeoutMs;
    private readonly int _responseTimeoutMs;
    private TcpClient? _client;
    private StreamReader? _reader;
    private StreamWriter? _writer;
    private readonly SemaphoreSlim _ioLock = new(1, 1);

    public TcpPlcClient(string host, int port, int pollIntervalMs = 10,
        int arrivalTimeoutMs = 10_000, int responseTimeoutMs = 5_000)
    {
        _host = host;
        _port = port;
        _pollIntervalMs = pollIntervalMs;
        _arrivalTimeoutMs = arrivalTimeoutMs;
        _responseTimeoutMs = responseTimeoutMs;
    }

    public bool IsConnected => _client?.Connected == true;

    public async Task ConnectAsync(CancellationToken ct)
    {
        _client = new TcpClient();
        await _client.ConnectAsync(_host, _port, ct);
        var stream = _client.GetStream();
        _reader = new StreamReader(stream, Encoding.UTF8);
        _writer = new StreamWriter(stream, Encoding.UTF8) { AutoFlush = true };

        // 握手：心跳
        await SendAndExpectAsync(PlcProtocol.Ping, PlcProtocol.Pong, ct);
    }

    public Task MoveToAsync(int pointIndex, CancellationToken ct)
        => SendAndExpectAsync($"{PlcProtocol.MoveTo} {pointIndex}", PlcProtocol.AckOk, ct);

    public async Task WaitForArrivalAsync(int pointIndex, CancellationToken ct)
    {
        var deadline = DateTime.UtcNow.AddMilliseconds(_arrivalTimeoutMs);
        while (!ct.IsCancellationRequested)
        {
            if (DateTime.UtcNow > deadline)
                throw new TimeoutException(
                    $"点位 {pointIndex} 等待到位信号超时({_arrivalTimeoutMs}ms)，轴可能卡死或信号丢失");

            string reply;
            await _ioLock.WaitAsync(ct);
            try
            {
                await _writer!.WriteLineAsync(PlcProtocol.Read);
                reply = await ReadLineWithTimeoutAsync();
            }
            finally
            {
                _ioLock.Release();
            }

            var parts = reply.Split(' ');
            if (parts.Length == 2 && parts[0] == PlcProtocol.StatusArrived && int.Parse(parts[1]) == pointIndex)
                return;

            await Task.Delay(_pollIntervalMs, ct);
        }

        throw new OperationCanceledException("等待到位信号被取消");
    }

    public Task SendResultAsync(PointInspectionResult result, CancellationToken ct)
    {
        var inv = System.Globalization.CultureInfo.InvariantCulture;
        var payload = result.IsOk ? "-" : result.DefectText;
        var line = $"{PlcProtocol.Result} {result.Point.Index} {(result.IsOk ? "OK" : "NG")} " +
                   $"{result.OffsetXmm.ToString("F4", inv)} {result.OffsetYmm.ToString("F4", inv)} {payload}";
        return SendAndExpectAsync(line, PlcProtocol.AckOk, ct);
    }

    public Task FinishAsync(CancellationToken ct)
        => SendAndExpectAsync(PlcProtocol.Finish, PlcProtocol.Done, ct);

    private async Task SendAndExpectAsync(string command, string expectedReply, CancellationToken ct)
    {
        await _ioLock.WaitAsync(ct);
        try
        {
            await _writer!.WriteLineAsync(command);
            var reply = await ReadLineWithTimeoutAsync();
            if (reply != expectedReply)
                throw new IOException($"PLC 应答异常: 期望 '{expectedReply}', 实际 '{reply}'");
        }
        finally
        {
            _ioLock.Release();
        }
    }

    /// <summary>带超时的同步读行：外部未取消却先超时，说明链路假死/断线。</summary>
    private async Task<string> ReadLineWithTimeoutAsync()
    {
        using var timeoutCts = new CancellationTokenSource(_responseTimeoutMs);
        try
        {
            return await _reader!.ReadLineAsync(timeoutCts.Token) ?? string.Empty;
        }
        catch (OperationCanceledException)
        {
            throw new TimeoutException($"PLC 应答超时({_responseTimeoutMs}ms)，连接可能已断开");
        }
    }

    public void Dispose()
    {
        _reader?.Dispose();
        _writer?.Dispose();
        _client?.Dispose();
        _ioLock.Dispose();
    }
}
