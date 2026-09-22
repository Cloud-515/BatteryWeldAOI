namespace BatteryWeldAOI.Core.Communication;

using System.Net;
using System.Net.Sockets;
using System.Text;

/// <summary>
/// PLC 模拟器：以 TCP 服务形式在本地运行，模拟下位机行为——
/// 接收运动指令、按设定耗时"运动"、置位到位信号、接收检测结果。
/// 用于在无真实 PLC/运动控制卡的开发环境里跑通整套 Stop-and-Go 流程。
/// </summary>
public sealed class PlcSimulator : IDisposable
{
    private readonly int _moveTimeMs;
    private readonly int _listenBacklogPort;
    private TcpListener? _listener;
    private CancellationTokenSource? _cts;
    private Task? _acceptTask;

    private readonly object _stateLock = new();
    private int _targetIndex = -1;
    private bool _arrived = true;
    private int _lastReportedIndex = -1;

    public event Action<string>? Log;

    public int Port { get; private set; }

    /// <summary>收到的检测结果日志（index, ok, dx, dy, defects)。</summary>
    public event Action<int, bool, double, double, string>? ResultReceived;

    public PlcSimulator(int port = 0, int moveTimeMs = 60)
    {
        _listenBacklogPort = port;
        _moveTimeMs = moveTimeMs;
    }

    public void Start()
    {
        _cts = new CancellationTokenSource();
        _listener = new TcpListener(IPAddress.Loopback, _listenBacklogPort);
        _listener.Start();
        Port = ((IPEndPoint)_listener.LocalEndpoint).Port;
        Log?.Invoke($"[PLC] 模拟器启动, 监听 127.0.0.1:{Port}, 单轴运动耗时 {_moveTimeMs}ms");
        _acceptTask = Task.Run(() => AcceptLoopAsync(_cts.Token));
    }

    private async Task AcceptLoopAsync(CancellationToken ct)
    {
        while (!ct.IsCancellationRequested)
        {
            TcpClient client;
            try
            {
                client = await _listener!.AcceptTcpClientAsync(ct);
            }
            catch (OperationCanceledException)
            {
                break;
            }
            catch (SocketException)
            {
                break;
            }

            Log?.Invoke("[PLC] 上位机已连接");
            _ = HandleClientAsync(client, ct);
        }
    }

    private async Task HandleClientAsync(TcpClient client, CancellationToken ct)
    {
        using (client)
        {
            var stream = client.GetStream();
            var reader = new StreamReader(stream, Encoding.UTF8);
            var writer = new StreamWriter(stream, Encoding.UTF8) { AutoFlush = true };

            var moveCts = CancellationTokenSource.CreateLinkedTokenSource(ct);
            var currentMove = Task.CompletedTask;

            try
            {
                while (!ct.IsCancellationRequested && client.Connected)
                {
                    var line = await reader.ReadLineAsync(ct);
                    if (line is null)
                        break;

                    var parts = line.Split(' ');
                    switch (parts[0])
                    {
                        case PlcProtocol.Ping:
                            await writer.WriteLineAsync(PlcProtocol.Pong);
                            break;

                        case PlcProtocol.MoveTo when parts.Length == 2 && int.TryParse(parts[1], out var idx):
                            lock (_stateLock)
                            {
                                _targetIndex = idx;
                                _arrived = false;
                            }
                            // 取消上一次未完成的运动（正常流程不会出现，防御处理）
                            moveCts.Cancel();
                            moveCts.Dispose();
                            moveCts = CancellationTokenSource.CreateLinkedTokenSource(ct);
                            var token = moveCts.Token;
                            var target = idx;
                            currentMove = Task.Run(async () =>
                            {
                                try
                                {
                                    await Task.Delay(_moveTimeMs, token);
                                    lock (_stateLock)
                                    {
                                        _arrived = true;
                                        _lastReportedIndex = target;
                                    }
                                    Log?.Invoke($"[PLC] 轴已到位: 点位 {target}");
                                }
                                catch (OperationCanceledException)
                                {
                                    // 运动被新指令覆盖
                                }
                            }, token);
                            await writer.WriteLineAsync(PlcProtocol.AckOk);
                            break;

                        case PlcProtocol.Read:
                            int targetIdx;
                            bool arrived;
                            lock (_stateLock)
                            {
                                targetIdx = _targetIndex;
                                arrived = _arrived && _lastReportedIndex == _targetIndex;
                            }
                            await writer.WriteLineAsync(arrived
                                ? $"{PlcProtocol.StatusArrived} {targetIdx}"
                                : $"{PlcProtocol.StatusMoving} {targetIdx}");
                            break;

                        case PlcProtocol.Result when parts.Length >= 4:
                            var rIdx = int.Parse(parts[1]);
                            var ok = parts[2] == "OK";
                            var dx = double.Parse(parts[3], System.Globalization.CultureInfo.InvariantCulture);
                            var dy = double.Parse(parts.Length > 4 ? parts[4] : "0", System.Globalization.CultureInfo.InvariantCulture);
                            var defects = parts.Length > 5 ? parts[5] : string.Empty;
                            ResultReceived?.Invoke(rIdx, ok, dx, dy, defects);
                            Log?.Invoke($"[PLC] 收到结果: 点位 {rIdx} = {(ok ? "OK" : "NG")}");
                            await writer.WriteLineAsync(PlcProtocol.AckOk);
                            break;

                        case PlcProtocol.Finish:
                            await writer.WriteLineAsync(PlcProtocol.Done);
                            Log?.Invoke("[PLC] 收到整包完成信号, 流程结束");
                            break;

                        default:
                            await writer.WriteLineAsync("ERR UNKNOWN_CMD");
                            break;
                    }
                }
            }
            catch (IOException)
            {
                // 客户端断开
            }
            catch (OperationCanceledException)
            {
                // 服务关闭
            }
            finally
            {
                moveCts.Cancel();
                moveCts.Dispose();
            }

            Log?.Invoke("[PLC] 上位机已断开");
        }
    }

    public void Dispose()
    {
        _cts?.Cancel();
        try { _listener?.Stop(); } catch (SocketException) { }
        try { _acceptTask?.Wait(TimeSpan.FromSeconds(2)); } catch (AggregateException) { }
        _cts?.Dispose();
    }
}
