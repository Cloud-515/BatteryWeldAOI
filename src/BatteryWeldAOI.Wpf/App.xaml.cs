namespace BatteryWeldAOI.Wpf;

using System.IO;
using System.Windows;

/// <summary>应用程序入口：附带 UI 线程异常兜底，崩溃时落盘日志并提示，而不是静默消失。</summary>
public partial class App : Application
{
    protected override void OnStartup(StartupEventArgs e)
    {
        base.OnStartup(e);
        DispatcherUnhandledException += (_, args) =>
        {
            try
            {
                var dir = Path.Combine(AppContext.BaseDirectory, "output");
                Directory.CreateDirectory(dir);
                File.AppendAllText(Path.Combine(dir, "crash.log"),
                    $"[{DateTime.Now:yyyy-MM-dd HH:mm:ss}] {args.Exception}\r\n\r\n");
            }
            catch
            {
                // 崩溃日志自身失败时不再抛出
            }

            MessageBox.Show($"发生未处理异常，程序将继续运行：\n\n{args.Exception.Message}",
                "BatteryWeldAOI", MessageBoxButton.OK, MessageBoxImage.Warning);
            args.Handled = true;
        };
    }
}
