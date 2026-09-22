namespace BatteryWeldAOI.Core.Pipeline;

using System.Collections.Concurrent;
using BatteryWeldAOI.Core.Models;
using BatteryWeldAOI.Core.Vision;
using OpenCvSharp;

/// <summary>
/// 图像处理流水线（生产者-消费者）：
/// 运动控制线程只负责"到位->拍照->入队"，图像处理由独立的 worker 池消费，
/// 两者互不阻塞——这是保证节拍的关键架构。有界队列在处理积压时自动反压。
/// </summary>
public sealed class ImagePipeline : IDisposable
{
    private readonly BlockingCollection<(WeldPoint Point, Mat Image)> _queue =
        new(boundedCapacity: 8);
    private readonly List<Task> _workers = new();
    private readonly WeldInspectionAlgorithm _algorithm;
    private readonly CancellationTokenSource _cts = new();

    /// <summary>
    /// 单点结果事件（在 worker 线程触发）。
    /// 参数二为标注图，事件返回后由流水线统一释放，订阅方不得持有引用。
    /// </summary>
    public event Action<PointInspectionResult, Mat>? ResultProcessed;

    public ImagePipeline(WeldInspectionAlgorithm algorithm, int workerCount = 2)
    {
        _algorithm = algorithm;
        for (var i = 0; i < workerCount; i++)
        {
            var workerId = i;
            _workers.Add(Task.Run(() => WorkerLoopAsync(workerId)));
        }
    }

    /// <summary>生产者入队（运动线程调用）；队列满时阻塞形成反压。</summary>
    public void Enqueue(WeldPoint point, Mat image) => _queue.Add((point, image));

    /// <summary>停止接收新图像；已入队图像会被处理完毕。</summary>
    public void CompleteAdding() => _queue.CompleteAdding();

    /// <summary>等待全部 worker 退出（即所有图像处理完毕）。</summary>
    public Task Completion => Task.WhenAll(_workers);

    private void WorkerLoopAsync(int workerId)
    {
        try
        {
            foreach (var (point, image) in _queue.GetConsumingEnumerable(_cts.Token))
            {
                using (image)
                {
                    PointInspectionResult result;
                    Mat annotated;
                    try
                    {
                        result = _algorithm.Inspect(image, point);
                        annotated = _algorithm.Annotate(image, result);
                    }
                    catch (Exception ex)
                    {
                        // 单点算法异常不击穿 worker：降级为错误结果，其余点位继续检测
                        result = new PointInspectionResult
                        {
                            Point = point,
                            HasError = true,
                            ErrorMessage = ex.Message
                        };
                        annotated = RenderErrorImage(image, point, ex);
                    }

                    using (annotated)
                    {
                        ResultProcessed?.Invoke(result, annotated);
                    }
                }
            }
        }
        catch (OperationCanceledException)
        {
            // Dispose 触发的退出
        }
    }

    /// <summary>
    /// 异常点位的标注图：优先在原图上写错误信息；原图本身不可用时退化为黑底提示图。
    /// 文字走 <see cref="AnnotationText"/>（GDI+ 中文渲染）——异常原因是中文，
    /// 用 <c>Cv2.PutText</c> 只会画出一串 "?"，异常图等于没有信息。
    /// </summary>
    private static Mat RenderErrorImage(Mat source, WeldPoint point, Exception ex)
    {
        var label = $"异常：{PointInspectionResult.ShortReason(ex.Message)}";
        try
        {
            var img = source.Clone();
            DrawErrorText(img, label, point, fontPx: Math.Max(13.0, Math.Min(img.Width, img.Height) * 0.030));
            return img;
        }
        catch (Exception)
        {
            var fallback = new Mat(480, 640, MatType.CV_8UC3, Scalar.Black);
            DrawErrorText(fallback, label, point, fontPx: 18);
            return fallback;
        }
    }

    private static void DrawErrorText(Mat img, string label, WeldPoint point, double fontPx)
    {
        var x = Math.Max(4, (int)Math.Round(Math.Min(img.Width, img.Height) * 0.020));
        var y = Math.Max((int)Math.Round(fontPx * 1.25), (int)Math.Round(Math.Min(img.Width, img.Height) * 0.043));
        var lineHeight = (int)Math.Round(AnnotationText.LineHeight(fontPx));
        var color = new Scalar(0, 0, 255);
        var available = Math.Max(40, img.Width - x * 2);

        AnnotationText.Draw(img, AnnotationText.Ellipsize(label, available, fontPx),
            new Point(x, y), fontPx, color);
        AnnotationText.Draw(img, AnnotationText.Ellipsize(point.Name, available, fontPx),
            new Point(x, y + lineHeight), fontPx, color);
    }

    public void Dispose()
    {
        _queue.CompleteAdding();
        _cts.Cancel();
        try { Task.WhenAll(_workers).Wait(TimeSpan.FromSeconds(5)); } catch (AggregateException) { }
        _cts.Dispose();
        _queue.Dispose();
    }
}
