#include "HistogramWidget.h"
#include <QPainter>
#include <QHBoxLayout>

template <class T>
static QVector<qreal> getBufferLevels(const T *buffer, int frames, int channels);

/**
 * 获取音频格式的最大峰值值。
 *
 * 此函数根据给定的QAudioFormat对象参数，计算并返回一个表示该音频格式下理论上的最大值。
 * 主要用于支持PCM格式的音频，对非PCM格式或无效的格式，函数将返回0。
 *
 * @param format QAudioFormat对象，包含待检查的音频格式信息。
 * @return 返回一个qreal类型值，表示音频格式的最大峰值。对于不支持的格式或无效的参数，返回0。
 */
qreal getPeakValue(const QAudioFormat &format)
{
    // 检查音频格式是否有效
    if (!format.isValid())
        return qreal(0);

    // 检查音频编码是否为PCM
    if (format.codec() != "audio/pcm")
        return qreal(0);

    // 根据样本类型计算峰值值
    switch (format.sampleType())
    {
    case QAudioFormat::Unknown:
        break;
    case QAudioFormat::Float:
        // 对于浮点样本，只支持32位，且返回一个略大于1的值
        if (format.sampleSize() != 32)
            return qreal(0);
        return qreal(1.00003);
    case QAudioFormat::SignedInt:
        // 对于有符号整数样本，根据样本大小返回相应的最大值
        if (format.sampleSize() == 32)
            return qreal(INT_MAX);
        if (format.sampleSize() == 16)
            return qreal(SHRT_MAX);
        if (format.sampleSize() == 8)
            return qreal(CHAR_MAX);
        break;
    case QAudioFormat::UnSignedInt:
        // 对于无符号整数样本，根据样本大小返回相应的最大值
        if (format.sampleSize() == 32)
            return qreal(UINT_MAX);
        if (format.sampleSize() == 16)
            return qreal(USHRT_MAX);
        if (format.sampleSize() == 8)
            return qreal(UCHAR_MAX);
        break;
    }

    // 如果没有匹配到任何已知情况，返回0
    return qreal(0);
}
template <class T>
/**
 * 获取缓冲区中每个通道的最大值。
 *
 * @param buffer 指向音频帧数据的指针，数据类型为T，假设为原始音频样本。
 * @param frames 音频帧的数量。
 * @param channels 音频的通道数。
 * @return QVector<qreal> 返回一个包含每个通道最大值的向量。
 */
QVector<qreal> getBufferLevels(const T *buffer, int frames, int channels)
{
    // 初始化一个向量来保存每个通道的最大值，初始值为0。
    QVector<qreal> max_values;
    max_values.fill(0, channels);

    // 遍历所有帧
    for (int i = 0; i < frames; ++i)
    {
        // 遍历当前帧中的每个通道
        for (int j = 0; j < channels; ++j)
        {
            // 计算当前样本的绝对值
            qreal value = qAbs(qreal(buffer[i * channels + j]));
            // 如果当前样本值大于当前通道的最大值，则更新最大值
            if (value > max_values.at(j))
                max_values.replace(j, value);
        }
    }

    return max_values;
}

/**
 * 获取音频缓冲区的电平值。
 *
 * 该函数分析给定的QAudioBuffer对象，计算每个通道的峰值电平，并返回一个包含每个通道当前电平值的向量。
 * 电平值是相对于缓冲区中找到的峰值电平的标准化值，使得缓冲区中的最大值为1。
 *
 * @param buffer QAudioBuffer对象，包含要分析的音频数据。
 * @return QVector<qreal> 包含每个通道电平值的向量。如果无法分析缓冲区，则返回空向量。
 */

QVector<qreal> getBufferLevels(const QAudioBuffer &buffer)
{
    QVector<qreal> values;

    // 如果缓冲区无效，则直接返回空向量
    if (!buffer.isValid())
        return values;

    // 检查音频格式是否有效，且是否为小端序
    if (!buffer.format().isValid() || buffer.format().byteOrder() != QAudioFormat::LittleEndian)
        return values;

    // 检查音频编解码器是否为PCM
    if (buffer.format().codec() != "audio/pcm")
        return values;

    int channelCount = buffer.format().channelCount();
    values.fill(0, channelCount);
    qreal peak_value = getPeakValue(buffer.format());
    // 如果无法计算峰值电平，则返回空向量
    if (qFuzzyCompare(peak_value, qreal(0)))
        return values;

    // 根据样本类型和大小，计算每个通道的电平值
    switch (buffer.format().sampleType())
    {
    case QAudioFormat::Unknown:
    case QAudioFormat::UnSignedInt:
        // 处理无符号整型样本，支持32位、16位和8位
        if (buffer.format().sampleSize() == 32)
            values = getBufferLevels(buffer.constData<quint32>(), buffer.frameCount(), channelCount);
        if (buffer.format().sampleSize() == 16)
            values = getBufferLevels(buffer.constData<quint16>(), buffer.frameCount(), channelCount);
        if (buffer.format().sampleSize() == 8)
            values = getBufferLevels(buffer.constData<quint8>(), buffer.frameCount(), channelCount);
        // 标准化电平值
        for (int i = 0; i < values.size(); ++i)
            values[i] = qAbs(values.at(i) - peak_value / 2) / (peak_value / 2);
        break;
    case QAudioFormat::Float:
        // 处理浮点型样本，支持32位
        if (buffer.format().sampleSize() == 32)
        {
            values = getBufferLevels(buffer.constData<float>(), buffer.frameCount(), channelCount);
            // 标准化电平值
            for (int i = 0; i < values.size(); ++i)
                values[i] /= peak_value;
        }
        break;
    case QAudioFormat::SignedInt:
        // 处理有符号整型样本，支持32位、16位和8位
        if (buffer.format().sampleSize() == 32)
            values = getBufferLevels(buffer.constData<qint32>(), buffer.frameCount(), channelCount);
        if (buffer.format().sampleSize() == 16)
            values = getBufferLevels(buffer.constData<qint16>(), buffer.frameCount(), channelCount);
        if (buffer.format().sampleSize() == 8)
            values = getBufferLevels(buffer.constData<qint8>(), buffer.frameCount(), channelCount);
        // 标准化电平值
        for (int i = 0; i < values.size(); ++i)
            values[i] /= peak_value;
        break;
    }

    return values;
}

QAudioLevel::QAudioLevel(QWidget *parent)
    : QWidget(parent)
{
}

void QAudioLevel::setLevel(qreal level)
{
    if (m_level != level)
    {
        m_level = level;
        update();
    }
}

void QAudioLevel::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);
    QPainter painter(this);
    // 渐变色，改为垂直方向
    QLinearGradient gradient(0, height(), 0, 0);
    int hue = static_cast<int>(m_level * 360.0);
    gradient.setColorAt(m_level, QColor::fromHsl(hue, 255, 127));
    // 定义每个小矩形的间距
    const int padding = 1;
    // 定义总共有多少个小矩形
    const int numRects = 50;
    // 计算每个矩形的高度
    qreal singleRectHeight = (height() - (numRects + 1) * padding) / numRects;
    // 使用m_level计算需要绘制多少个小矩形
    int activeRects = qRound(m_level * numRects);

    painter.setBrush(QBrush(gradient));
    for (int i = 0; i < activeRects; ++i)
    {
        // 从底部开始计算每个矩形的顶部位置
        qreal rectTop = height() - ((i + 1) * (singleRectHeight + padding));
        QRectF rect(0, rectTop, width(), singleRectHeight);
        painter.drawRect(rect);
    }

    // 绘制剩余的小矩形（不活跃部分）
    painter.setBrush(Qt::black);
    for (int i = activeRects; i < numRects; ++i)
    {
        // 从底部开始计算每个矩形的顶部位置
        qreal rectTop = height() - ((i + 1) * (singleRectHeight + padding));
        QRectF rect(0, rectTop, width(), singleRectHeight);
        painter.drawRect(rect);
    }
}

HistogramWidget::HistogramWidget(QWidget *parent) : QWidget(parent)
{
    setLayout(new QHBoxLayout);
}

HistogramWidget::~HistogramWidget()
{
}

/**
 * 处理音频缓冲区，更新音频级别显示器。
 *
 * @param buffer QAudioBuffer对象，包含待处理的音频数据。
 */
void HistogramWidget::processBuffer(const QAudioBuffer &buffer)
{
    // 检查音频级别计数是否与音频缓冲区的声道数匹配
    if (m_audioLevels.count() != buffer.format().channelCount())
    {
        // 如果不匹配，则删除现有音频级别对象，并根据声道数创建新的音频级别对象
        qDeleteAll(m_audioLevels);
        m_audioLevels.clear();
        for (int i = 0; i < buffer.format().channelCount(); ++i)
        {
            QAudioLevel *level = new QAudioLevel(this);
            m_audioLevels.append(level);
            layout()->addWidget(level);
        }
    }

    // 计算音频缓冲区的级别并更新音频级别显示器
    QVector<qreal> levels = getBufferLevels(buffer);
    for (int i = 0; i < levels.count(); ++i)
    {
        m_audioLevels.at(i)->setLevel(levels.at(i));
    }
}

void HistogramWidget::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);

    if (!m_audioLevels.isEmpty())
        return;

    QPainter painter(this);
    painter.fillRect(0, 0, width(), height(), QColor::fromRgb(0, 0, 0));
}
