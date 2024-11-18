
#ifndef HISTOGRAMWIDGET_H
#define HISTOGRAMWIDGET_H
#include <QVideoFrame>
#include <QAudioBuffer>
#include <QWidget>

class QAudioLevel : public QWidget
{
public:
    explicit QAudioLevel(QWidget *parent = nullptr);
    void setLevel(qreal level);

protected:
    void paintEvent(QPaintEvent *event);

private:
    qreal m_level = 0;
};

class HistogramWidget : public QWidget
{
    Q_OBJECT

public:
    explicit HistogramWidget(QWidget *parent = nullptr);
    ~HistogramWidget();
    void setLevels(int levels) { m_levels = levels; }

public slots:
    void processBuffer(const QAudioBuffer &buffer);

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    int m_levels = 128;
    QVector<QAudioLevel *> m_audioLevels;
};

#endif