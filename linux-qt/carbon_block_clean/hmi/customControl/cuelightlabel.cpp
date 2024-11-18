#include "cuelightlabel.h"
#include <QPainter>
#include <QPen>
#include <QBrush>
#include <QToolTip>
#include <QObject>

CueLightLabel::CueLightLabel(QWidget *parent) : QLabel(parent), mCurrentStatus(Offline)
{
    setStyleSheet("QToolTip { color: #ffffff; background-color: #081640; }");
}

CueLightLabel::~CueLightLabel()
{
    delete mAnimation;
}

void CueLightLabel::setStatus(CueLightStatus status)
{
    if (mCurrentStatus != status)
    {
        mCurrentStatus = status;
        update();
    }
}

void CueLightLabel::setBreathingEffectEnabled(bool enabled)
{
    if (enabled)
    {
        if (!mAnimation)
        {
            mAnimation = new QPropertyAnimation(this, "color");
            mAnimation->setDuration(1000);
            mAnimation->setStartValue(QColor(Qt::transparent));
            mAnimation->setEndValue(QColor(Qt::black));
            mAnimation->setLoopCount(-1);
            mAnimation->setEasingCurve(QEasingCurve::InOutSine);
            connect(mAnimation, &QPropertyAnimation::valueChanged, this, QOverload<>::of(&QLabel::update));
        }
        mAnimation->start();
    }
    else
    {
        if (mAnimation)
        {
            mAnimation->stop();
        }
    }
}

void CueLightLabel::paintEvent(QPaintEvent *event)
{
    QLabel::paintEvent(event);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    QColor baseColor;
    switch (mCurrentStatus)
    {
    case Online:
        baseColor = Qt::green;
        break;
    case Offline:
        baseColor = Qt::red;
        break;
    case Dropped:
        baseColor = Qt::gray;
        break;
    case Warning:
        baseColor = Qt::yellow;
        break;
    }

    QColor color = baseColor;
    if (mAnimation && mAnimation->state() == QAbstractAnimation::Running)
    {
        QColor fadeColor = qvariant_cast<QColor>(mAnimation->currentValue());
        color.setAlpha(fadeColor.alpha());
    }

    painter.setPen(Qt::NoPen);
    painter.setBrush(color);

    int side = qMin(width(), height()) / 2;
    QRect circleRect((width() - side) / 2, (height() - side) / 2, side, side);

    painter.drawEllipse(circleRect);
}
