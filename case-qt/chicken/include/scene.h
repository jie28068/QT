#ifndef SCENE_H
#define SCENE_H
#include <math.h>

#include <QGraphicsScene>
#include "mouse.h"
#include <QMovie>
/// @brief
class Scene : public QGraphicsScene
{
    Q_OBJECT
public:
    explicit Scene(QObject *parent = 0);
    ~Scene();

signals:

public slots:
    void refreshMouse(int count = 7);
    void refreshRice(int count = 5);

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *event) override;
};
#endif