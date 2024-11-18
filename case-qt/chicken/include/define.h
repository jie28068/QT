#ifndef DEFINE
#define DEFINE
#include <QList>
#include <QGraphicsItem>
#include <QDebug>
namespace DfinePro
{
    static const int itemTypes = 0;
    enum itemType
    {
        MOUSE = 0,
        RICE,
    };
    extern QList<QGraphicsItem *> itemsRice;
} // namespace DfinePro
#endif