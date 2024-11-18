#include "customDockWidget.h"

CustomDockWidget::CustomDockWidget(const QString &title, QWidget *parent, Qt::WindowFlags flags) : QDockWidget(title, parent, flags)
{

    // 不支持浮动与关闭
    setFeatures(features() & ~QDockWidget::DockWidgetFloatable);
    setFeatures(features() & ~QDockWidget::DockWidgetClosable);

    // 任何区域
    setAllowedAreas(Qt::AllDockWidgetAreas);
}
