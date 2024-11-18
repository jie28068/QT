#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QWidget>
#include <QMainWindow>
#include <QTextEdit>
#include <QHBoxLayout>
#include <QToolBar>
#include <QPushButton>
#include <QLabel>
#include <QTableWidget>
#include <QDragEnterEvent>
#include <QMap>

class ControlBase;
class QtVariantProperty;
class QtProperty;
class QtBrowserIndex;
class CustomDockWidget;
class ControlDialogTreeView;
class MainWindow : public QMainWindow
{
public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    void initUI();
    void addProperty(QtVariantProperty *property, const QString &id);

private slots:
    void updatePositions();
    void itemClicked(const QModelIndex &index);
    void valueChanged(QtProperty *property, const QVariant &value);
    void clickedControlView(QMimeData *data);
    void clickedControlTreeView(QMimeData *data);

private:
    CustomDockWidget *m_dockListView;
    CustomDockWidget *m_dockConstraint;
    CustomDockWidget *m_dockProperty;
    CustomDockWidget *m_dockTreeView;
    ControlDialogTreeView *treeView;

private:
    class QtVariantPropertyManager *variantManager;
    class QtTreePropertyBrowser *propertyEditor;
    QMap<QtProperty *, QString> propertyToId;
    QMap<QString, QtVariantProperty *> idToProperty;
    ControlBase *currentItem;
};

#endif