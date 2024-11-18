#ifndef TYPEDEF_H
#define TYPEDEF_H
#include "KLModelDefinitionCore.h"
#include <QObject>
#include <QString>
#include <QVector>

namespace ItemType {
static const QString groupTypeItem = "组";
static const QString variableTypeItem = "变量";
static const QString otherTypeItem = "模块";
}

// 树列号
enum COLUMN
{
    COLUMN_NODE = 0,
    COLUMN_TYPE
};

// 信息
typedef struct Node_t {
    QString Node; // 节点
    QString Type; // 节点类型
    Node_t()
    {
        Node = "";
        Type = "";
    }
} Node;

namespace Developer {
// 特殊字段
static const QString isSharedGroup = "同步共享组变量";
static const QString newKey = "new Key";
static const QString newValue = "new Value";
static const QString uuid = "UUID";
// 控制组名称
static const QStringList controlGroupNameLists = QStringList()
        << Kcc::BlockDefinition::RoleDataDefinition::MainGroup << Kcc::BlockDefinition::RoleDataDefinition::InputSignal
        << Kcc::BlockDefinition::RoleDataDefinition::OutputSignal
        << Kcc::BlockDefinition::RoleDataDefinition::DiscreteStateVariable
        << Kcc::BlockDefinition::RoleDataDefinition::ContinueStateVariable
        << Kcc::BlockDefinition::RoleDataDefinition::InternalVariable
        << Kcc::BlockDefinition::RoleDataDefinition::Parameter << Kcc::BlockDefinition::RoleDataDefinition::ControlGroup
        << Kcc::BlockDefinition::RoleDataDefinition::PortGroup;
// 电气组名称
static const QStringList elecGroupNameLists = QStringList()
        << Kcc::BlockDefinition::RoleDataDefinition::MainGroup
        << Kcc::BlockDefinition::RoleDataDefinition::ElectricalParameter
        << Kcc::BlockDefinition::RoleDataDefinition::SimulationParameter
        << Kcc::BlockDefinition::RoleDataDefinition::InitializationParameter
        << Kcc::BlockDefinition::RoleDataDefinition::LoadFlowParameter
        << Kcc::BlockDefinition::RoleDataDefinition::LoadFlowResultParameter
        << Kcc::BlockDefinition::RoleDataDefinition::ResultSaveVariables
        << Kcc::BlockDefinition::RoleDataDefinition::ElectricalGroup
        << Kcc::BlockDefinition::RoleDataDefinition::PortGroup << Kcc::BlockDefinition::RoleDataDefinition::InputSignal
        << Kcc::BlockDefinition::RoleDataDefinition::OutputSignal;
// 设备类型
static const QStringList deviceGroupNameLists = QStringList()
        << Kcc::BlockDefinition::RoleDataDefinition::MainGroup
        << Kcc::BlockDefinition::RoleDataDefinition::DeviceTypeParameter
        << Kcc::BlockDefinition::RoleDataDefinition::DeviceTypeGroup;

// 非参数组
static const QStringList otherGroupNameLists = QStringList()
        << Kcc::BlockDefinition::RoleDataDefinition::MainGroup << Kcc::BlockDefinition::RoleDataDefinition::ControlGroup
        << Kcc::BlockDefinition::RoleDataDefinition::ElectricalGroup;
// 其他组
static const QStringList otherGroupNameList = QStringList()
        << Kcc::BlockDefinition::RoleDataDefinition::MainGroup
        << Kcc::BlockDefinition::RoleDataDefinition::ElectricalParameter
        << Kcc::BlockDefinition::RoleDataDefinition::SimulationParameter
        << Kcc::BlockDefinition::RoleDataDefinition::InitializationParameter
        << Kcc::BlockDefinition::RoleDataDefinition::LoadFlowParameter
        << Kcc::BlockDefinition::RoleDataDefinition::LoadFlowResultParameter
        << Kcc::BlockDefinition::RoleDataDefinition::ResultSaveVariables
        << Kcc::BlockDefinition::RoleDataDefinition::ElectricalGroup
        << Kcc::BlockDefinition::RoleDataDefinition::PortGroup << Kcc::BlockDefinition::RoleDataDefinition::InputSignal
        << Kcc::BlockDefinition::RoleDataDefinition::OutputSignal
        << Kcc::BlockDefinition::RoleDataDefinition::DeviceTypeParameter
        << Kcc::BlockDefinition::RoleDataDefinition::DeviceTypeGroup
        << Kcc::BlockDefinition::RoleDataDefinition::ControlGroup << Kcc::BlockDefinition::RoleDataDefinition::Parameter
        << Kcc::BlockDefinition::RoleDataDefinition::DiscreteStateVariable
        << Kcc::BlockDefinition::RoleDataDefinition::ContinueStateVariable
        << Kcc::BlockDefinition::RoleDataDefinition::InternalVariable;

// 图片资源组
static const QStringList PNGImageLists = QStringList() << Kcc::BlockDefinition::RoleDataDefinition::PNG_NORMAL_PIC
                                                       << Kcc::BlockDefinition::RoleDataDefinition::PNG_DISABLE_PIC
                                                       << Kcc::BlockDefinition::RoleDataDefinition::PNG_WARNING_PIC
                                                       << Kcc::BlockDefinition::RoleDataDefinition::PNG_ERROR_PIC;
static const QStringList SVGImageLists = QStringList()
        << Kcc::BlockDefinition::RoleDataDefinition::SVG_0_PIC << Kcc::BlockDefinition::RoleDataDefinition::SVG_90_PIC
        << Kcc::BlockDefinition::RoleDataDefinition::SVG_180_PIC
        << Kcc::BlockDefinition::RoleDataDefinition::SVG_270_PIC;
}
#endif // TYPEDEF_H
