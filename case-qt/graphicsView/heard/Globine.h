#pragma once

namespace Gloabine
{
    enum PaintStatus
    {
        custom, // 自定义多边形
        polygon // 已有多边形
    };

    enum PaintModel
    {
        rectangle, // 矩形
        round,     // 圆形
        triangle,  // 多边形
        line,      // 直线
    };

    enum PaintDraws
    {
        drawing,  // 绘制中
        drawEnd,  // 绘制结束
        drawBegin // 绘制开始
    };
};