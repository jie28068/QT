/*
 * @Author: 奥特曼狂扁小怪兽 2583782734@qq.com
 * @Date: 2024-01-06 12:34:08
 * @LastEditors: 奥特曼狂扁小怪兽 2583782734@qq.com
 * @LastEditTime: 2024-01-16 15:05:22
 */
/*
 *  ┌───┐   ┌───┬───┬───┬───┐ ┌───┬───┬───┬───┐ ┌───┬───┬───┬───┐ ┌───┬───┬───┐
 *  │Esc│   │ F1│ F2│ F3│ F4│ │ F5│ F6│ F7│ F8│ │ F9│F10│F11│F12│ │P/S│S L│P/B│  ┌┐    ┌┐    ┌┐
 *  └───┘   └───┴───┴───┴───┘ └───┴───┴───┴───┘ └───┴───┴───┴───┘ └───┴───┴───┘  └┘    └┘    └┘
 *  ┌───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───────┐ ┌───┬───┬───┐ ┌───┬───┬───┬───┐
 *  │~ `│! 1│@ 2│# 3│$ 4│% 5│^ 6│& 7│* 8│( 9│) 0│_ -│+ =│ BacSp │ │Ins│Hom│PUp│ │N L│ / │ * │ - │
 *  ├───┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─────┤ ├───┼───┼───┤ ├───┼───┼───┼───┤
 *  │ Tab │ Q │ W │ E │ R │ T │ Y │ U │ I │ O │ P │{ [│} ]│ | \ │ │Del│End│PDn│ │ 7 │ 8 │ 9 │   │
 *  ├─────┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴─────┤ └───┴───┴───┘ ├───┼───┼───┤ + │
 *  │ Caps │ A │ S │ D │ F │ G │ H │ J │ K │ L │: ;│" '│ Enter  │               │ 4 │ 5 │ 6 │   │
 *  ├──────┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴────────┤     ┌───┐     ├───┼───┼───┼───┤
 *  │ Shift  │ Z │ X │ C │ V │ B │ N │ M │< ,│> .│? /│  Shift   │     │ ↑ │     │ 1 │ 2 │ 3 │   │
 *  ├─────┬──┴─┬─┴──┬┴───┴───┴───┴───┴───┴──┬┴───┼───┴┬────┬────┤ ┌───┼───┼───┐ ├───┴───┼───┤ E││
 *  │ Ctrl│    │Alt │         Space         │ Alt│    │    │Ctrl│ │ ← │ ↓ │ → │ │   0   │ . │←─┘│
 *  └─────┴────┴────┴───────────────────────┴────┴────┴────┴────┘ └───┴───┴───┘ └───────┴───┴───┘
 */

#ifndef GLOBAL_H
#define GLOBAL_H

#include <QStringList>
#include <QPixmap>

namespace MacroDf
{
    static const QStringList comboxlists = QStringList() << "2x2"
                                                         << "3x3"
                                                         << "4x4"
                                                         << "5x5";
    static const int widgetSize = 400;
    // 游戏模式
    enum Model
    {
        customsPass = 0, // 关卡
        relaxation,      // 休闲
    };
    // 闯关难度
    enum Difficulty
    {
        briefness = 0, // 简单
        ordinary,      // 普通
        hard,          // 困难
    };
    // 模式数据
    typedef struct ModelRelaxation_t
    {
        ModelRelaxation_t()
        {
            QPixmap Pixmap(":/images/2");
            pixmap = Pixmap;
            str = "2x2";
            time = 60;
        }
        QPixmap pixmap;
        QString str;
        int time;
    } ModelRelaxation;
    // 数据
    typedef struct Varable_t
    {
        Varable_t() : m_relax(nullptr), levelDesignCount(0) {}
        Model m_model;                     // 玩法模式
        Difficulty m_diff;                 // 模式难度(闯关)
        ModelRelaxation *m_relax;          // 关卡数据
        QList<ModelRelaxation *> *m_lists; // 关卡列表(闯关)
        int levelDesignCount;              // 当前关卡(闯关)
    } Varable;

    extern Varable *mVarable;

    /// 获取关卡-行*列
    static int getCloum()
    {
        int m = 2;
        if (MacroDf::mVarable != nullptr)
        {
            if (MacroDf::mVarable->m_model == Model::relaxation)
            {
                if (MacroDf::mVarable->m_relax)
                {
                    m = MacroDf::mVarable->m_relax->str.at(0).digitValue();
                }
            }
            else if (MacroDf::mVarable->m_model == Model::customsPass)
            {
                if (MacroDf::mVarable->m_lists)
                {
                    m = MacroDf::mVarable->m_lists->at(MacroDf::mVarable->levelDesignCount)->str.at(0).digitValue();
                }
            }
        }
        return m;
    }

    // 图片资源结构体
    struct Piece
    {
        QPixmap pixmap;
        QRect rect;
        QPoint location;
        Piece() {}
        Piece(QPixmap Vpixmap, QPoint Vlocation, QRect Vrect) : pixmap(Vpixmap), location(Vlocation), rect(Vrect) {}
        Piece(const Piece &other)
        {
            pixmap = other.pixmap;
            rect = other.rect;
            location = other.location;
        }
    };
}

#endif