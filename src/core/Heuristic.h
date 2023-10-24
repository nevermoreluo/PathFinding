/*
 * Created by SG220 on 2023/10/18.
 * Copyright (C) 2023 Nevermoreluo. All rights reserved.
 * Ownership: Nevermoreluo ()
 * License: All rights reserved. Unauthorized copying, modification, 
 * or distribution of this software, or any portion thereof, is strictly prohibited.
 * Description: This file contains the implementation of the pathfinding software.
 */

#ifndef PATHFINDING_HEURISTIC_H
#define PATHFINDING_HEURISTIC_H

#include "core/Consts.h"

namespace PathFinding{


/*
 * 寻路算法的启发算法
 * 用于计算从任何出发点到目标点的最小成本估计值
 * 详情可以参照哈佛大学的关于启发算法的网页
 * https://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html
 */
namespace Heuristic {

/*
 * 曼哈顿算法
 * 其实就是只能按上下左右四方向按方格走的路径距离，不可以斜边走
 */
distance_t manhattan(distance_t dx, distance_t dy);

/*
 * 欧式几何算法
 * 即计算点到点的直线距离
 */
distance_t euclidean(distance_t dx, distance_t dy);

/*
 * 八进距离
 * 依旧按格子走，但是这次是8方向允许格子按斜边走的两点距离，此时假设格子都是1*1的格子
 * 水平或垂直移动成本为1，斜边移动成本为根号2
 */
distance_t octile(distance_t dx, distance_t dy);


/*
 * 切比雪夫距离
 * 实际是八进距离的变种，此时忽视斜边距离是根号2的实际，依旧按1处理步进，即每次移动一个格子的成本都是1
 */
distance_t chebyshev(distance_t dx, distance_t dy);

}
}

#endif //PATHFINDING_HEURISTIC_H
