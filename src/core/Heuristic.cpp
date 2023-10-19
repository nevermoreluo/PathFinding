/*
 * Created by SG220 on 2023/10/18.
 * Copyright (C) 2023 Senken. All rights reserved. 
 * Ownership: Senken (www.senken.com.cn)
 * License: All rights reserved. Unauthorized copying, modification, 
 * or distribution of this software, or any portion thereof, is strictly prohibited.
 * Description: This file contains the implementation of the pathfinding software.
 */

#include "Heuristic.h"
#include <cmath>
#include <algorithm>
namespace PathFinding {
namespace Heuristic {

distance_t manhattan(distance_t dx, distance_t dy)
{
    return dx + dy;
}

distance_t euclidean(distance_t dx, distance_t dy)
{
    return std::sqrt(dx * dx + dy * dy);
}

distance_t octile(distance_t dx, distance_t dy)
{
    auto F = std::sqrt(2.0) - 1;
    return (dx < dy) ? F * dx + dy : F * dy + dx;
}

distance_t chebyshev(distance_t dx, distance_t dy)
{
    return std::max(dx, dy);
}

}
}


