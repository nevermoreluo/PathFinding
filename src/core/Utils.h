/*
 * Created by SG220 on 2023/10/18.
 * Copyright (C) 2023 Senken. All rights reserved. 
 * Ownership: Senken (www.senken.com.cn)
 * License: All rights reserved. Unauthorized copying, modification, 
 * or distribution of this software, or any portion thereof, is strictly prohibited.
 * Description: This file contains the implementation of the pathfinding software.
 */

#ifndef PATHFINDING_UTILS_H
#define PATHFINDING_UTILS_H

#include "core/Consts.h"
#include <iostream>
#include <vector>
#include <cmath>
#include "core/Node.h"
#include "core/Grid.h"


namespace PathFinding {

namespace Utils {

using Path = std::vector<std::vector<int>>;

Path backtrace(std::shared_ptr<Node> node);

Path biBacktrace(std::shared_ptr<Node> nodeA, std::shared_ptr<Node> nodeB);

double pathLength(Path path);

Path interpolate(distance_t x0, distance_t y0, distance_t x1, distance_t y1);

Path expandPath(Path path);

Path smoothenPath(Grid grid, Path path);

Path compressPath(Path path);
}

}


#endif //PATHFINDING_UTILS_H
