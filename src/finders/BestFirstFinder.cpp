/*
 * Created by SG220 on 2023/10/24.
 * Copyright (C) 2023 Nevermore. All rights reserved. 
 * Ownership: Nevermore ()
 * License: All rights reserved. Unauthorized copying, modification, 
 * or distribution of this software, or any portion thereof, is strictly prohibited.
 * Description: This file contains the implementation of the pathfinding_main software.
 */

#include "BestFirstFinder.h"
#include <functional>

using namespace PathFinding;
BestFirstFinder::BestFirstFinder(bool allowDiagonal, bool dontCrossCorners,
                                 DiagonalMovement diagonalMovement, double weight)
                                 : AStarFinder(allowDiagonal, dontCrossCorners, diagonalMovement, weight)
{
    std::function<double(int, int)> orig = heuristic;
    this->heuristic = [orig](distance_t dx, distance_t dy) { return orig(dx, dy) * 1000000.0;};
}
