/*
 * Created by SG220 on 2023/10/18.
 * Copyright (C) 2023 Senken. All rights reserved. 
 * Ownership: Senken (www.senken.com.cn)
 * License: All rights reserved. Unauthorized copying, modification, 
 * or distribution of this software, or any portion thereof, is strictly prohibited.
 * Description: This file contains the implementation of the pathfinding software.
 */

#ifndef PATHFINDING_ASTARFINDER_H
#define PATHFINDING_ASTARFINDER_H

#include <vector>
#include "core/Consts.h"
#include "core/Utils.h"
#include "core/Heuristic.h"
#include <queue>


namespace PathFinding {

class AStarFinder {
public:
    explicit AStarFinder(bool allowDiagonal = true, bool dontCrossCorners = false,
                DiagonalMovement diagonalMovement = IfAtMostOneObstacle, distance_t weight = 1.0);

    std::vector<std::vector<int>> findPath(int startX, int startY, int endX, int endY, std::shared_ptr<Grid> grid);

private:
    bool allowDiagonal;
    bool dontCrossCorners;
    DiagonalMovement diagonalMovement;
    distance_t weight;

    distance_t (*heuristic)(distance_t, distance_t) = nullptr;
};

}

#endif //PATHFINDING_ASTARFINDER_H
