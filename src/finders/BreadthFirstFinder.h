/*
 * Created by SG220 on 2023/10/25.
 * Copyright (C) 2023 Nevermore. All rights reserved. 
 * Ownership: Nevermore ()
 * License: All rights reserved. Unauthorized copying, modification, 
 * or distribution of this software, or any portion thereof, is strictly prohibited.
 * Description: This file contains the implementation of the pathfinding_main software.
 */

#ifndef PATHFINDING_MAIN_BREADTHFIRSTFINDER_H
#define PATHFINDING_MAIN_BREADTHFIRSTFINDER_H

#include "finders/AStarFinder.h"

namespace PathFinding {
class BreadthFirstFinder : public AStarFinder {
public:
    explicit BreadthFirstFinder(bool allowDiagonal = true, bool dontCrossCorners = false,
                                DiagonalMovement diagonalMovement = Never, distance_t weight = 1.0);

    using AStarFinder::findPath;

protected:
    std::vector<std::vector<int>>
    findPath(const std::vector<std::vector<int>> &startPoints,
             int endX, int endY, std::shared_ptr<Grid> grid) override;
};
}

#endif //PATHFINDING_MAIN_BREADTHFIRSTFINDER_H
