/*
 * Created by SG220 on 2023/10/25.
 * Copyright (C) 2023 Nevermore. All rights reserved. 
 * Ownership: Nevermore ()
 * License: All rights reserved. Unauthorized copying, modification, 
 * or distribution of this software, or any portion thereof, is strictly prohibited.
 * Description: This file contains the implementation of the pathfinding_main software.
 */

#include "BiDijkstraFinder.h"

using namespace PathFinding;

BiDijkstraFinder::BiDijkstraFinder(bool allowDiagonal, bool dontCrossCorners,
                                     DiagonalMovement diagonalMovement, double weight)
        : BiAStarFinder(allowDiagonal, dontCrossCorners, diagonalMovement, weight)
{
    this->heuristic = [](distance_t dx, distance_t dy) { return 0.0;};
}
