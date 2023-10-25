/*
 * Created by SG220 on 2023/10/25.
 * Copyright (C) 2023 Nevermore. All rights reserved. 
 * Ownership: Nevermore ()
 * License: All rights reserved. Unauthorized copying, modification, 
 * or distribution of this software, or any portion thereof, is strictly prohibited.
 * Description: This file contains the implementation of the pathfinding_main software.
 */

#ifndef PATHFINDING_MAIN_BIDIJKSTRAFINDER_H
#define PATHFINDING_MAIN_BIDIJKSTRAFINDER_H

#include "finders/BiAStarFinder.h"

namespace PathFinding {
class BiDijkstraFinder: public BiAStarFinder {
public:
    explicit BiDijkstraFinder(bool allowDiagonal = true, bool dontCrossCorners = false,
                               DiagonalMovement diagonalMovement = Never, distance_t weight = 1.0);
};
}

#endif //PATHFINDING_MAIN_BIDIJKSTRAFINDER_H
