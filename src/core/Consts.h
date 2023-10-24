/*
 * Created by SG220 on 2023/10/18.
 * Copyright (C) 2023 Nevermoreluo. All rights reserved.
 * Ownership: Nevermoreluo ()
 * License: All rights reserved. Unauthorized copying, modification, 
 * or distribution of this software, or any portion thereof, is strictly prohibited.
 * Description: This file contains the implementation of the pathfinding software.
 */

#ifndef PATHFINDING_CONSTS_H
#define PATHFINDING_CONSTS_H

namespace PathFinding {

using distance_t = double;

enum DiagonalMovement {
    Always = 1,
    Never = 2,
    IfAtMostOneObstacle = 3,
    OnlyWhenNoObstacles = 4
};

}

#endif //PATHFINDING_CONSTS_H
