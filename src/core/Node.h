/*
 * Created by SG220 on 2023/10/18.
 * Copyright (C) 2023 Senken. All rights reserved. 
 * Ownership: Senken (www.senken.com.cn)
 * License: All rights reserved. Unauthorized copying, modification, 
 * or distribution of this software, or any portion thereof, is strictly prohibited.
 * Description: This file contains the implementation of the pathfinding software.
 */

#ifndef PATHFINDING_NODE_H
#define PATHFINDING_NODE_H

#include <memory>
#include "core/Consts.h"

namespace PathFinding {

struct Node {
    Node(int _x=0, int _y=0, bool _walkable = true);


    int x;
    int y;


    distance_t g = 0;
    distance_t h = 0;
    distance_t f = 0;

    bool opened = false;
    bool closed = false;
    bool walkable = false;
    std::shared_ptr<Node> parent = nullptr;

};
}

#endif //PATHFINDING_NODE_H
