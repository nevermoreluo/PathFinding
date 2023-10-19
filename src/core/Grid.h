/*
 * Created by SG220 on 2023/10/18.
 * Copyright (C) 2023 Senken. All rights reserved. 
 * Ownership: Senken (www.senken.com.cn)
 * License: All rights reserved. Unauthorized copying, modification, 
 * or distribution of this software, or any portion thereof, is strictly prohibited.
 * Description: This file contains the implementation of the pathfinding software.
 */

#ifndef PATHFINDING_GRID_H
#define PATHFINDING_GRID_H

#include <vector>
#include <memory>
#include "core/Consts.h"
#include "core/Node.h"

namespace PathFinding {
using node_vector_t = std::vector<std::vector<std::shared_ptr<Node>>>;


class Grid {
public:
    int width;
    int height;
    node_vector_t nodes;

    Grid(std::vector<std::vector<int>>& matrix_input, int height, std::vector<std::vector<int>> matrix = {});

    Grid(int width, int height, std::vector<std::vector<int>> matrix = {});

    node_vector_t buildNodes(int width, int height, std::vector<std::vector<int>> matrix);

    std::shared_ptr<Node> getNodeAt(int x, int y);

    bool isWalkableAt(int x, int y);

    bool isInside(int x, int y);

    void setWalkableAt(int x, int y, bool walkable);

    std::vector<std::shared_ptr<Node>> getNeighbors(std::shared_ptr<Node> node, DiagonalMovement diagonalMovement);

    Grid clone();
};

}

#endif //PATHFINDING_GRID_H
