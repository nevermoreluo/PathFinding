/*
 * Created by SG220 on 2023/10/18.
 * Copyright (C) 2023 Nevermoreluo. All rights reserved.
 * Ownership: Nevermoreluo ()
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
using node_vector_t = Node***;


class Grid {
public:
    Grid(const std::vector<std::vector<int>>& matrix_input);
    Grid(const int** matrix);

    Grid(int width, int height, std::vector<std::vector<int>> matrix = {});

    ~Grid();

    node_vector_t buildNodes(int width, int height, const std::vector<std::vector<int>>& matrix);

    NodePtr getNodeAt(int x, int y);

    bool isWalkableAt(int x, int y);

    bool isInside(int x, int y);

    void setWalkableAt(int x, int y, bool walkable);

    std::vector<NodePtr> getNeighbors(NodePtr node, DiagonalMovement diagonalMovement);

    Grid clone();

private:
    //Node*** buildRowNodes(int width, int height, const std::vector<std::vector<int>>& matrix);

private:
    int width;
    int height;
    node_vector_t nodes;
};

}

#endif //PATHFINDING_GRID_H
