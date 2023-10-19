/*
 * Created by SG220 on 2023/10/18.
 * Copyright (C) 2023 Senken. All rights reserved. 
 * Ownership: Senken (www.senken.com.cn)
 * License: All rights reserved. Unauthorized copying, modification, 
 * or distribution of this software, or any portion thereof, is strictly prohibited.
 * Description: This file contains the implementation of the pathfinding software.
 */

#include "Grid.h"
#include <stdexcept>

using namespace PathFinding;


Grid::Grid(std::vector<std::vector<int>>& matrix_input, int height, std::vector<std::vector<int>> matrix){
    height = matrix_input.size();
    width = matrix_input[0].size();
    matrix = matrix_input;
    this->nodes = buildNodes(width, height, matrix);
}

Grid::Grid(int width, int height, std::vector<std::vector<int>> matrix) {
    this->width = width;
    this->height = height;

    this->nodes = buildNodes(width, height, matrix);
}

node_vector_t Grid::buildNodes(int width, int height, std::vector<std::vector<int>> matrix) {
    node_vector_t nodes(height, std::vector<std::shared_ptr<Node>>(width));

    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            nodes[i][j] = std::make_shared<Node>(j, i);
        }
    }

    if (matrix.empty()) {
        return nodes;
    }

    if (matrix.size() != height || matrix[0].size() != width) {
        throw std::runtime_error("Matrix size does not fit");
    }

    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            if (matrix[i][j]) {
                nodes[i][j]->walkable = false;
            }
        }
    }

    return nodes;
}

std::shared_ptr<Node> Grid::getNodeAt(int x, int y) {
    return nodes[y][x];
}

bool Grid::isWalkableAt(int x, int y) {
    return isInside(x, y) && nodes[y][x]->walkable;
}

bool Grid::isInside(int x, int y) {
    return (x >= 0 && x < width) && (y >= 0 && y < height);
}

void Grid::setWalkableAt(int x, int y, bool walkable) {
    nodes[y][x]->walkable = walkable;
}

std::vector<std::shared_ptr<Node>> Grid::getNeighbors(std::shared_ptr<Node> node, DiagonalMovement diagonalMovement) {
    int x = node->x;
    int y = node->y;
    std::vector<std::shared_ptr<Node>> neighbors;
    bool s0 = false, d0 = false;
    bool s1 = false, d1 = false;
    bool s2 = false, d2 = false;
    bool s3 = false, d3 = false;

    if (isWalkableAt(x, y - 1)) {
        neighbors.push_back(nodes[y - 1][x]);
        s0 = true;
    }

    if (isWalkableAt(x + 1, y)) {
        neighbors.push_back(nodes[y][x + 1]);
        s1 = true;
    }

    if (isWalkableAt(x, y + 1)) {
        neighbors.push_back(nodes[y + 1][x]);
        s2 = true;
    }

    if (isWalkableAt(x - 1, y)) {
        neighbors.push_back(nodes[y][x - 1]);
        s3 = true;
    }

    if (diagonalMovement == DiagonalMovement::Never) {
        return neighbors;
    }

    if (diagonalMovement == DiagonalMovement::OnlyWhenNoObstacles) {
        d0 = s3 && s0;
        d1 = s0 && s1;
        d2 = s1 && s2;
        d3 = s2 && s3;
    } else if (diagonalMovement == DiagonalMovement::IfAtMostOneObstacle) {
        d0 = s3 || s0;
        d1 = s0 || s1;
        d2 = s1 || s2;
        d3 = s2 || s3;
    } else if (diagonalMovement == DiagonalMovement::Always) {
        d0 = true;
        d1 = true;
        d2 = true;
        d3 = true;
    } else {
        throw std::runtime_error("Incorrect value of diagonalMovement");
    }

    if (d0 && isWalkableAt(x - 1, y - 1)) {
        neighbors.push_back(nodes[y - 1][x - 1]);
    }

    if (d1 && isWalkableAt(x + 1, y - 1)) {
        neighbors.push_back(nodes[y - 1][x + 1]);
    }

    if (d2 && isWalkableAt(x + 1, y + 1)) {
        neighbors.push_back(nodes[y + 1][x + 1]);
    }

    if (d3 && isWalkableAt(x - 1, y + 1)) {
        neighbors.push_back(nodes[y + 1][x - 1]);
    }

    return neighbors;
}

Grid Grid::clone() {
    Grid newGrid(width, height);
    node_vector_t newNodes(height, std::vector<std::shared_ptr<Node>>(width));

    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            newNodes[i][j] = std::make_shared<Node>(j, i, nodes[i][j]->walkable);
        }
    }

    newGrid.nodes = newNodes;

    return newGrid;
}

