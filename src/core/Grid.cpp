/*
 * Created by SG220 on 2023/10/18.
 * Copyright (C) 2023 Nevermoreluo. All rights reserved.
 * Ownership: Nevermoreluo ()
 * License: All rights reserved. Unauthorized copying, modification, 
 * or distribution of this software, or any portion thereof, is strictly prohibited.
 * Description: This file contains the implementation of the pathfinding software.
 */

#include "Grid.h"
#include <stdexcept>

using namespace PathFinding;


Grid::Grid(const std::vector<std::vector<int>>& matrix_input){
    this->height = matrix_input.size();
    width = matrix_input[0].size();
    this->nodes = buildNodes(width, height, matrix_input);
}

Grid::Grid(const int** matrix)
{
    width = sizeof(matrix) / sizeof(matrix[0]);
    height = sizeof(matrix[0]) / sizeof(int);
    std::vector<std::vector<int>> matrix_vec(width, std::vector<int>(height));;
    for (int i = 0; i < width; ++i) {
        for (int j = 0; j < height; ++j) {
            matrix_vec[i][j] = matrix[i][j];
        }
    }
    this->nodes = buildNodes(width, height, matrix_vec);
}


Grid::Grid(int width, int height, std::vector<std::vector<int>> matrix) {
    this->width = width;
    this->height = height;
    this->nodes = buildNodes(width, height, matrix);
}

Grid::~Grid()
{
    // 释放内存
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            delete nodes[i][j];  // 释放单个A对象的内存
        }
        delete[] nodes[i];  // 释放行的内存
    }
    delete[] nodes;  // 释放二维数组的内存
}


Node*** Grid::buildNodes(int _width, int _height, const std::vector<std::vector<int>>& matrix)
{
    Node*** _nodes = new Node**[_height];
    for (int i = 0; i < _height; i++) {
        _nodes[i] = new Node*[_width];
        for (int j = 0; j < _width; j++) {
            _nodes[i][j] = new Node(j, i);  // 分配一个新的A对象
        }
    }

    if (matrix.empty()) {
        return _nodes;
    }

    if (matrix.size() != _height || matrix[0].size() != _width) {
        throw std::runtime_error("Matrix size does not fit");
    }

    for (int i = 0; i < _height; ++i) {
        for (int j = 0; j < _width; ++j) {
            if (matrix[i][j]) {
                _nodes[i][j]->walkable = false;
            }
        }
    }
    return _nodes;
}

//node_vector_t Grid::buildNodes(int _width, int _height, const std::vector<std::vector<int>>& matrix) {
//    node_vector_t _nodes(_height, std::vector<NodePtr>(_width));
//
//    for (int i = 0; i < _height; ++i) {
//        for (int j = 0; j < _width; ++j) {
//            _nodes[i][j] = new Node(j, i);
//        }
//    }
//
//    if (matrix.empty()) {
//        return _nodes;
//    }
//
//    if (matrix.size() != _height || matrix[0].size() != _width) {
//        throw std::runtime_error("Matrix size does not fit");
//    }
//
//    for (int i = 0; i < _height; ++i) {
//        for (int j = 0; j < _width; ++j) {
//            if (matrix[i][j]) {
//                _nodes[i][j]->walkable = false;
//            }
//        }
//    }
//
//    return _nodes;
//}

NodePtr Grid::getNodeAt(int x, int y) {
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

std::vector<NodePtr> Grid::getNeighbors(NodePtr node, DiagonalMovement diagonalMovement) {
    int x = node->x;
    int y = node->y;
    std::vector<NodePtr> neighbors;
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

    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            newGrid.nodes[i][j]->walkable = nodes[i][j]->walkable;
        }
    }

    return newGrid;
}

