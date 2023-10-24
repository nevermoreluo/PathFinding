/*
 * Created by SG220 on 2023/10/18.
 * Copyright (C) 2023 Nevermoreluo. All rights reserved.
 * Ownership: Nevermoreluo ()
 * License: All rights reserved. Unauthorized copying, modification, 
 * or distribution of this software, or any portion thereof, is strictly prohibited.
 * Description: This file contains the implementation of the pathfinding software.
 */

#include "AStarFinder.h"

#include <utility>

using namespace PathFinding;
using namespace PathFinding::Utils;
using namespace PathFinding::Heuristic;

AStarFinder::AStarFinder(bool allowDiagonal, bool dontCrossCorners , DiagonalMovement diagonalMovement, double weight) {
    this->allowDiagonal = allowDiagonal;
    this->dontCrossCorners = dontCrossCorners;
    this->diagonalMovement = diagonalMovement;
    this->weight = weight;

    if (!this->diagonalMovement) {
        if (!this->allowDiagonal) {
            this->diagonalMovement = Never;
        } else {
            if (this->dontCrossCorners) {
                this->diagonalMovement = OnlyWhenNoObstacles;
            } else {
                this->diagonalMovement = IfAtMostOneObstacle;
            }
        }
    }

    if (this->diagonalMovement == Never) {
        this->heuristic = manhattan;
    } else {
        this->heuristic = octile;
    }
}

std::vector<std::vector<int>> AStarFinder::findPath(int startX, int startY, int endX, int endY, const std::vector<std::vector<int>>& matrix)
{
    auto grid = std::make_shared<Grid>(matrix);
    return findPath(startX, startY, endX, endY, grid);
}

std::vector<std::vector<int>> AStarFinder::findPath(int startX, int startY, int endX, int endY, const int** matrix)
{
    auto grid = std::make_shared<Grid>(matrix);
    return findPath(startX, startY, endX, endY, grid);

}

std::vector<std::vector<int>> AStarFinder::findPath(const std::vector<std::vector<int>>& startPoints, int endX, int endY, const std::vector<std::vector<int>>& matrix)
{
    auto grid = std::make_shared<Grid>(matrix);
    return findPath(startPoints, endX, endY, grid);
}


Path AStarFinder::findPath(const std::vector<std::vector<int>>& startPoints, int endX, int endY, std::shared_ptr<Grid> grid) {
    auto cmp = [](const NodePtr& left, const NodePtr& right) {
        return left->f  > right->f;
    };
    std::priority_queue<NodePtr, std::vector<NodePtr>, decltype(cmp)> openList(cmp);
    std::vector<NodePtr> closedList;

    for (auto& pos: startPoints) {
        if (pos.size() != 2){
            throw std::runtime_error("startPoints need at least two point");
        }
        NodePtr startNode = grid->getNodeAt(pos[0], pos[1]);
        startNode->g = 0;
        startNode->f = 0;

        openList.push(startNode);
        startNode->opened = 1;
    }

    NodePtr endNode = grid->getNodeAt(endX, endY);
    distance_t SQRT2 = std::sqrt(2);
    NodePtr node;
    std::vector<NodePtr> neighbors;
    NodePtr neighbor;
    int x, y;
    distance_t ng;

    while (!openList.empty()) {
        node = openList.top();
        openList.pop();
        node->closed = true;
        closedList.push_back(node);

        if (node == endNode) {
            return Utils::backtrace(endNode);
        }

        neighbors = grid->getNeighbors(node, diagonalMovement);
        for (const auto & i : neighbors) {
            neighbor = i;

            if (neighbor->closed) {
                continue;
            }

            x = neighbor->x;
            y = neighbor->y;

            ng = node->g + ((x - node->x == 0 || y - node->y == 0) ? 1 : SQRT2);

            if (!neighbor->opened || ng < neighbor->g) {
                neighbor->g = ng;
                neighbor->h = neighbor->h || weight * heuristic(std::abs(x - endX), std::abs(y - endY));
                neighbor->f = neighbor->g + neighbor->h;
                neighbor->parent = node;

                if (!neighbor->opened) {
                    openList.push(neighbor);
                    neighbor->opened = 1;
                }
            }
        }
    }

    return {};
}

Path AStarFinder::findPath(int startX, int startY, int endX, int endY, std::shared_ptr<Grid> grid) {
    std::vector<std::vector<int>> startPoints = {{startX, startY}};
    return findPath(startPoints, endX, endY, std::move(grid));
}




