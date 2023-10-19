/*
 * Created by SG220 on 2023/10/18.
 * Copyright (C) 2023 Senken. All rights reserved. 
 * Ownership: Senken (www.senken.com.cn)
 * License: All rights reserved. Unauthorized copying, modification, 
 * or distribution of this software, or any portion thereof, is strictly prohibited.
 * Description: This file contains the implementation of the pathfinding software.
 */

#include "AStarFinder.h"

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

Path AStarFinder::findPath(int startX, int startY, int endX, int endY, std::shared_ptr<Grid> grid) {
    auto cmp = [](const std::shared_ptr<Node>& left, const std::shared_ptr<Node>& right) {
        return left->f  > right->f;
    };

    std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, decltype(cmp)> openList(cmp);
    std::vector<std::shared_ptr<Node>> closedList;
    std::shared_ptr<Node> startNode = grid->getNodeAt(startX, startY);
    std::shared_ptr<Node> endNode = grid->getNodeAt(endX, endY);
    distance_t SQRT2 = std::sqrt(2);
    std::shared_ptr<Node> node;
    std::vector<std::shared_ptr<Node>> neighbors;
    std::shared_ptr<Node> neighbor;
    int x, y;
    distance_t ng;

    startNode->g = 0;
    startNode->f = 0;

    openList.push(startNode);
    startNode->opened = true;

    while (!openList.empty()) {
//            std::sort(openList.begin(), openList.end(), [](std::shared_ptr<Node> a, std::shared_ptr<Node> b) { return a->f < b->f; });
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
                    neighbor->opened = true;
                } else {
//                        updateItem(neighbor, openList);
                }
            }
        }
    }

    return {};
}




