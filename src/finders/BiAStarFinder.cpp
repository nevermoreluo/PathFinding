/*
 * Created by SG220 on 2023/10/24.
 * Copyright (C) 2023 Nevermore. All rights reserved. 
 * Ownership: Nevermore ()
 * License: All rights reserved. Unauthorized copying, modification, 
 * or distribution of this software, or any portion thereof, is strictly prohibited.
 * Description: This file contains the implementation of the pathfinding_main software.
 */

#include "BiAStarFinder.h"

using namespace PathFinding;
using namespace PathFinding::Heuristic;


BiAStarFinder::BiAStarFinder(bool allowDiagonal, bool dontCrossCorners,
        DiagonalMovement diagonalMovement, double weight)
    : AStarFinder(allowDiagonal, dontCrossCorners, diagonalMovement, weight)
{

}

std::vector<std::vector<int>>
BiAStarFinder::findPath(const std::vector<std::vector<int>> &startPoints, int endX, int endY,
                        std::shared_ptr<Grid> grid) {
    auto cmp = [](const NodePtr &left, const NodePtr &right) {
        return left->f > right->f;
    };
    std::priority_queue<NodePtr, std::vector<NodePtr>, decltype(cmp)> startOpenList(cmp);
    std::priority_queue<NodePtr, std::vector<NodePtr>, decltype(cmp)> endOpenList(cmp);
    std::vector<NodePtr> closedList;
    int BY_START = 1, BY_END = 2;
    NodePtr endNode = grid->getNodeAt(endX, endY);
    distance_t SQRT2 = std::sqrt(2);
    NodePtr node;
    std::vector<NodePtr> neighbors;
    NodePtr neighbor;
    int x, y;
    distance_t ng;
    int startX = startPoints[0][0], startY = startPoints[0][1];

    for (auto &pos: startPoints) {
        if (pos.size() != 2) {
            throw std::runtime_error("startPoints need at least two point");
        }
        NodePtr startNode = grid->getNodeAt(pos[0], pos[1]);
        startNode->g = 0;
        startNode->f = 0;

        startOpenList.push(startNode);
        startNode->opened = BY_START;
    }

    endNode->g = 0;
    endNode->f = 0;
    endOpenList.push(endNode);
    endNode->opened = BY_END;

    while (!startOpenList.empty() && !endOpenList.empty()) {
        node = startOpenList.top();
        startOpenList.pop();
        node->closed = true;

        neighbors = grid->getNeighbors(node, diagonalMovement);
        for (int i = 0; i < neighbors.size(); i++) {
            neighbor = neighbors[i];

            if (neighbor->closed) {
                continue;
            }
            if (neighbor->opened == BY_END) {
                return Utils::biBacktrace(node, neighbor);
            }

            x = neighbor->x;
            y = neighbor->y;

            ng = node->g + ((x - node->x == 0 || y - node->y == 0) ? 1 : SQRT2);

            if (!neighbor->opened || ng < neighbor->g) {
                neighbor->g = ng;
                neighbor->h = neighbor->h || weight * heuristic(abs(x - endX), abs(y - endY));
                neighbor->f = neighbor->g + neighbor->h;
                neighbor->parent = node;

                if (!neighbor->opened) {
                    startOpenList.push(neighbor);
                    neighbor->opened = BY_START;
                }
            }
        }

        node = endOpenList.top();
        endOpenList.pop();
        node->closed = true;

        neighbors = grid->getNeighbors(node, diagonalMovement);
        for (int i = 0; i < neighbors.size(); i++) {
            neighbor = neighbors[i];

            if (neighbor->closed) {
                continue;
            }
            if (neighbor->opened == BY_START) {
                return Utils::biBacktrace(neighbor, node);
            }

            x = neighbor->x;
            y = neighbor->y;

            ng = node->g + ((x - node->x == 0 || y - node->y == 0) ? 1 : SQRT2);

            if (!neighbor->opened || ng < neighbor->g) {
                neighbor->g = ng;
                neighbor->h = neighbor->h || weight * heuristic(abs(x - startX), abs(y - startY));
                neighbor->f = neighbor->g + neighbor->h;
                neighbor->parent = node;

                if (!neighbor->opened) {
                    endOpenList.push(neighbor);
                    neighbor->opened = BY_END;
                }
            }
        }
    }

    return std::vector<std::vector<int>>();
}