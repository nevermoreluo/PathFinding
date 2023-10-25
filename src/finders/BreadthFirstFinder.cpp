/*
 * Created by SG220 on 2023/10/25.
 * Copyright (C) 2023 Nevermore. All rights reserved. 
 * Ownership: Nevermore ()
 * License: All rights reserved. Unauthorized copying, modification, 
 * or distribution of this software, or any portion thereof, is strictly prohibited.
 * Description: This file contains the implementation of the pathfinding_main software.
 */

#include "BreadthFirstFinder.h"

using namespace PathFinding;

BreadthFirstFinder::BreadthFirstFinder(bool allowDiagonal, bool dontCrossCorners,
                                 DiagonalMovement diagonalMovement, double weight)
        : AStarFinder(allowDiagonal, dontCrossCorners, diagonalMovement, weight)
{

}

std::vector<std::vector<int>>
BreadthFirstFinder::findPath(const std::vector<std::vector<int>> &startPoints, int endX, int endY,
                        std::shared_ptr<Grid> grid) {
    std::queue<NodePtr> openList;
    NodePtr endNode = grid->getNodeAt(endX, endY);
    std::vector<NodePtr> neighbors;
    NodePtr neighbor;
    NodePtr node;

    for (auto &pos: startPoints) {
        if (pos.size() != 2) {
            throw std::runtime_error("startPoints need at least two point");
        }
        NodePtr startNode = grid->getNodeAt(pos[0], pos[1]);

        openList.push(startNode);
        startNode->opened = true;
    }

    // while the queue is not empty
    while (!openList.empty()) {
        // take the front node from the queue
        node = openList.front();
        openList.pop();
        node->closed = true;

        // reached the end position
        if (node == endNode) {
            return Utils::backtrace(endNode);
        }

        neighbors = grid->getNeighbors(node, diagonalMovement);
        for (int i = 0, l = neighbors.size(); i < l; ++i) {
            neighbor = neighbors[i];

            // skip this neighbor if it has been inspected before
            if (neighbor->closed || neighbor->opened) {
                continue;
            }

            openList.push(neighbor);
            neighbor->opened = true;
            neighbor->parent = node;
        }
    }

    // fail to find the path
    return std::vector<std::vector<int>>();
}
