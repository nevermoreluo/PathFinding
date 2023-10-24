/*
 * Created by SG220 on 2023/10/18.
 * Copyright (C) 2023 Nevermoreluo. All rights reserved.
 * Ownership: Nevermoreluo ()
 * License: All rights reserved. Unauthorized copying, modification, 
 * or distribution of this software, or any portion thereof, is strictly prohibited.
 * Description: This file contains the implementation of the pathfinding software.
 */

#ifndef PATHFINDING_ASTARFINDER_H
#define PATHFINDING_ASTARFINDER_H

#include <vector>
#include "core/Consts.h"
#include "core/Utils.h"
#include "core/Heuristic.h"
#include <queue>
#include <functional>


namespace PathFinding {

class AStarFinder {
public:
    AStarFinder(bool allowDiagonal = true, bool dontCrossCorners = false,
                DiagonalMovement diagonalMovement = Never, distance_t weight = 1.0);

    std::vector<std::vector<int>> findPath(int startX, int startY, int endX, int endY, const std::vector<std::vector<int>>& matrix);

    std::vector<std::vector<int>> findPath(int startX, int startY, int endX, int endY, const int** matrix);

    /*
     *
     * 很多情况下我们需要多个相邻的起点或者终点，例如目标点是个巨大的门或者是个很多位置的长凳，就可以有多个交互位置
     * 如果需要多个终点就把终点输入startPoints内 起点设置为endX endY，把结果反转一下即可
     */
    std::vector<std::vector<int>> findPath(const std::vector<std::vector<int>>& startPoints, int endX, int endY, const std::vector<std::vector<int>>& matrix);

protected:
    virtual std::vector<std::vector<int>> findPath(const std::vector<std::vector<int>>& startPoints, int endX, int endY, std::shared_ptr<Grid> grid);
    std::vector<std::vector<int>> findPath(int startX, int startY, int endX, int endY, std::shared_ptr<Grid> grid);

protected:
    std::function<double(int, int)> heuristic = nullptr;
    DiagonalMovement diagonalMovement;
    distance_t weight;

private:
    bool allowDiagonal;
    bool dontCrossCorners;


};

}

#endif //PATHFINDING_ASTARFINDER_H
