/*
 * Created by SG220 on 2023/10/19.
 * Copyright (C) 2023 Nevermoreluo. All rights reserved.
 * Ownership: Nevermoreluo ()
 * License: All rights reserved. Unauthorized copying, modification, 
 * or distribution of this software, or any portion thereof, is strictly prohibited.
 * Description: This file contains the implementation of the pathfinding software.
 */

#include "src/finders/AStarFinder.h"
#include "src/finders/BiBestFirstFinder.h"

#include <gtest/gtest.h>

//std::shared_ptr<Grid> grid = std::make_shared<Grid>(matrix, 3);
TEST(FaceCheck, BasicAssertions) {
    // base check
    using namespace PathFinding;
    std::vector<std::vector<int>> matrix{{0,0,0,0,0}};

    AStarFinder aStarFinder;
    auto path = aStarFinder.findPath(0, 0, matrix[0].size()-1, matrix.size()-1, matrix);
    EXPECT_EQ(path, std::vector<std::vector<int>>({{0,0},{1,0}, {2,0}, {3,0}, {4,0}}));

}
