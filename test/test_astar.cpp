/*
 * Created by SG220 on 2023/10/19.
 * Copyright (C) 2023 Senken. All rights reserved. 
 * Ownership: Senken (www.senken.com.cn)
 * License: All rights reserved. Unauthorized copying, modification, 
 * or distribution of this software, or any portion thereof, is strictly prohibited.
 * Description: This file contains the implementation of the pathfinding software.
 */

#include "src/finders/AStarFinder.h"

#include <gtest/gtest.h>

//std::shared_ptr<Grid> grid = std::make_shared<Grid>(matrix, 3);
TEST(FaceCheck, BasicAssertions) {
    // base check
    EXPECT_STREQ("hello", "hello");

    using namespace PathFinding;
    std::vector<std::vector<int>> matrix{{0, 0, 0},{0,1,0}, {0,0,0}};
    PathFinding::Grid g(3, 3);

//    AStarFinder aStarFinder;
//    auto path = aStarFinder.findPath(0, 0, 2, 2, grid);
//    for (auto p : path){
//        for (auto i: p){
//            printf("%d", i);
//        }
//        printf("\n");
//    }
}
