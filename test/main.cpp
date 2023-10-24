/*
 * Created by SG220 on 2023/10/19.
 * Copyright (C) 2023 Nevermoreluo. All rights reserved.
 * Ownership: Nevermoreluo ()
 * License: All rights reserved. Unauthorized copying, modification, 
 * or distribution of this software, or any portion thereof, is strictly prohibited.
 * Description: This file contains the implementation of the pathfinding software.
 */
#include "finders/AStarFinder.h"
#include "finders/BiAStarFinder.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cstdlib>
#include <ctime>

#define DATA_FILE "data"


std::vector<std::vector<std::vector<int>>> readConfig()
{
    std::vector<std::vector<std::vector<int>>> matrix_arr;
    std::ifstream file(DATA_FILE);
    if (!file.is_open()) {
        std::cout << "Failed to open the file." << std::endl;
        return matrix_arr;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::vector<std::vector<int>> matrix;
        std::vector<std::string> tokens;
        std::stringstream ss(line);
        std::string token;
        while (std::getline(ss, token, '#')) {
            std::vector<int> arr;
            std::stringstream tokenss(token);
            std::string walkable;
            while (std::getline(tokenss, walkable, ',')){
                arr.push_back(walkable=="1" ? 1 : 0);
            }
            matrix.push_back(arr);
        }
        matrix_arr.push_back(matrix);
    }
    file.close();

    return matrix_arr;
}


void writeConfig()
{
    std::ofstream outputFile(DATA_FILE);
    if (outputFile.is_open()) {
        srand(static_cast<unsigned int>(time(0)));  // 设置随机数种子
        std::string outline;
        int lines = 10;
        for (int n = 0; n < lines; n++){
            int rows = rand() % 5 + 1;  // 生成1到5之间的随机行数
            int cols = rand() % 10 + 2;  // 生成1到5之间的随机列数

            for (int i = 0; i < rows; ++i) {
                for (int j = 0; j < cols; ++j) {
                    outline += std::to_string(rand() % 2);
                    if (j != cols - 1) {
                        outline += ",";
                    }
                }
                if (i != rows - 1) {
                    outline += "#";
                }
            }
            if (n != lines - 1){
                outline += "\n";
            }
        }
        outputFile << outline;

        outputFile.close();

        std::cout << "Data written to the file successfully.\n";
    } else {
        std::cout << "Failed to open the file.\n";
    }
}



int main(){
    using namespace PathFinding;
    // make a test, input with file, output to result file, compare with another pathfinding lib out put

    // writeConfig();

    auto matrix_arr = readConfig();

    std::string result_str;
    for (const auto& matrix: matrix_arr){
        size_t w = matrix[0].size();
        size_t h = matrix.size();
        std::vector<std::vector<int>> path;
        {
            BiAStarFinder aStarFinder(true, false);
            const std::vector<std::vector<int>> starts = std::vector<std::vector<int>>{{0, 0}};
            path = aStarFinder.findPath(starts,
                                        w - 1,
                                        h - 1,
                                         matrix);
        }
        for (auto pos: path){
            for (auto p: pos){
                result_str += std::to_string(p);
            }
            result_str += ",";
        }
        result_str += "   " + std::to_string(w) + "*" + std::to_string(h) + "\n";

    }

    printf("%s", result_str.c_str());
    return 0;
}

