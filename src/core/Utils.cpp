/*
 * Created by SG220 on 2023/10/18.
 * Copyright (C) 2023 Nevermoreluo. All rights reserved.
 * Ownership: Nevermoreluo ()
 * License: All rights reserved. Unauthorized copying, modification, 
 * or distribution of this software, or any portion thereof, is strictly prohibited.
 * Description: This file contains the implementation of the pathfinding software.
 */

#include "Utils.h"
#include <algorithm>

namespace PathFinding{
namespace Utils{

Path backtrace(NodePtr node) {
    NodePtr tmp_node = node;
    Path path;
    path.push_back({tmp_node->x, tmp_node->y});
    while (tmp_node->parent) {
        tmp_node = tmp_node->parent;
        path.push_back({tmp_node->x, tmp_node->y});
    }
    std::reverse(path.begin(), path.end());
    return path;
}

Path biBacktrace(NodePtr nodeA, NodePtr nodeB) {
    Path pathA = PathFinding::Utils::backtrace(nodeA);
    Path pathB = PathFinding::Utils::backtrace(nodeB);
    std::reverse(pathB.begin(), pathB.end());
    pathA.insert(pathA.end(), pathB.begin(), pathB.end());
    return pathA;
}

double pathLength(Path path) {
    double sum = 0;
    for (int i = 1; i < path.size(); ++i) {
        int dx = path[i - 1][0] - path[i][0];
        int dy = path[i - 1][1] - path[i][1];
        sum += sqrt(dx * dx + dy * dy);
    }
    return sum;
}

Path interpolate(int x0, int y0, int x1, int y1) {
    std::vector<std::vector<int>> line;
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    while (true) {
        line.push_back({x0, y0});
        if (x0 == x1 && y0 == y1) {
            break;
        }
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
    return line;
}

Path expandPath(Path path) {
    Path expanded;
    int len = path.size();
    if (len < 2) {
        return expanded;
    }
    for (int i = 0; i < len - 1; ++i) {
        int x0 = path[i][0];
        int y0 = path[i][1];
        int x1 = path[i + 1][0];
        int y1 = path[i + 1][1];
        Path interpolated = interpolate(x0, y0, x1, y1);
        for (int j = 0; j < interpolated.size() - 1; ++j) {
            expanded.push_back(interpolated[j]);
        }
    }
    expanded.push_back(path[len - 1]);
    return expanded;
}

Path smoothenPath(Grid grid, Path path) {
    distance_t len = path.size();
    distance_t x0 = path[0][0];
    distance_t y0 = path[0][1];
    int x1 = path[len - 1][0];
    int y1 = path[len - 1][1];
    int sx = 0, sy = 0, ex = 0, ey = 0;
    Path newPath;
    newPath.push_back({sx, sy});
    for (int i = 2; i < len; ++i) {
        int ex = path[i][0];
        int ey = path[i][1];
        Path line = interpolate(sx, sy, ex, ey);
        bool blocked = false;
        for (int j = 1; j < line.size(); ++j) {
            std::vector<int> testCoord = line[j];
            if (!grid.isWalkableAt(testCoord[0], testCoord[1])) {
                blocked = true;
                break;
            }
        }
        if (blocked) {
            std::vector<int> lastValidCoord = path[i - 1];
            newPath.push_back(lastValidCoord);
            sx = lastValidCoord[0];
            sy = lastValidCoord[1];
        }
    }
    newPath.push_back({x1, y1});
    return newPath;
}

Path compressPath(Path path) {
    if (path.size() < 3) {
        return path;
    }
    Path compressed;
    int sx = path[0][0];
    int sy = path[0][1];
    int px = path[1][0];
    int py = path[1][1];
    int dx = px - sx;
    int dy = py - sy;
    double sq = sqrt(dx * dx + dy * dy);
    dx /= sq;
    dy /= sq;
    compressed.push_back({sx, sy});
    for (int i = 2; i < path.size(); i++) {
        int lx = px;
        int ly = py;
        int ldx = dx;
        int ldy = dy;
        px = path[i][0];
        py = path[i][1];
        dx = px - lx;
        dy = py - ly;
        sq = sqrt(dx * dx + dy * dy);
        dx /= sq;
        dy /= sq;
        if (dx != ldx || dy != ldy) {
            compressed.push_back({lx, ly});
        }
    }
    compressed.push_back({px, py});
    return compressed;
}

}
}




