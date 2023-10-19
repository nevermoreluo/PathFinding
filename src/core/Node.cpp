/*
 * Created by SG220 on 2023/10/18.
 * Copyright (C) 2023 Senken. All rights reserved. 
 * Ownership: Senken (www.senken.com.cn)
 * License: All rights reserved. Unauthorized copying, modification, 
 * or distribution of this software, or any portion thereof, is strictly prohibited.
 * Description: This file contains the implementation of the pathfinding software.
 */

#include "Node.h"

using namespace PathFinding;

Node::Node(int _x, int _y, bool _walkable): x(_x), y(_y), walkable(_walkable)
{

}