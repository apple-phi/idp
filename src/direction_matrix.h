#pragma once
#include <ArxContainer.h>

enum class Dir
{
};

namespace Direction
{
    extern int dir_matrix[20][20];
    extern arx::map<arx::pair<int, int>, int> navigation_map;
}