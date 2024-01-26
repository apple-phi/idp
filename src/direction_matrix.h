#pragma once
#include <ArxContainer.h>

enum class Dir
{
};

namespace Direction
{
    extern int dir_matrix[18][18];
    extern int nav_matrix[18][4];
    extern arx::map<arx::pair<int, int>, int> navigation_map;
}