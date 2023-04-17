/*
 * @Author: 董泰宏 2396203400@qq.com
 * @Date: 2023-04-14 10:24:18
 * @LastEditors: 董泰宏 2396203400@qq.com
 * @LastEditTime: 2023-04-17 10:37:18
 * @FilePath: /AstarVisualization/src/main.cpp
 * @Description:
 * Copyright (c) 2023 by 董泰宏 email: 2396203400@qq.com, All Rights Reserved.
 */
#include "astar.h"

int main(int argc, const char* argv[]) {
  std::cout << "---initializing map---" << std::endl;
  char mapdata[map_x][map_y];
  for (int i = 0; i < map_x; i++) {
    for (int j = 0; j < map_y; j++) {
      mapdata[i][j] = '1';
    }
  }
  //障碍物位置
  for (int i = 4; i < 8; i++) mapdata[i][20] = '0';
  for (int i = 24; i < 38; i++) mapdata[i][32] = '0';
  for (int i = 14; i < 28; i++) mapdata[i][10] = '0';
  for (int i = 4; i < 38; i++) mapdata[25][i] = '0';

  vector<vector<Point*>> Map;
  for (int i = 0; i < map_x; i++) {
    vector<Point*> tmp;
    for (int j = 0; j < map_y; j++) {
      Point* point = new Point();
      point->x = i;
      point->y = j;
      if (mapdata[i][j] == '0') point->type = Type::OBSTACLE;
      tmp.push_back(point);
    }
    Map.push_back(tmp);
  }
  Astar astar(Map[1][1], Map[map_x - 2][map_y - 2], Map);
  std::cout << "---start path searching---" << std::endl;
  vector<Point*> final_path = astar.SolvePath();
  if (final_path.size() == 0) {
    return 0;
  }
  return 0;
}