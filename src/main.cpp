/*
 * @Author: 董泰宏 2396203400@qq.com
 * @Date: 2023-04-14 10:24:18
 * @LastEditors: 董泰宏 2396203400@qq.com
 * @LastEditTime: 2023-05-04 16:25:07
 * @FilePath: /AstarVisualization/src/main.cpp
 * @Description:
 * Copyright (c) 2023 by 董泰宏 email: 2396203400@qq.com, All Rights Reserved.
 */
#include "astar.h"

int main(int argc, const char* argv[]) {
  // 1.随机生成障碍物，需要注意：很可能会生成无解的地图，所以看到地图无解时就重新运行此程序
  std::cout << "---initializing map---" << std::endl;
  char mapdata[map_x][map_y];
  for (int i = 0; i < map_x; i++) {
    for (int j = 0; j < map_y; j++) {
      mapdata[i][j] = '1';
    }
  }
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> disRow(0, map_x - 1);
  std::uniform_int_distribution<> disCol(0, map_y - 1);
  const double prob = 0.3;
  for (int i = 0; i < map_x; ++i) {
    for (int j = 0; j < map_y; ++j) {
      double randNum = static_cast<double>(disRow(gen)) / (map_x - 1);
      if (randNum < prob) {
        cv::rectangle(visualMap,
                      cv::Rect(i * cell_w, j * cell_h, cell_w, cell_h),
                      cv::Scalar(0, 0, 0), -1);
        mapdata[i][j] = '0';
      }
    }
  }
  mapdata[0][0] = '1';                  //起点不能有障碍物
  mapdata[map_x - 2][map_y - 2] = '1';  //终点不能有障碍物
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

  // 2.开始搜索全局路径
  std::cout << "---start path searching---" << std::endl;
  Astar astar(Map[0][0], Map[map_x - 2][map_y - 2], Map);
  vector<Point*> final_path = astar.SolvePath();
  if (final_path.size() == 0) {
    return 0;
  }
  return 0;
}