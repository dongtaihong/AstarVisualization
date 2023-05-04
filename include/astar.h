/*
 * @Author: 董泰宏 2396203400@qq.com
 * @Date: 2023-04-14 10:24:39
 * @LastEditors: 董泰宏 2396203400@qq.com
 * @LastEditTime: 2023-05-04 15:59:21
 * @FilePath: /AstarVisualization/include/astar.h
 * @Description:
 * Copyright (c) 2023 by 董泰宏 email: 2396203400@qq.com, All Rights Reserved.
 */
#pragma once
#include <algorithm>
#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <random>
#include <vector>
using std::vector;

//定义地图的珊格数
constexpr int map_x = 30;
constexpr int map_y = 30;

//定义画图的格子像素大小与总尺寸
constexpr int cell_w = 20;
constexpr int cell_h = 20;
constexpr int map_w = map_x * cell_w;
constexpr int map_h = map_y * cell_h;
extern cv::Mat visualMap;

enum class Type { FRESH, OPENED, CLOSED, OBSTACLE };

//定义每个格子
class Point {
 public:
  Point();
  ~Point() = default;
  int x;
  int y;
  Type type;
  int f;  //寻路的总代价
  int g;  //到起始点的距离
  int h;  //到终点的启发距离
  Point* parent;
  bool operator==(const Point* p) {
    if (x == p->x && y == p->y) return true;
    return false;
  }
};

// astar算法
class Astar {
 public:
  Astar(Point* start_p, Point* end_p, vector<vector<Point*>> Map);
  ~Astar() = default;
  vector<Point*> SolvePath();

 private:
  int getH(Point* point);
  int getF(Point* point);
  vector<Point*> getNeighborPoint(Point* point);
  Point* start;
  Point* end;
  Point* cur;
  vector<vector<Point*>> map;
  vector<Point*> OpenList;
  vector<Point*> CloseList;
  vector<Point*> NeighborList;
};