#include "astar.h"

cv::Mat visualMap(map_h, map_w, CV_8UC3, cv::Scalar(255, 255, 255));

/**
 * @description: 建立网格地图
 * @return {*}
 */
void GridMap(Point* start_point, Point* end_point) {
  // 绘制水平方向的网格线
  for (int r = 0; r <= map_x; r++) {
    cv::Point p1(0, r * cell_h);
    cv::Point p2(map_w, r * cell_h);
    cv::line(visualMap, p1, p2, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
  }

  // 绘制垂直方向的网格线
  for (int c = 0; c <= map_y; c++) {
    cv::Point p1(c * cell_w, 0);
    cv::Point p2(c * cell_w, map_h);
    cv::line(visualMap, p1, p2, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
  }
  //起点、终点
  cv::rectangle(visualMap,
                cv::Rect(start_point->x * cell_w, start_point->y * cell_h,
                         cell_w, cell_h),
                cv::Scalar(0, 0, 255), -1);

  cv::rectangle(
      visualMap,
      cv::Rect(end_point->x * cell_w, end_point->y * cell_h, cell_w, cell_h),
      cv::Scalar(0, 0, 255), -1);
}

/**
 * @description: 将openlist,closelist等搜索过的格子用不同的颜色表示
 * @param {vector<Point*>} openlist
 * @param {vector<Point*>} closelist
 * @return {*}
 */
void painting(vector<Point*> openlist, vector<Point*> closelist,
              Point* current) {
  //先openlist
  for (auto i : openlist) {
    cv::rectangle(
        visualMap,
        cv::Rect(i->x * cell_w, i->y * cell_h, cell_w * 0.8, cell_h * 0.8),
        cv::Scalar(170, 216, 152), -1);
  }
  //再closelist
  for (auto i : closelist) {
    cv::rectangle(
        visualMap,
        cv::Rect(i->x * cell_w, i->y * cell_h, cell_w * 0.8, cell_h * 0.8),
        cv::Scalar(255, 164, 178), -1);
  }
  //最后cur point
  Point* tmp = current;
  while (tmp->parent != nullptr) {
    cv::rectangle(
        visualMap,
        cv::Rect(tmp->x * cell_w, tmp->y * cell_h, cell_w * 0.8, cell_h * 0.8),
        cv::Scalar(71, 41, 252), -1);
    tmp = tmp->parent;
  }
}

/**
 * @description: Point类的构造函数
 * @return {*}
 */
Point::Point()
    : x(0), y(0), type(Type::FRESH), f(0), g(0), h(0), parent(nullptr) {}

/**
 * @description: 初始化Astar算法，并传入起点、终点和地图
 * @param {Point*} start_p
 * @param {Point*} end_p
 * @param {vector<vector<Point*>>} Map
 * @return {*}
 */
Astar::Astar(Point* start_p, Point* end_p, vector<vector<Point*>> Map) {
  start = start_p;
  end = end_p;
  map = Map;
}

/**
 * @description: 启发函数——曼哈顿距离，如果启发函数设置为0，则A*退化成Dijkstra
 * @param {Point*} point
 * @return {*}
 */
int Astar::getH(Point* point) {
  return 1.5 * (abs(end->x - point->x) + abs(end->y - point->y));
  // return 0;
}

/**
 * @description: 计算F值
 * @param {Point*} point
 * @return {*}
 */
int Astar::getF(Point* point) { return point->g + getH(point); }

/**
 * @description: 找到point的全部邻居节点，如果是OBSTACLE则不加入
 * @param {Point*} point
 * @return {*}
 */
vector<Point*> Astar::getNeighborPoint(Point* point) {
  NeighborList.clear();
  if (point->x < map_x - 1) {
    if (map[point->x + 1][point->y]->type != Type::OBSTACLE) {
      NeighborList.push_back(map[point->x + 1][point->y]);
    }
  }
  if (point->x > 0) {
    if (map[point->x - 1][point->y]->type != Type::OBSTACLE) {
      NeighborList.push_back(map[point->x - 1][point->y]);
    }
  }
  if (point->y < map_y - 1) {
    if (map[point->x][point->y + 1]->type != Type::OBSTACLE) {
      NeighborList.push_back(map[point->x][point->y + 1]);
    }
  }
  if (point->y > 0) {
    if (map[point->x][point->y - 1]->type != Type::OBSTACLE) {
      NeighborList.push_back(map[point->x][point->y - 1]);
    }
  }
  return NeighborList;
}

/**
 * @description: A星算法具体实现
 * @return {*}
 */
vector<Point*> Astar::SolvePath() {
  //基本错误情况判断
  if (start == end) return vector<Point*>();
  if (end->type == Type::OBSTACLE) return vector<Point*>();
  //开始进行搜索
  OpenList.push_back(start);
  start->type = Type::OPENED;
  start->f = getF(start);
  int flag = 0;
  cv::namedWindow("A*", cv::WINDOW_AUTOSIZE);
  GridMap(start, end);
  while (OpenList.size() > 0) {
    cur = OpenList[0];
    OpenList.erase(OpenList.begin());
    cur->type = Type::CLOSED;
    CloseList.push_back(cur);
    if (cur == end) {
      std::cout << "找到最短路径" << flag << std::endl;

      //显示openlist用灰绿色，closelist用紫色，起点终点用红色，路径用红色，障碍物用黑色
      painting(OpenList, CloseList, cur);
      cv::imshow("A*", visualMap);
      cv::waitKey(50000);

      vector<Point*> globalPath;
      while (cur->parent != nullptr) {
        globalPath.push_back(cur);
        cv::rectangle(
            visualMap,
            cv::Rect(cur->x * cell_w, cur->y * cell_h, cell_w, cell_h),
            cv::Scalar(0, 0, 255), -1);
        cur = cur->parent;
      }
      globalPath.push_back(start);
      std::reverse(globalPath.begin(), globalPath.end());
      return globalPath;
    }
    vector<Point*> NeighborList = getNeighborPoint(cur);
    for (auto& p : NeighborList) {
      if (p->type == Type::CLOSED) {
        continue;
      }
      if (p->type != Type::OPENED) {
        p->parent = cur;
        p->g = cur->g + 1;
        p->h = getH(p);
        p->f = getF(p);
        OpenList.push_back(p);
        p->type = Type::OPENED;
      } else {
        if (p->g > cur->g + 1) {
          p->parent = cur;
          p->g = cur->g + 1;
        }
      }
    }
    sort(OpenList.begin(), OpenList.end(),
         [](const Point* p1, const Point* p2) { return p1->f < p2->f; });
    flag++;
    painting(OpenList, CloseList, cur);
    cv::imshow("A*", visualMap);
    //每一帧图片之间的间隔，20对应20ms，如果为了放慢速度可自行增大
    cv::waitKey(20);
  }
  std::cout << "该地图无解！" << std::endl;
  cv::waitKey(10000);
  return vector<Point*>();
}