# AstarVisualization
使用opencv展示A*算法整个搜索过程
## 依赖
--opencv4.7.0
```shell
安装参照 https://blog.csdn.net/weixin_43863869/article/details/128552342
```
## 编译运行
```shell
git clone https://github.com/dongtaihong/AstarVisualization.git
cd AstarVisualization 
mkdir build && cd build
cmake .. && make
./astar
```
## 效果
**说明**：为了能够充分展示搜索的过程，在每一帧图片之间阻塞了20ms时间，具体阻塞时间可在atar.cpp的201行处调整  

![image](./image/astar.png)