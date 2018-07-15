#include "../include/global_planner_qt/costmapthread.h"
#include "../include/global_planner_qt/param.h"
#include <ros/ros.h>

namespace global_planner {

CostMapThread::CostMapThread()
{

}

void CostMapThread::run()
{
  while (!is_map_init || !is_roboter_pos_init) {
    sleep(1);
  }
  ros::Rate loop_rate(1);
  while (ros::ok()) {
    calc_costMap();
    loop_rate.sleep();
  }
}

void CostMapThread::calc_costMap()
{
  int redRange = 15;
  int orangeRange = 16;
  int Upper = orangeRange * 15 + 120;
  int roboter_row = roboter_pos.first;
  int roboter_col = roboter_pos.second;
  for (int row = roboter_row - costMap_area; row < roboter_row + costMap_area; row++) {
    for (int col = roboter_col - costMap_area; col < roboter_col + costMap_area; col++) {
      if (gridMap[row][col] >= 50 && gridMap[row][col] <= 100) {   // Block
        for (int n = -redRange; n <= redRange; n++) {
          for (int m = -redRange; m <= redRange; m++) {
            if (gridMap[row+n][col+m] != 100 && gridMap[row+n][col+m] != 110) {
              gridMap[row+n][col+m] = 110;    // 110 indicate red area
            }
          }
        }
        for (int i = 1; i <= orangeRange; i++) {
          draw_costMap(row, col, i + redRange, Upper - i * 15);
        }
      }
    }
  }
}

void CostMapThread::draw_costMap(int row, int col, int n, int value)
{
  for (int i = col - n; i <= col + n; i++) {
    if (gridMap[row-n][i] != 100 && gridMap[row-n][i] != 110) {
      if (gridMap[row-n][i] >= 120) {   // 120 is lowest value for orange area
        if (value > gridMap[row-n][i]) {
          gridMap[row-n][i] = value;
        }
      } else {
        gridMap[row-n][i] = value;    // 0 or -1
      }
    }
    if (gridMap[row+n][i] != 100 && gridMap[row+n][i] != 110) {
      if (gridMap[row+n][i] >= 120) {   // 120 is lowest value for orange area
        if (value > gridMap[row+n][i]) {
          gridMap[row+n][i] = value;
        }
      } else {
        gridMap[row+n][i] = value;    // 0 or -1
      }
    }
  }
  for (int i = row - n; i <= row + n; i++) {
    if (gridMap[i][col-n] != 100 && gridMap[i][col-n] != 110) {
      if (gridMap[i][col-n] >= 120) {   // 120 is lowest value for orange area
        if (value > gridMap[i][col-n]) {
          gridMap[i][col-n] = value;
        }
      } else {
        gridMap[i][col-n] = value;    // 0 or -1
      }
    }
    if (gridMap[i][col+n] != 100 && gridMap[i][col+n] != 110) {
      if (gridMap[i][col+n] >= 120) {   // 120 is lowest value for orange area
        if (value > gridMap[i][col+n]) {
          gridMap[i][col+n] = value;
        }
      } else {
        gridMap[i][col+n] = value;    // 0 or -1
      }
    }
  }

}

}

