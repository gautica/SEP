#include "../include/gui/htnplanner.h"
#include "../include/gui/param.h"

HTNPlanner::HTNPlanner()
{
  timeTable.resize(3);
  for (int i = 0; i < 3; i++) {
    timeTable[i].resize(6);
    for (int j = 0; j < 6; j++) {
      timeTable[i][j] = true;
    }
  }
}

void HTNPlanner::make_plan(int product)
{
  machine_chain.clear();
  machine_chain.push_back(WAREHOUSE);
  switch (product) {
  case BLUE_PRODUCT:
    machine_chain.push_back(BLUE_MACHINE_1);
    break;
  case RED_PRODUCT:
    machine_chain.push_back(RED_MACHINE_1);
    break;
  case YELLOW_PRODUCT:
    machine_chain.push_back(YELLOW_MACHINE_1);
    break;
  case VIOLET_PRODUCT:
    create_violet();
    break;
  case ORANGE_PRODUCT:
    create_orange();
    break;
  case GREEN_PRODUCT:
    create_green();
    break;
  case BLACK_PRODUCT:
    create_black();
    break;
  default:
    break;
  }
  update_table();
}

void HTNPlanner::create_violet()
{
  if (timeTable[0][0] == true && timeTable[1][2] == true) {
    machine_chain.push_back(RED_MACHINE_1);
    machine_chain.push_back(BLUE_MACHINE_1);
    wait_time = 0;
  } else if (timeTable[1][0] == true && timeTable[0][2] == true) {
    machine_chain.push_back(BLUE_MACHINE_1);
    machine_chain.push_back(RED_MACHINE_1);
    wait_time = 0;
  } else if (timeTable[0][0] == false && timeTable[1][2] == false && timeTable[1][0] == false && timeTable[0][2] == false ) {
    // must twice Wait
    machine_chain.push_back(RED_MACHINE_1);
    machine_chain.push_back(BLUE_MACHINE_1);
    wait_time = 2;
  } else {
    if (find_free(0, 0, 1, 2)) {
      machine_chain.push_back(RED_MACHINE_1);
      machine_chain.push_back(BLUE_MACHINE_1);
    } else {
      machine_chain.push_back(BLUE_MACHINE_1);
      machine_chain.push_back(RED_MACHINE_1);
    }
    wait_time = 1;
  }
}

void HTNPlanner::create_orange()
{
  if (timeTable[0][2] == true && timeTable[1][4] == true) {
    machine_chain.push_back(YELLOW_MACHINE_1);
    machine_chain.push_back(RED_MACHINE_1);
    wait_time = 0;
  } else if (timeTable[1][2] == true && timeTable[0][4] == true) {
    machine_chain.push_back(RED_MACHINE_1);
    machine_chain.push_back(YELLOW_MACHINE_1);
    wait_time = 0;
  } else if (timeTable[0][2] == false && timeTable[1][4] == false && timeTable[1][2] == false && timeTable[0][4] == false) {
    // must twice Wait
    machine_chain.push_back(YELLOW_MACHINE_1);
    machine_chain.push_back(RED_MACHINE_1);
    wait_time = 2;
  } else {
    if (find_free(0, 2, 1, 4)) {
      machine_chain.push_back(YELLOW_MACHINE_1);
      machine_chain.push_back(RED_MACHINE_1);
    } else {
      machine_chain.push_back(RED_MACHINE_1);
      machine_chain.push_back(YELLOW_MACHINE_1);
    }
    wait_time = 1;
  }
}

void HTNPlanner::create_green()
{
  if (timeTable[0][0] == true && timeTable[1][4] == true) {
    machine_chain.push_back(YELLOW_MACHINE_1);
    machine_chain.push_back(BLUE_MACHINE_1);
    wait_time = 0;
  } else if (timeTable[1][0] == true && timeTable[0][4] == true) {
    machine_chain.push_back(BLUE_MACHINE_1);
    machine_chain.push_back(YELLOW_MACHINE_1);
    wait_time = 0;
  } else if (timeTable[0][0] == false && timeTable[1][4] == false && timeTable[1][0] == false && timeTable[0][4] == false) {
    // must twice Wait
    machine_chain.push_back(YELLOW_MACHINE_1);
    machine_chain.push_back(BLUE_MACHINE_1);
    wait_time = 2;
  } else {
    if (find_free(0, 0, 1, 4)) {
      machine_chain.push_back(YELLOW_MACHINE_1);
      machine_chain.push_back(BLUE_MACHINE_1);
    } else {
      machine_chain.push_back(BLUE_MACHINE_1);
      machine_chain.push_back(YELLOW_MACHINE_1);
    }
    wait_time = 1;
  }
}

void HTNPlanner::create_black()
{
  create_green();
  int temp1 = wait_time;
  if (timeTable[2][2] == false) {
    temp1++;
  }

  create_orange();
  int temp2 = wait_time;
  if (timeTable[2][0] == false) {
    temp2++;
  }

  create_violet();
  int temp3 = wait_time;
  if (timeTable[2][4] == false) {
    temp3++;
  }
  machine_chain.clear();
  machine_chain.push_back(WAREHOUSE);
  if (temp1 < temp2 && temp1 < temp3) {
    machine_chain.push_back(RED_MACHINE_1);
    create_green();
  } else if (temp2 < temp3) {
    machine_chain.push_back(BLUE_MACHINE_1);
    create_orange();
  } else {
    machine_chain.push_back(YELLOW_MACHINE_1);
    create_violet();
  }
}

bool HTNPlanner::find_free(const int &row1, const int &col1, const int &row2, const int &col2)
{
  return timeTable[row1][col1] || timeTable[row2][col2];
}

void HTNPlanner::update_table()
{
  int index = 0;
  for (int i = machine_chain.size() - 1; i >= 0 ; i--) {
    switch (machine_chain[i]) {
      case BLUE_MACHINE_1:
        timeTable[index][0] = false;
        break;
      case RED_MACHINE_1:
        timeTable[index][2] = false;
        break;
      case YELLOW_MACHINE_1:
        timeTable[index][4] = false;
        break;
      default:
        break;
    }
    index++;
  }
}




