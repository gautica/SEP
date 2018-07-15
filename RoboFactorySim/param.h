#ifndef PARAM_H
#define PARAM_H
#include <vector>

enum Product
{
  NO_PRODUCT = -1,
  BLUE_PRODUCT = 0,
  YELLOW_PRODUCT = 1,
  RED_PRODUCT = 2,
  VIOLET_PRODUCT = 3,
  ORANGE_PRODUCT = 4,
  GREEN_PRODUCT = 5,
  BLACK_PRODUCT = 6
};

enum GameMode
{
  KI_VS_KI = 0,
  PLAYER_VS_PLAYER = 1,
  AS_TEAM = 2
};

extern bool gamepadEnable;
extern std::vector<unsigned int> products;

#endif // PARAM_H
