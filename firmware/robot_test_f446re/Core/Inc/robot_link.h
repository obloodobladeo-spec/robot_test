#ifndef ROBOT_LINK_H
#define ROBOT_LINK_H
#include "main.h"
HAL_StatusTypeDef RobotLink_Init(void);
void RobotLink_Task(uint32_t now);
void RobotLink_IrqHandler(void);
#endif
