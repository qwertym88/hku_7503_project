#pragma once
#include "lifter_control.h"

// Driver owns positional PWM and settling deadlines in an independent 10ms task.
void Lifter_Init(void);
void Lifter_Heartbeat(void);
lifter_result_t Lifter_Command(uint8_t value);
void Lifter_Stop(lifter_fault_t reason);
void Lifter_Get_Snapshot(lifter_control_t *out);
bool Lifter_Blocks_Chassis(void);
bool Lifter_Is_Moving(void);
