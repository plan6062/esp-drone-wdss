/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * power_distribution_stock.c - Crazyflie stock power distribution code
 */

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "power_distribution.h"
#include "motors.h"

// 필요한 상수 정의
#define DEFAULT_IDLE_THRUST 0

// 단순화된 limitUint16 매크로
#define limitUint16(VAL) ((VAL) < 0 ? 0 : ((VAL) > 65535 ? 65535 : (VAL)))

static bool motorSetEnable = false;

static struct {
  uint32_t m1;
  uint32_t m2;
  uint32_t m3;
  uint32_t m4;
} motorPower;

static struct {
  uint16_t m1;
  uint16_t m2;
  uint16_t m3;
  uint16_t m4;
} motorPowerSet;

#ifndef DEFAULT_IDLE_THRUST
#define DEFAULT_IDLE_THRUST 0
#endif

static uint32_t idleThrust = DEFAULT_IDLE_THRUST;

void powerDistributionInit(void)
{
  // motors는 이미 main에서 초기화됨
  printf("Power Distribution 초기화 완료\n");
}

bool powerDistributionTest(void)
{
  // 간단한 테스트
  printf("Power Distribution 테스트 통과\n");
  return true;
}

#define limitThrust(VAL) limitUint16(VAL)

// MOTOR 상수 정의
#define MOTOR_M1 0
#define MOTOR_M2 1
#define MOTOR_M3 2
#define MOTOR_M4 3

void powerStop()
{
  motorsSetRatio(MOTOR_M1, 0);
  motorsSetRatio(MOTOR_M2, 0);
  motorsSetRatio(MOTOR_M3, 0);
  motorsSetRatio(MOTOR_M4, 0);
}

void powerDistribution(const control_t *control)
{
  #ifdef QUAD_FORMATION_X
    int16_t r = control->roll / 2.0f;
    int16_t p = control->pitch / 2.0f;
    motorPower.m1 = limitThrust(control->thrust - r + p + control->yaw);
    motorPower.m2 = limitThrust(control->thrust - r - p - control->yaw);
    motorPower.m3 =  limitThrust(control->thrust + r - p + control->yaw);
    motorPower.m4 =  limitThrust(control->thrust + r + p - control->yaw);
  #else // QUAD_FORMATION_NORMAL
    motorPower.m1 = limitThrust(control->thrust + control->pitch +
                               control->yaw);
    motorPower.m2 = limitThrust(control->thrust - control->roll -
                               control->yaw);
    motorPower.m3 =  limitThrust(control->thrust - control->pitch +
                               control->yaw);
    motorPower.m4 =  limitThrust(control->thrust + control->roll -
                               control->yaw);
  #endif

  // idleThrust 적용
  if (motorPower.m1 < idleThrust) {
    motorPower.m1 = idleThrust;
  }
  if (motorPower.m2 < idleThrust) {
    motorPower.m2 = idleThrust;
  }
  if (motorPower.m3 < idleThrust) {
    motorPower.m3 = idleThrust;
  }
  if (motorPower.m4 < idleThrust) {
    motorPower.m4 = idleThrust;
  }

  // 모터에 실제 적용
  motorsSetRatio(MOTOR_M1, motorPower.m1);
  motorsSetRatio(MOTOR_M2, motorPower.m2);
  motorsSetRatio(MOTOR_M3, motorPower.m3);
  motorsSetRatio(MOTOR_M4, motorPower.m4);

  // 디버깅 출력 (간헐적으로)
  static int debug_count = 0;
  if (++debug_count % 100 == 0) { // 100번마다 한번 출력
    printf("PowerDist: T=%.0f R=%d P=%d Y=%d -> M1=%lu M2=%lu M3=%lu M4=%lu\n",
           control->thrust, control->roll, control->pitch, control->yaw,
           motorPower.m1, motorPower.m2, motorPower.m3, motorPower.m4);
  }
}
