/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012 BitCraze AB
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
 * trajectory.h - Module to receive trajectory from the host
 */

#ifndef __TRAJECTORY_H__
#define __TRAJECTORY_H__

#include <stdbool.h>
#include <stdint.h>

/* Public functions */
void trajectoryInit(void);
bool trajectoryTest(void);

// Basic trajectory point containing target position and velocity
typedef struct {
  float x;            // m
  float y;            // m
  float z;            // m
  float velocity_x;   // m/s
  float velocity_y;   // m/s
  float velocity_z;   // m/s
  float yaw;          // rad
} trajectoryPoint_t;

// Sets the current goal of the active trajectory
void trajectoryGetCurrentGoal(trajectoryPoint_t* goal);

typedef enum {
  TRAJECTORY_STATE_IDLE = 0,
  TRAJECTORY_STATE_FLYING = 1,
} trajectoryState_t;

void trajectoryGetState(trajectoryState_t* state);


#endif /* __TRAJECTORY_H__ */
