/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2016 Bitcraze AB
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
 *
 */
#ifndef POSITION_CONTROLLER_MELLINGER_H_
#define POSITION_CONTROLLER_MELLINGER_H_

#include "stabilizer_types.h"
#include "trajectory.h"

#ifdef __cplusplus
extern "C" {
#endif

void positionControllerMellingerUpdate(
  const pose_t* poseEstimate,       // current state
  const trajectoryPoint_t* target,  // target state
  float dt,                         // dt since last call [s]
  float* eulerRollDesired,          // output [deg]
  float* eulerPitchDesired,         // output [deg]
  float* eulerYawDesired,           // output [deg]
  uint16_t* thrustDesired           // output
);

#ifdef __cplusplus
}
#endif

#endif /* POSITION_CONTROLLER_MELLINGER_H_ */
