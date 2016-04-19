/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
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
 * position_controller_mellinger.c: Position controller based on Mellingers work
 */

#include <math.h>

// #include "commander.h"
// #include "log.h"
// #include "param.h"
// #include "pid.h"
// #include "num.h"
#include "position_controller_mellinger.h"

typedef struct {
  float x;
  float y;
  float z;
} vector3f;

// computes the dot-product between the two vectors
static float dot(
  const vector3f* a,
  const vector3f* b)
{
  return a->x * b->x + a->y * b->y + a->z * b->z;
}
// computes vector norm
static float norm(
  const vector3f* a)
{
  return sqrt(pow(a->x, 2) + pow(a->y, 2) + pow(a->z, 2));
}

static void normalize(
  vector3f* a)
{
  float n = norm(a);
  if (n > 0) {
    a->x /= n;
    a->y /= n;
    a->z /= n;
  }
}

// c = axb
static void cross(
  const vector3f* a,
  const vector3f* b,
  vector3f* c)
{
  c->x = a->y * b->z - a->z * b->y;
  c->y = a->z * b->x - a->x * b->z;
  c->z = a->x * b->y - a->y * b->x;
}

static vector3f oldPosition;

static bool firstUpdate = true;

static const float g = 9.81;
static const float mass = 0.030;

static float kp_xy = 0.3;
static float kd_xy = 0.2;
static float kp_z = 0.2;
static float kd_z = 0.1;

static float massThrust = 130500;

void positionControllerMellingerUpdate(
  const pose_t* poseEstimate,       // current state
  const trajectoryPoint_t* target,  // target state
  float dt,                         // dt since last call [s]
  float* eulerRollDesired,          // output [deg]
  float* eulerPitchDesired,         // output [deg]
  float* eulerYawDesired,           // output [deg]
  uint16_t* thrustDesired           // output
  )
{
  vector3f velocity;
  vector3f r_error;
  vector3f v_error;
  vector3f target_thrust;
  vector3f z_axis;
  float current_thrust;
  vector3f z_axis_desired;
  vector3f x_axis_desired;
  vector3f y_axis_desired;
  vector3f x_c_des;

  // Current velocity
  if (!firstUpdate
      && dt > 0) {
    velocity.x = (poseEstimate->position.x - oldPosition.x) / dt;
    velocity.y = (poseEstimate->position.y - oldPosition.y) / dt;
    velocity.z = (poseEstimate->position.z - oldPosition.z) / dt;
  }

  // Position Error
  r_error.x = target->x - poseEstimate->position.x;
  r_error.y = target->y - poseEstimate->position.y;
  r_error.z = target->z - poseEstimate->position.z;

  // Velocity Error
  v_error.x = target->velocity_x - velocity.x;
  v_error.y = target->velocity_y - velocity.y;
  v_error.z = target->velocity_z - velocity.z;

  // Desired thrust (ignoring target accellerations)
  target_thrust.x = kp_xy * r_error.x + kd_xy * v_error.x + mass * 0;
  target_thrust.y = kp_xy * r_error.y + kd_xy * v_error.y + mass * 0;
  target_thrust.z = kp_z * r_error.z + kd_z * v_error.z + mass * g;

  // Z-Axis
  z_axis.x = -sin(poseEstimate->attitude.pitch) * cos(poseEstimate->attitude.roll);
  z_axis.y = sin(poseEstimate->attitude.roll);
  z_axis.z = cos(poseEstimate->attitude.pitch) * cos(poseEstimate->attitude.roll);

  // Current thrust
  current_thrust = massThrust * dot(&target_thrust, &z_axis);

  // Calculate axis
  z_axis_desired = target_thrust;
  normalize(&z_axis_desired);

  // x_axis_desired = z_axis_desired x [sin(yaw), cos(yaw), 0]^T
  x_c_des.x = cos(target->yaw);
  x_c_des.y = sin(target->yaw);
  x_c_des.z = 0;
  cross(&z_axis_desired, &x_c_des, &y_axis_desired);
  normalize(&y_axis_desired);

  cross(&y_axis_desired, &z_axis_desired, &x_axis_desired);
  // cross(&z_axis_desired, &tmp, &x_axis_desired);

  // y_axis_desired = z_axis_desired x x_axis_desired
  // cross(&z_axis_desired, &x_axis_desired, &y_axis_desired);

  // Output
  *eulerRollDesired = atan2(y_axis_desired.z, z_axis_desired.z) * 180.0 / M_PI;
  *eulerPitchDesired = -asin(x_axis_desired.z) * 180.0 / M_PI;
  *eulerYawDesired = target->yaw * 180.0 / M_PI; // assuming we have direct control!
  *thrustDesired = current_thrust;

  // update state
  oldPosition.x = poseEstimate->position.x;
  oldPosition.y = poseEstimate->position.y;
  oldPosition.z = poseEstimate->position.z;
  firstUpdate = false;
}

// PARAM_GROUP_START(ctrlMel)
// PARAM_ADD(PARAM_FLOAT, kp, &kp)
// PARAM_ADD(PARAM_FLOAT, kd, &kd)
// PARAM_ADD(PARAM_FLOAT, massThrust, &massThrust)
// PARAM_GROUP_STOP(ctrlMel)
