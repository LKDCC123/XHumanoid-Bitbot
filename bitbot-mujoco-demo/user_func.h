#pragma once

#include "bitbot_mujoco/kernel/mujoco_kernel.hpp"
#include "bitbot_mujoco/device/mujoco_joint.h"
#include "bitbot_mujoco/device/mujoco_force_sensor.h"
#include "bitbot_mujoco/device/mujoco_imu.h"

#define _USE_MATH_DEFINES
#include <math.h>

enum Events
{
  Wait = 1001,
  Test,
  Add,
  Minus
};

enum class States : bitbot::StateId
{
  Waiting = 1001,
  JointSinPos
};

struct UserData
{
  double sin = 10;
};

using Kernel = bitbot::MujocoKernel<UserData, "time", "sin">;

void ConfigFunc(const bitbot::MujocoBus& bus, UserData&);

std::optional<bitbot::StateId> EventWait(bitbot::EventValue value, UserData& user_data);
std::optional<bitbot::StateId> EventTest(bitbot::EventValue value, UserData& user_data);
std::optional<bitbot::StateId> EventAdd(bitbot::EventValue value, UserData& user_data);
std::optional<bitbot::StateId> EventMinus(bitbot::EventValue value, UserData& user_data);

void StateWaiting(const bitbot::KernelInterface& kernel, Kernel::ExtraData& extra_data, UserData& user_data);

void StateJointSinPos(const bitbot::KernelInterface& kernel, Kernel::ExtraData& extra_data, UserData& user_data);

void FinishFunc(UserData&);
