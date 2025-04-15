#pragma once

#include "bitbot_mujoco/device/mujoco_force_sensor.h"
#include "bitbot_mujoco/device/mujoco_framelinvel.h"
#include "bitbot_mujoco/device/mujoco_imu.h"
#include "bitbot_mujoco/device/mujoco_joint.h"
#include "bitbot_mujoco/kernel/mujoco_kernel.hpp"

#define _USE_MATH_DEFINES
#include <math.h>

#include "LLog.hpp"
#include "eigen3/Eigen/Dense"
// #include "vicon_capture.hpp"

extern lee::blocks::LLog<float> Logger_More;

enum Events {
  SinPos = 1001,
  SwitchMode,
  InitPose,
  InitForData,
  OriginTest,
  StepFunction,
  PolicyRun,
  CompensationTest,
  AnkleTest,

  VeloxIncrease = 2001,
  VeloxDecrease = 2002,
  VeloyIncrease = 2003,
  VeloyDecrease = 2004,
  YOffsetIncrease = 2010,
  YOffsetDecrease = 2011,
  YawOffsetIncrease = 2012,
  YawOffsetDecrease = 2013,

  ContinousVeloXChange = 3001,
  ContinousVeloYawChange = 3003,

  ContinousOffsetYChange = 3011,
  ContinousOffsetYawChange = 3012
};

enum class States : bitbot::StateId {
  Waiting = 1001,
  PF2SinPos,
  PF2SwitchMode,
  PF2InitPose,
  PF2InitForData,
  PF2OriginTest,
  PF2StepFunction,
  PF2PolicyRun,
  PF2CompensationTest,
  AnkleTest
};

struct UserData {
  // Joint pos
  double l_p_pos = 0.0;
  double l_r_pos = 0.0;
  double r_p_pos = 0.0;
  double r_r_pos = 0.0;
  // Joint vel
  double l_p_vel = 0.0;
  double l_r_vel = 0.0;
  double r_p_vel = 0.0;
  double r_r_vel = 0.0;
};

using Kernel =
    bitbot::MujocoKernel<UserData, "l_p_pos", "l_p_vel", "l_r_pos", "l_r_vel",
                         "r_p_pos", "r_p_vel", "r_r_pos", "r_r_vel">;

using KernelBus = bitbot::MujocoBus;
using DeviceImu = bitbot::MujocoImu;
using DeviceJoint = bitbot::MujocoJoint;

std::optional<bitbot::StateId> EventSinPos(bitbot::EventValue value,
                                           UserData &user_data);
std::optional<bitbot::StateId> EventInitPose(bitbot::EventValue value,
                                             UserData &user_data);
std::optional<bitbot::StateId> EventSwitchMode(bitbot::EventValue value,
                                               UserData &user_data);
std::optional<bitbot::StateId> EventInitForData(bitbot::EventValue value,
                                                UserData &user_data);
std::optional<bitbot::StateId> EventOriginTest(bitbot::EventValue value,
                                               UserData &user_data);
std::optional<bitbot::StateId> EventStepFunction(bitbot::EventValue value,
                                                 UserData &user_data);
std::optional<bitbot::StateId> EventPolicyRun(bitbot::EventValue value,
                                              UserData &user_data);
std::optional<bitbot::StateId> EventCompensationTest(bitbot::EventValue value,
                                                     UserData &user_data);
std::optional<bitbot::StateId> EventKdAdd(bitbot::EventValue value,
                                          UserData &user_data);
std::optional<bitbot::StateId> EventKdDecrease(bitbot::EventValue value,
                                               UserData &user_data);

std::optional<bitbot::StateId> EventAnkleTest(bitbot::EventValue value,
                                              UserData &user_data);

std::optional<bitbot::StateId> EventVeloXIncrease(bitbot::EventValue keyState,
                                                  UserData &);
std::optional<bitbot::StateId> EventVeloXDecrease(bitbot::EventValue keyState,
                                                  UserData &);
std::optional<bitbot::StateId> EventVeloYIncrease(bitbot::EventValue keyState,
                                                  UserData &);
std::optional<bitbot::StateId> EventVeloYDecrease(bitbot::EventValue keyState,
                                                  UserData &);

std::optional<bitbot::StateId> EventYOffsetIncrease(bitbot::EventValue KeyState,
                                                    UserData &);
std::optional<bitbot::StateId> EventYOffsetDecrease(bitbot::EventValue KeyState,
                                                    UserData &);
std::optional<bitbot::StateId>
EventYawOffsetIncrease(bitbot::EventValue KeyState, UserData &);
std::optional<bitbot::StateId>
EventYawOffsetDecrease(bitbot::EventValue KeyState, UserData &);

std::optional<bitbot::StateId>
EventContinousVeloXChange(bitbot::EventValue KeyState, UserData &);
std::optional<bitbot::StateId>
EventContinousVeloYChange(bitbot::EventValue KeyState, UserData &);
std::optional<bitbot::StateId>
EventContinousVeloYawChange(bitbot::EventValue KeyState, UserData &);

std::optional<bitbot::StateId>
EventContinousOffsetXChange(bitbot::EventValue KeyState, UserData &);
std::optional<bitbot::StateId>
EventContinousOffsetYChange(bitbot::EventValue KeyState, UserData &);
std::optional<bitbot::StateId>
EventContinousOffsetYawChange(bitbot::EventValue KeyState, UserData &);

void ConfigFunc(const KernelBus &bus, UserData &);

void StateWaiting(const bitbot::KernelInterface &kernel,
                  Kernel::ExtraData &extra_data, UserData &user_data);

void StateJointSwitchMode(const bitbot::KernelInterface &kernel,
                          Kernel::ExtraData &extra_data, UserData &user_data);
// zyx-231007

void StateJointSinPos(const bitbot::KernelInterface &kernel,
                      Kernel::ExtraData &extra_data, UserData &user_data);

void StateJointInitPose_ForData(const bitbot::KernelInterface &kernel,
                                Kernel::ExtraData &extra_data,
                                UserData &user_data);
// zyx-231008

void StateJointInitPose(const bitbot::KernelInterface &kernel,
                        Kernel::ExtraData &extra_data, UserData &user_data);
// zyx-231008

void StateJointOriginTest(const bitbot::KernelInterface &kernel,
                          Kernel::ExtraData &extra_data, UserData &user_data);
// zyx-231012

void StateStepFunction(const bitbot::KernelInterface &kernel,
                       Kernel::ExtraData &extra_data, UserData &user_data);

void StatePolicyRun(const bitbot::KernelInterface &kernel,
                    Kernel::ExtraData &extra_data, UserData &user_data);
// zyx-231019

void StateCompensationTest(const bitbot::KernelInterface &kernel,
                           Kernel::ExtraData &extra_data, UserData &user_data);
// zyx-231103

void StateAnkleTest(const bitbot::KernelInterface &kernel,
                    Kernel::ExtraData &extra_data, UserData &user_data);

void TorqueController();
void CompensationController();
void PolicyController(uint64_t cur_time);
void InitPos_ForData(float current_time);

void InitPos(float current_time);

void InitPolicy();
// void CallPolicy();

void SetJointTorque_User();

void SinPosCompute(uint64_t start, uint64_t end);

void log_result(uint64_t cur_time);

void GetJointObservation(Kernel::ExtraData &extra_data);
void SetJointAction();
