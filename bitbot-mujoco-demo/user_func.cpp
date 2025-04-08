#include "user_func.h"

#include <chrono>
#include <memory>

bitbot::MujocoJoint *joint_x, *joint_y = nullptr;

void ConfigFunc(const bitbot::MujocoBus& bus, UserData&)
{
  joint_x = bus.GetDevice<bitbot::MujocoJoint>(1).value();
  joint_y = bus.GetDevice<bitbot::MujocoJoint>(2).value();
}

std::optional<bitbot::StateId> EventWait(bitbot::EventValue, UserData&)
{
  return static_cast<bitbot::StateId>(States::Waiting);
}

std::optional<bitbot::StateId> EventTest(bitbot::EventValue value, UserData&)
{
  if(value == static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up))
    return static_cast<bitbot::StateId>(States::JointSinPos);
  else
    return std::nullopt;
}

std::optional<bitbot::StateId> EventAdd(bitbot::EventValue value, UserData& user_data)
{
  if(user_data.sin < 60)
    user_data.sin += 1;
  
  return std::nullopt;
}

std::optional<bitbot::StateId> EventMinus(bitbot::EventValue value, UserData& user_data)
{
  if(user_data.sin > 1)
    user_data.sin -= 1;
  
  return std::nullopt;
}

void StateWaiting(const bitbot::KernelInterface& kernel, Kernel::ExtraData& extra_data, UserData& user_data)
{
  
}

void StateJointSinPos(const bitbot::KernelInterface& kernel, Kernel::ExtraData& extra_data, UserData& user_data)
{
  constexpr double deg2rad = M_PI / 180.0;
  constexpr double rad2deg = 180.0 / M_PI;

  static double start_time = 0;
  static bool init = false;

  if(joint_x->GetActualPosition()*rad2deg > 20)
  {
    std::cout << "stop!" << std::endl;
    kernel.EmitEvent(static_cast<bitbot::EventId>(bitbot::KernelEvent::STOP), 0);
  }

  if(!init)
  {
    start_time = kernel.GetPeriodsCount()*0.004;
    init=true;
  }

  double t = kernel.GetPeriodsCount()*0.004 - start_time;

  joint_x->SetTargetPosition(user_data.sin*deg2rad*sin(4*M_PI*t));
  extra_data.Set<0>(t);
  extra_data.Set<"sin">(user_data.sin);
}

void FinishFunc(UserData&)
{
  std::cout << "finish" << std::endl;
}
