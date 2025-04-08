#include "bitbot_mujoco/kernel/mujoco_kernel.hpp"
#include "bitbot_mujoco/device/mujoco_joint.h"

#include "user_func.h"

#define _USE_MATH_DEFINES
#include <math.h>

int main(int argc, char const *argv[])
{
  // 定义kernel对象，参数为配置文件
  Kernel kernel("../../bitbot_pendulum.xml");

  // 注册Config函数
  kernel.RegisterConfigFunc(&ConfigFunc);
  // 注册Finish函数
  kernel.RegisterFinishFunc(&FinishFunc);

  // 注册event
  kernel.RegisterEvent("waiting", static_cast<bitbot::StateId>(Events::Wait), &EventWait, true);
  kernel.RegisterEvent("test", static_cast<bitbot::StateId>(Events::Test), &EventTest);
  kernel.RegisterEvent("add", static_cast<bitbot::StateId>(Events::Add), &EventAdd, true);
  kernel.RegisterEvent("minus", static_cast<bitbot::StateId>(Events::Minus), &EventMinus, true);

  // 注册state
  kernel.RegisterState("waiting", static_cast<bitbot::StateId>(States::Waiting), &StateWaiting, {
    static_cast<bitbot::StateId>(Events::Test)
  });
  kernel.RegisterState("joint_test", static_cast<bitbot::StateId>(States::JointSinPos), &StateJointSinPos, {});

  // 设置用户的第一个state
  kernel.SetFirstState(static_cast<bitbot::StateId>(States::Waiting));

  // 运行
  kernel.Run();
  
  return 0;
}
