#include "bitbot_cifx/kernel/cifx_kernel.hpp"
#include "bitbot_kernel/utils/cpu_affinity.h"
#include "bitbot_kernel/utils/priority.h"
#include "user_func.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <sys/mman.h>

#include <csignal>

void SignalHandler(int sig) {
  std::cout << "Signal received. Saving log..." << std::endl;
  auto now = std::chrono::system_clock::now();
  std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm *now_tm = std::localtime(&now_time);
  std::stringstream ss;
  ss << std::put_time(now_tm, "%Y-%m-%d-%H-%M-%S");
  std::string current_time = ss.str();
  std::string log_path =
      "/home/huahui/Project/hhfc-bitbot/log/extra/" + current_time + ".csv";
  std::string log_path_policy =
      "/home/huahui/Project/hhfc-bitbot/log/policy/" + current_time + ".csv";

  Logger_More.saveLog(log_path.c_str());
  Logger_Policy.saveLog(log_path_policy.c_str());

  std::cout << "Log is saved. Exiting..." << std::endl;

  exit(1);
};

int main(int argc, char const *argv[]) {
  if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
    printf("mlockall failed: \n");
    if (errno == ENOMEM) {
      printf(
          "\nIt is likely your user does not have enough memory limits, you "
          "can change the limits by adding the "
          "following line to /etc/security/limits.conf:\n\n");
    }
    return -1;
  }
  if (!setProcessHighPriority(99)) {
    printf("Failed to set process scheduling policy\n");
    return -1;
  }
  // int cores[4] = {0, 1, 2, 3};
  // StickThisThreadToCores(cores, 4);

  Kernel kernel("/home/huahui/Project/hhfc-bitbot/pf2.xml");

  kernel.RegisterConfigFunc(ConfigFunc);

  // 注册 Event
  kernel.RegisterEvent("switch_cst",
                       static_cast<bitbot::EventId>(Events::SwitchMode),
                       &EventSwitchMode);
  kernel.RegisterEvent("sin_pos", static_cast<bitbot::EventId>(Events::SinPos),
                       &EventSinPos);
  kernel.RegisterEvent("init_pose",
                       static_cast<bitbot::EventId>(Events::InitPose),
                       &EventInitPose);
  kernel.RegisterEvent("init_pose_for_data",
                       static_cast<bitbot::EventId>(Events::InitForData),
                       &EventInitForData);
  kernel.RegisterEvent("origin_test",
                       static_cast<bitbot::EventId>(Events::OriginTest),
                       &EventOriginTest);
  /*kernel.RegisterEvent("step_function",*/
  /*                     static_cast<bitbot::EventId>(Events::StepFunction),*/
  /*                     &EventStepFunction);*/
  kernel.RegisterEvent("policy_run",
                       static_cast<bitbot::EventId>(Events::PolicyRun),
                       &EventPolicyRun);
  kernel.RegisterEvent("compensation_test",
                       static_cast<bitbot::EventId>(Events::CompensationTest),
                       &EventCompensationTest);

  // 注册速度控制器
  kernel.RegisterEvent("velo_x_increase",
                       static_cast<bitbot::EventId>(Events::VeloxIncrease),
                       &EventVeloXIncrease);
  kernel.RegisterEvent("velo_x_decrease",
                       static_cast<bitbot::EventId>(Events::VeloxDecrease),
                       &EventVeloXDecrease);
  kernel.RegisterEvent("velo_y_increase",
                       static_cast<bitbot::EventId>(Events::VeloyIncrease),
                       &EventVeloYIncrease);
  kernel.RegisterEvent("velo_y_decrease",
                       static_cast<bitbot::EventId>(Events::VeloyDecrease),
                       &EventVeloYDecrease);

  kernel.RegisterEvent("velo_yaw_increase",
                       static_cast<bitbot::EventId>(Events::VeloyawIncrease),
                       &EventVeloYawIncrease);
  kernel.RegisterEvent("velo_yaw_decrease",
                       static_cast<bitbot::EventId>(Events::VeloyawDecrease),
                       &EventVeloYawDecrease);

  // 注册补偿控制器
  kernel.RegisterEvent("offset_yaw_increase",
                       static_cast<bitbot::EventId>(Events::YawOffsetIncrease),
                       &EventYawOffsetIncrease);
  kernel.RegisterEvent("offset_yaw_decrease",
                       static_cast<bitbot::EventId>(Events::YawOffsetDecrease),
                       &EventYawOffsetDecrease);
  kernel.RegisterEvent("offset_y_increase",
                       static_cast<bitbot::EventId>(Events::YOffsetIncrease),
                       &EventYOffsetIncrease);
  kernel.RegisterEvent("offset_y_decrease",
                       static_cast<bitbot::EventId>(Events::YOffsetDecrease),
                       &EventYOffsetDecrease);

  // 注册手柄摇杆控制
  kernel.RegisterEvent(
      "velo_x_continuous",
      static_cast<bitbot::EventId>(Events::ContinousVeloXChange),
      &EventContinousVeloXChange);
  // kernel.RegisterEvent("",static_cast<bitbot::EventId>(Events::ContinousVeloYChange),&EventContinousVeloYChange);
  kernel.RegisterEvent(
      "velo_yaw_continuous",
      static_cast<bitbot::EventId>(Events::ContinousVeloYawChange),
      &EventContinousVeloYawChange);

  // Ankle Test event
  kernel.RegisterEvent("ankle_test",
                       static_cast<bitbot::EventId>(Events::AnkleTest),
                       &EventAnkleTest);
  kernel.RegisterEvent("joint_sin_test",
                       static_cast<bitbot::EventId>(Events::JointSinTest),
                       &EventJointSinTest);

  // I don't know what does this mean. I'm not care. I just want to write
  // something here. I don't know whether this code contains bug. If it does, oh
  // I'm so sad for that, but this commit is not the reason.
  kernel.RegisterEvent("hip_p_inc",
                       static_cast<bitbot::EventId>(Events::HipPitchIncrease),
                       &EventHipPitchIncrease);
  kernel.RegisterEvent("hip_p_dec",
                       static_cast<bitbot::EventId>(Events::HipPitchDecrease),
                       &EventHipPitchDecrease);
  kernel.RegisterEvent("knee_p_inc",
                       static_cast<bitbot::EventId>(Events::KneePitchIncrease),
                       &EventKneePitchIncrease);
  kernel.RegisterEvent("knee_p_dec",
                       static_cast<bitbot::EventId>(Events::KneePitchDecrease),
                       &EventKneePitchDecrease);
  kernel.RegisterEvent("ankle_p_inc",
                       static_cast<bitbot::EventId>(Events::AnklePitchIncrease),
                       &EventAnklePitchIncrease);
  kernel.RegisterEvent("ankle_p_dec",
                       static_cast<bitbot::EventId>(Events::AnklePitchDecrease),
                       &EventAnklePitchDecrease);

  // 注册 State
  kernel.RegisterState("waiting", static_cast<bitbot::StateId>(States::Waiting),
                       &StateWaiting,
                       {
                           static_cast<bitbot::EventId>(Events::SwitchMode),
                           static_cast<bitbot::EventId>(Events::InitPose),
                           static_cast<bitbot::EventId>(Events::AnkleTest),
                           static_cast<bitbot::EventId>(Events::JointSinTest),
                       });

  kernel.RegisterState("switch_cst",
                       static_cast<bitbot::StateId>(States::PF2SwitchMode),
                       &StateJointSwitchMode,
                       {static_cast<bitbot::EventId>(Events::SinPos),
                        (Events::OriginTest), (Events::InitPose)});

  kernel.RegisterState("init_pose",
                       static_cast<bitbot::StateId>(States::PF2InitPose),
                       &StateJointInitPose,
                       {
                           static_cast<bitbot::EventId>(Events::SinPos),
                           /*(Events::StepFunction),*/
                           (Events::PolicyRun),
                           (Events::CompensationTest),
                           (Events::JointSinTest),
                           Events::HipPitchIncrease,
                           Events::HipPitchDecrease,
                           Events::KneePitchIncrease,
                           Events::KneePitchDecrease,
                           Events::AnklePitchIncrease,
                           Events::AnklePitchDecrease,
                       });

  kernel.RegisterState("init_pose_for_data",
                       static_cast<bitbot::StateId>(States::PF2InitForData),
                       &StateJointInitPose_ForData,
                       {static_cast<bitbot::EventId>(Events::SinPos)});

  kernel.RegisterState("sin_pos",
                       static_cast<bitbot::StateId>(States::PF2SinPos),
                       &StateJointSinPos, {});

  kernel.RegisterState("origin_test",
                       static_cast<bitbot::StateId>(States::PF2OriginTest),
                       &StateJointOriginTest, {});

  /*kernel.RegisterState("step_function",*/
  /*                     static_cast<bitbot::StateId>(States::PF2StepFunction),*/
  /*                     &StateStepFunction, {});*/

  kernel.RegisterState(
      "policy_run", static_cast<bitbot::StateId>(States::PF2PolicyRun),
      &StatePolicyRun,
      {static_cast<bitbot::EventId>(Events::VeloxDecrease),
       static_cast<bitbot::EventId>(Events::VeloxIncrease),
       static_cast<bitbot::EventId>(Events::VeloyDecrease),
       static_cast<bitbot::EventId>(Events::VeloyIncrease),
       static_cast<bitbot::EventId>(Events::VeloyawDecrease),
       static_cast<bitbot::EventId>(Events::VeloyawIncrease),
       static_cast<bitbot::EventId>(Events::ContinousVeloXChange),
       static_cast<bitbot::EventId>(Events::ContinousVeloYawChange)});

  kernel.RegisterState(
      "compensation_test",
      static_cast<bitbot::StateId>(States::PF2CompensationTest),
      &StateCompensationTest, {});

  // Ankle Test state
  kernel.RegisterState("ankle_test",
                       static_cast<bitbot::StateId>(States::AnkleTest),
                       &StateAnkleTest, {});
  kernel.RegisterState("joint_sin_test",
                       static_cast<bitbot::StateId>(States::JointSinTest),
                       &StateJointSinTest, {});

  kernel.SetFirstState(static_cast<bitbot::StateId>(States::Waiting));

  // 20231029 LLog - zyx
  Logger_More.initMemory(10, 1000);
  Logger_Policy.initMemory(48, 1000);

  // get current time in string
  //  Get current time as a string
  /*auto now = std::chrono::system_clock::now();*/
  /*std::time_t now_time = std::chrono::system_clock::to_time_t(now);*/
  /*std::tm *now_tm = std::localtime(&now_time);*/
  /*std::stringstream ss;*/
  /*ss << std::put_time(now_tm, "%Y-%m-%d-%H-%M-%S");*/
  /*std::string current_time = ss.str();*/

  signal(SIGINT, SignalHandler);
  signal(SIGTERM, SignalHandler);

  kernel.Run();  // Run the kernel
  /**/
  /*std::string log_path =*/
  /*    "/home/huahui/Project/hhfc-bitbot/log/extra/" + current_time + ".csv";*/
  /*Logger_More.saveLog(log_path.c_str());*/
  // 20231029 LLog - zyx
  SignalHandler(0);

  return 0;
}
