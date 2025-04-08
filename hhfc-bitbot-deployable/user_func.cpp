#include "user_func.h"

#include <float.h>

#include <chrono>
#include <cmath>
#include <cmath>  // std::abs
#include <ctime>
#include <iostream>  // std::cout
#include <memory>
#include <random>
#include <thread>

#include "./leeMatrix.h"
// #include "bitbot_cifx/device/ahrs_xsens.h"
#include "bitbot_cifx/device/force_sri6d.h"
#include "include/parallel_ankle.hpp"
#include "inekf-warper/ConfigParser.hpp"
#include "inekf-warper/ContactEstimator.hpp"
#include "inekf-warper/InEKF_Warper.hpp"
#include "inference_net.hpp"
#include "vicon_capture.hpp"

ViconCapture vicon;

// std::mutex mymutex1;
// std::condition_variable condition;

// Variables for Vicon
//////////////////////////////
bool using_vicon = false;
bool using_state_estimation = false;
bool using_inekf_state_estimation = false;

const float pose_estimation_period = 1 / 300.0;
int past_Pose_frame = 0;

Eigen::Matrix4f LastTbwMat;
Eigen::Vector3f linear_velo_world;
Eigen::Vector3f linear_velo_rel;
Eigen::Vector3f linear_velo_base;

//////////////////////////////
constexpr float deg2rad = M_PI / 180.0;
constexpr float rad2deg = 180.0 / M_PI;
constexpr float rpm2radps = 2.0 * M_PI / 60.0;

bitbot::JointElmo *joint[2][7];  // zyx-231007
bitbot::ImuMti300 *imu;          // zyx-231008
const float off_terrain_threshold = 30;

const float IMU_Vicon_rel_pos[3] = {0, 0.037, 0.225};

// For parallel ankle
float joint_target_position[2][7] = {0};
float joint_target_torque[2][7] = {0};
float joint_current_position[2][7] = {0};
float joint_current_velocity[2][7] = {0};
float motor_target_position[2][7] = {0};
float motor_target_torque[2][7] = {0};
float motor_current_position[2][7] = {0};
float motor_current_velocity[2][7] = {0};

// TODO: This should be constant
std::array<std::array<float, 7>, 2> default_position = {
    {{0.165647, 0.0, 0, -0.529741, -0.301101, 0., 0},    // left
     {0.165647, 0.0, 0, -0.529741, -0.301101, 0., 0}}};  // right
/*std::array<std::array<float, 7>, 2> default_position = {*/
/*    {{0.47108, 0.0, 0, -1.19296, -0.575991, 0., 0},    // left*/
/*     {0.47108, 0.0, 0, -1.19296, -0.575991, 0., 0}}};  // right*/
// const std::array<std::array<float, 3>, 2> default_position = {
//     {{0.0, 0.30, -0.60},   // left
//      {0.0, 0.30, -0.60}} }; // right
// const std::array<std::array<float, 3>, 2> default_position = {
//     {{0, 0., 0.},   // left
//      {0, 0., 0.}} }; // right
const size_t LEFT = 0;
const size_t RIGHT = 1;
const size_t FEM_PITCH = 0;
const size_t TIB_PITCH = 1;

// Bro D
const float p_gains[2][7] = {{120., 120.0, 80.0, 120., 20., 20., 200.},
                             {120., 120.0, 80.0, 120., 20., 20., 200.}};
const float d_gains[2][7] = {
    {3.0, 4.0, 2.0, 3.0, 0.5, 0.5, 1.8},
    {3.0, 4.0, 2.0, 3.0, 0.5, 0.5, 1.8}};  // zyx-231007

/*// Dknt*/
/*const float p_gains[2][7] = {{120., 135.0, 80.0, 120., 40., 40., 200.},*/
/*                             {120., 135.0, 80.0, 120., 40., 40., 200.}};*/
/*const float d_gains[2][7] = {*/
/*    {1.2, 1.2, 1.0, 1.2, 1.0, 1.0, 1.8},*/
/*    {1.2, 1.2, 1.0, 1.2, 1.0, 1.0, 1.8}};  // zyx-232.07*/

// WSY
/*const float p_gains[2][7] = {{120., 140.0, 80.0, 120., 40., 40., 200.},*/
/*                             {120., 140.0, 80.0, 120., 40., 40., 200.}};*/
/*const float d_gains[2][7] = {*/
/*    {1.2, 1.2, 1.0, 1.2, 1.0, 1.0, 1.8},*/
/*    {1.2, 1.2, 1.0, 1.2, 1.0, 1.0, 1.8}};  // zyx-232.07*/

float wheel_error_int[2] = {0, 0};
const float Torque_User_Limit[2][7] = {
    {90.0, 120.0, 100.0, 160., 50., 50., 100.},
    {90.0, 120.0, 100.0, 160., 50., 50., 100.}};  // zyx-231008

// Output of policy net. Action interpolation buffer
// action[policy_frequence+1][6]
std::vector<std::vector<float>> action_interpolated;

float step_frequency = 1;
float control_frequency = 1000;

// WT What is compenstae?
const float torque_compensate[2][7] = {{0.0, 0.0, 0.0, 0., 0., 0., 0.},
                                       {0.0, 0.0, 0.0, 0., 0., 0., 0.}};
const float compensate_threshold[2][7] = {
    {1.0, 1.0, 1.0, 1.0, 1., 1., 1.},
    {1.0, 1.0, 1.0, 1., 1., 1., 1.}};  // In rads

// Not used
const float LowerJointPosLimit[2][7] = {
    {-1.5708, -0.47123889803846897, -1.2217, -1.5708, -0.8726646259971648,
     -0.8726646259971648, -100.0 * deg2rad},
    {-1.5708, -0.70, -1.2217, -1.5708, -0.8726646259971648, -0.8726646259971648,
     -100.0 * deg2rad}};  // zyx-231008
const float UpperJointPosLimit[2][7] = {
    {1.5708, 0.70, 1.2217, 0.0, 0.8726646259971648, 0.8726646259971648,
     100.0 * deg2rad},
    {1.5708, 0.47123889803846897, 1.2217, 0.0, 0.8726646259971648,
     0.8726646259971648, 100.0 * deg2rad}};  // zyx-231008

uint64_t sin_pos_init_time = 0;  // zyx-231008
bool has_sin_pos_init = false;
uint64_t init_pos_start_time = 0;
bool has_init_pos = false;
uint64_t init_policy_time = 0;
bool has_init_policy = false;
bool has_step_init = false;
std::chrono::time_point<std::chrono::system_clock> execrise_start =
    std::chrono::system_clock::now();

// Global user dat
bool pd_test = false;

bool has_test_init_pos = false;

const int policy_frequency = 10;
float gravity_vec[3] = {0.0, 0.0, -1.0};

float CoM_angle[3] = {0};        // CoM rpy orientation
float CoM_linear_velo[3] = {0};  // CoM linear velocity
float CoM_angle_velo[3] = {0};   // CoM angular velocity
float CoM_acc[3] = {0};          // CoM acceleration

std::vector<float> dof_pos_obs(12);
std::vector<float> dof_vel_obs(12);
std::vector<float> last_action(12);

float clock_input[2] = {0};

std::vector<float> predicted_lin_velo(3);  // lin velo predicted by SE
Eigen::Vector3f inekf_predict_lin_velo;
Eigen::Matrix3f inekf_predict_pos;
Eigen::Vector3f inekf_predict_Proj_grav;
std::array<bool, 2> inekf_predict_contact_status;

Eigen::Matrix<float, 3, 1> CoMAngleVeloMat;
Eigen::Matrix<float, 3, 1> CoMVeloMat;
Eigen::Matrix<float, 3, 1> gravity_vec_mat;

Eigen::Matrix<float, 3, 1> projected_gravity_mat;

std::vector<float> commands = {0.0, 0.0, 0.0};

int control_periods = 0;
size_t run_ctrl_cnt = 0;
size_t start_delay_cnt = 1000;  // delay control at the begining some time for
                                // net history input stability

float_inference_net::Ptr inference_net;
float_inference_net::NetConfigT net_config =
    {
        .input_config = {.obs_scales_ang_vel = 1.0,
                         .obs_scales_lin_vel = 2.0,
                         .scales_commands = 1.0,
                         .obs_scales_dof_pos = 1.0,
                         .obs_scales_dof_vel = 0.05,
                         .obs_scales_euler = 1.0,
                         .clip_observations = 18.0,
                         .ctrl_model_input_size = 15 * 47,
                         .stack_length = 15,
                         .ctrl_model_unit_input_size = 47},
        .output_config =
            {.ctrl_clip_action = 18.0,
             .action_scale = 0.5,
             .ctrl_action_lower_limit =
                 {
                     -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30,
                     /*-1.5708,*/
                     /*-0.20,*/
                     /*-1.2217,*/
                     /*-1.5708,*/
                     /*-0.7853981633974483,*/
                     /*-0.3490658503988659,*/
                     /*-1.5708,*/
                     /*-0.70,*/
                     /*-1.2217,*/
                     /*-1.5708,*/
                     /*-0.7853981633974483,*/
                     /*-0.3490658503988659,*/
                 },
             .ctrl_action_upper_limit =
                 {
                     30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30
                     /*1.5708,*/
                     /*0.70,*/
                     /*1.2217,*/
                     /*0.3,*/
                     /*0.6108,*/
                     /*0.174532,*/
                     /*1.5708,*/
                     /*0.20,*/
                     /*1.2217,*/
                     /*0.3,*/
                     /*0.6108,*/
                     /*0.174532,*/
                 },
             .ctrl_model_output_size = 12},
        .action_default_pos = {
            default_position[0][0],
            default_position[0][1],
            default_position[0][2],
            default_position[0][3],
            default_position[0][4],  // Left ank pit
            default_position[0][5],  // Left ank roll
            default_position[1][0],
            default_position[1][1],
            default_position[1][2],
            default_position[1][3],
            default_position[1][4],
            default_position[1][5],
        }};
lee::blocks::LLog<float> Logger_More;
lee::blocks::LLog<float> Logger_Policy;

// Noise generator
std::default_random_engine angle_noise_generator;
std::default_random_engine angle_velo_noise_generator;
std::default_random_engine acc_noise_generator;

std::normal_distribution<float> angle_noise_distribution(
    0, 0.01);  // TODO: set this distrib
std::normal_distribution<float> angle_velo_noise_distribution(0, 0.08);
std::normal_distribution<float> acc_noise_distribution(0, 0.08);

std::array<float, 3> ang_vel_GT_no_noise;  // Angular velocity with out noise

class filter {
 public:
  filter(int buffer_sz = 3) {
    this->buffer_sz = buffer_sz;
    this->buffer.resize(buffer_sz);
    for (auto &i : this->buffer) i = 0;
  }
  void clear() {
    for (auto &i : this->buffer) {
      i = 0;
    }
  }

  float operator()(float input) {
    buffer[this->pointer] = input;
    this->pointer++;
    this->pointer %= this->buffer_sz;
    float sum = 0;
    for (auto &i : this->buffer) {
      sum += i;
    }
    return sum / buffer_sz;
  }

 private:
  int buffer_sz;
  std::vector<float> buffer;
  int pointer = 0;
};

// Velocity Average Filter
filter velo_filters[2][7];
filter velo_filter_net[2][7];

/**
 * @brief Config function
 * @param[in] bus CIFX bus, from witch ELMO devices are gotten.
 * @param[in]
 */
void ConfigFunc(const bitbot::CifxBus &bus, UserData &) {
  joint[1][6] = bus.GetDevice<bitbot::JointElmo>(0).value();   // zyx-231007
  joint[0][6] = bus.GetDevice<bitbot::JointElmo>(1).value();   // zyx-231007
  imu = bus.GetDevice<bitbot::ImuMti300>(2).value();           // zyx-231008
  joint[1][0] = bus.GetDevice<bitbot::JointElmo>(3).value();   // zyx-231007
  joint[1][1] = bus.GetDevice<bitbot::JointElmo>(4).value();   // zyx-231007
  joint[1][2] = bus.GetDevice<bitbot::JointElmo>(5).value();   // zyx-231007
  joint[1][3] = bus.GetDevice<bitbot::JointElmo>(6).value();   // zyx-231007
  joint[1][4] = bus.GetDevice<bitbot::JointElmo>(7).value();   // zyx-231007
  joint[1][5] = bus.GetDevice<bitbot::JointElmo>(8).value();   // zyx-231007
  joint[0][0] = bus.GetDevice<bitbot::JointElmo>(9).value();   // zyx-231007
  joint[0][1] = bus.GetDevice<bitbot::JointElmo>(10).value();  // zyx-231007
  joint[0][2] = bus.GetDevice<bitbot::JointElmo>(11).value();  // zyx-231007
  joint[0][3] = bus.GetDevice<bitbot::JointElmo>(12).value();  // zyx-231007
  joint[0][4] = bus.GetDevice<bitbot::JointElmo>(13).value();  // zyx-231007
  joint[0][5] = bus.GetDevice<bitbot::JointElmo>(14).value();  // zyx-231007

  // std::cin.get();

  InitPolicy();  // zyx-231019

  // std::thread policy_thread(&CallPolicy);
  // policy_thread.detach();

  srand(time(0));
  clock_input[1] += M_PI;
  clock_input[0] += 0.00001;
  clock_input[1] += 0.00001;
  if (using_vicon) {
    std::cout << "initializing vicon..." << std::endl;
    vicon.Connect("192.168.0.2");
  }
}

// void JointSinPos(float time) {}

std::optional<bitbot::StateId> EventSinPos(bitbot::EventValue, UserData &) {
  return static_cast<bitbot::StateId>(States::PF2SinPos);
}
std::optional<bitbot::StateId> EventSwitchMode(bitbot::EventValue, UserData &) {
  return static_cast<bitbot::StateId>(States::PF2SwitchMode);
}  // zyx-231007
std::optional<bitbot::StateId> EventInitPose(bitbot::EventValue, UserData &) {
  return static_cast<bitbot::StateId>(States::PF2InitPose);
}  // zyx-231007
std::optional<bitbot::StateId> EventInitForData(bitbot::EventValue,
                                                UserData &) {
  return static_cast<bitbot::StateId>(States::PF2InitForData);
}  // zyx-231007
std::optional<bitbot::StateId> EventOriginTest(bitbot::EventValue, UserData &) {
  return static_cast<bitbot::StateId>(States::PF2OriginTest);
}  // zyx-231007
/*std::optional<bitbot::StateId> EventStepFunction(bitbot::EventValue,*/
/*                                                 UserData &) {*/
/*  return static_cast<bitbot::StateId>(States::PF2StepFunction);*/
/*}  // zyx-231012*/
std::optional<bitbot::StateId> EventPolicyRun(bitbot::EventValue, UserData &) {
  return static_cast<bitbot::StateId>(States::PF2PolicyRun);
}  // zyx-231019
std::optional<bitbot::StateId> EventCompensationTest(bitbot::EventValue,
                                                     UserData &) {
  return static_cast<bitbot::StateId>(States::PF2CompensationTest);
}  // zyx-231103
std::optional<bitbot::StateId> EventAnkleTest(bitbot::EventValue value,
                                              UserData &user_data) {
  std::cout << "Ankle Test" << std::endl;
  return static_cast<bitbot::StateId>(States::AnkleTest);
}
std::optional<bitbot::StateId> EventJointSinTest(bitbot::EventValue value,
                                                 UserData &user_data) {
  std::cout << "hahaha" << std::endl;
  return static_cast<bitbot::StateId>(States::JointSinTest);
}

// velocity control callback
std::optional<bitbot::StateId> EventVeloXIncrease(bitbot::EventValue keyState,
                                                  UserData &) {
  if (keyState == static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up)) {
    commands[0] += 0.05;
    std::cout << "current velocity: x=" << commands[0] << " y=" << commands[1]
              << std::endl;
  }
  return static_cast<bitbot::StateId>(States::PF2PolicyRun);
}

std::optional<bitbot::StateId> EventVeloXDecrease(bitbot::EventValue keyState,
                                                  UserData &) {
  if (keyState == static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up)) {
    commands[0] -= 0.05;
    std::cout << "current velocity: x=" << commands[0] << " y=" << commands[1]
              << std::endl;
  }
  return static_cast<bitbot::StateId>(States::PF2PolicyRun);
}

std::optional<bitbot::StateId> EventVeloYIncrease(bitbot::EventValue keyState,
                                                  UserData &) {
  if (keyState == static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up)) {
    commands[1] -= 0.05;
    std::cout << "current velocity: x=" << commands[0] << " y=" << commands[1]
              << std::endl;
  }
  return static_cast<bitbot::StateId>(States::PF2PolicyRun);
}

std::optional<bitbot::StateId> EventVeloYDecrease(bitbot::EventValue keyState,
                                                  UserData &) {
  if (keyState == static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up)) {
    commands[1] += 0.05;
    std::cout << "current velocity: x=" << commands[0] << " y=" << commands[1]
              << std::endl;
  }
  return static_cast<bitbot::StateId>(States::PF2PolicyRun);
}
std::optional<bitbot::StateId> EventVeloYawIncrease(bitbot::EventValue keyState,
                                                    UserData &) {
  if (keyState == static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up)) {
    commands[2] += 0.05;
    std::cout << " yaw=" << commands[2] << std::endl;
  }
  return static_cast<bitbot::StateId>(States::PF2PolicyRun);
}

std::optional<bitbot::StateId> EventVeloYawDecrease(bitbot::EventValue keyState,
                                                    UserData &) {
  if (keyState == static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up)) {
    commands[2] -= 0.05;
    std::cout << " yaw=" << commands[2] << std::endl;
  }
  return static_cast<bitbot::StateId>(States::PF2PolicyRun);
}

std::optional<bitbot::StateId> EventYOffsetIncrease(bitbot::EventValue keyState,
                                                    UserData &) {
  if (keyState == static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up)) {
    inference_net->y_offset += 0.005;
    std::cout << "current y offset=" << inference_net->y_offset << std::endl;
  }
  return static_cast<bitbot::StateId>(States::PF2PolicyRun);
}

std::optional<bitbot::StateId> EventYOffsetDecrease(bitbot::EventValue keyState,
                                                    UserData &) {
  if (keyState == static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up)) {
    inference_net->y_offset -= 0.005;
    std::cout << "current y offset=" << inference_net->y_offset << std::endl;
  }
  return static_cast<bitbot::StateId>(States::PF2PolicyRun);
}

std::optional<bitbot::StateId> EventYawOffsetIncrease(
    bitbot::EventValue keyState, UserData &) {
  if (keyState == static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up)) {
    inference_net->yaw_offset += 0.01;
    std::cout << "current yaw offset=" << inference_net->yaw_offset
              << std::endl;
  }
  return static_cast<bitbot::StateId>(States::PF2PolicyRun);
}

std::optional<bitbot::StateId> EventYawOffsetDecrease(
    bitbot::EventValue keyState, UserData &) {
  if (keyState == static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up)) {
    inference_net->yaw_offset -= 0.01;
    std::cout << "current yaw offset=" << inference_net->yaw_offset
              << std::endl;
  }
  return static_cast<bitbot::StateId>(States::PF2PolicyRun);
}

std::optional<bitbot::StateId> EventContinousVeloXChange(
    bitbot::EventValue KeyState, UserData &) {
  double val = *(reinterpret_cast<double *>(&KeyState));
  commands[0] = val * 1.2;
  std::cout << "current X velo:" << commands[0] << std::endl;
  return static_cast<bitbot::StateId>(States::PF2PolicyRun);
}

std::optional<bitbot::StateId> EventContinousVeloYawChange(
    bitbot::EventValue KeyState, UserData &) {
  double val = *(reinterpret_cast<double *>(&KeyState));
  commands[2] = val;
  std::cout << "current yaw velo:" << commands[2] << std::endl;
  return static_cast<bitbot::StateId>(States::PF2PolicyRun);
}

std::optional<bitbot::StateId> EventContinousVeloYChange(
    bitbot::EventValue KeyState, UserData &) {
  return static_cast<bitbot::StateId>(States::PF2PolicyRun);
}

void ShowCurrentDefPos() {
  std::cout << "ShowCurrentDefPos" << std::endl;
  for (size_t i = 0; i < 2; ++i) {
    for (size_t j = 0; j < 7; ++j) {
      std::cout << "Id " << i << " " << j << ": " << default_position[i][j]
                << std::endl;
    }
  }
}

// 0.5 deg. /2 because the backend responses twice
constexpr float pitch_tuning_inc = 0.008726646259971648 / 2.0;
std::optional<bitbot::StateId> EventHipPitchIncrease(
    bitbot::EventValue KeyState, UserData &) {
  default_position[0][0] += pitch_tuning_inc;
  default_position[1][0] += pitch_tuning_inc;
  ShowCurrentDefPos();
  return {};
}
std::optional<bitbot::StateId> EventHipPitchDecrease(
    bitbot::EventValue KeyState, UserData &) {
  default_position[0][0] -= pitch_tuning_inc;
  default_position[1][0] -= pitch_tuning_inc;
  ShowCurrentDefPos();
  return {};
}
std::optional<bitbot::StateId> EventKneePitchIncrease(
    bitbot::EventValue KeyState, UserData &) {
  default_position[0][3] += pitch_tuning_inc;
  default_position[1][3] += pitch_tuning_inc;
  ShowCurrentDefPos();
  return {};
}
std::optional<bitbot::StateId> EventKneePitchDecrease(
    bitbot::EventValue KeyState, UserData &) {
  default_position[0][3] -= pitch_tuning_inc;
  default_position[1][3] -= pitch_tuning_inc;
  ShowCurrentDefPos();
  return {};
}
std::optional<bitbot::StateId> EventAnklePitchIncrease(
    bitbot::EventValue KeyState, UserData &) {
  default_position[0][4] += pitch_tuning_inc;
  default_position[1][4] += pitch_tuning_inc;
  ShowCurrentDefPos();
  return {};
}
std::optional<bitbot::StateId> EventAnklePitchDecrease(
    bitbot::EventValue KeyState, UserData &) {
  default_position[0][4] -= pitch_tuning_inc;
  default_position[1][4] -= pitch_tuning_inc;
  ShowCurrentDefPos();
  return {};
}

void StateWaiting(const bitbot::KernelInterface &kernel,
                  Kernel::ExtraData &extra_data, UserData &user_data) {
  GetJointObservation(extra_data);
}

void StateJointSwitchMode(const bitbot::KernelInterface &kernel,
                          Kernel::ExtraData &extra_data, UserData &user_data) {
  for (int i = 0; i < 2; i++)
    for (int j = 0; j < 3; j++) {
      joint[i][j]->SetTargetTorque(0.0);
      joint[i][j]->SetMode(bitbot::CANopenMotorMode::CST);
    }
}  // zyx-231007

void StateJointInitPose_ForData(const bitbot::KernelInterface &kernel,
                                Kernel::ExtraData &extra_data,
                                UserData &user_data) {
  if (!has_init_pos) {
    init_pos_start_time = kernel.GetPeriodsCount();
    has_init_pos = true;
  }
  InitPos_ForData((kernel.GetPeriodsCount() - init_pos_start_time) * 0.001);
};

void StatePolicyRun(const bitbot::KernelInterface &kernel,
                    Kernel::ExtraData &extra_data, UserData &user_data) {
  if (!has_init_policy) {
    init_policy_time = kernel.GetPeriodsCount();
    has_init_policy = true;
  }

  GetJointObservation(extra_data);
  PolicyController(kernel.GetPeriodsCount());
};

void StateCompensationTest(const bitbot::KernelInterface &kernel,
                           Kernel::ExtraData &extra_data, UserData &user_data) {
  CompensationController();
}

// FIXME: delete this function and merge it into initpos()
/**
 * @brief Discarded
 *
 * @param[] current_time
 */
void InitPos_ForData(float current_time) {
  // const static float total_time = 0.5;
}

void StateJointInitPose(const bitbot::KernelInterface &kernel,
                        Kernel::ExtraData &extra_data, UserData &user_data) {
  GetJointObservation(extra_data);
  if (!has_init_pos) {
    init_pos_start_time = kernel.GetPeriodsCount();
    has_init_pos = true;
  }
  InitPos((kernel.GetPeriodsCount() - init_pos_start_time) * 0.001);
}  // zyx-231007

void StateJointSinTest(const bitbot::KernelInterface &kernel,
                       Kernel::ExtraData &extra_data, UserData &user_data) {
  GetJointObservation(extra_data);
  static float pose_test_start_time = kernel.GetPeriodsCount();

  float current_time =
      (kernel.GetPeriodsCount() - pose_test_start_time) * 0.001;

  // Ankle index Left pitch
  const size_t test_idx0 = 1;
  const size_t test_idx1 = 5;
  // roll
  /*const float amplidute = 0.17453292519943295;*/
  // pitch
  const float amplidute = 0.3490658503988659;
  const float frequency = 0.2;

  static float initial_pos = joint_current_position[test_idx0][test_idx1];
  float target_pos =
      initial_pos + amplidute * std::sin(2 * M_PI * frequency * current_time);
  std::cout << "target_pos: " << target_pos << std::endl;
  joint_target_position[test_idx0][test_idx1] = target_pos;

  //
  TorqueController();
}

void StateJointImpTest(const bitbot::KernelInterface &kernel,
                       Kernel::ExtraData &extra_data, UserData &user_data) {
  GetJointObservation(extra_data);
  static float pose_test_start_time = kernel.GetPeriodsCount();

  float current_time =
      (kernel.GetPeriodsCount() - pose_test_start_time) * 0.001;

  // Ankle index
  const size_t test_idx0 = 0;
  const size_t test_idx1 = 4;
  const float amplidute = 0.1;
  const float frequency = 1.0;

  static float initial_pos = joint_current_position[test_idx0][test_idx1];
  float target_pos =
      initial_pos + amplidute * std::sin(2 * M_PI / frequency * current_time);
  joint_target_position[test_idx0][test_idx1] = target_pos;

  //
  TorqueController();
}
/**
 * @brief Discatded again.
 *
 * @param[] kernel
 * @param[] extra_data
 * @param[] user_data
 */
/*void StateStepFunction(const bitbot::KernelInterface &kernel,*/
/*                       Kernel::ExtraData &extra_data, UserData &user_data) {*/
/*  //*/
/*}*/

/**
 * @brief Initialize robot joint position, called in StateJointInitPose
 *
 * @param[in] current_time Current time in seconds
 */
void InitPos(float current_time) {
  const static float total_time = 3.5;

  const static float target_v0 = 0, target_a0 = 0;
  const static float target_v1 = 0, target_a1 = 0;

  static double p0[2][7];
  static double v0[2][7];
  static double a0[2][7];
  static bool flag = false;
  if (!flag) {
    for (size_t i = 0; i < 2; i++) {
      for (size_t j = 0; j < 7; j++) {
        p0[i][j] = joint_current_position[i][j];
        v0[i][j] = 0;
        a0[i][j] = 0;
      }
    }
    flag = true;
  }

  for (size_t i = 0; i < 2; i++) {
    for (size_t j = 0; j < 7; j++) {
      realtime_1D_interpolation_5(&p0[i][j], &v0[i][j], &a0[i][j],
                                  default_position[i][j], 0, 0, current_time,
                                  total_time, 0.001);
      joint_target_position[i][j] = p0[i][j];
    }
  }

  TorqueController();
}

void StateJointSinPos(const bitbot::KernelInterface &kernel,
                      Kernel::ExtraData &extra_data, UserData &user_data) {
  if (!has_sin_pos_init) {
    sin_pos_init_time = kernel.GetPeriodsCount();
    has_sin_pos_init = true;
  }
  SinPosCompute(sin_pos_init_time, kernel.GetPeriodsCount());
}
ParallelAnkle<float> left_ankle({.l_bar1 = 0.06,
                                 .l_rod1 = 0.241639,
                                 .r_a1 = {-0.00775, 0.06565, 0.2405},
                                 .r_b1_0 = {-0.06775, 0.06565, 0.2405},
                                 .r_c1_0 = {-0.04432, 0.06565, 0.0},

                                 .l_bar2 = 0.06,
                                 .l_rod2 = 0.159473,
                                 .r_a2 = {-0.00736, -0.06565, 0.1578},
                                 .r_b2_0 = {-0.06736, -0.06565, 0.1578},
                                 .r_c2_0 = {-0.04432, -0.06565, 0.0}},
                                1e-6);

// Fixing
ParallelAnkle<float> right_ankle({.l_bar1 = 0.06,
                                  .l_rod1 = 0.159473,
                                  .r_a1 = {-0.00736, 0.06565, 0.1578},
                                  .r_b1_0 = {-0.06736, 0.06565, 0.1578},
                                  .r_c1_0 = {-0.04432, 0.06565, 0.0},

                                  .l_bar2 = 0.06,
                                  .l_rod2 = 0.241639,
                                  .r_a2 = {-0.00775, -0.06565, 0.2405},
                                  .r_b2_0 = {-0.06775, -0.06565, 0.2405},
                                  .r_c2_0 = {-0.04432, -0.06565, 0.0}},
                                 1e-6);

void StateAnkleTest(const bitbot::KernelInterface &kernel,
                    Kernel::ExtraData &extra_data, UserData &user_data) {
  GetJointObservation(extra_data);
}

/**
 * @brief This function is discarded.
 *
 * @param[] start
 * @param[] end
 */
void SinPosCompute(uint64_t start, uint64_t end) {
  // float zyx_delta_time = 0.001 * (end - start);
}

/**
 * @brief Initialize policy net
 *
 */
void InitPolicy() {
  // Action interpolation buffer
  action_interpolated = std::vector<std::vector<float>>(policy_frequency + 1,
                                                        std::vector<float>(12));
  // Create the policy network instance
  // last 4000
  inference_net = std::make_unique<float_inference_net>(
      "/home/huahui/Project/hhfc-bitbot/checkpoint/"
      "policy_1.pt",     // control model  policy_202412271
      net_config,        // net config
      true,              // use async mode
      policy_frequency,  // policy frequency
      &Logger_Policy);
}

void GetJointObservation(Kernel::ExtraData &extra_data) {
  for (size_t i = 0; i < 2; i++) {
    for (size_t j = 0; j < 7; j++) {
      if (j == 4 || j == 5) {
        motor_current_position[i][j] = -joint[i][j]->GetActualPosition();
        motor_current_velocity[i][j] =
            velo_filters[i][j](-joint[i][j]->GetActualVelocity());
      } else {
        motor_current_position[i][j] = joint[i][j]->GetActualPosition();
        motor_current_velocity[i][j] =
            velo_filters[i][j](joint[i][j]->GetActualVelocity());
        joint_current_position[i][j] = motor_current_position[i][j];
        joint_current_velocity[i][j] = motor_current_velocity[i][j];
      }
    }
  }
  auto left_res = left_ankle.ForwardKinematics(motor_current_position[0][4],
                                               motor_current_position[0][5]);
  auto right_res = right_ankle.ForwardKinematics(motor_current_position[1][5],
                                                 motor_current_position[1][4]);

  auto left_vel_res = left_ankle.VelocityMapping(motor_current_velocity[0][4],
                                                 motor_current_velocity[0][5]);
  auto right_vel_res = left_ankle.VelocityMapping(motor_current_velocity[1][5],
                                                  motor_current_velocity[1][4]);
  joint_current_position[0][4] = left_res(0, 0);
  joint_current_position[0][5] = left_res(1, 0);
  joint_current_position[1][4] = right_res(0, 0);
  joint_current_position[1][5] = right_res(1, 0);
  joint_current_velocity[0][4] = left_vel_res(0, 0);
  joint_current_velocity[0][5] = left_vel_res(1, 0);
  joint_current_velocity[1][4] = right_vel_res(0, 0);
  joint_current_velocity[1][5] = right_vel_res(1, 0);

  extra_data.Set<"l_p_pos">(joint_current_position[0][4] / M_PI * 180.0);
  extra_data.Set<"l_r_pos">(joint_current_position[0][5] / M_PI * 180.0);
  extra_data.Set<"r_p_pos">(joint_current_position[1][4] / M_PI * 180.0);
  extra_data.Set<"r_r_pos">(joint_current_position[1][5] / M_PI * 180.0);
  extra_data.Set<"l_p_vel">(joint_current_velocity[0][4] / M_PI * 180.0);
  extra_data.Set<"l_r_vel">(joint_current_velocity[0][5] / M_PI * 180.0);
  extra_data.Set<"r_p_vel">(joint_current_velocity[1][4] / M_PI * 180.0);
  extra_data.Set<"r_r_vel">(joint_current_velocity[1][5] / M_PI * 180.0);
  extra_data.Set<"l_p_tor">(joint_target_torque[0][4]);
  extra_data.Set<"l_r_tor">(joint_target_torque[0][5]);
  extra_data.Set<"r_p_tor">(joint_target_torque[1][4]);
  extra_data.Set<"r_r_tor">(joint_target_torque[1][5]);
}

void SetJointAction() {
  // For normal joints
  for (size_t i = 0; i < 2; i++) {
    for (size_t j = 0; j < 7; j++) {
      if (j == 4 || j == 5) continue;
      motor_target_position[i][j] = joint_target_position[i][j];
      motor_target_torque[i][j] = joint_target_torque[i][j];
    }
  }

  // For ankle joints
  auto left_mot_pos_res = left_ankle.InverseKinematics(
      joint_target_position[0][4], joint_target_position[0][5]);
  auto right_mot_pos_res = right_ankle.InverseKinematics(
      joint_target_position[1][4], joint_target_position[1][5]);
  motor_target_position[0][4] = -left_mot_pos_res[0];
  motor_target_position[0][5] = -left_mot_pos_res[1];
  motor_target_position[1][4] = -right_mot_pos_res[1];
  motor_target_position[1][5] = -right_mot_pos_res[0];

  auto left_tor_res = left_ankle.TorqueRemapping(joint_target_torque[0][4],
                                                 joint_target_torque[0][5]);
  auto right_tor_res = right_ankle.TorqueRemapping(joint_target_torque[1][4],
                                                   joint_target_torque[1][5]);
  motor_target_torque[0][4] = -left_tor_res[0];
  motor_target_torque[0][5] = -left_tor_res[1];
  motor_target_torque[1][4] = -right_tor_res[1];
  motor_target_torque[1][5] = -right_tor_res[0];
}

/**
 * @brief Policy Controller
 *
 * @param[in] cur_time Current time in steps
 */
void PolicyController(uint64_t cur_time) {
  // Orientation from IMU
  CoM_angle[0] = imu->GetRoll() / 180 * M_PI;  // imu angle
  CoM_angle[1] = imu->GetPitch() / 180 * M_PI;
  CoM_angle[2] = imu->GetYaw() / 180 * M_PI;

  // This is sort of filter.
  // Acceleration from IMU
  CoM_acc[0] = (abs(imu->GetAccX()) > 50) ? CoM_acc[0] : imu->GetAccX();
  CoM_acc[1] = (abs(imu->GetAccY()) > 50) ? CoM_acc[1] : imu->GetAccY();
  CoM_acc[2] = (abs(imu->GetAccZ()) > 50) ? CoM_acc[2] : imu->GetAccZ();

  // Angular velocity from IMU
  float angle_velo_x = imu->GetGyroX();  // imu angular velocity
  float angle_velo_y = imu->GetGyroY();
  float angle_velo_z = imu->GetGyroZ();

  if (std::isnan(angle_velo_x)) angle_velo_x = 0;
  if (std::isnan(angle_velo_y)) angle_velo_y = 0;
  if (std::isnan(angle_velo_z)) angle_velo_z = 0;

  for (size_t i = 0; i < 3; i++) {
    if (std::isnan(CoM_angle[i])) CoM_angle[i] = 0;
  }

  CoM_angle_velo[0] = std::abs(angle_velo_x) > 10
                          ? CoM_angle_velo[0]
                          : angle_velo_x;  // filter the noise
  CoM_angle_velo[1] =
      std::abs(angle_velo_y) > 10 ? CoM_angle_velo[1] : angle_velo_y;
  CoM_angle_velo[2] =
      std::abs(angle_velo_z) > 10 ? CoM_angle_velo[2] : angle_velo_z;
  Eigen::Vector3f angle_velo_eigen(CoM_angle_velo[0], CoM_angle_velo[1],
                                   CoM_angle_velo[2]);

  // inference
  clock_t start_time, end_time;
  start_time = clock();

  if ((cur_time - init_policy_time) % policy_frequency == 0) {
    /*const float stance_T = 0.32;*/
    /*const float stance_T = 0.40;*/
    const float stance_T = 0.375;
    const float dt = 0.001;

    for (int i = 0; i < 2; i++) {
      clock_input[i] +=
          2 * M_PI / (2 * stance_T) * dt * static_cast<float>(policy_frequency);
      if (clock_input[i] > 2 * M_PI) {
        clock_input[i] -= 2 * M_PI;
      }
    }

    Eigen::Matrix3f RotationMatrix;
    const Eigen::AngleAxisf roll(CoM_angle[0], Eigen::Vector3f::UnitX());
    const Eigen::AngleAxisf pitch(CoM_angle[1], Eigen::Vector3f::UnitY());
    const Eigen::AngleAxisf yaw(CoM_angle[2], Eigen::Vector3f::UnitZ());
    RotationMatrix = yaw * pitch * roll;

    std::vector<float> eu_ang{CoM_angle[0], CoM_angle[1], CoM_angle[2]};

    CoMAngleVeloMat << CoM_angle_velo[0], CoM_angle_velo[1], CoM_angle_velo[2];
    gravity_vec_mat << gravity_vec[0], gravity_vec[1], gravity_vec[2];

    projected_gravity_mat = RotationMatrix.transpose() * gravity_vec_mat;

    std::vector<float> base_ang_vel = {
        CoMAngleVeloMat(0, 0), CoMAngleVeloMat(1, 0), CoMAngleVeloMat(2, 0)};
    std::vector<float> project_gravity = {projected_gravity_mat(0, 0),
                                          projected_gravity_mat(1, 0),
                                          projected_gravity_mat(2, 0)};

    std::vector<float> clock_input_vec = {float(sin(clock_input[0])),
                                          float(cos(clock_input[0]))};

    // TODO: Optimize this
    dof_pos_obs[0] = joint_current_position[0][0];  // Left hip pitch
    dof_pos_obs[1] = joint_current_position[0][1];  // Left hip roll
    dof_pos_obs[2] = joint_current_position[0][2];  // Left hip yaw
    dof_pos_obs[3] = joint_current_position[0][3];  // Left knee
    dof_pos_obs[4] = joint_current_position[0][4];  // Left ankle pitch
    dof_pos_obs[5] = joint_current_position[0][5];  // Left ankle roll
    //
    dof_pos_obs[6] = joint_current_position[1][0];   // Right hip pitch
    dof_pos_obs[7] = joint_current_position[1][1];   // Right hip roll
    dof_pos_obs[8] = joint_current_position[1][2];   // Right hip yaw
    dof_pos_obs[9] = joint_current_position[1][3];   // Right knee
    dof_pos_obs[10] = joint_current_position[1][4];  // Right ankle pitch
    dof_pos_obs[11] = joint_current_position[1][5];  // Right ankle roll

    // Add filter
    dof_vel_obs[0] =
        velo_filter_net[0][0](joint_current_velocity[0][0]);  // Left hip pitch
    dof_vel_obs[1] =
        velo_filter_net[0][1](joint_current_velocity[0][1]);  // Left hip roll
    dof_vel_obs[2] =
        velo_filter_net[0][2](joint_current_velocity[0][2]);  // Left hip yaw
    dof_vel_obs[3] =
        velo_filter_net[0][3](joint_current_velocity[0][3]);  // Left knee
    dof_vel_obs[4] = velo_filter_net[0][4](
        joint_current_velocity[0][4]);  // Left ankle pitch
    dof_vel_obs[5] =
        velo_filter_net[0][5](joint_current_velocity[0][5]);  // Left ankle roll
    //
    dof_vel_obs[6] =
        velo_filter_net[1][0](joint_current_velocity[1][0]);  // Right hip pitch
    dof_vel_obs[7] =
        velo_filter_net[1][1](joint_current_velocity[1][1]);  // Right hip roll
    dof_vel_obs[8] =
        velo_filter_net[1][2](joint_current_velocity[1][2]);  // Right hip yaw
    dof_vel_obs[9] =
        velo_filter_net[1][3](joint_current_velocity[1][3]);  // Right knee
    dof_vel_obs[10] = velo_filter_net[1][4](
        joint_current_velocity[1][4]);  // Right ankle pitch
    dof_vel_obs[11] = velo_filter_net[1][5](
        joint_current_velocity[1][5]);  // Right ankle roll

    // Euler angle
    /*bool ok = inference_net->InferenceOnceErax(*/
    /*    clock_input_vec, commands, dof_pos_obs, dof_vel_obs, last_action,*/
    /*    base_ang_vel, eu_ang);*/
    // Projected gravity
    bool ok = inference_net->InferenceOnceErax(
        clock_input_vec, commands, dof_pos_obs, dof_vel_obs, last_action,
        base_ang_vel, project_gravity);

    if (!ok) {
      std::cout << "inference failed" << std::endl;
    }
  }

  // get inference result when inference finished
  if (auto inference_status = inference_net->GetStatus();
      inference_status == float_inference_net::StatusT::FINISHED) {
    auto ok =
        inference_net->GetInfereceResult(last_action, action_interpolated);
    // TODO: Reset order
    {
    }
    if (!ok) {
      std::cout << "get inference result failed" << std::endl;
    }
    control_periods = 0;  // reset control periods when new target is generated

    // Log data into policy logger
    inference_net->log_result();
  }

  if (run_ctrl_cnt > start_delay_cnt) {
    // TODO: Optimize this
    joint_target_position[0][6] = 0.0;  // Left shoulder
    joint_target_position[0][0] =
        action_interpolated[control_periods][0];  // Left hip pitch
    joint_target_position[0][1] =
        action_interpolated[control_periods][1];  // Left hip roll
    joint_target_position[0][2] =
        action_interpolated[control_periods][2];  // Left hip yaw
    joint_target_position[0][3] =
        action_interpolated[control_periods][3];  // Left knee
    joint_target_position[0][4] =
        action_interpolated[control_periods][4];  // Left ankle pitch
    joint_target_position[0][5] =
        action_interpolated[control_periods][5];  // Left ankle roll

    joint_target_position[1][6] = 0.0;  // Right shoudler
    joint_target_position[1][0] =
        action_interpolated[control_periods][6];  // Right hip pitch
    joint_target_position[1][1] =
        action_interpolated[control_periods][7];  // Right hip roll
    joint_target_position[1][2] =
        action_interpolated[control_periods][8];  // Right hip yaw
    joint_target_position[1][3] =
        action_interpolated[control_periods][9];  // Right knee
    joint_target_position[1][4] =
        action_interpolated[control_periods][10];  // Right ankle pitch
    joint_target_position[1][5] =
        action_interpolated[control_periods][11];  // Right ankle roll

  } else {
    for (int i = 0; i < 2; i++)
      for (int j = 0; j < 7; j++) {
        joint_target_position[i][j] = joint_target_position[i][j];
      }
  }

  control_periods += 1;
  control_periods =
      (control_periods > policy_frequency) ? policy_frequency : control_periods;

  end_time = clock();
  if (((float)(end_time - start_time) / CLOCKS_PER_SEC) > 0.001) {
    std::cout << "Calling time: "
              << (float)(end_time - start_time) / CLOCKS_PER_SEC << "s"
              << std::endl;
  }

  TorqueController();
  run_ctrl_cnt++;
}

void StateJointOriginTest(const bitbot::KernelInterface &kernel,
                          Kernel::ExtraData &extra_data, UserData &user_data) {
  GetJointObservation(extra_data);
  if (!pd_test) {
    for (int i = 0; i < 2; i++)
      for (int j = 0; j < 3; j++) {
        joint_target_position[i][j] =
            static_cast<float>(joint[i][j]->GetActualPosition());
        pd_test = true;
      }
  }

  TorqueController();
}

void log_result(uint64_t cur_time, bool log_net) {
  Logger_More.startLog();
  // Logger_More.addLog(has_sin_pos_init, "has_sin_pos_init");
  /*Logger_More.addLog(has_init_policy, "has_init_policy");*/
  /*Logger_More.addLog(cur_time - init_policy_time, "time");*/

  // Joint data
  for (size_t i = 0; i < 2; ++i) {
    for (size_t j = 0; j < 7; ++j) {
      Logger_More.addLog(
          joint_current_position[i][j],
          std::format("joint_current_position_{}{}", i, j).c_str());
    }
  }
  for (size_t i = 0; i < 2; ++i) {
    for (size_t j = 0; j < 7; ++j) {
      Logger_More.addLog(
          joint_current_velocity[i][j],
          std::format("joint_current_velocity_{}{}", i, j).c_str());
    }
  }
  for (size_t i = 0; i < 2; ++i) {
    for (size_t j = 0; j < 7; ++j) {
      Logger_More.addLog(
          joint_target_position[i][j],
          std::format("joint_target_position_{}{}", i, j).c_str());
    }
  }
  for (size_t i = 0; i < 2; ++i) {
    for (size_t j = 0; j < 7; ++j) {
      Logger_More.addLog(joint_target_torque[i][j],
                         std::format("joint_target_torque_{}{}", i, j).c_str());
    }
  }

  // Motor Data
  for (size_t i = 0; i < 2; ++i) {
    for (size_t j = 0; j < 7; ++j) {
      Logger_More.addLog(
          motor_current_position[i][j],
          std::format("motor_current_position_{}{}", i, j).c_str());
    }
  }
  for (size_t i = 0; i < 2; ++i) {
    for (size_t j = 0; j < 7; ++j) {
      Logger_More.addLog(
          motor_current_velocity[i][j],
          std::format("motor_current_velocity_{}{}", i, j).c_str());
    }
  }
  for (size_t i = 0; i < 2; ++i) {
    for (size_t j = 0; j < 7; ++j) {
      Logger_More.addLog(
          motor_target_position[i][j],
          std::format("motor_target_position_{}{}", i, j).c_str());
    }
  }
  for (size_t i = 0; i < 2; ++i) {
    for (size_t j = 0; j < 7; ++j) {
      Logger_More.addLog(motor_target_torque[i][j],
                         std::format("motor_target_torque_{}{}", i, j).c_str());
    }
  }

  // Sensor data
  std::map<size_t, std::string> rpy_name_mapping = {
      {0, "roll"}, {1, "pitch"}, {2, "yaw"}};
  for (size_t i = 0; i < 3; ++i) {
    Logger_More.addLog(CoM_angle[i],
                       std::format("eular_{}", rpy_name_mapping[i]).c_str());
  }

  std::map<size_t, std::string> body_axis_mapping = {
      {0, "x"}, {1, "y"}, {2, "z"}};
  for (size_t i = 0; i < 3; ++i) {
    Logger_More.addLog(
        CoM_angle_velo[i],
        std::format("angle_vel_{}", body_axis_mapping[i]).c_str());
  }
  for (size_t i = 0; i < 3; ++i) {
    Logger_More.addLog(
        CoM_acc[i],
        std::format("acceleration_{}", body_axis_mapping[i]).c_str());
  }
  for (size_t i = 0; i < 3; ++i) {
    Logger_More.addLog(
        projected_gravity_mat(i, 0),
        std::format("projected_gravity_{}", body_axis_mapping[i]).c_str());
  }

  std::map<size_t, std::string> commands_mapping = {
      {0, "v_x"}, {1, "v_y"}, {2, "omega"}};
  for (size_t i = 0; i < 3; ++i) {
    Logger_More.addLog(commands[i],
                       std::format("commands_{}", commands_mapping[i]).c_str());
  }

  if (log_net) {
    inference_net->log_result();  // HACK: temporary log result, should be move
                                  // to the inference thread later
  }
}

/**
 * @brief Torque Controller
 *
 */
void TorqueController() {
  // Logger_More.startLog();

  // Compute torque for non-wheel joints.
  for (int i = 0; i < 2; i++)
    for (int j = 0; j < 7; j++) {
      float position_error = joint_target_position[i][j] -
                             joint_current_position[i][j];  // position error
      /*joint_current_velocity[i][j] = velo_filters[i][j](*/
      /*    joint_current_velocity[i][j]);  // filter the velocity*/

      joint_target_torque[i][j] = p_gains[i][j] * position_error;  // P
      joint_target_torque[i][j] +=
          -d_gains[i][j] * joint_current_velocity[i][j];  // D

      // Torque compensation is zero...
      if (abs(joint_target_position[i][j] - joint_current_position[i][j]) >
          compensate_threshold[i][j] * deg2rad) {
        /*std::cout << "Compensate!!!" << std::endl;*/
        joint_target_torque[i][j] +=
            torque_compensate[i][j] *
            (joint_target_position[i][j] - joint_current_position[i][j]) /
            abs((joint_target_position[i][j] - joint_current_position[i][j]));
      }
    }

  log_result(0);

  SetJointAction();
  SetJointTorque_User();
}

void CompensationController() {
  for (int i = 0; i < 2; i++)
    for (int j = 0; j < 7; j++) {
      joint_current_velocity[i][j] = velo_filters[i][j](
          static_cast<float>(joint[i][j]->GetActualVelocity()));
      if (abs(joint_current_velocity[i][j]) >
          compensate_threshold[i][j] * deg2rad) {
        joint_current_position[i][j] =
            static_cast<float>(joint[i][j]->GetActualPosition());
        joint_current_velocity[i][j] = velo_filters[i][j](
            static_cast<float>(joint[i][j]->GetActualVelocity()));
        joint_target_torque[i][j] = torque_compensate[i][j] *
                                    joint_current_velocity[i][j] /
                                    abs(joint_current_velocity[i][j]);
        // joint_target_torque[i][j] = torque_compensate *
        // (joint_target_position[i][j] - joint_current_position[i][j]) /
        //                       abs((joint_target_position[i][j] -
        //                       joint_current_position[i][j]));
      } else {
        joint_target_torque[i][j] = 0.0;
      }
      // joint_target_position[i][j] = joint[i][j]->GetActualPosition();
      joint[i][j]->SetTargetTorque(
          static_cast<double>(joint_target_torque[i][j]));

      /*Logger_More.startLog();*/
      // Logger_More.addLog(has_sin_pos_init, "has_sin_pos_init");
      // Logger_More.addLog(has_init_policy, "has_init_policy");
      // Logger_More.addLog(cur_time - init_policy_time, "time");
    }

  // SetJointTorque_User();
}

/**
 * @brief Set joint torque limit and execuate
 *
 */
void SetJointTorque_User() {
  // Compute limit
  for (int i = 0; i < 2; i++)
    for (int j = 0; j < 7; j++) {
      if (motor_target_torque[i][j] > Torque_User_Limit[i][j]) {
        motor_target_torque[i][j] = Torque_User_Limit[i][j];
      } else if (motor_target_torque[i][j] < -Torque_User_Limit[i][j]) {
        motor_target_torque[i][j] = -Torque_User_Limit[i][j];
      }

      if (static_cast<float>(motor_current_position[i][j]) <
          LowerJointPosLimit[i][j]) {
        motor_target_torque[i][j] = 0;
      } else if (static_cast<float>(motor_current_position[i][j]) >
                 UpperJointPosLimit[i][j]) {
        motor_target_torque[i][j] = 0;
      }
    }
  /*for (size_t i = 0; i < 2; ++i) {*/
  /*  for (size_t j = 0; j < 7; ++j) {*/
  /*    std::cout << "i, j: " << i << ", " << j << " , "*/
  /*              << motor_target_torque[i][j] << std::endl;*/
  /*  }*/
  /*}*/
  // Set limit
  for (int i = 0; i < 2; i++)
    for (int j = 0; j < 7; j++) {
      joint[i][j]->SetTargetTorque(
          static_cast<double>(motor_target_torque[i][j]));
      joint[i][j]->SetTargetPosition(
          static_cast<double>(motor_target_position[i][j]));
    }
}
