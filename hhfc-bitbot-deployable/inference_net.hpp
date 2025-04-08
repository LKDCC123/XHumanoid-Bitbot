#ifndef __INFERENCE_NET_H__
#define __INFERENCE_NET_H__
#include <torch/script.h>

#include <array>
#include <chrono>
#include <ctime>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <typeinfo>

#include "LLog.hpp"
#include "ZenBuffer.hpp"
#include "bitbot_kernel/utils/priority.h"
template <typename T = float>
class InferenceNet {
 public:
  enum class StatusT { STOPPED, IDLE, STARTING, INFERRING, FINISHED };

 public:
  struct NetConfigT {
    struct InputConfigT {
      T obs_scales_ang_vel;
      T obs_scales_lin_vel = 0.1;
      T scales_commands;
      T obs_scales_dof_pos;
      T obs_scales_dof_vel;
      T obs_scales_euler;
      T clip_observations;
      int ctrl_model_input_size;
      int stack_length;
      int ctrl_model_unit_input_size;
    } input_config;

    struct OutputConfigT {
      T ctrl_clip_action;
      T action_scale;
      std::array<T, 12> ctrl_action_lower_limit;
      std::array<T, 12> ctrl_action_upper_limit;
      int ctrl_model_output_size;
    } output_config;

    std::vector<T> action_default_pos;
  };

 public:
  using InputT = std::vector<T>&;
  using OutputT = std::vector<T>&;
  using IOT = OutputT;
  using BatchIOT = std::vector<std::vector<T>>&;

  using Ptr = std::unique_ptr<InferenceNet<T>>;

 public:
  InferenceNet(const std::string& ctrl_model_path, NetConfigT& net_config,
               bool async = false, int interpolate = -1,
               lee::blocks::LLog<T>* logger = nullptr);

  ~InferenceNet();

  bool InferenceOnceErax(InputT clock, InputT command, InputT q, InputT dq,
                         InputT last_action, InputT base_angle_vel,
                         InputT base_eular);

  bool GetInfereceResult(OutputT action, BatchIOT interpolated_action);

  StatusT GetStatus() const;

 private:
  NetConfigT::InputConfigT input_cfg;
  NetConfigT::OutputConfigT output_cfg;

  const int interpolate;
  const bool async;

  lee::blocks::LLog<T>* logger;

  std::unique_ptr<std::thread> policy_thread;
  bool policy_thread_running;
  StatusT runner_status;
  std::chrono::time_point<std::chrono::system_clock> start_time;
  std::chrono::time_point<std::chrono::system_clock> end_time;

  std::unique_ptr<torch::jit::script::Module> ctrl_model;

  torch::Tensor action_default_pos;

  // torch::Tensor base_lin_vel_tensor_GT;
  torch::Tensor base_ang_vel_tensor;
  torch::Tensor base_eular_tensor;
  torch::Tensor project_gravity_tensor;
  torch::Tensor command_tensor;
  torch::Tensor dof_pos_obs_tensor;
  torch::Tensor dof_vel_obs_tensor;
  torch::Tensor last_action_tensor;
  torch::Tensor joint_last_error_tensor;
  torch::Tensor joint_last_vel_tensor;
  torch::Tensor clock_tensor;

  // torch::Tensor predict_other_tensor;
  torch::Tensor action_tensor;
  torch::Tensor scaled_action_tensor;
  std::vector<std::vector<T>> interpolated_action;

  /*std::unique_ptr<zzs::RingBuffer<torch::Tensor>> history_input_ptr;*/
  std::unique_ptr<zzs::RingBuffer<torch::Tensor>> history_input_ptr_erax;

 private:
  void run();

  void perform_interpolation();
  void perform_inference_erax();

  T clip(T value, T min, T max) { return std::max(min, std::min(max, value)); }

  void warm_up_net();

 public:  // HACK: temporary public for testing, fix multi thread logging issue
          // later
  void log_result();

  T y_offset;
  T yaw_offset;
};

using double_inference_net = InferenceNet<double>;
using float_inference_net = InferenceNet<float>;

/**********************************************************************************/
/**********************************************************************************/
/**********************************************************************************/

template <typename T>
InferenceNet<T>::InferenceNet(const std::string& ctrl_model_path,
                              NetConfigT& net_config, bool async,
                              int interpolate, lee::blocks::LLog<T>* logger)
    : async(async),
      interpolate(interpolate),
      logger(logger),
      policy_thread_running(false),
      runner_status(StatusT::STOPPED),
      input_cfg(net_config.input_config),
      start_time(std::chrono::system_clock::now()),
      end_time(std::chrono::system_clock::now()),
      output_cfg(net_config.output_config),
      y_offset(0.0),
      yaw_offset(0) {
  try {
    this->ctrl_model = std::make_unique<torch::jit::script::Module>(
        torch::jit::load(ctrl_model_path, torch::kCPU));
  } catch (const std::exception& e) {
    std::cerr << typeid(this).name() << ": failed to load model, " << e.what()
              << '\n';
  }

  // allocate interpolation buffer
  if (interpolate > 0) {
    this->interpolated_action.resize(interpolate + 1);
    for (size_t i = 0; i < this->interpolated_action.size(); i++) {
      this->interpolated_action[i].resize(
          this->output_cfg.ctrl_model_output_size);
    }
  }

  // load config
  this->action_default_pos =
      torch::from_blob(
          net_config.action_default_pos.data(),
          {static_cast<long int>(net_config.action_default_pos.size())})
          .clone();

  // start policy thread
  if (async) {
    this->policy_thread_running = true;
    this->policy_thread =
        std::make_unique<std::thread>(&InferenceNet<T>::run, this);
    this->policy_thread->detach();
  } else {
    this->runner_status = StatusT::IDLE;
  }

  torch::Tensor input_tensor =
      torch::zeros({this->input_cfg.ctrl_model_unit_input_size}, torch::kCPU);
  this->history_input_ptr_erax =
      std::make_unique<zzs::RingBuffer<torch::Tensor>>(
          this->input_cfg.stack_length, input_tensor);
}

template <typename T>
InferenceNet<T>::~InferenceNet() {
  if (this->policy_thread != nullptr) {
    this->policy_thread_running = false;
    this->policy_thread.reset();
  }
}

template <typename T>
void InferenceNet<T>::warm_up_net() {
  const int warm_up = 100;
  torch::Tensor ctrl_warm_up_input =
      torch::randn({input_cfg.ctrl_model_input_size});
  for (int i = 0; i < warm_up; i++) {
    auto out = this->ctrl_model->forward({ctrl_warm_up_input});
  }
}

template <typename T>
bool InferenceNet<T>::InferenceOnceErax(InputT clock, InputT command, InputT q,
                                        InputT dq, InputT last_action,
                                        InputT base_angle_vel,
                                        InputT base_eular) {
  if (this->runner_status != StatusT::IDLE) {
    std::cout << typeid(this).name()
              << ": perform_inference is called in wrong status, please check "
                 "the code! "
              << int(this->runner_status) << std::endl;
    fflush(stdout);
    return false;
  }
  torch::Tensor clk_tensor = torch::from_blob(clock.data(), {2}).clone();
  this->clock_tensor = clk_tensor;
  torch::Tensor cmd_tensor = torch::from_blob(command.data(), {3}).clone();
  cmd_tensor[0] = cmd_tensor[0] * this->input_cfg.obs_scales_lin_vel;
  cmd_tensor[1] = cmd_tensor[1] * this->input_cfg.obs_scales_lin_vel;
  cmd_tensor[2] = cmd_tensor[2] * this->input_cfg.obs_scales_ang_vel;
  this->command_tensor = cmd_tensor;
  torch::Tensor q_tensor =
      torch::from_blob(q.data(), {12}).clone() - this->action_default_pos;
  this->dof_pos_obs_tensor = q_tensor;
  torch::Tensor dq_tensor = torch::from_blob(dq.data(), {12}).clone();
  this->dof_vel_obs_tensor = dq_tensor;
  torch::Tensor lst_act_tensor =
      torch::from_blob(last_action.data(), {12}).clone();
  this->last_action_tensor = lst_act_tensor;
  torch::Tensor base_ang_vel_tensor_ =
      torch::from_blob(base_angle_vel.data(), {3}).clone();
  this->base_ang_vel_tensor = base_ang_vel_tensor_;
  torch::Tensor base_eu_tensor =
      torch::from_blob(base_eular.data(), {3}).clone();
  this->base_eular_tensor = base_eu_tensor;

  torch::Tensor input_tensor =
      torch::cat(
          {clk_tensor, cmd_tensor * this->input_cfg.scales_commands,
           q_tensor * this->input_cfg.obs_scales_dof_pos,
           dq_tensor * this->input_cfg.obs_scales_dof_vel, lst_act_tensor,
           base_ang_vel_tensor_ * this->input_cfg.obs_scales_ang_vel,
           base_eu_tensor * this->input_cfg.obs_scales_euler},
          0)
          .clip(-this->input_cfg.clip_observations,
                this->input_cfg.clip_observations);
  this->history_input_ptr_erax->push(input_tensor);

  if (!this->async) {
    this->perform_inference_erax();
  } else {
    this->runner_status = StatusT::STARTING;
  }
  return true;
}

template <typename T>
bool InferenceNet<T>::GetInfereceResult(OutputT action,
                                        BatchIOT interpolated_action) {
  if (this->runner_status == StatusT::IDLE) {
    std::cout << typeid(this).name()
              << ": GetInfereceResult is called in wrong status, please check "
                 "the code!"
              << std::endl;
    fflush(stdout);
    return false;
  }

  if (this->runner_status != StatusT::FINISHED) {
    return false;
  }

  action.assign(
      this->action_tensor.data_ptr<T>(),
      this->action_tensor.data_ptr<T>() + this->action_tensor.numel());
  interpolated_action = this->interpolated_action;

  this->runner_status = StatusT::IDLE;  // reset status after get result
  return true;
}

template <typename T>
InferenceNet<T>::StatusT InferenceNet<T>::GetStatus() const {
  return this->runner_status;
}

template <typename T>
void InferenceNet<T>::run() {
  // if (!setProcessHighPriority(20)) {
  //   printf("Failed to set process scheduling policy\n");
  // }
  this->warm_up_net();

  this->runner_status = StatusT::IDLE;
  while (this->policy_thread_running) {
    std::this_thread::sleep_for(std::chrono::microseconds(1));

    if (this->GetStatus() == StatusT::STARTING) {
      this->perform_inference_erax();
    }
  }
}

template <typename T>
void InferenceNet<T>::perform_inference_erax() {
  this->start_time = std::chrono::system_clock::now();
  this->runner_status = StatusT::INFERRING;

  torch::Tensor ctrl_net_input = this->history_input_ptr_erax->front();
  for (size_t i = 1; i < this->history_input_ptr_erax->size(); i++) {
    torch::Tensor tmp_ptr = this->history_input_ptr_erax.get()->operator()(i);
    ctrl_net_input = torch::cat({ctrl_net_input, tmp_ptr}, 0);
  }

  ctrl_net_input = ctrl_net_input.view({1, -1});
  ctrl_net_input = ctrl_net_input.squeeze(0);

  auto out = this->ctrl_model->forward({ctrl_net_input});

  this->action_tensor = (out.toTensor())
                            .clip(-this->output_cfg.ctrl_clip_action,
                                  this->output_cfg.ctrl_clip_action);
  // this->predict_lin_vel_tensor=out->elements()[1].toTensor();

  this->scaled_action_tensor =
      this->action_tensor * this->output_cfg.action_scale +
      this->action_default_pos;

  // post inference
  if (this->interpolate > 0) {
    this->perform_interpolation();
  }

  this->end_time =
      std::chrono::system_clock::now();  // HACK: fix multi thread logging
                                         // issue. temp add here
  if (this->logger != nullptr) {
    // NOTE: for time calculation accuracy, please do NOT add code after
    // log_result this->log_result(); //HACK: fix multi thread logging issue.
    // temp disable
  }

  this->runner_status = StatusT::FINISHED;
}

template <typename T>
void InferenceNet<T>::perform_interpolation() {
  torch::Tensor last_action_cliped =
      this->last_action_tensor * this->output_cfg.action_scale +
      this->action_default_pos;

  torch::Tensor action_diff = this->scaled_action_tensor - last_action_cliped;

  for (size_t i = 0; i < this->interpolate + 1; i++) {
    for (size_t j = 0; j < this->output_cfg.ctrl_model_output_size; j++) {
      T temp = last_action_cliped[j].item<T>() +
               action_diff[j].item<T>() *
                   (static_cast<T>(i) / static_cast<T>(this->interpolate));

      // Limits are add here
      this->interpolated_action[i][j] =
          this->clip(temp, this->output_cfg.ctrl_action_lower_limit[j],
                     this->output_cfg.ctrl_action_upper_limit[j]);
    }
  }
}

template <typename T>
void InferenceNet<T>::log_result() {
  this->logger->startLog();
  // 2
  this->logger->addLog(this->clock_tensor[0].item<T>(), "input_0_clock_0");
  this->logger->addLog(this->clock_tensor[1].item<T>(), "input_1_clock_1");

  // 5
  this->logger->addLog(this->command_tensor[0].item<T>(), "input_2_command_vx");
  this->logger->addLog(this->command_tensor[1].item<T>(), "input_3_command_vy");
  this->logger->addLog(this->command_tensor[2].item<T>(),
                       "input_4_command_omega");

  // 17
  for (size_t i = 0; i < 2; ++i) {
    for (size_t j = 0; j < 6; ++j) {
      this->logger->addLog(
          this->dof_pos_obs_tensor[i * 6 + j].item<T>(),
          std::format("input_{}_dof_pos_{}{}", 5 + i * 6 + j, i, j).c_str());
    }
  }

  // 29
  for (size_t i = 0; i < 2; ++i) {
    for (size_t j = 0; j < 6; ++j) {
      this->logger->addLog(
          this->dof_vel_obs_tensor[i * 6 + j].item<T>(),
          std::format("input_{}_dof_vel_{}{}", 17 + i * 6 + j, i, j).c_str());
    }
  }

  // 41
  for (size_t i = 0; i < 2; ++i) {
    for (size_t j = 0; j < 6; ++j) {
      this->logger->addLog(
          this->last_action_tensor[i * 6 + j].item<T>(),
          std::format("input_{}_last_action_{}{}", 29 + i * 6 + j, i, j)
              .c_str());
    }
  }

  // 44
  this->logger->addLog(this->base_ang_vel_tensor[0].item<T>(),
                       "input_41_base_ang_vel_x");
  this->logger->addLog(this->base_ang_vel_tensor[1].item<T>(),
                       "input_42_base_ang_vel_y");
  this->logger->addLog(this->base_ang_vel_tensor[2].item<T>(),
                       "input_43_base_ang_vel_z");

  // 47
  // This should be in rad
  /*this->logger->addLog(this->base_eular_tensor[0].item<T>(),*/
  /*                     "input_44_eular_roll");*/
  /*this->logger->addLog(this->base_eular_tensor[1].item<T>(),*/
  /*                     "input_45_eular_pitch");*/
  /*this->logger->addLog(this->base_eular_tensor[2].item<T>(),*/
  /*                     "input_46_eular_yaw");*/
  this->logger->addLog(this->base_eular_tensor[0].item<T>(),
                       "input_44_gravity_x");
  this->logger->addLog(this->base_eular_tensor[1].item<T>(),
                       "input_45_gravity_y");
  this->logger->addLog(this->base_eular_tensor[2].item<T>(),
                       "input_46_gravity_z");

  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
      this->end_time - this->start_time);  // 微秒
  int64_t count = (duration.count() < 0) ? 0 : duration.count();
  this->logger->addLog(static_cast<T>(count), "net_inference_time");
}

#endif  //__INFERENCE_NET_H__
