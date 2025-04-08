/**
 * @file parallel_ankle.hpp
 * @brief Parallel ankle forward kinematics and torque transformation
 * @date 2025-02-18
 * @author Dknt
 */

#ifndef PARALLEL_ANKLE
#define PARALLEL_ANKLE

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <format>
#include <iostream>

/**
 * @brief Parallel ankle kinematics and torque transformation.
 *        Refer to this paper: On the Comprehensive Kinematics Analysis of a
 *                             Humanoid Parallel Ankle
 *                             Mechanism. 10.1115/1.4040886
 */
template <typename T = double>
class ParallelAnkle {
 public:
  /**
   * struct AnkleParameters - Parameters for the parallel ankle.
   */
  struct AnkleParameters {
    T l_bar1 = 0.1;
    T l_rod1 = 0.2;
    Eigen::Matrix<T, 3, 1> r_a1 = Eigen::Matrix<T, 3, 1>::Zero();
    Eigen::Matrix<T, 3, 1> r_b1_0 = Eigen::Matrix<T, 3, 1>::Zero();
    Eigen::Matrix<T, 3, 1> r_c1_0 = Eigen::Matrix<T, 3, 1>::Zero();
    T l_bar2 = 0.1;
    T l_rod2 = 0.2;
    Eigen::Matrix<T, 3, 1> r_a2 = Eigen::Matrix<T, 3, 1>::Zero();
    Eigen::Matrix<T, 3, 1> r_b2_0 = Eigen::Matrix<T, 3, 1>::Zero();
    Eigen::Matrix<T, 3, 1> r_c2_0 = Eigen::Matrix<T, 3, 1>::Zero();
  };

  /**
   * @brief Constuctor
   *
   * @param[in] params Ankles structure parameters
   * @param[in] threshold Thereshold to terminate iterative forward kinematics
   */
  ParallelAnkle(const AnkleParameters &params, const T threshold = 1e-6);

  /**
   * @brief Compute forward kinematics. From     *
   * @param[in] theta1 Joint position 1 (left)
   * @param[in] theta2 Joint position 2 (right)
   * @return Foot pitch and roll [pitch, roll]
   */
  Eigen::Matrix<T, 2, 1> ForwardKinematics(const T theta1, const T theta2);

  /**
   * @brief Remap torque from pitch roll to joint torques. This method should be
   * used after calling ForwardKinematics
   *
   * @param[in] torque_pitch Torque in pitch
   * @param[in] torque_roll Torque in roll
   * @return Joint torques [j1, j2]
   */
  inline Eigen::Matrix<T, 2, 1> TorqueRemapping(const T torque_pitch,
                                                const T torque_roll) {
    // [roll, pitch]
    Eigen::Matrix<T, 2, 1> torque_ankle(torque_roll, torque_pitch);
    Eigen::Matrix<T, 2, 1> torque_motor =
        this->J_c_.inverse().inverse() * torque_ankle;
    return torque_motor;
  }

  inline Eigen::Matrix<T, 2, 1> VelocityMapping(const T omega1,
                                                const T omega2) {
    Eigen::Matrix<T, 2, 1> vel_motor(omega1, omega2);
    // [roll, pitch]
    Eigen::Matrix<T, 2, 1> vel_ankle_rp = this->J_c_.inverse() * vel_motor;
    Eigen::Matrix<T, 2, 1> vel_ankle(vel_ankle_rp(1, 0), vel_ankle_rp(0, 0));
    return vel_ankle;
  }

 public:
  /**
   * @brief Inverse Kinematics. From foot pitch roll to joint positions.
   *
   * @param[in] pitch Pitch in rad
   * @param[in] roll Roll in rad
   *
   * @return Joint positions [j1, j2]
   */
  Eigen::Matrix<T, 2, 1> InverseKinematics(const T pitch, const T roll);

  /**
   * @brief Compute Jacobian
   *
   * @param[in] pitch Pitch in rad
   * @param[in] roll Roll in rad
   * @param[in] theta1 Left joint (y+) position in rad
   * @param[in] theta2 Right joint (y-) position in rad
   */
  void ComputeJacobian(const T pitch, const T roll, const T theta1,
                       const T theta2);

 private:
  size_t error_count_ = 0;
  size_t fatal_count_ = 0;
  AnkleParameters params_;
  T l_rod1_;
  T l_bar1_;
  T l_rod2_;
  T l_bar2_;
  T threshold_;

  Eigen::Matrix<T, 3, 1> r_a1_;
  Eigen::Matrix<T, 3, 1> r_b1_0_;
  Eigen::Matrix<T, 3, 1> r_c1_0_;
  Eigen::Matrix<T, 3, 1> r_a2_;
  Eigen::Matrix<T, 3, 1> r_b2_0_;
  Eigen::Matrix<T, 3, 1> r_c2_0_;

  // Values for iteration
  T last_pitch_ = 0.0;
  T last_roll_ = 0.0;
  Eigen::Matrix2<T> J_c_ = Eigen::Matrix<T, 2, 2>::Identity();
};

template <typename T>
ParallelAnkle<T>::ParallelAnkle(const AnkleParameters &params,
                                const T threshold)
    : params_(params), threshold_(threshold) {
  l_rod1_ = params_.l_rod1;
  l_bar1_ = params_.l_bar1;
  l_rod2_ = params_.l_rod2;
  l_bar2_ = params_.l_bar2;

  r_a1_ = params_.r_a1;
  r_b1_0_ = params_.r_b1_0;
  r_c1_0_ = params_.r_c1_0;
  r_a2_ = params_.r_a2;
  r_b2_0_ = params_.r_b2_0;
  r_c2_0_ = params_.r_c2_0;
}

template <typename T>
Eigen::Matrix<T, 2, 1> ParallelAnkle<T>::InverseKinematics(const T pitch,
                                                           const T roll) {
  // TODO: Optimize this
  Eigen::Matrix<T, 3, 3> Rof = Eigen::Matrix<T, 3, 3>(
      Eigen::AngleAxis<T>(pitch, Eigen::Matrix<T, 3, 1>::UnitY()) *
      Eigen::AngleAxis<T>(roll, Eigen::Matrix<T, 3, 1>::UnitX()));

  // Joint C position
  Eigen::Matrix<T, 3, 1> r_c1 = Rof * (this->r_c1_0_);
  Eigen::Matrix<T, 3, 1> r_c2 = Rof * (this->r_c2_0_);

  T a1 = (r_c1 - this->r_a1_)(0);
  T b1 = (this->r_a1_ - r_c1)(2);
  T c1 = (std::pow(this->l_rod1_, 2) - std::pow(this->l_bar1_, 2) -
          std::pow((r_c1 - this->r_a1_).norm(), 2)) /
         (2 * this->l_bar1_);

  T a2 = (r_c2 - this->r_a2_)(0);
  T b2 = (this->r_a2_ - r_c2)(2);
  T c2 = (std::pow(this->l_rod2_, 2) - std::pow(this->l_bar2_, 2) -
          std::pow((r_c2 - this->r_a2_).norm(), 2)) /
         (2 * this->l_bar2_);

  Eigen::Matrix<T, 2, 1> theta_res(2);

  theta_res(0, 0) = std::asin(
      (b1 * c1 + std::sqrt(b1 * b1 * c1 * c1 -
                           (a1 * a1 + b1 * b1) * (c1 * c1 - a1 * a1))) /
      (a1 * a1 + b1 * b1));
  theta_res(1, 0) = std::asin(
      (b2 * c2 + std::sqrt(b2 * b2 * c2 * c2 -
                           (a2 * a2 + b2 * b2) * (c2 * c2 - a2 * a2))) /
      (a2 * a2 + b2 * b2));

  return theta_res;
}

template <typename T>
void ParallelAnkle<T>::ComputeJacobian(const T pitch, const T roll,
                                       const T theta1, const T theta2) {
  // Joint B position
  Eigen::Matrix<T, 3, 1> r_b1 =
      this->r_a1_ + Eigen::Matrix<T, 3, 3>(Eigen::AngleAxis<T>(
                        theta1, Eigen::Matrix<T, 3, 1>::UnitY())) *
                        (this->r_b1_0_ - this->r_a1_);
  Eigen::Matrix<T, 3, 1> r_b2 =
      this->r_a2_ + Eigen::Matrix<T, 3, 3>(Eigen::AngleAxis<T>(
                        theta2, Eigen::Matrix<T, 3, 1>::UnitY())) *
                        (this->r_b2_0_ - this->r_a2_);

  // Orientation of foot w.r.t. the base
  // TODO: Optimize this
  Eigen::Matrix<T, 3, 3> Rof = Eigen::Matrix<T, 3, 3>(
      Eigen::AngleAxis<T>(pitch, Eigen::Matrix<T, 3, 1>::UnitY()) *
      Eigen::AngleAxis<T>(roll, Eigen::Matrix<T, 3, 1>::UnitX()));

  // Joint C position
  Eigen::Matrix<T, 3, 1> r_c1 = Rof * (this->r_c1_0_);
  Eigen::Matrix<T, 3, 1> r_c2 = Rof * (this->r_c2_0_);

  Eigen::Matrix<T, 3, 1> r_bar_1 = r_b1 - this->r_a1_;
  Eigen::Matrix<T, 3, 1> r_bar_2 = r_b2 - this->r_a2_;
  Eigen::Matrix<T, 3, 1> r_rod_1 = r_c1 - r_b1;
  Eigen::Matrix<T, 3, 1> r_rod_2 = r_c2 - r_b2;

  // Compute Jacobian
  Eigen::Matrix<T, 2, 6> J_x;
  Eigen::Matrix<T, 2, 2> J_theta;
  Eigen::Matrix<T, 2, 6> J;
  Eigen::Matrix<T, 6, 2> G;

  J_x.block(0, 0, 1, 3) = r_rod_1.transpose();
  J_x.block(0, 3, 1, 3) = (r_c1.cross(r_rod_1)).transpose();
  J_x.block(1, 0, 1, 3) = r_rod_2.transpose();
  J_x.block(1, 3, 1, 3) = (r_c2.cross(r_rod_2)).transpose();

  J_theta.setZero();
  J_theta(0, 0) = Eigen::Matrix<T, 1, 3>::UnitY().dot(r_bar_1.cross(r_rod_1));
  J_theta(1, 1) = Eigen::Matrix<T, 1, 3>::UnitY().dot(r_bar_2.cross(r_rod_2));

  G.setZero();
  G(3, 0) = std::cos(pitch);
  G(5, 0) = -std::sin(pitch);
  G(4, 1) = 1.0;

  this->J_c_ = J_theta.inverse() * J_x * G;
}

template <typename T>
Eigen::Matrix<T, 2, 1> ParallelAnkle<T>::ForwardKinematics(const T theta1,
                                                           const T theta2) {
  if (std::isnan(this->last_pitch_) || std::isnan(this->last_roll_)) {
    std::cout << "Fatal in ankle solving!!! Press SPACE NOW!!!"
              << this->fatal_count_++ << std::endl;
    this->last_pitch_ = 0.0;
    this->last_roll_ = 0.0;
  }
  Eigen::Matrix<T, 2, 1> theta_ref(theta1, theta2);

  for (size_t i = 0; i < 10; ++i) {
    Eigen::Matrix<T, 2, 1> theta_k;
    theta_k = this->InverseKinematics(this->last_pitch_, this->last_roll_);
    Eigen::Matrix<T, 2, 1> err = theta_k - theta_ref;

    /*std::cout << "Current i: " << i << std::endl;*/
    /*std::cout << "err: " << err.transpose() << std::endl;*/
    /*std::cout << "theta_k: " << theta_k.transpose() << std::endl;*/

    this->ComputeJacobian(last_pitch_, last_roll_, theta_k(0, 0),
                          theta_k(1, 0));

    // Check
    if (std::isnan(this->last_pitch_) || std::isnan(this->last_roll_)) {
      // First check.
      std::cout << "Ankle solving failed!!! Consider PRESS SPACE!!!"
                << this->error_count_++ << std::endl;
      std::cout << "theta1: " << theta1 << " theta2: " << theta2 << std::endl;
      this->last_pitch_ = 0.0;
      this->last_roll_ = 0.0;
      theta_k = this->InverseKinematics(this->last_pitch_, this->last_roll_);
      this->ComputeJacobian(last_pitch_, last_roll_, theta_k(0, 0),
                            theta_k(1, 0));
      break;
    } else if (err.norm() < this->threshold_) {
      /*std::cout << "Iteration: " << i << std::endl;*/
      break;
    } else {
      Eigen::Matrix<T, 2, 1> x_c_k(this->last_roll_, this->last_pitch_);
      x_c_k -= this->J_c_.inverse() * err;

      /*std::cout << "xck: " << x_c_k.transpose() << std::endl << std::endl;*/

      this->last_pitch_ = x_c_k(1, 0);
      this->last_roll_ = x_c_k(0, 0);
    }

    /*std::cout << "Jacobian:\n" << this->J_c_ << std::endl;*/
  }

  /*std::cout << std::format("Pitch: {} Roll: {}", this->last_pitch_,*/
  /*                         this->last_roll_)*/
  /*          << std::endl;*/
  Eigen::Matrix<T, 2, 1> pitch_roll(this->last_pitch_, this->last_roll_);
  return pitch_roll;
}

#endif  // !PARALLEL_ANKLE
