/****************************************************************************
MIT License

Copyright (c) 2024 zishun zhou

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*****************************************************************************/

#pragma once
#ifndef INEKF_WARPER_HPP
#define INEKF_WARPER_HPP

#include <array>
#include <string>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <type_traits>
#include <memory>
#include "invariant-ekf/InEKF.h"
#include "invariant-ekf/NoiseParams.h"
#include "invariant-ekf/RobotState.h"
#include "libdef.h"

namespace zzs
{
	template <size_t LEG_NUM, size_t JOINT_NUM, typename SCALAR>
	class INEKF_API StateEstimator
	{
		static_assert(std::is_floating_point<SCALAR>::value, "SCALAR must be floating point type");

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		enum class JointRotDir_e
		{
			AXIS_X = 0,
			AXIS_Y = 1,
			AXIS_Z = 2,

			AXIS_REV_X = 3,
			AXIS_REV_Y = 4,
			AXIS_REV_Z = 5
		};
		using Ptr = std::unique_ptr<StateEstimator<LEG_NUM, JOINT_NUM, SCALAR>>;

	public:
		StateEstimator()
			: dt(0)
		{
			this->PrintWelcomeMessage();
		}

		StateEstimator(const SCALAR dt,
					   const std::array<std::array<std::array<SCALAR, 3>, JOINT_NUM + 1>, LEG_NUM> &rot_ang,
					   const std::array<std::array<std::array<SCALAR, 3>, JOINT_NUM + 1>, LEG_NUM> &trans_vec,
					   const std::array<std::array<JointRotDir_e, JOINT_NUM>, LEG_NUM> &joint_rot_dir,
					   const std::array<std::array<SCALAR, JOINT_NUM>, LEG_NUM> &init_Joint_ang,
					   const std::array<std::array<SCALAR, JOINT_NUM>, LEG_NUM> &joint_noise,
					   const inekf::RobotState &init_InEKF_state, const inekf::NoiseParams &init_InEKF_noise_param) : dt(dt)
		{
			std::cout << "leg number=" << LEG_NUM << std::endl;
			std::cout << "joint number=" << JOINT_NUM << std::endl;

			for (size_t i = 0; i < LEG_NUM; i++)
			{
				// deal with link
				for (size_t j = 0; j < JOINT_NUM + 1; j++)
				{
					// rotate in rpy order

					Eigen::Matrix3<SCALAR> rotation_matrix =
						(Eigen::AngleAxis<SCALAR>(rot_ang[i][j][2], Eigen::Vector3<SCALAR>::UnitZ()) *
						 Eigen::AngleAxis<SCALAR>(rot_ang[i][j][1], Eigen::Vector3<SCALAR>::UnitY()) *
						 Eigen::AngleAxis<SCALAR>(rot_ang[i][j][0], Eigen::Vector3<SCALAR>::UnitX()))
							.matrix();

					ConstTransMat[i][j] = Eigen::Matrix4<SCALAR>::Identity();
					ConstTransMat[i][j].template block<3, 3>(0, 0) = rotation_matrix;
					ConstTransMat[i][j].template block<3, 1>(0, 3) << trans_vec[i][j][0], trans_vec[i][j][1], trans_vec[i][j][2];
				}

				// deal with joint
				this->JointCovMat[i] = Eigen::Matrix<SCALAR, JOINT_NUM, JOINT_NUM>::Identity();
				for (size_t j = 0; j < JOINT_NUM; j++)
				{
					switch (joint_rot_dir[i][j])
					{
					case JointRotDir_e::AXIS_X:
						this->JointRotSign[i][j][0] = 1;
						this->JointRotSign[i][j][1] = 0;
						this->JointRotSign[i][j][2] = 0;
						break;
					case JointRotDir_e::AXIS_Y:
						this->JointRotSign[i][j][0] = 0;
						this->JointRotSign[i][j][1] = 1;
						this->JointRotSign[i][j][2] = 0;
						break;
					case JointRotDir_e::AXIS_Z:
						this->JointRotSign[i][j][0] = 0;
						this->JointRotSign[i][j][1] = 0;
						this->JointRotSign[i][j][2] = 1;
						break;

					case JointRotDir_e::AXIS_REV_X:
						this->JointRotSign[i][j][0] = -1;
						this->JointRotSign[i][j][1] = 0;
						this->JointRotSign[i][j][2] = 0;
						break;
					case JointRotDir_e::AXIS_REV_Y:
						this->JointRotSign[i][j][0] = 0;
						this->JointRotSign[i][j][1] = -1;
						this->JointRotSign[i][j][2] = 0;
						break;
					case JointRotDir_e::AXIS_REV_Z:
						this->JointRotSign[i][j][0] = 0;
						this->JointRotSign[i][j][1] = 0;
						this->JointRotSign[i][j][2] = -1;
						break;
					}

					this->JointCovMat[i](j, j) = joint_noise[i][j];
					this->JointTransMat[i][j] = Eigen::Matrix4<SCALAR>::Identity();
					this->JointTransMat[i][j].template block<3, 3>(0, 0) =
						Eigen::AngleAxis<SCALAR>(init_Joint_ang[i][j], this->JointRotSign[i][j]).toRotationMatrix();
				}
			}

			this->UpdateJacobian(init_Joint_ang);

			// InEKF_Filter = inekf::InEKF(init_InEKF_state, init_InEKF_noise_param);
			this->InEKF_Filter.setState(init_InEKF_state);
			this->InEKF_Filter.setNoiseParams(init_InEKF_noise_param);
			std::cout << "Noise parameters are initialized to: \n";
			std::cout << this->InEKF_Filter.getNoiseParams() << std::endl;
			std::cout << "Joint Noise:\n";
			for (auto &joint_vec : joint_noise)
			{
				std::cout << "[";
				for (auto &joint : joint_vec)
				{
					std::cout << joint << " ";
				}
				std::cout << "]\n";
			}

			for (size_t i = 0; i < LEG_NUM; i++)
			{
				std::cout << "leg " << i << " jacobi matrix:\n"
						  << this->JacoMat[i] << std::endl;
			}

			/*for (size_t i = 0; i < LEG_NUM; i++)
			{
				Eigen::Matrix4<SCALAR> AccumulateTransMat = Eigen::Matrix4<SCALAR>::Identity();
				for (size_t j = 0; j < JOINT_NUM; j++)
				{
					AccumulateTransMat = AccumulateTransMat * (this->ConstTransMat[i][j] * this->JointTransMat[i][j]);
					std::cout << "leg " << i << "joint " << j << "Pose:\n";
					std::cout << AccumulateTransMat << std::endl;
				}
				AccumulateTransMat = AccumulateTransMat * this->ConstTransMat[i][JOINT_NUM];
				std::cout << "leg " << i << "End Pose:\n";
				std::cout << AccumulateTransMat << std::endl;
			}*/

			/*std::cout << "Robot's state is initialized to: \n";
			std::cout << this->InEKF_Filter->getState() << std::endl;*/

			this->PrintWelcomeMessage();
		}

		Eigen::Vector3<SCALAR> operator()(
			const std::array<std::array<SCALAR, JOINT_NUM>, LEG_NUM> &joint_angle,
			const std::array<std::array<SCALAR, JOINT_NUM>, LEG_NUM> &joint_velo,
			const std::array<bool, LEG_NUM> &ContactStatus)
		{
			return this->ComputeLinarVelocityWithPureKinematic(joint_angle, joint_velo, ContactStatus);
		}

		// the name of this method is so loooooong
		Eigen::Vector3<SCALAR> ComputeLinarVelocityWithPureKinematic(
			const std::array<std::array<SCALAR, JOINT_NUM>, LEG_NUM> &joint_angle,
			const std::array<std::array<SCALAR, JOINT_NUM>, LEG_NUM> &joint_velo,
			const std::array<bool, LEG_NUM> &ContactStatus)
		{
			Eigen::Vector3<SCALAR> BaseLinearVelocity = Eigen::Vector3<SCALAR>::Zero();
			this->UpdateJacobian(joint_angle);
			for (size_t i = 0; i < LEG_NUM; i++)
			{
				if (!ContactStatus[i])
					continue;
				else
				{
					// 计算末端点的线速度
					Eigen::Vector<SCALAR, JOINT_NUM> q = Eigen::Vector<SCALAR, JOINT_NUM>::Map(joint_velo[i].data());

					// NOTE: 其实不太需要质心角速度
					// Eigen::Matrix3<SCALAR> RotationMatrix;
					// const Eigen::AngleAxis<SCALAR> roll(BaseAngVel[0], Eigen::Vector3<SCALAR>::UnitX());
					// const Eigen::AngleAxis<SCALAR> pitch(BaseAngVel[1], Eigen::Vector3<SCALAR>::UnitY());
					// const Eigen::AngleAxis<SCALAR> yaw(BaseAngVel[2], Eigen::Vector3<SCALAR>::UnitZ());
					// RotationMatrix = yaw * pitch * roll;
					// Eigen::AngleAxis<SCALAR> AngleAxis;
					// AngleAxis.fromRotationMatrix(RotationMatrix);
					// Eigen::Vector3<SCALAR> AngleAxisVec = AngleAxis.axis() * AngleAxis.angle();
					// Eigen::Vector3<SCALAR> EndPos = (this->EndTransMat[i].template block<3, 1>(0, 3));
					////std::cout<<"EndPos="<<EndPos<<std::endl;

					Eigen::Vector<SCALAR, 6> EndEffectorVel = -this->JacoMat[i] * q;
					// NOTE: 其实不太需要质心角速度
					BaseLinearVelocity = (EndEffectorVel.template block<3, 1>(0, 0)); // -AngleAxisVec.cross(EndPos);
					break;
				}
			}
			return BaseLinearVelocity;
		}

		std::tuple<Eigen::Vector3<SCALAR>, Eigen::Matrix3<SCALAR>> operator()(
			const std::array<std::array<SCALAR, JOINT_NUM>, LEG_NUM> &joint_angle,
			const std::array<std::array<SCALAR, JOINT_NUM>, LEG_NUM> &joint_velo,
			const std::array<bool, LEG_NUM> &ContactStatus,
			const Eigen::Vector<SCALAR, 6> &IMU_Measurement)
		{
			return this->ComputeLinarVelocityWithKinematicAndIMU(joint_angle, joint_velo, ContactStatus, IMU_Measurement);
		}

		std::tuple<Eigen::Vector3<SCALAR>, Eigen::Matrix3<SCALAR>> ComputeLinarVelocityWithKinematicAndIMU(
			const std::array<std::array<SCALAR, JOINT_NUM>, LEG_NUM> &joint_angle,
			const std::array<std::array<SCALAR, JOINT_NUM>, LEG_NUM> &joint_velo,
			const std::array<bool, LEG_NUM> &ContactStatus,
			const Eigen::Vector<SCALAR, 6> &IMU_Measurement)
		{
			this->UpdateJacobian(joint_angle);
			Eigen::Matrix<double, 6, 1> IMU_Measurement_double = IMU_Measurement.template cast<double>();

			std::vector<std::pair<int, bool>> contact_pairs(LEG_NUM);
			inekf::vectorKinematics measured_kinematics;
			measured_kinematics.reserve(LEG_NUM);

			for (int i = 0; i < LEG_NUM; i++)
			{
				bool contact_ = ContactStatus[i]; // remove const prop
				contact_pairs[i] = std::make_pair(i, contact_);
				inekf::Kinematics frame(i, this->EndTransMat[i].template cast<double>(), this->ObsvCovMat[i].template cast<double>());
				measured_kinematics.emplace_back(frame);
			}

			// update ekf
			this->InEKF_Filter.Propagate(IMU_Measurement_double, this->dt);
			this->InEKF_Filter.setContacts(contact_pairs);
			this->InEKF_Filter.CorrectKinematics(measured_kinematics);

			// 改名叫auto++得了
			auto curr_status = this->InEKF_Filter.getState();
			auto vel_w = curr_status.getVelocity().cast<SCALAR>();
			auto pos_w = curr_status.getRotation().cast<SCALAR>();
			auto vel_b = pos_w.transpose() * vel_w;
			return std::make_tuple(vel_b, pos_w);
		}

		void AlignWithGravityCoordinate(const std::array<SCALAR, 3> &rpy)
		{
			Eigen::Vector3<SCALAR> rpy_eigen = {rpy[0], rpy[1], rpy[2]};
			Eigen::Matrix4<SCALAR> trans_matrix = this->rpy2rot(rpy_eigen);
			auto current_states = this->InEKF_Filter.getState();
			current_states.setRotation(trans_matrix.template block<3, 3>(0, 0).template cast<double>());
			this->InEKF_Filter.setState(current_states);
		}

	private:
		void PrintWelcomeMessage()
		{
			std::string line0 = "\033[32m=========================================================================================================================\033[0m";
			std::string line1 = "\033[32m|| SSSSS  TTTTTTT  AAAAA  TTTTTTT  EEEEE        EEEEE   SSSSS  TTTTTTT  I   M     M    AAAAA   TTTTTTT  OOOOO   RRRRR  ||\033[0m";
			std::string line2 = "\033[32m|| S         T    A     A    T     E            E      S          T     I   MM   MM   A     A     T     O   O   R   R  ||\033[0m";
			std::string line3 = "\033[32m|| SSSSS     T    AAAAAAA    T     EEEE         EEEE    SSSSS     T     I   M M M M   AAAAAAA     T     O   O   RRRR   ||\033[0m";
			std::string line4 = "\033[32m||     S     T    A     A    T     E            E            S    T     I   M  M  M   A     A     T     O   O   R  R   ||\033[0m";
			std::string line5 = "\033[32m|| SSSSS     T    A     A    T     EEEEE        EEEEE   SSSSS     T     I   M     M   A     A     T     OOOOO   R   RR ||\033[0m";
			std::string line6 = "\033[32m=========================================================================================================================\033[0m";

			std::cout << std::endl
					  << std::endl
					  << std::endl;
			std::cout << "\033[31mwelcome to use this\033[0m" << std::endl;
			std::cout << line0 << std::endl;
			std::cout << line1 << std::endl;
			std::cout << line2 << std::endl;
			std::cout << line3 << std::endl;
			std::cout << line4 << std::endl;
			std::cout << line5 << std::endl;
			std::cout << line6 << std::endl
					  << std::endl;
		}

		Eigen::Matrix4<SCALAR> rpy2rot(const Eigen::Vector3<SCALAR> &rpy)
		{
			Eigen::Matrix4<SCALAR> rot_mat = Eigen::Matrix4<SCALAR>::Identity();
			rot_mat.template block<3, 3>(0, 0) =
				(Eigen::AngleAxis<SCALAR>(rpy[2], Eigen::Vector3<SCALAR>::UnitZ()) *
				 Eigen::AngleAxis<SCALAR>(rpy[1], Eigen::Vector3<SCALAR>::UnitY()) *
				 Eigen::AngleAxis<SCALAR>(rpy[0], Eigen::Vector3<SCALAR>::UnitX()))
					.matrix();
			return rot_mat;
		}

		void UpdateJacobian(const std::array<std::array<SCALAR, JOINT_NUM>, LEG_NUM> &curr_ang)
		{
			for (size_t i = 0; i < LEG_NUM; i++)
			{
				// update rot matrix
				for (size_t j = 0; j < JOINT_NUM; j++)
				{
					this->JointTransMat[i][j] = Eigen::Matrix4<SCALAR>::Identity();
					this->JointTransMat[i][j].template block<3, 3>(0, 0) = Eigen::AngleAxis<SCALAR>(curr_ang[i][j], this->JointRotSign[i][j]).toRotationMatrix();
				}

				Eigen::Matrix<SCALAR, 3, JOINT_NUM> JacoRot;
				Eigen::Matrix<SCALAR, 3, JOINT_NUM> JacoTrans;

				Eigen::Matrix4<SCALAR> AccumulateTransMat = Eigen::Matrix4<SCALAR>::Identity();
				std::array<Eigen::Vector4<SCALAR>, JOINT_NUM> JointPos;
				Eigen::Vector4<SCALAR> EndEffectorPos;

				// 累乘计算各关节旋转向量在机器人本体坐标系下的表示，和各关节坐标在本体坐标系下的表示
				for (size_t j = 0; j < JOINT_NUM; j++)
				{
					AccumulateTransMat *= (this->ConstTransMat[i][j] * this->JointTransMat[i][j]);
					Eigen::Vector3<SCALAR> rot_vec = (AccumulateTransMat.template block<3, 3>(0, 0)) * this->JointRotSign[i][j];
					JacoRot.template block<3, 1>(0, j) = rot_vec;
					JointPos[j] = AccumulateTransMat * Eigen::Vector4<SCALAR>::UnitW();
				}

				// 计算末端点的坐标
				AccumulateTransMat *= this->ConstTransMat[i][JOINT_NUM];
				EndEffectorPos = AccumulateTransMat * Eigen::Vector4<SCALAR>::UnitW();

				// 更新末端姿态
				this->EndTransMat[i] = AccumulateTransMat;

				// 计算末端点的线速度
				for (size_t j = 0; j < JOINT_NUM; j++)
				{
					Eigen::Vector3<SCALAR> PosDiff = (EndEffectorPos.template block<3, 1>(0, 0)) - (JointPos[j].template block<3, 1>(0, 0));
					Eigen::Vector3<SCALAR> EndLinVel = (JacoRot.template block<3, 1>(0, j)).cross(PosDiff);
					JacoTrans.template block<3, 1>(0, j) = EndLinVel;
				}

				// 前三维是线速度，后三维是角速度
				this->JacoMat[i].template block<3, JOINT_NUM>(0, 0) = JacoTrans;
				this->JacoMat[i].template block<3, JOINT_NUM>(3, 0) = JacoRot;

				// 计算观测噪声协方差矩阵
				this->ObsvCovMat[i] = this->JacoMat[i] * this->JointCovMat[i] * this->JacoMat[i].transpose();
			}
		}

	private:
		const SCALAR g = 9.81;
		const SCALAR dt;

		inekf::InEKF InEKF_Filter;

		std::array<std::array<Eigen::Matrix<SCALAR, 4, 4>, JOINT_NUM + 1>, LEG_NUM> ConstTransMat;
		std::array<std::array<Eigen::Matrix<SCALAR, 4, 4>, JOINT_NUM>, LEG_NUM> JointTransMat;

		std::array<Eigen::Matrix4<SCALAR>, LEG_NUM> EndTransMat;

		// indicate the sign of joint rotation, 1 for positive, -1 for negative and 0 for no rotation
		std::array<std::array<Eigen::Vector3<SCALAR>, JOINT_NUM>, LEG_NUM> JointRotSign;

		// Jacobian matrix
		std::array<Eigen::Matrix<SCALAR, 6, JOINT_NUM>, LEG_NUM> JacoMat;

		// Joint covariance matrix
		std::array<Eigen::Matrix<SCALAR, JOINT_NUM, JOINT_NUM>, LEG_NUM> JointCovMat;

		// Observation covariance matrix, square matrix of Jacobian times Joint covariance matrix
		std::array<Eigen::Matrix<SCALAR, 6, 6>, LEG_NUM> ObsvCovMat;
	};

	template <size_t JOINT_NUM>
	using BipedalStateEstimatorXf = StateEstimator<2, JOINT_NUM, float>;

	template <size_t JOINT_NUM>
	using BipedalStateEstimatorXd = StateEstimator<2, JOINT_NUM, double>;

	using BipedalStateEstimator3f = BipedalStateEstimatorXf<3>;
	using BipedalStateEstimator3d = BipedalStateEstimatorXd<3>;

	using BipedalStateEstimator4f = BipedalStateEstimatorXf<4>;
	using BipedalStateEstimator4d = BipedalStateEstimatorXd<4>;

};

#endif // INEKF_WARPER_HPP