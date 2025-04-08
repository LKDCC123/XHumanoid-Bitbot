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
#ifndef __CONFIG_PARSER_HPP__
#define __CONFIG_PARSER_HPP__

#include <string>
#include <iostream>
#include <array>
#include <pugixml.hpp>
#include <pugiconfig.hpp>
#include <memory>
#include <type_traits>
#include <Eigen/Dense>
#include <optional>
#include "InEKF_Warper.hpp"
#include "ContactEstimator.hpp"
#include "libdef.h"

namespace zzs
{
	template <size_t LEG_NUM, size_t JOINT_NUM, typename SCALAR>
	class INEKF_API ConfigParser
	{
	public:
		static_assert(std::is_floating_point<SCALAR>::value, "SCALAR must be floating point type");
		using Ptr = std::shared_ptr<ConfigParser<LEG_NUM, JOINT_NUM, SCALAR>>;

		static Ptr Create(const std::string &config_file_path)
		{
			return std::make_shared<ConfigParser<LEG_NUM, JOINT_NUM, SCALAR>>(config_file_path);
		}

	public:
		ConfigParser(const std::string &config_file_path)
			: ReadyToCreateContactEstimator(false),
			  ReadyToCreateInEKFWarper(false),
			  HasCreateContactEstimator(false),
			  HasCreateInEKFWarper(false),
			  init_Joint_ang({0})
		{
			this->config_file_path_ = config_file_path;
			pugi::xml_parse_result result = doc_.load_file(config_file_path.c_str());
			if (!result)
			{
				throw std::runtime_error("Failed to load config file");
			}
			if (this->doc_.child("bitbot").type() == pugi::node_null)
			{
				throw std::runtime_error("This is not a bitbot config file.");
			}

			this->root_ = this->doc_.child("bitbot");
			this->joint_node_ = this->root_.child("joint");
			this->ekf_node_ = this->root_.child("ekf_noise");
			this->contact_node_ = this->root_.child("contact");

			this->dt_ = static_cast<SCALAR>(this->root_.attribute("dt").as_double());

			if (this->ParseContact())
				this->ReadyToCreateContactEstimator = true;
			if (this->ParseEKF() && this->ParseJoint())
				this->ReadyToCreateInEKFWarper = true;
		}

		auto CreateInEKFWarper()
		{
			if (HasCreateInEKFWarper || !ReadyToCreateInEKFWarper)
			{
				throw std::runtime_error("InEKF Warper has been created or not ready to create InEKF Warper");
			}

			auto inekf_warper = std::make_unique<zzs::StateEstimator<LEG_NUM, JOINT_NUM, SCALAR>>(
				this->dt_,
				this->rot_ang,
				this->trans_vec,
				this->joint_rot_dir,
				this->init_Joint_ang,
				this->joint_noise,
				this->robot_state_,
				this->imu_noise_params_);
			this->HasCreateInEKFWarper = true;
			// return std::move(inekf_warper);
			return inekf_warper;
		}

		auto CreateContactEstimator()
		{
			if (HasCreateContactEstimator || !ReadyToCreateContactEstimator)
			{
				throw std::runtime_error("Contact Estimator has been created or not ready to create Contact Estimator");
			}

			auto contact_estimator = std::make_unique<zzs::BipedalContactEstimator<SCALAR>>(
				this->contact_threshold_,
				this->debounce_time_,
				this->dt_);
			this->HasCreateContactEstimator = true;
			// return std::move(contact_estimator);
			return contact_estimator;
		}

		auto getContactThreshold() const
		{
			return this->contact_threshold_;
		}

		auto getDebounceTime() const
		{
			return this->debounce_time_;
		}

		auto getDt() const
		{
			return this->dt_;
		}

		auto getJointRotDir() const
		{
			return this->joint_rot_dir;
		}

		auto getJointNoise() const
		{
			return this->joint_noise;
		}

		auto getJointTranslation() const
		{
			return std::make_tuple(this->rot_ang, this->trans_vec);
		}

		auto getRobotStateParam() const
		{
			return std::make_tuple(this->robot_state_, this->imu_noise_params_);
		}

	private:
		auto ParseOrigin(const pugi::xml_node &node)
		{
			using ReturnType = std::tuple<std::array<SCALAR, 3>, std::array<SCALAR, 3>>;

			if (node.type() == pugi::node_null)
				return std::optional<ReturnType>(std::nullopt);
			try
			{
				std::string xyz = node.attribute("xyz").as_string();
				std::string rpy = node.attribute("rpy").as_string();
				std::array<SCALAR, 3> xyz_array;
				std::array<SCALAR, 3> rpy_array;
				std::string::size_type sz;
				xyz_array[0] = static_cast<SCALAR>(std::stod(xyz, &sz));
				xyz = xyz.substr(sz + 1);
				xyz_array[1] = static_cast<SCALAR>(std::stod(xyz, &sz));
				xyz = xyz.substr(sz + 1);
				xyz_array[2] = static_cast<SCALAR>(std::stod(xyz, &sz));

				rpy_array[0] = static_cast<SCALAR>(std::stod(rpy, &sz));
				rpy = rpy.substr(sz + 1);
				rpy_array[1] = static_cast<SCALAR>(std::stod(rpy, &sz));
				rpy = rpy.substr(sz + 1);
				rpy_array[2] = static_cast<SCALAR>(std::stod(rpy, &sz));

				return std::optional<ReturnType>(std::make_tuple(xyz_array, rpy_array));
			}
			catch (const std::exception &e)
			{
				std::cout << e.what() << std::endl;
				return std::optional<ReturnType>(std::nullopt);
			}
		}

		auto ParseJointRotDir(const pugi::xml_node &node)
		{
			using RetureType = typename StateEstimator<LEG_NUM, JOINT_NUM, SCALAR>::JointRotDir_e;
			if (node.type() == pugi::node_null)
				return std::optional<RetureType>(std::nullopt);

			std::string dir;
			try
			{
				dir = node.attribute("xyz").as_string();
			}
			catch (const std::exception &e)
			{
				std::cout << "parse joint rotation direction error," << e.what() << std::endl;
				return std::optional<RetureType>(std::nullopt);
			}

			if (dir == "1 0 0")
				return std::optional<RetureType>(RetureType::AXIS_X);
			else if (dir == "0 1 0")
				return std::optional<RetureType>(RetureType::AXIS_Y);
			else if (dir == "0 0 1")
				return std::optional<RetureType>(RetureType::AXIS_Z);
			else if (dir == "-1 0 0")
				return std::optional<RetureType>(RetureType::AXIS_REV_X);
			else if (dir == "0 -1 0")
				return std::optional<RetureType>(RetureType::AXIS_REV_Y);
			else if (dir == "0 0 -1")
				return std::optional<RetureType>(RetureType::AXIS_REV_Z);
			else
			{
				std::cerr << "Joint Rotation Direction \"" << dir << "\" is not correct or not support!" << std::endl;
				return std::optional<RetureType>(std::nullopt);
			}
		}

		bool ParseJoint()
		{
			if (this->joint_node_.type() == pugi::node_null)
				return false;

			std::map<int, pugi::xml_node> leg_node_map;
			try
			{
				pugi::xml_node leg_node = this->joint_node_.child("leg");
				while (leg_node)
				{
					int leg_id = leg_node.attribute("id").as_int();
					leg_node_map[leg_id] = leg_node;
					leg_node = leg_node.next_sibling("leg");
				}

				if (leg_node_map.size() != LEG_NUM)
				{
					std::cerr << "Leg number is not correct" << std::endl;
					return false;
				}

				size_t leg_idx = 0;
				for (auto &[leg_key, leg_value] : leg_node_map)
				{
					std::map<int, pugi::xml_node> joint_node_map;
					pugi::xml_node joint_node = leg_value.child("joint");
					while (joint_node)
					{
						int joint_id = joint_node.attribute("id").as_int();
						joint_node_map[joint_id] = joint_node;
						joint_node = joint_node.next_sibling("joint");
					}

					if (joint_node_map.size() != JOINT_NUM)
					{
						std::cerr << "Joint number is not correct" << std::endl;
						return false;
					}

					size_t joint_idx = 0;
					for (auto &[joint_key, joint_value] : joint_node_map)
					{
						double noise = joint_value.attribute("noise").as_double();
						this->joint_noise[leg_idx][joint_idx] = static_cast<SCALAR>(noise);

						auto rot_origin = ParseOrigin(joint_value.child("origin"));
						if (rot_origin.has_value())
						{
							std::tie(this->trans_vec[leg_idx][joint_idx], this->rot_ang[leg_idx][joint_idx]) = rot_origin.value();
						}
						else
						{
							std::cerr << "Parse Origin Error" << std::endl;
							return false;
						}

						auto joint_rot_dir = ParseJointRotDir(joint_value.child("axis"));
						if (joint_rot_dir.has_value())
						{
							this->joint_rot_dir[leg_idx][joint_idx] = joint_rot_dir.value();
						}
						else
						{
							std::cerr << "Parse Joint Rotation Direction Error" << std::endl;
							return false;
						}

						joint_idx++;
					}

					auto end_pose = ParseOrigin(leg_value.child("end").child("origin"));
					if (end_pose.has_value())
					{
						std::tie(this->trans_vec[leg_idx][JOINT_NUM], this->rot_ang[leg_idx][JOINT_NUM]) =
							end_pose.value();
					}
					else
					{
						std::cerr << "Parse End Pose Error" << std::endl;
						return false;
					}

					leg_idx++;
				}
			}
			catch (const std::exception &e)
			{
				std::cout << "Parse Joint Error: " << e.what() << std::endl;
				return false;
			}

			return true;
		}

		bool ParseEKF()
		{
			if (this->ekf_node_.type() == pugi::node_null)
				return false;

			try
			{
				double gyro_noise = this->ekf_node_.attribute("gyro_noise").as_double();
				double acc_noise = this->ekf_node_.attribute("acc_noise").as_double();
				double contact_noise = this->ekf_node_.attribute("contact_noise").as_double();
				double gyro_bias_noise = this->ekf_node_.attribute("gyro_bias").as_double();
				double acc_bias_noise = this->ekf_node_.attribute("acc_bias").as_double();

				this->imu_noise_params_.setGyroscopeNoise(gyro_noise);
				this->imu_noise_params_.setAccelerometerNoise(acc_noise);
				this->imu_noise_params_.setContactNoise(contact_noise);
				this->imu_noise_params_.setGyroscopeBiasNoise(gyro_bias_noise);
				this->imu_noise_params_.setAccelerometerBiasNoise(acc_bias_noise);
			}
			catch (const std::exception &e)
			{
				std::cerr << e.what() << std::endl;
				return false;
			}

			// Initialize state mean
			Eigen::Matrix3d R0;
			Eigen::Vector3d v0, p0, bg0, ba0;
			R0 << 1, 0, 0, // initial orientation
				0, 1, 0,   // IMU frame is rotated 90deg about the x-axis
				0, 0, 1;
			v0 << 0, 0, 0;	// initial velocity
			p0 << 0, 0, 0;	// initial position
			bg0 << 0, 0, 0; // initial gyroscope bias
			ba0 << 0, 0, 0; // initial accelerometer bias

			this->robot_state_.setRotation(R0);
			this->robot_state_.setVelocity(v0);
			this->robot_state_.setPosition(p0);
			this->robot_state_.setGyroscopeBias(bg0);
			this->robot_state_.setAccelerometerBias(ba0);

			return true;
		}

		bool ParseContact()
		{
			if (this->contact_node_.type() == pugi::node_null)
				return false;
			try
			{
				std::string name = this->contact_node_.attribute("name").as_string();
				std::cout << "Contact Estimator Name: " << name << std::endl;
				this->contact_threshold_ = static_cast<SCALAR>(this->contact_node_.attribute("threshold").as_double());
				this->debounce_time_ = static_cast<SCALAR>(this->contact_node_.attribute("debounce").as_double());
			}
			catch (const std::exception &e)
			{
				std::cout << e.what() << std::endl;
				return false;
			}
			return true;
		}

	private:
		bool HasCreateContactEstimator;
		bool HasCreateInEKFWarper;

		bool ReadyToCreateContactEstimator;
		bool ReadyToCreateInEKFWarper;

		std::string config_file_path_;
		pugi::xml_document doc_;
		pugi::xml_node root_;
		pugi::xml_node joint_node_;
		pugi::xml_node ekf_node_;
		pugi::xml_node contact_node_;

		SCALAR dt_;
		SCALAR g_;
		SCALAR debounce_time_;
		SCALAR contact_threshold_;

		inekf::NoiseParams imu_noise_params_;
		inekf::RobotState robot_state_;

		std::array<std::array<std::array<SCALAR, 3>, JOINT_NUM + 1>, LEG_NUM> rot_ang;
		std::array<std::array<std::array<SCALAR, 3>, JOINT_NUM + 1>, LEG_NUM> trans_vec;
		std::array<std::array<SCALAR, JOINT_NUM>, LEG_NUM> init_Joint_ang;
		std::array<std::array<SCALAR, JOINT_NUM>, LEG_NUM> joint_noise;
		std::array<std::array<typename StateEstimator<LEG_NUM, JOINT_NUM, SCALAR>::JointRotDir_e, JOINT_NUM>, LEG_NUM> joint_rot_dir;
	};
};

#endif // !__CONFIG_PARSER_HPP__
