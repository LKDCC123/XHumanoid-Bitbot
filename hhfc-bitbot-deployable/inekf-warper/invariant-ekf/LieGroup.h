/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   LieGroup.h
 *  @author Ross Hartley
 *  @brief  Header file for various Lie Group functions 
 *  @date   September 25, 2018
 **/

#ifndef LIEGROUP_H
#define LIEGROUP_H 
#include <Eigen/Dense>
#include <iostream>
#include "libdef.h"

namespace inekf {

extern const double TOLERANCE;

INEKF_API Eigen::Matrix3d skew(const Eigen::Vector3d& v);
INEKF_API Eigen::Matrix3d Exp_SO3(const Eigen::Vector3d& w);
INEKF_API Eigen::MatrixXd Exp_SEK3(const Eigen::VectorXd& v);
INEKF_API Eigen::MatrixXd Adjoint_SEK3(const Eigen::MatrixXd& X);

} // end inekf namespace
#endif 
