/*
 * LinearDampingForceModel.cpp
 *
 *  Created on: Oct 17, 2014
 *      Author: cady
 */

#include <Eigen/Dense>
#include "yaml.h"

#include <ssc/kinematics.hpp>
#include "Body.hpp"
#include "LinearDampingForceModel.hpp"
#include "external_data_structures_parsers.hpp"

std::string LinearDampingForceModel::model_name() {return "linear damping";}

LinearDampingForceModel::LinearDampingForceModel(const Eigen::Matrix<double,6,6>& D_, const std::string& body_name, const EnvironmentAndFrames& env) :
		DampingForceModel(LinearDampingForceModel::model_name(), body_name, env, D_)
{}

Eigen::Matrix<double, 6, 1> LinearDampingForceModel::get_force_and_torque(const Eigen::Matrix<double,6,6>& D, const Eigen::Matrix<double, 6, 1>& W) const
{
    return (Eigen::Matrix<double, 6, 1>)(-D * W);
}

LinearDampingForceModel::Input LinearDampingForceModel::parse(const std::string& yaml)
{
    Eigen::Matrix<double,6,6> ret;
    std::stringstream stream(yaml);
    YAML::Parser parser(stream);
    YAML::Node node;
    parser.GetNextDocument(node);
    YamlDynamics6x6Matrix M;
    try
    {
        parse_YamlDynamics6x6Matrix(node["damping matrix at the center of gravity projected in the body frame"], M, false);
    }
    catch(const InvalidInputException& e)
    {
        THROW(__PRETTY_FUNCTION__, InvalidInputException, "In node 'damping matrix at the center of gravity projected in the body frame': " << e.get_message());
    }
    for (size_t j = 0 ; j < 6 ; ++j) ret(0,(int)j) = M.row_1[j];
    for (size_t j = 0 ; j < 6 ; ++j) ret(1,(int)j) = M.row_2[j];
    for (size_t j = 0 ; j < 6 ; ++j) ret(2,(int)j) = M.row_3[j];
    for (size_t j = 0 ; j < 6 ; ++j) ret(3,(int)j) = M.row_4[j];
    for (size_t j = 0 ; j < 6 ; ++j) ret(4,(int)j) = M.row_5[j];
    for (size_t j = 0 ; j < 6 ; ++j) ret(5,(int)j) = M.row_6[j];
    return ret;
}
