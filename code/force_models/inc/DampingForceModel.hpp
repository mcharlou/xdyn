/*
 * DampingForceModel.hpp
 *
 *  Created on: Oct 17, 2014
 *      Author: cady
 */

#ifndef DAMPINGFORCEMODEL_HPP_
#define DAMPINGFORCEMODEL_HPP_

#include "ForceModelAtH.hpp"
#include <Eigen/Dense>
#include <ssc/kinematics.hpp>

class Body;

/** \brief Provides an interface to QuadraticDampingForceModel & LinearDampingForceModel
 *  \addtogroup model_wrappers
 *  \ingroup model_wrappers
 */
class DampingForceModel : public ForceModelAtH
{
    public:
        DampingForceModel(const std::string& name, const std::string& body_name, const EnvironmentAndFrames& env, const Eigen::Matrix<double,6,6>& D);
        Vector6d get_force(const BodyStates& states, const double t, const EnvironmentAndFrames& env, const std::map<std::string,double>& commands) const override;

    private:
        virtual Vector6d get_force_and_torque(const Eigen::Matrix<double,6,6>& D, const Vector6d& W) const = 0;
        DampingForceModel();
        Eigen::Matrix<double,6,6> D; //!< 6x6 matrix corresponding to the quadratic damping matrix expressed in the body frame
};

#endif /* DAMPINGFORCEMODEL_HPP_ */
