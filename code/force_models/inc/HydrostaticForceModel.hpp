/*
 * HydrostaticForceModel.hpp
 *
 *  Created on: Oct 17, 2014
 *      Author: cady
 */

#ifndef HYDROSTATICFORCEMODEL_HPP_
#define HYDROSTATICFORCEMODEL_HPP_

#include "ForceModelAtG.hpp"
#include <Eigen/Dense>
#include <ssc/kinematics.hpp>

#include "EnvironmentAndFrames.hpp"

class Body;

/** \brief Provides an interface to QuadraticDampingForceModel & LinearDampingForceModel
 *  \addtogroup model_wrappers
 *  \ingroup model_wrappers
 */
class HydrostaticForceModel : public ForceModelAtG
{
    public:
        HydrostaticForceModel(const std::string& body_name, const EnvironmentAndFrames& env);
        Vector6d get_force(const BodyStates& states, const double t, const EnvironmentAndFrames& env, const std::map<std::string,double>& commands) const override;
        static std::string model_name();
        bool is_a_surface_force_model() const override;
        void extra_observations(Observer& observer) const override;
        ssc::kinematics::Point get_centre_of_buoyancy() const;

    private:
        HydrostaticForceModel();
        std::shared_ptr<Eigen::Vector3d> centre_of_buoyancy;
};

#endif /* HYDROSTATICFORCEMODEL_HPP_ */
