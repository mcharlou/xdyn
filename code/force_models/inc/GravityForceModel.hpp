/*
 * GravityForceModel.hpp
 *
 *  Created on: Jun 17, 2014
 *      Author: cady
 */

#ifndef GRAVITYFORCEMODEL_HPP_
#define GRAVITYFORCEMODEL_HPP_

#include "ForceModelAtG.hpp"
#include <ssc/kinematics.hpp>

class Body;
struct EnvironmentAndFrames;

class GravityForceModel : public ForceModelAtG
{
    public:
        GravityForceModel(const std::string& body_name, const EnvironmentAndFrames& env);
        Vector6d get_force(const BodyStates& states, const double t, const EnvironmentAndFrames& env, const std::map<std::string,double>& commands) const override;
        double potential_energy(const BodyStates& states, const EnvironmentAndFrames& env, const std::vector<double>& x) const;
        static std::string model_name();

    private:
        GravityForceModel();
        GravityForceModel(const double g);
};

#endif /* GRAVITYFORCEMODEL_HPP_ */
