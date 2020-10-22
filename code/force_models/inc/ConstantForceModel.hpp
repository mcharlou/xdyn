/*
 * ConstantForce.hpp
 *
 *  Created on: Sep 7, 2018
 *      Author: cady
 */

#ifndef FORCE_MODELS_INC_CONSTANTFORCEMODEL_HPP_
#define FORCE_MODELS_INC_CONSTANTFORCEMODEL_HPP_


#include "EnvironmentAndFrames.hpp"
#include "ForceModel.hpp"
#include <ssc/kinematics.hpp>

class ConstantForceModel : public ForceModel
{
    public:
        struct Input
        {
            Input();
            std::string frame;
            double x;
            double y;
            double z;
            double X;
            double Y;
            double Z;
            double K;
            double M;
            double N;
        };

        ConstantForceModel(const Input& input, const std::string& body_name, const EnvironmentAndFrames& env);
        Vector6d get_force(const BodyStates& states, const double t, const EnvironmentAndFrames& env, const std::map<std::string,double>& commands) const override;
        static Input parse(const std::string& yaml);
        static std::string model_name();

    private:
        ConstantForceModel();
        Eigen::Vector3d force;
        Eigen::Vector3d torque;
};



#endif /* FORCE_MODELS_INC_CONSTANTFORCEMODEL_HPP_ */
