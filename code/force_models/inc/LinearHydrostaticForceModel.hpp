/*
 * LinearHydrostaticForceModel.hpp
 *
 *  Created on: Aug 12, 2015
 *      Author: cady
 */

#ifndef LINEARHYDROSTATICFORCEMODEL_HPP_
#define LINEARHYDROSTATICFORCEMODEL_HPP_

#include <ssc/kinematics.hpp>

#include "EnvironmentAndFrames.hpp"
#include "ForceModelAtG.hpp"

class LinearHydrostaticForceModel : public ForceModelAtG
{
    public:
        struct Input
        {
            Input();
            double z_eq;
            double theta_eq;
            double phi_eq;
            std::vector<double> K1;
            std::vector<double> K2;
            std::vector<double> K3;
            double x1;
            double y1;
            double x2;
            double y2;
            double x3;
            double y3;
            double x4;
            double y4;
        };
        LinearHydrostaticForceModel(const Input& input, const std::string& body_name, const EnvironmentAndFrames& env);
        Vector6d get_force(const BodyStates& states, const double t, const EnvironmentAndFrames& env, const std::map<std::string,double>& commands) const override;
        static Input parse(const std::string& yaml);
        static std::string model_name();

    private:
        LinearHydrostaticForceModel();
        std::vector<double> get_zH(const double t, const EnvironmentAndFrames& env) const;
        double compute_zbar(const std::vector<double>& z) const;
        double compute_thetabar(const std::vector<double>& z) const;
        double compute_phibar(const std::vector<double>& z) const;
        Eigen::Matrix<double,3,3> K;
        ssc::kinematics::Point P1;
        ssc::kinematics::Point P2;
        ssc::kinematics::Point P3;
        ssc::kinematics::Point P4;
        double z_eq;
        double theta_eq;
        double phi_eq;
        double d12;
        double d34;
        double d13;
        double d24;
};

#endif /* GRAVITYFORCEMODEL_HPP_ */
