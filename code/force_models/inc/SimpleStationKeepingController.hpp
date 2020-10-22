/*
 * SimpleStationKeepingController.hpp
 *
 *  Created on: Jan 29, 2015
 *      Author: cady
 */

#ifndef SIMPLESTATIONKEEPINGCONTROLLER_HPP_
#define SIMPLESTATIONKEEPINGCONTROLLER_HPP_

#include "ForceModel.hpp"

class SimpleStationKeepingController : public ForceModel
{
    public:
        struct Yaml
        {
            Yaml();
            std::string name;
            double ksi_x;
            double T_x;
            double ksi_y;
            double T_y;
            double ksi_psi;
            double T_psi;
        };
        static Yaml parse(const std::string& yaml);
        static std::string model_name();

        SimpleStationKeepingController(const Yaml& input, const std::string& body_name, const EnvironmentAndFrames& env);
        Vector6d get_force(const BodyStates& states, const double t, const EnvironmentAndFrames& env, const std::map<std::string,double>& commands) const override;

    private:
        SimpleStationKeepingController();
        double ksi_x;
        double omega_x;
        double ksi_y;
        double omega_y;
        double ksi_psi;
        double omega_psi;
};

#endif /* SIMPLESTATIONKEEPINGCONTROLLER_HPP_ */
