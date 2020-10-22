/*
 * RadiationDampingForceModel.hpp
 *
 *  Created on: Dec 1, 2014
 *      Author: cec
 */

#ifndef RadiationDampingForceModel_HPP_
#define RadiationDampingForceModel_HPP_

#include <ssc/macros.hpp>
#include <memory>

#include "ForceModel.hpp"
#include "YamlRadiationDamping.hpp"
#include "EnvironmentAndFrames.hpp"

class HDBParser;

class RadiationDampingForceModel : public ForceModel
{
    public:
        struct Input
        {
            Input() : hdb(), yaml(){}
            std::shared_ptr<HDBParser> hdb;
            YamlRadiationDamping yaml;
        };
        RadiationDampingForceModel(const Input& input, const std::string& body_name, const EnvironmentAndFrames& env);
        Vector6d get_force(const BodyStates& states, const double t, const EnvironmentAndFrames& env, const std::map<std::string,double>& commands) const override;
        static Input parse(const std::string& yaml, const bool parse_hdb=true);
        static std::string model_name();
        double get_Tmax() const override;

    private:
        RadiationDampingForceModel();
        class Impl;
        std::shared_ptr<Impl> pimpl;

};

#endif /* RadiationDampingForceModel_HPP_ */
