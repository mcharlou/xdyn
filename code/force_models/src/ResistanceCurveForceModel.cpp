/*
 * ResistanceCurveForceModel.cpp
 *
 *  Created on: Oct 24, 2014
 *      Author: cady
 */

#include "Body.hpp"
#include "environment_parsers.hpp"
#include <ssc/interpolation.hpp>
#include <ssc/yaml_parser.hpp>
#include "yaml.h"

#include "ResistanceCurveForceModel.hpp"

std::string ResistanceCurveForceModel::model_name() {return "resistance curve";}

class ResistanceCurveForceModel::Impl
{
    public:
        Impl(const Yaml& data) : S(data.Va, data.R, /*allow queries outside bounds*/ true), vmin(data.Va.front()), vmax(data.Va.back())
        {
        }

        double get_resistance(const double Va)
        {
            if (Va<vmin) std::cerr << "Warning: resistance curve is tabulated from " << vmin << " m/s, but received Va = " << Va << " m/s" << std::endl;
            if (Va>vmax) std::cerr << "Warning: resistance curve is tabulated up to " << vmax << " m/s, but received Va = " << Va << " m/s" << std::endl;
            return S.f(Va);
        }

    private:
        Impl();
        ssc::interpolation::SplineVariableStep S;
        double vmin;
        double vmax;
};

ResistanceCurveForceModel::Yaml::Yaml() : Va(), R()
{
}

ResistanceCurveForceModel::ResistanceCurveForceModel(const Yaml& data, const std::string& body_name, const EnvironmentAndFrames&) :
		ForceModelAtG(ResistanceCurveForceModel::model_name(), body_name), pimpl(new Impl(data))
{
}

ResistanceCurveForceModel::Yaml ResistanceCurveForceModel::parse(const std::string& yaml)
{
    std::stringstream stream(yaml);
    YAML::Parser parser(stream);
    YAML::Node node;
    parser.GetNextDocument(node);
    Yaml ret;
    ssc::yaml_parser::parse_uv(node["speed"], ret.Va);
    ssc::yaml_parser::parse_uv(node["resistance"], ret.R);
    return ret;
}

Vector6d ResistanceCurveForceModel::get_force(const BodyStates& states, const double t, const EnvironmentAndFrames& env, const std::map<std::string,double>& commands) const
{
    ssc::kinematics::Vector6d tau = ssc::kinematics::Vector6d::Zero();
    tau(0) = -pimpl->get_resistance(states.u());
    return tau;
}
