/*
 * ManeuveringForceModel.hpp
 *
 *  Created on: Jan 19, 2015
 *      Author: cady
 */

#ifndef MANEUVERINGFORCEMODEL_HPP_
#define MANEUVERINGFORCEMODEL_HPP_

#include <map>

#include <ssc/data_source.hpp>
#include <ssc/macros.hpp>

#include "ForceModel.hpp"
#include "YamlCoordinates.hpp"
#include "ManeuveringInternal.hpp"


#include TR1INC(memory)

class ManeuveringForceModel : public ForceModel
{
    public:

        struct Yaml
        {
            Yaml();
            YamlCoordinates point_of_application;
            std::map<std::string, std::string> var2expr;
        };
        ManeuveringForceModel(const Yaml& data, const std::string& body_name, const EnvironmentAndFrames& env);
        static Yaml parse(const std::string& yaml);
        ssc::kinematics::Wrench operator()(const BodyStates& states, const double t) const;
        static const std::string model_name;

    private:
        ManeuveringForceModel();
        ssc::kinematics::Point point_of_application;
        std::map<std::string, maneuvering::NodePtr> m;
};

#endif /* MANEUVERINGFORCEMODEL_HPP_ */