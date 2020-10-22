/*
 * ResistanceCurveForceModel.hpp
 *
 *  Created on: Oct 24, 2014
 *      Author: cady
 */

#ifndef RESISTANCECURVEFORCEMODEL_HPP_
#define RESISTANCECURVEFORCEMODEL_HPP_

#include <ssc/macros.hpp>

#include "ForceModelAtG.hpp"

struct EnvironmentAndFrames;

/** \brief Resistance curve given by interpolation table
 *  \details
 *  \addtogroup model_wrappers
 *  \ingroup model_wrappers
 *  \section ex1 Example
 *  \snippet model_wrappers/unit_tests/src/ResistanceCurveForceModelTest.cpp ResistanceCurveForceModelTest example
 *  \section ex2 Expected output
 *  \snippet model_wrappers/unit_tests/src/ResistanceCurveForceModelTest.cpp ResistanceCurveForceModelTest expected output
 */
class ResistanceCurveForceModel : public ForceModelAtG
{
    public:
        struct Yaml
        {
            Yaml();
            std::vector<double> Va;
            std::vector<double> R;
        };
        ResistanceCurveForceModel(const Yaml& data, const std::string& body_name, const EnvironmentAndFrames& env);
        static Yaml parse(const std::string& yaml);
        Vector6d get_force(const BodyStates& states, const double t, const EnvironmentAndFrames& env, const std::map<std::string,double>& commands) const override;
        static std::string model_name();

    private:
        ResistanceCurveForceModel();
        class Impl;
        std::shared_ptr<Impl> pimpl;

};

#endif /* RESISTANCECURVEFORCEMODEL_HPP_ */
