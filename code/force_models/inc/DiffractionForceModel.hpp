/*
 * DiffractionForceModel.hpp
 *
 */

#ifndef DIFFRACTIONFORCEMODEL_HPP_
#define DIFFRACTIONFORCEMODEL_HPP_

#include <ssc/macros.hpp>

#include "EnvironmentAndFrames.hpp"
#include "ForceModel.hpp"

#include "YamlDiffraction.hpp"

/** \brief Diffraction forces
 *  \details
 *  \addtogroup model_wrappers
 *  \ingroup model_wrappers
 *  \section ex1 Example
 *  \snippet model_wrappers/unit_tests/src/DiffractionForceModelTest.cpp DiffractionForceModelTest example
 *  \section ex2 Expected output
 *  \snippet model_wrappers/unit_tests/src/DiffractionForceModelTest.cpp DiffractionForceModelTest expected output
 */
class DiffractionForceModel : public ForceModel
{
    public:
        typedef YamlDiffraction Input;
        DiffractionForceModel(const Input& data, const std::string& body_name, const EnvironmentAndFrames& env);
        DiffractionForceModel(const Input& data, const std::string& body_name, const EnvironmentAndFrames& env, const std::string& hdb_file_contents);
        Vector6d get_force(const BodyStates& states, const double t, const EnvironmentAndFrames& env, const std::map<std::string,double>& commands) const override;
        static Input parse(const std::string& yaml);
        static std::string model_name();

    private:
        DiffractionForceModel();
        class Impl;
        std::shared_ptr<Impl> pimpl;

};

#endif /* DIFFRACTIONFORCEMODEL_HPP_ */
