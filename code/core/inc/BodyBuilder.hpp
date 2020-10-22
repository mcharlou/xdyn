/*
 * BodyBuilder.hpp
 *
 *  Created on: Jun 17, 2014
 *      Author: cady
 */

#ifndef BODYBUILDER_HPP_
#define BODYBUILDER_HPP_

#include <vector>

#include "Body.hpp"
#include "YamlRotation.hpp"
#include "GeometricTypes3d.hpp"
#include "ForceModel.hpp"

struct YamlDynamics6x6Matrix;
struct YamlAngle;

/** \author cec
 *  \date Jun 17, 2014, 12:39:59 PM
 *  \brief Builds a Body object from the YAML & STL describing it
 *  \ingroup simulator
 *  \section ex1 Example
 *  \snippet simulator/unit_tests/src/BodyBuilderTest.cpp BodyBuilderTest example
 *  \section ex2 Expected output
 *  \snippet simulator/unit_tests/src/BodyBuilderTest.cpp BodyBuilderTest expected output
 */
class BodyBuilder
{
    public:
        /** \details It makes no sense to build a 'Body' object without knowing
         *           the rotation conventions, which is why this is the only
         *           constructor available.
         */
        BodyBuilder(const YamlRotation& convention);

        /** \brief Build a 'Body' object from YAML & STL data
         *  \returns New Body object
         */
        BodyPtr build(const YamlBody& input, const VectorOfVectorOfPoints& mesh, const size_t idx, const double t0, const EnvironmentAndFrames& env) const;

        /** \details Only used for testing purposes when we don't want to go
         *           through the hassle of defining the inertia matrix & initial
         *           positions & when we want to build the forces separately
         *  \returns New Body object, without any forces
         */
        BodyPtr build(const std::string& name, const VectorOfVectorOfPoints& mesh, const size_t idx, const double t0, const bool has_surface_forces = false) const;

        /** \details Only used for testing purposes when when we want to build
         * 			 the forces separately
		 *  \returns New Body object, without any forces
		 */
        BodyPtr build(const YamlBody& input, const VectorOfVectorOfPoints& mesh, const size_t idx, const double t0, const bool has_surface_forces = false) const;

        /**  \brief Add the capacity to parse certain YAML inputs for forces
          *  \details This method must not be called with any parameters: the
          *  default parameter is only there so we can use boost::enable_if. This
          *  allows us to use can_parse for several types derived from a few
          *  base classes (WaveModelInterface, ForceModel...) & the compiler will
          *  automagically choose the right version of can_parse.
          *  \returns *this (so we can chain calls to can_parse)
          *  \snippet simulator/unit_tests/src/SimulatorBuilderTest.cpp SimulatorBuilderTest can_parse_example
          */
        template <typename T> BodyBuilder& can_parse(typename boost::enable_if<boost::is_base_of<ForceModel,T> >::type* dummy = 0)
        {
            (void)dummy; // Ignore "unused variable" warning: we just need "dummy" for boost::enable_if
            force_parsers.push_back(ForceModel::build_parser<T>());
            return *this;
        }

    private:
        BodyBuilder(); // Disabled

        void add_inertia(BodyStates& states, const YamlDynamics6x6Matrix& rigid_body_inertia, const YamlDynamics6x6Matrix& added_mass) const;

        /**  \details Converts the external YAML data structure (several std::vectors)
         *            to an Eigen::Matrix used for calculations
         */
        Eigen::Matrix<double,6,6> convert(const YamlDynamics6x6Matrix& M) const;

        /** \brief Puts the mesh in the body frame
         *  \details Uses the body frame's initial position relative to the mesh
         */
        void change_mesh_ref_frame(BodyStates& states, const VectorOfVectorOfPoints& mesh) const;

        BodyStates get_initial_states(const YamlBody& input, const VectorOfVectorOfPoints& mesh, const double t0) const;
        std::vector<ForcePtr> get_forces(const YamlBody& input, const std::string body_name, const EnvironmentAndFrames& env, const std::vector<ForceParser>& force_parsers) const;
        ForcePtr parse_force(const YamlModel& model, const std::string body_name, const EnvironmentAndFrames& env, const std::vector<ForceParser>& force_parsers) const;
        double get_max_history_length(const std::vector<ForcePtr>& forces) const;

        YamlRotation rotations; //!< Rotation convention (describes how we can build a rotation matrix from three angles)
        std::vector<ForceParser> force_parsers;
};

bool isSymmetric(const Eigen::MatrixXd& m);
bool isSymmetricDefinitePositive(const Eigen::MatrixXd& m);

#endif /* BODYBUILDER_HPP_ */
