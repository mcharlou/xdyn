/*
 * GMForceModel.hpp
 *
 *  Created on: April 10, 2015
 *      Author: cady
 */

#ifndef GMFORCEMODEL_HPP_
#define GMFORCEMODEL_HPP_

#include <ssc/kinematics.hpp>

#include "ImmersedSurfaceForceModel.hpp"

#include "EnvironmentAndFrames.hpp"

class Body;

class GMForceModel : public ImmersedSurfaceForceModel
{
    public:
        struct Yaml : public YamlModel
        {
            Yaml();
            std::string name_of_hydrostatic_force_model;
            double roll_step;
            ForceParser try_to_parse;
        };
        GMForceModel(const Yaml& data, const std::string& body_name, const EnvironmentAndFrames& env);
        std::function<DF(const FacetIterator &,
                         const size_t,
                         const EnvironmentAndFrames &,
                         const BodyStates &,
                         const double
                        )> get_dF(const FacetIterator& begin_facet,
                                  const FacetIterator& end_facet,
                                  const EnvironmentAndFrames& env,
                                  const BodyStates& states,
                                  const double t
                                 ) const;
        static Yaml parse(const std::string& yaml);
        Vector6d get_force(const BodyStates& states, const double t, const EnvironmentAndFrames& env, const std::map<std::string,double>& commands) const;
        void extra_observations(Observer& ) const;
        static std::string model_name();
        double get_GM() const;

    private:
        GMForceModel();
        double get_gz_for_shifted_states(const BodyStates& states, const double t, const EnvironmentAndFrames& env) const;
        BodyStates get_shifted_states(const BodyStates& states, const double t) const;
        double pe(const BodyStates& states, const std::vector<double>& x, const EnvironmentAndFrames& env) const;

        ForcePtr underlying_hs_force_model;
        double dphi;
        // TODO: storing data behind pointers just for the sake of violating constness is bad practice, this should be updated to reflect the changes in ForceModel
        std::shared_ptr<double> GM;
        std::shared_ptr<double> GZ;
        std::shared_ptr<Body> body_for_gm;
};

#endif /* GMFORCEMODEL_HPP_ */
