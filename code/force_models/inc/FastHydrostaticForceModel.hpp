/*
 * FastHydrostaticForceModel.hpp
 *
 *  Created on: Jun 16, 2014
 *      Author: cady
 */

#ifndef FASTHYDROSTATICFORCEMODEL_HPP_
#define FASTHYDROSTATICFORCEMODEL_HPP_

#include "ImmersedSurfaceForceModel.hpp"

class FastHydrostaticForceModel : public ImmersedSurfaceForceModel
{
    public:
        FastHydrostaticForceModel(const std::string& body_name, const EnvironmentAndFrames& env);
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
        static std::string model_name();


    protected:
        FastHydrostaticForceModel(const std::string& force_name, const std::string& body_name, const EnvironmentAndFrames& env);
        //void update_gz(const EnvironmentAndFrames& env);

    private:
        FastHydrostaticForceModel();
        //void extra_observations(Observer& observer) const;
        virtual EPoint get_application_point(const FacetIterator& that_facet, const BodyStates& states, const double zG) const;
        double pe(const BodyStates& states, const std::vector<double>& x, const EnvironmentAndFrames& env) const;

        //double gz;
};

#endif /* FASTHYDROSTATICFORCEMODEL_HPP_ */
