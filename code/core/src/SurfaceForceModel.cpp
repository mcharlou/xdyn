/*
 * SurfaceForceModel.cpp
 *
 *  Created on: Oct 2, 2014
 *      Author: cady
 */

#include "BodyStates.hpp"
#include "YamlPosition.hpp"
#include "EnvironmentAndFrames.hpp"

#include "SurfaceForceModel.hpp"

SurfaceForceModel::SurfaceForceModel(const std::string& name, const std::string body_name, const EnvironmentAndFrames& env) :
		ForceModelAtG(name, body_name),
        g_in_NED(ssc::kinematics::Point("NED", 0, 0, env.g)),
        zg_calculator(new ZGCalculator())
{}

/*SurfaceForceModel::SurfaceForceModel(const std::string& name, const std::string body_name, const EnvironmentAndFrames& env, const YamlPosition& internal_frame) :
		ForceModel(name, body_name, env, internal_frame),
        g_in_NED(ssc::kinematics::Point("NED", 0, 0, env->g)),
        zg_calculator(new ZGCalculator())
{}*/

SurfaceForceModel::~SurfaceForceModel()
{}

Vector6d SurfaceForceModel::get_force(const BodyStates& states, const double t, const EnvironmentAndFrames& env, const std::map<std::string,double>&) const
{
    zg_calculator->update_transform(env.k->get("NED", states.name));
    Vector6d F = Vector6d::Zero();
    const double orientation_factor = states.intersector->mesh->orientation_factor;

    const auto b = begin(states.intersector);
    const auto e = end(states.intersector);
    std::function<SurfaceForceModel::DF(const FacetIterator &,
                                        const size_t,
                                        const EnvironmentAndFrames &,
                                        const BodyStates &,
                                        const double)> dF_lambda =
        get_dF(b, e, env, states, t);
    
    size_t facet_index = 0;
    for (auto that_facet = begin(states.intersector) ; that_facet != e ; ++that_facet)
    {
        const DF f = dF_lambda(that_facet, facet_index, env, states, t);
        const double x = (f.C(0)-states.G.v(0));
        const double y = (f.C(1)-states.G.v(1));
        const double z = (f.C(2)-states.G.v(2));
        F(0) += orientation_factor*f.dF(0);
        F(1) += orientation_factor*f.dF(1);
        F(2) += orientation_factor*f.dF(2);
        F(3) += orientation_factor*(y*f.dF(2)-z*f.dF(1));
        F(4) += orientation_factor*(z*f.dF(0)-x*f.dF(2));
        F(5) += orientation_factor*(x*f.dF(1)-y*f.dF(0));
        ++facet_index;
    }
    return F;
}

double SurfaceForceModel::potential_energy(const BodyStates& states, const EnvironmentAndFrames& env, const std::vector<double>& x) const
{
    return pe(states, x, env);
}

bool SurfaceForceModel::is_a_surface_force_model() const
{
    return true;
}
