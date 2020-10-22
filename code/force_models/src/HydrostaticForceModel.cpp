/*
 * DampingForceModel.cpp
 *
 *  Created on: Oct 17, 2014
 *      Author: cady
 */

#include <Eigen/Dense>
#include <ssc/kinematics.hpp>

#include "Body.hpp"
#include "calculate_gz.hpp"
#include "Observer.hpp"
#include "QuadraticDampingForceModel.hpp"
#include "YamlPosition.hpp"

#include "HydrostaticForceModel.hpp"

std::string HydrostaticForceModel::model_name() {return "hydrostatic";}

HydrostaticForceModel::HydrostaticForceModel(const std::string& body_name, const EnvironmentAndFrames&) :
		ForceModelAtG(HydrostaticForceModel::model_name(), body_name),
		centre_of_buoyancy(new Eigen::Vector3d())
{
}

bool HydrostaticForceModel::is_a_surface_force_model() const
{
    return true;
}

Vector6d HydrostaticForceModel::get_force(const BodyStates& states, const double t, const EnvironmentAndFrames& env, const std::map<std::string,double>& commands) const
{
    const auto mesh = std::string("mesh(") + body_name + ")";
    auto Tned2body = env.k->get(body_name, "NED");
    Tned2body.swap();
    auto TG2body = env.k->get(states.G.get_frame(), body_name);

    auto C = states.intersector->center_of_mass_immersed();

    if (C.all_facets_are_in_same_plane) C.volume = 0;

    for (size_t i = 0 ; i < 3 ; ++i) centre_of_buoyancy->operator()(i) = C.G(i);

    // Coordinates of the center of buoyancy in the BODY frame
    const ssc::kinematics::Point B(body_name, C.G);

    ssc::kinematics::Vector6d w;

    w << 0,
         0,
         -env.rho*env.g*C.volume,
         0,
         0,
         0;

    // The coordinates of the center of buoyancy (in the NED frame) are given by Tned2body.inverse()*B
    ssc::kinematics::Wrench ret(Tned2body.inverse()*B,w);
    const auto G = TG2body*states.G;
    ret = ret.change_frame_but_keep_ref_point(Tned2body);
    ssc::kinematics::Wrench ret2(B, ret.force, ret.torque);
    ret2 = ret2.change_point_of_application(G);
    Vector6d ret3;
    ret3 << ret2.force, ret2.torque;
    return ret3;
}

ssc::kinematics::Point HydrostaticForceModel::get_centre_of_buoyancy() const
{
    return ssc::kinematics::Point(get_body_name(), centre_of_buoyancy->operator()(0),
                                                   centre_of_buoyancy->operator()(1),
                                                   centre_of_buoyancy->operator()(2));
}

void HydrostaticForceModel::extra_observations(Observer& observer) const
{
    observer.write(centre_of_buoyancy->operator()(0),DataAddressing({"efforts",get_body_name(),get_name(),"extra observations","Bx"},std::string("Bx")));
    observer.write(centre_of_buoyancy->operator()(1),DataAddressing({"efforts",get_body_name(),get_name(),"extra observations","By"},std::string("By")));
    observer.write(centre_of_buoyancy->operator()(2),DataAddressing({"efforts",get_body_name(),get_name(),"extra observations","Bz"},std::string("Bz")));
    //const double gz = calculate_gz(*this, *env);
    //observer.write(gz,DataAddressing(std::vector<std::string>{"efforts",get_body_name(),get_name(),get_body_name(),"GZ"},std::string("GZ(")+get_name()+","+get_body_name()+")"));
}
