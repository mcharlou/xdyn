/*
 * ForceModel.cpp
 *
 *  Created on: Oct 27, 2014
 *      Author: cady
 */

#include <memory>

#include "ForceModel.hpp"
#include "Observer.hpp"
#include "BodyStates.hpp"
#include "NumericalErrorException.hpp"
#include "InternalErrorException.hpp"
#include "InvalidInputException.hpp"
#include <ssc/kinematics.hpp>
#include <ssc/numeric/almost_equal.hpp>
#include <ssc/data_source.hpp>
#include "yaml2eigen.hpp"
#include "EnvironmentAndFrames.hpp"

bool samePoints(const ssc::kinematics::Point& P, const ssc::kinematics::Point& Q);
bool samePoints(const ssc::kinematics::Point& P, const ssc::kinematics::Point& Q)
{
    if (P.get_frame() != Q.get_frame()) return false;
    if (not(almost_equal(P.x(), Q.x())))    return false;
    if (not(almost_equal(P.y(), Q.y())))    return false;
    if (not(almost_equal(P.z(), Q.z())))    return false;
                                        return true;
}

ForceModel::ForceModel(const std::string& force_name_, const std::string body_name_) :
    force_name(force_name_),
    body_name(body_name_),
    force_in_body_frame(),
    force_in_ned_frame(),
	frame(body_name_),
	commands()/*,
	from_internal_frame_to_a_known_frame()*/
{}

ForceModel::ForceModel(const std::string& force_name_, const std::string body_name_, const EnvironmentAndFrames& env_, const YamlPosition& internal_frame) :
    force_name(force_name_),
    body_name(body_name_),
    force_in_body_frame(),
    force_in_ned_frame(),
	frame(force_name_),
	commands()/*,
	from_internal_frame_to_a_known_frame(make_transform(internal_frame, force_name, env.rot))*/
{env_.k->add(make_transform(internal_frame, force_name, env_.rot));}

ForceModel::ForceModel(const std::string& force_name_, const std::string body_name_, const std::vector<std::string>& commands_) :
    force_name(force_name_),
    body_name(body_name_),
    force_in_body_frame(),
    force_in_ned_frame(),
	frame(body_name_),
	commands(commands_)/*,
	from_internal_frame_to_a_known_frame()*/
{}

ForceModel::ForceModel(const std::string& force_name_, const std::string body_name_, const EnvironmentAndFrames& env_, const YamlPosition& internal_frame, const std::vector<std::string>& commands_) :
    force_name(force_name_),
    body_name(body_name_),
    force_in_body_frame(),
    force_in_ned_frame(),
	frame(force_name_),
	commands(commands_)/*,
	from_internal_frame_to_a_known_frame(make_transform(internal_frame, force_name, env.rot))*/
{env_.k->add(make_transform(internal_frame, force_name, env_.rot));}

bool ForceModel::is_a_surface_force_model() const
{
    return false;
}

std::string ForceModel::get_name() const
{
    return force_name;
}

std::string ForceModel::get_body_name() const
{
    return body_name;
}

std::string ForceModel::expression_frame() const
{
	return frame;
}

ssc::kinematics::Point ForceModel::application_point() const
{
	return ssc::kinematics::Point(expression_frame(),0,0,0);
}

ssc::kinematics::Wrench ForceModel::compute_wrench_from_force(const BodyStates& states, const EnvironmentAndFrames& env, Vector6d& F)
{
//	std::cout << "Force:" << F.transpose() << std::endl;
	const Eigen::Vector3d force(F(0),F(1),F(2));
	const Eigen::Vector3d torque(F(3),F(4),F(5));
	ssc::kinematics::Point P = application_point();
//	std::cout << "Application point:" << P << std::endl;
//	std::cout << "Expression frame:" << expression_frame() << std::endl;
	if(P.get_frame()!=expression_frame())
	{
		P = env.k->get(P.get_frame(),expression_frame())*P;
	}
//	std::cout << "Application point (in expression frame):" << P << std::endl;
	ssc::kinematics::Wrench tau(P, F); // At this point tau is expressed in the expression frame at the application point
//	std::cout << "Wrench (in application frame and at application point):" << tau << std::endl;
	tau = tau.change_point_of_application(ssc::kinematics::Point(expression_frame(),0,0,0)); // Now tau is expressed in the expression frame at the origin of the expression frame
//	std::cout << "Wrench (in application frame and at the origin of that frame):" << tau << std::endl;
	ssc::kinematics::Transform T = env.k->get(body_name, expression_frame());
//	std::cout << "Transform:" << T << std::endl;
	ssc::kinematics::RotationMatrix R = T.get_rot();
	tau = ssc::kinematics::Wrench(T.get_point(),R*tau.force,R*tau.torque); // Here tau is expressed in the body frame at the origin of the expression frame, a change of point remains necessary
//	std::cout << "Wrench (in body frame and at the origin of the expression frame):" << tau << std::endl;
	tau = tau.change_point_of_application(states.G); // Now tau is expressed in the body frame at G
//	std::cout << "Wrench (in body frame and at G):" << tau << std::endl;

	return tau;

	// Origin of the internal frame is P
	// G is the point (not the origin) of the body frame where the forces are summed
	// Ob is the origin of the body frame

	/*const auto rot_from_internal_frame_to_body_frame = T.get_rot().transpose();
	const auto OP = T.get_point().v;
	const auto OG = states.G.v;
	const auto GP = -OG + OP;
	const auto force_in_G_expressed_in_body_frame = rot_from_internal_frame_to_body_frame*force;
	const auto torque_in_G_expressed_in_body_frame = rot_from_internal_frame_to_body_frame*(torque+GP.cross(force));

	const ssc::kinematics::Wrench tau_in_body_frame_at_G(states.G, force_in_G_expressed_in_body_frame, torque_in_G_expressed_in_body_frame);*/

	/*ssc::kinematics::Wrench tau = this->get_force(states, t, get_commands(command_listener,t));
	if(tau.get_frame() != body_name)
	{
		const ssc::kinematics::Transform T = kinematics->get(tau.get_frame(), body_name);
    	force_in_body_frame = tau.change_frame_but_keep_ref_point(T).change_point_of_application(states.G);

    }
    else if(not(samePoints(tau.get_point(),states.G)))
    {
    	force_in_body_frame = tau.change_point_of_application(states.G);
    }
    else
    {
    	force_in_body_frame = tau;
    }*/
}

#define CHECK(force_name,component,value,body_name,t) if (std::isnan(value)) {THROW(__PRETTY_FUNCTION__,NumericalErrorException,"NaN detected in component '" << component << "' of force '" << force_name << "' acting on body '" << body_name << "' at t = " << t);}
ssc::kinematics::Wrench ForceModel::operator()(const BodyStates& states, const double t, const EnvironmentAndFrames& env, ssc::data_source::DataSource& command_listener)
{
	update(states, t, env, command_listener);
	return force_in_body_frame;
}

/*ssc::kinematics::Wrench ForceModel::operator()(const BodyStates& states, const double t, const EnvironmentAndFrames& env)
{
	if(not(commands.empty()))
	{
		THROW(__PRETTY_FUNCTION__, InvalidInputException,"This force model requires commands, thus a command listener must be provided");
	}
	std::map<std::string,double> commands_({});
	auto force = get_force(states, t, env, commands_);
	auto wrench = compute_wrench_from_force(states, env, force);
	CHECK(force_name,"Fx",force_in_body_frame.X(),body_name,t);
	CHECK(force_name,"Fy",force_in_body_frame.Y(),body_name,t);
	CHECK(force_name,"Fz",force_in_body_frame.Z(),body_name,t);
	CHECK(force_name,"Mx",force_in_body_frame.K(),body_name,t);
	CHECK(force_name,"My",force_in_body_frame.M(),body_name,t);
	CHECK(force_name,"Mz",force_in_body_frame.N(),body_name,t);
	return wrench;
}*/

std::map<std::string,double> ForceModel::get_commands(ssc::data_source::DataSource& command_listener, const double t) const
{
    std::map<std::string,double> ret;
    if(not(commands.empty()))
    {
    	command_listener.check_in(__PRETTY_FUNCTION__);
    	command_listener.set("t", t);
    	command_listener.check_out();
    	for (auto that_command = commands.begin() ; that_command != commands.end() ; ++that_command)
    	{
    		ret[*that_command] = get_command(*that_command, command_listener);
    	}
    	//auto m = command_listener.get_all<double>();
    	//ret.insert(m.begin(),m.end());
    }
    return ret;
}

double ForceModel::get_command(const std::string& command_name, ssc::data_source::DataSource& command_listener) const
{
    double ret = 0;
    try
    {
        command_listener.check_in(__PRETTY_FUNCTION__);
        //std::cout << "Setting signal t: " << t << " in DataSource" << std::endl;
        //command_listener.set("t", t);
        ret = command_listener.get<double>(force_name + "(" + command_name + ")");
        //std::cout << "Retrieved " << name + "(" + command_name + ")" << "=" << ret << " from DataSource" << std::endl;
        command_listener.check_out();
    }
    catch (const ssc::data_source::DataSourceException& e)
    {
        THROW(__PRETTY_FUNCTION__, InvalidInputException,
                "Unable to retrieve command '" << command_name << "' for '" << force_name << "': " << e.get_message()
                << " Check that the YAML file containing the commands was supplied to the simulator & that the command exists in that file."
                );
    }
    return ret;
}

ssc::kinematics::Wrench ForceModel::project_into_NED_frame(const ssc::kinematics::Wrench& F, const ssc::kinematics::RotationMatrix& R)
{
	// TODO: Set the gravity center of the ship in NED frame. Better : Move this responsibility to the Body class
    return ssc::kinematics::Wrench(ssc::kinematics::Point("NED"),R*F.force,R*F.torque);
}

ssc::kinematics::Wrench ForceModel::get_force_in_body_frame() const
{
    return force_in_body_frame;
}

ssc::kinematics::Wrench ForceModel::get_force_in_ned_frame() const
{
    return force_in_ned_frame;
}

void ForceModel::feed(Observer& observer) const
{
    observer.write(force_in_body_frame.X(),DataAddressing({"efforts",body_name,force_name,body_name,"Fx"},std::string("Fx(")+force_name+","+body_name+","+body_name+")"));
    observer.write(force_in_body_frame.Y(),DataAddressing({"efforts",body_name,force_name,body_name,"Fy"},std::string("Fy(")+force_name+","+body_name+","+body_name+")"));
    observer.write(force_in_body_frame.Z(),DataAddressing({"efforts",body_name,force_name,body_name,"Fz"},std::string("Fz(")+force_name+","+body_name+","+body_name+")"));
    observer.write(force_in_body_frame.K(),DataAddressing({"efforts",body_name,force_name,body_name,"Mx"},std::string("Mx(")+force_name+","+body_name+","+body_name+")"));
    observer.write(force_in_body_frame.M(),DataAddressing({"efforts",body_name,force_name,body_name,"My"},std::string("My(")+force_name+","+body_name+","+body_name+")"));
    observer.write(force_in_body_frame.N(),DataAddressing({"efforts",body_name,force_name,body_name,"Mz"},std::string("Mz(")+force_name+","+body_name+","+body_name+")"));

    observer.write(force_in_ned_frame.X(),DataAddressing({"efforts",body_name,force_name,"NED","Fx"},std::string("Fx(")+force_name+","+body_name+",NED)"));
    observer.write(force_in_ned_frame.Y(),DataAddressing({"efforts",body_name,force_name,"NED","Fy"},std::string("Fy(")+force_name+","+body_name+",NED)"));
    observer.write(force_in_ned_frame.Z(),DataAddressing({"efforts",body_name,force_name,"NED","Fz"},std::string("Fz(")+force_name+","+body_name+",NED)"));
    observer.write(force_in_ned_frame.K(),DataAddressing({"efforts",body_name,force_name,"NED","Mx"},std::string("Mx(")+force_name+","+body_name+",NED)"));
    observer.write(force_in_ned_frame.M(),DataAddressing({"efforts",body_name,force_name,"NED","My"},std::string("My(")+force_name+","+body_name+",NED)"));
    observer.write(force_in_ned_frame.N(),DataAddressing({"efforts",body_name,force_name,"NED","Mz"},std::string("Mz(")+force_name+","+body_name+",NED)"));
    extra_observations(observer);
}

void ForceModel::extra_observations(Observer& ) const
{}

void ForceModel::update(const BodyStates& states, const double t, const EnvironmentAndFrames& env, ssc::data_source::DataSource& command_listener)
{
	auto commands_ = get_commands(command_listener, t);
	auto force = get_force(states, t, env, commands_);
	auto wrench = compute_wrench_from_force(states, env, force);
	if(not samePoints(states.G,wrench.get_point()))
	{
		THROW(__PRETTY_FUNCTION__, InternalErrorException, "Error when computing force '" << force_name << "' on body '" << body_name << "': the force was provided at point '" << wrench.get_point() << "', which is not the centre of gravity of the body.");
	}
	force_in_body_frame = wrench;
	CHECK(force_name,"Fx",force_in_body_frame.X(),body_name,t);
	CHECK(force_name,"Fy",force_in_body_frame.Y(),body_name,t);
	CHECK(force_name,"Fz",force_in_body_frame.Z(),body_name,t);
	CHECK(force_name,"Mx",force_in_body_frame.K(),body_name,t);
	CHECK(force_name,"My",force_in_body_frame.M(),body_name,t);
	CHECK(force_name,"Mz",force_in_body_frame.N(),body_name,t);
	force_in_ned_frame = project_into_NED_frame(force_in_body_frame, states.get_rot_from_ned_to_body());
}


double ForceModel::get_Tmax() const
{
    return 0.;
}

