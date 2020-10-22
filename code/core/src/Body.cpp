/*
 * Body.cpp
 *
 *  Created on: Jun 16, 2014
 *      Author: cady
 */


#include "EnvironmentAndFrames.hpp"
#include "Observer.hpp"
#include "SurfaceElevationInterface.hpp"
#include "YamlBody.hpp"
#include "NumericalErrorException.hpp"
#include "BodyStates.hpp"

#include "Body.hpp"

Body::Body(const size_t i, const BlockedDOF& blocked_states_) :
			states(),
			forces(),
			sum_of_forces_in_body_frame(),
			sum_of_forces_in_NED_frame(),
			fictitious_forces_in_body_frame(),
			fictitious_forces_in_NED_frame(),
			has_surface_forces(false),
			idx(i),
			blocked_states(blocked_states_)
{}

Body::Body(const BodyStates& s, const size_t i, const BlockedDOF& blocked_states_) :
			states(s),
			forces(),
			sum_of_forces_in_body_frame(),
			sum_of_forces_in_NED_frame(),
			fictitious_forces_in_body_frame(),
			fictitious_forces_in_NED_frame(),
			has_surface_forces(false),
			idx(i),
			blocked_states(blocked_states_)
{}

Body::~Body()
{
}

BodyStates Body::get_states() const
{
    return states;
}

std::vector<ForcePtr> Body::get_forces() const
{
	return forces;
}

void Body::set_forces(std::vector<ForcePtr> forces_)
{
	forces = forces_;
	has_surface_forces = false;
	for (auto force:forces)
	{
		if(force->is_a_surface_force_model()) has_surface_forces = true;
	}
}

double Body::get_Tmax() const
{
	return states.Tmax;
}

void Body::set_Tmax(const double Tmax)
{
	states.set_Tmax(Tmax);
}

#define SQUARE(x) ((x)*(x))

ssc::kinematics::Point Body::get_origin(const StateType& x) const
{
    return ssc::kinematics::Point("NED", *_X(x,idx),
                                         *_Y(x,idx),
                                         *_Z(x,idx));
}

ssc::kinematics::Point Body::get_position_of_body_relative_to_mesh() const
{
    return ssc::kinematics::Point(std::string("mesh(")+states.name+")",
                                  states.x_relative_to_mesh,
                                  states.y_relative_to_mesh,
                                  states.z_relative_to_mesh);
}

ssc::kinematics::Point Body::get_G_in_body_frame() const
{
	return states.G;
}

ssc::kinematics::Point Body::get_H_in_body_frame() const
{
	return states.hydrodynamic_forces_calculation_point;
}

ssc::kinematics::Transform Body::get_transform_from_mesh_to_body() const
{
    return ssc::kinematics::Transform(get_position_of_body_relative_to_mesh(), states.mesh_to_body, states.name);
}

ssc::kinematics::Transform Body::get_transform_from_ned_to_body(const StateType& x) const
{
    return ssc::kinematics::Transform(get_origin(x), states.get_rot_from_ned_to(x, idx), states.name);
}

ssc::kinematics::Transform Body::get_transform_from_ned_to_local_ned(const StateType& x) const
{
    return ssc::kinematics::Transform(get_origin(x), std::string("NED(") + states.name + ")");
}

void Body::update_kinematics(StateType x, const ssc::kinematics::KinematicsPtr& k) const
{
    k->add(get_transform_from_ned_to_body(x));
    k->add(get_transform_from_ned_to_local_ned(x));
}

StateType Body::block_states_if_necessary(StateType x, const double t) const
{
    blocked_states.force_states(x,t);
    return x;
}

void Body::update_body_states(StateType x, const double t)
{
    blocked_states.force_states(x,t);
    states.record(t, *_X(x,idx), *_Y(x,idx), *_Z(x,idx), *_U(x,idx), *_V(x,idx), *_W(x,idx), *_P(x,idx), *_Q(x,idx), *_R(x,idx), *_QR(x,idx), *_QI(x,idx), *_QJ(x,idx), *_QK(x,idx));
}

void Body::update_projection_of_z_in_mesh_frame(const double g,
                                                const ssc::kinematics::KinematicsPtr& k)
{
    const ssc::kinematics::Point g_in_NED("NED", 0, 0, g);
    const ssc::kinematics::RotationMatrix ned2mesh = k->get("NED", std::string("mesh(") + states.name + ")").get_rot();
    states.g_in_mesh_frame = ned2mesh*g_in_NED.v;
}

#define CHECK(x,y,t) if (std::isnan(x)) {THROW(__PRETTY_FUNCTION__,NumericalErrorException,"NaN detected in state " << y << ", at t = " << t);}
void Body::update(const EnvironmentAndFrames& env, const StateType& x, const double t)
{
    CHECK(*_X(x,idx),"X",t);
    CHECK(*_Y(x,idx),"Y",t);
    CHECK(*_Z(x,idx),"Z",t);
    CHECK(*_U(x,idx),"U",t);
    CHECK(*_V(x,idx),"V",t);
    CHECK(*_W(x,idx),"W",t);
    CHECK(*_P(x,idx),"P",t);
    CHECK(*_Q(x,idx),"Q",t);
    CHECK(*_R(x,idx),"R",t);
    CHECK(*_QR(x,idx),"QR",t);
    CHECK(*_QI(x,idx),"QI",t);
    CHECK(*_QJ(x,idx),"QJ",t);
    CHECK(*_QK(x,idx),"QK",t);
    update_kinematics(x,env.k);
    update_body_states(x, t);
    update_intersection_with_free_surface(env, t);
    update_projection_of_z_in_mesh_frame(env.g, env.k);
}

void Body::calculate_state_derivatives(const StateType& x,
									   StateType& dx_dt,
									   const double t,
									   const EnvironmentAndFrames& env,
									   ssc::data_source::DataSource& command_listener)
{
	update(env,x,t);
	compute_fictitious_forces(x);
	compute_sum_of_forces(t, env, command_listener);
	// du/dt, dv/dt, dw/dt, dp/dt, dq/dt, dr/dt
    Eigen::Map<Eigen::Matrix<double,6,1> > dXdt(_U(dx_dt,idx));

    dXdt = states.inverse_of_the_total_inertia->operator*(sum_of_forces_in_body_frame.to_vector());

    // dx/dt, dy/dt, dz/dt
    const ssc::kinematics::RotationMatrix& R = env.k->get("NED", states.name).get_rot();
    const Eigen::Map<const Eigen::Vector3d> uvw(_U(x,idx));
    const Eigen::Vector3d XpYpZp(R*uvw);
    *_X(dx_dt,idx) = XpYpZp(0);
    *_Y(dx_dt,idx) = XpYpZp(1);
    *_Z(dx_dt,idx) = XpYpZp(2);

    // dqr/dt, dqi/dt, dqj/dt, dqk/dt
    const Eigen::Quaternion<double> q1(*_QR(x,idx),
                                       *_QI(x,idx),
                                       *_QJ(x,idx),
                                       *_QK(x,idx));
    const Eigen::Quaternion<double> q2(0,*_P(x,idx),*_Q(x,idx),*_R(x,idx));
    const Eigen::Quaternion<double>& dq_dt = q1*q2;
    *_QR(dx_dt,idx) = 0.5*(double)dq_dt.w();
    *_QI(dx_dt,idx) = 0.5*(double)dq_dt.x();
    *_QJ(dx_dt,idx) = 0.5*(double)dq_dt.y();
    *_QK(dx_dt,idx) = 0.5*(double)dq_dt.z();

    blocked_states.force_state_derivatives(dx_dt, t);
}

Eigen::Vector3d Body::get_uvw(const StateType& x) const
{
    return Eigen::Vector3d::Map(_U(x,idx));
}

Eigen::Vector3d Body::get_pqr(const StateType& x) const
{
    return Eigen::Vector3d::Map(_P(x,idx));
}

void Body::compute_fictitious_forces(const StateType& x)
{
	// Coriolis and centrifugal forces (Euler force is neglected)
	const Matrix6x6* M = states.solid_body_inertia.get();
	const Eigen::Vector3d v = get_uvw(x);
	const Eigen::Vector3d omega = get_pqr(x);

    const Matrix3x3 A11 = M->block<3,3>(0,0);
    const Matrix3x3 A12 = M->block<3,3>(0,3);
    const Matrix3x3 A21 = M->block<3,3>(3,0);
    const Matrix3x3 A22 = M->block<3,3>(3,3);

    const Eigen::Vector3d s1 = A11*v+A12*omega;
    const Eigen::Vector3d s2 = A21*v+A22*omega;

    const Eigen::Vector3d force = s1.cross(omega);
    const Eigen::Vector3d torque = s1.cross(v)+s2.cross(omega);

    fictitious_forces_in_body_frame = ssc::kinematics::Wrench(states.G, force, torque);
    fictitious_forces_in_NED_frame = ForceModel::project_into_NED_frame(fictitious_forces_in_body_frame,states.get_rot_from_ned_to_body());
}

void Body::compute_sum_of_forces(double t, const EnvironmentAndFrames& env, ssc::data_source::DataSource& command_listener)
{
	sum_of_forces_in_body_frame = fictitious_forces_in_body_frame;
	for (auto force:forces)
	{
		//force->update(states, t, pimpl->env.k);
		const ssc::kinematics::Wrench tau = force->operator()(states, t, env, command_listener);
		sum_of_forces_in_body_frame = sum_of_forces_in_body_frame + tau;
	}
	sum_of_forces_in_NED_frame = ForceModel::project_into_NED_frame(sum_of_forces_in_body_frame,states.get_rot_from_ned_to_body());
}

BlockedDOF::Vector Body::get_delta_F(const StateType& dx_dt) const
{
    return blocked_states.get_delta_F(dx_dt,*states.total_inertia,sum_of_forces_in_body_frame);
}

void Body::output(const StateType& x, const StateType& dx_dt, Observer& observer, const YamlRotation& c) const
{
	output_states(x, observer, c);
	output_forces(observer);
	output_blocked_states_forces(dx_dt, observer);

}

void Body::output_states(const StateType& x, Observer& observer, const YamlRotation& c) const
{
	observer.write(*_X(x,idx), DataAddressing({"states",states.name,"X"},std::string("x(")+states.name+")"));
    observer.write(*_Y(x,idx), DataAddressing({"states",states.name,"Y"},std::string("y(")+states.name+")"));
    observer.write(*_Z(x,idx), DataAddressing({"states",states.name,"Z"},std::string("z(")+states.name+")"));
    observer.write(*_U(x,idx), DataAddressing({"states",states.name,"U"},std::string("u(")+states.name+")"));
    observer.write(*_V(x,idx), DataAddressing({"states",states.name,"V"},std::string("v(")+states.name+")"));
    observer.write(*_W(x,idx), DataAddressing({"states",states.name,"W"},std::string("w(")+states.name+")"));
    observer.write(*_P(x,idx), DataAddressing({"states",states.name,"P"},std::string("p(")+states.name+")"));
    observer.write(*_Q(x,idx), DataAddressing({"states",states.name,"Q"},std::string("q(")+states.name+")"));
    observer.write(*_R(x,idx), DataAddressing({"states",states.name,"R"},std::string("r(")+states.name+")"));
    observer.write(*_QR(x,idx),DataAddressing({"states",states.name,"Quat","Qr"},std::string("qr(")+states.name+")"));
    observer.write(*_QI(x,idx),DataAddressing({"states",states.name,"Quat","Qi"},std::string("qi(")+states.name+")"));
    observer.write(*_QJ(x,idx),DataAddressing({"states",states.name,"Quat","Qj"},std::string("qj(")+states.name+")"));
    observer.write(*_QK(x,idx),DataAddressing({"states",states.name,"Quat","Qk"},std::string("qk(")+states.name+")"));
    const auto angles = get_angles(x, c);
    observer.write(angles.phi, DataAddressing({"states",states.name,"PHI"},std::string("phi(")+states.name+")"));
    observer.write(angles.theta, DataAddressing({"states",states.name,"THETA"},std::string("theta(")+states.name+")"));
    observer.write(angles.psi, DataAddressing({"states",states.name,"PSI"},std::string("psi(")+states.name+")"));

    auto U_EMA = states.low_frequency_velocity.get_vector();
    observer.write((double)U_EMA(0),DataAddressing({"states",get_name(),"EMA(U)"},std::string("u_EMA(")+get_name()+")"));
    observer.write((double)U_EMA(1),DataAddressing({"states",get_name(),"EMA(V)"},std::string("v_EMA(")+get_name()+")"));
    observer.write((double)U_EMA(2),DataAddressing({"states",get_name(),"EMA(W)"},std::string("w_EMA(")+get_name()+")"));
    observer.write((double)U_EMA(3),DataAddressing({"states",get_name(),"EMA(P)"},std::string("p_EMA(")+get_name()+")"));
    observer.write((double)U_EMA(4),DataAddressing({"states",get_name(),"EMA(Q)"},std::string("q_EMA(")+get_name()+")"));
    observer.write((double)U_EMA(5),DataAddressing({"states",get_name(),"EMA(R)"},std::string("r_EMA(")+get_name()+")"));
}

#define OUTPUT_WRENCH(wrench,name,body,frame) \
		observer.write(wrench.X(),DataAddressing({"efforts",body,name,frame,"Fx"},std::string("Fx(")+name+","+body+","+frame+")"));\
		observer.write(wrench.Y(),DataAddressing({"efforts",body,name,frame,"Fy"},std::string("Fy(")+name+","+body+","+frame+")"));\
		observer.write(wrench.Z(),DataAddressing({"efforts",body,name,frame,"Fz"},std::string("Fz(")+name+","+body+","+frame+")"));\
		observer.write(wrench.K(),DataAddressing({"efforts",body,name,frame,"Mx"},std::string("Mx(")+name+","+body+","+frame+")"));\
		observer.write(wrench.M(),DataAddressing({"efforts",body,name,frame,"My"},std::string("My(")+name+","+body+","+frame+")"));\
		observer.write(wrench.N(),DataAddressing({"efforts",body,name,frame,"Mz"},std::string("Mz(")+name+","+body+","+frame+")"));

void Body::output_forces(Observer& observer) const
{
	OUTPUT_WRENCH(sum_of_forces_in_body_frame,"sum of forces",get_name(),get_name());
	OUTPUT_WRENCH(sum_of_forces_in_NED_frame,"sum of forces",get_name(),"NED");
	OUTPUT_WRENCH(fictitious_forces_in_body_frame,"fictitious forces",get_name(),get_name());
	for(auto force:forces)
	{
		force->feed(observer);
	}
}

void Body::output_blocked_states_forces(const StateType& dx_dt, Observer& observer) const
{
	auto dF = get_delta_F(dx_dt);
	observer.write((double)dF(0),DataAddressing({"efforts",get_name(),"blocked states",get_name(),"Fx"},std::string("Fx(blocked states,")+get_name()+","+get_name()+")"));
	observer.write((double)dF(1),DataAddressing({"efforts",get_name(),"blocked states",get_name(),"Fy"},std::string("Fy(blocked states,")+get_name()+","+get_name()+")"));
	observer.write((double)dF(2),DataAddressing({"efforts",get_name(),"blocked states",get_name(),"Fz"},std::string("Fz(blocked states,")+get_name()+","+get_name()+")"));
	observer.write((double)dF(3),DataAddressing({"efforts",get_name(),"blocked states",get_name(),"Mx"},std::string("Mx(blocked states,")+get_name()+","+get_name()+")"));
	observer.write((double)dF(4),DataAddressing({"efforts",get_name(),"blocked states",get_name(),"My"},std::string("My(blocked states,")+get_name()+","+get_name()+")"));
	observer.write((double)dF(5),DataAddressing({"efforts",get_name(),"blocked states",get_name(),"Mz"},std::string("Mz(blocked states,")+get_name()+","+get_name()+")"));
}

std::string Body::get_name() const
{
    return states.name;
}

ssc::kinematics::RotationMatrix Body::get_rot_from_ned_to(const StateType& x) const
{
    return states.get_rot_from_ned_to(x,idx);
}

ssc::kinematics::EulerAngles Body::get_angles(const StateType& all_states, const YamlRotation& c) const
{
    return states.get_angles(all_states, idx, c);
}

std::tuple<double,double,double,double> Body::get_quaternions(const ssc::kinematics::EulerAngles& angle, const YamlRotation& c) const
{
    return states.convert(angle,c);
}

void Body::set_states_history(const AbstractStates<History>& s)
{
    states = s;
}

void Body::reset_history()
{
    states.reset();
}

void Body::update_intersection_with_free_surface(const EnvironmentAndFrames& env, const double t)
{
    if (env.w.use_count())// && has_surface_forces) TODO: check should be made at call, and this entire method should not be public
    {
        try
        {
            env.w->update_surface_elevation(states.M, env.k,t);
        }
        catch (const ssc::exception_handling::Exception& e)
        {
            THROW(__PRETTY_FUNCTION__, ssc::exception_handling::Exception, "This simulation uses surface force models (eg. Froude-Krylov) which are integrated on the hull. This requires computing the intersection between the hull and the free surface and hence calculating the wave heights. While calculating these wave heights, " << e.get_message());
        }
        const std::vector<double> dz = env.w->get_relative_wave_height();
        states.intersector->update_intersection_with_free_surface(env.w->get_relative_wave_height(),
                                                                  env.w->get_surface_elevation());
    }
}
