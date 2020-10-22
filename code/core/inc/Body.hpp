/*
 * Body.hpp
 *
 *  Created on: Jun 16, 2014
 *      Author: cady
 */

#ifndef BODY_HPP_
#define BODY_HPP_

#include <tuple>
#include <memory>
#include <Eigen/Dense>

#include "BlockedDOF.hpp"
#include "BodyStates.hpp"
#include "StateMacros.hpp"
#include "ForceModel.hpp"

#include <ssc/kinematics.hpp>

struct YamlBody;
struct YamlRotation;

class Observer;
class Body;
typedef std::shared_ptr<Body> BodyPtr;

class Body
{
public:
	virtual ~Body();
	Body(const size_t idx, const BlockedDOF& blocked_states);
	Body(const BodyStates& states, const size_t idx, const BlockedDOF& blocked_states);

	BodyStates get_states() const;
	std::vector<ForcePtr> get_forces() const;
	void set_forces(std::vector<ForcePtr> forces);

	/** \brief Use SurfaceElevation to compute wave height & update accordingly
	 */
	void update_intersection_with_free_surface(const EnvironmentAndFrames& env, const double t);

	ssc::kinematics::Point get_origin(const StateType& x) const;
	ssc::kinematics::Point get_position_of_body_relative_to_mesh() const;
	ssc::kinematics::Point get_G_in_body_frame() const; // Center of gravity in body frame
	ssc::kinematics::Point get_H_in_body_frame() const; // Calculation point of hydrodynamic forces in body frame
	ssc::kinematics::Transform get_transform_from_mesh_to_body() const;
	ssc::kinematics::Transform get_transform_from_ned_to_body(const StateType& x) const;
	ssc::kinematics::Transform get_transform_from_ned_to_local_ned(const StateType& x) const;

	/**  \brief Update Body structure taking the new coordinates & wave heights into account
	 */
	void update(const EnvironmentAndFrames& env, const StateType& x, const double t);
	void update_kinematics(StateType x, const ssc::kinematics::KinematicsPtr& k) const;
	void update_body_states(StateType x, const double t);
	StateType block_states_if_necessary(StateType x, const double t) const;

	/**  \brief Update down vector (expressed in body's mesh frame), taking the new coordinates into account
	 */
	void update_projection_of_z_in_mesh_frame(const double g,
											  const ssc::kinematics::KinematicsPtr& k);

	void calculate_state_derivatives(//const ssc::kinematics::Wrench& sum_of_forces,
									 const StateType& x,
									 StateType& dx_dt,
									 const double t,
									 const EnvironmentAndFrames& env,
									 ssc::data_source::DataSource& command_listener);

	Eigen::Vector3d get_uvw(const StateType& x) const;
	Eigen::Vector3d get_pqr(const StateType& x) const;
	std::string get_name() const;
	ssc::kinematics::RotationMatrix get_rot_from_ned_to(const StateType& x) const;
	ssc::kinematics::EulerAngles get_angles(const StateType& all_states, const YamlRotation& c) const;
	std::tuple<double,double,double,double> get_quaternions(const ssc::kinematics::EulerAngles& angle, const YamlRotation& c) const;

	void output(const StateType& x, const StateType& dx_dt, Observer& observer, const YamlRotation& c) const;
	BlockedDOF::Vector get_delta_F(const StateType& dx_dt) const;

	double get_Tmax() const;
	void set_Tmax(const double Tmax);
	void set_states_history(const AbstractStates<History>& states);
	void reset_history();
protected:
	BodyStates states;
	std::vector<ForcePtr> forces;
	ssc::kinematics::Wrench sum_of_forces_in_body_frame;
	ssc::kinematics::Wrench sum_of_forces_in_NED_frame;
	ssc::kinematics::Wrench fictitious_forces_in_body_frame;
	ssc::kinematics::Wrench fictitious_forces_in_NED_frame;
	bool has_surface_forces;

private:
	Body(); // Deactivated
	void compute_fictitious_forces(const StateType& x);
	void compute_sum_of_forces(double t, const EnvironmentAndFrames& env, ssc::data_source::DataSource& command_listener);

	void output_states(const StateType& x, Observer& observer, const YamlRotation& c) const;
	void output_forces(Observer& observer) const;
	void output_blocked_states_forces(const StateType& dx_dt, Observer& observer) const;

	size_t idx; //!< Index of the first state
	BlockedDOF blocked_states;
};

#endif /* BODY_HPP_ */
