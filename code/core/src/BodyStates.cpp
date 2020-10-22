/*
 * BodyStates.cpp
 *
 *  Created on: Jan 8, 2015
 *      Author: cady
 */

#include <utility>
#include <iostream>
#include <Eigen/Dense>
#include <math.h>

#include "BodyStates.hpp"
#include "EnvironmentAndFrames.hpp"
#include "StateMacros.hpp"
#include "SurfaceElevationInterface.hpp"
#include "YamlBody.hpp"
#include "yaml2eigen.hpp"
#include "VelocityMovingAverage.hpp"

BodyStates::BodyStates(const double Tmax_) : AbstractStates<History>(Tmax_),
		name(),
		Tmax(Tmax_),
		G(),
		mesh(),
		total_inertia(),
		solid_body_inertia(),
		inverse_of_the_total_inertia(),
		x_relative_to_mesh(),
		y_relative_to_mesh(),
		z_relative_to_mesh(),
		mesh_to_body(),
		M(),
		intersector(),
		g_in_mesh_frame(),
		hydrodynamic_forces_calculation_point(),
		low_frequency_velocity(Tmax_),
		convention()
{
}

BodyStates& BodyStates::operator=(const AbstractStates<History>& rhs)
{
    AbstractStates<History>::operator=(rhs);
    low_frequency_velocity.reset(*this);
    return *this;
}

ssc::kinematics::EulerAngles BodyStates::convert(const ssc::kinematics::RotationMatrix& R, const YamlRotation& rotations)
{
    using namespace ssc::kinematics;
    if (rotations.order_by == "angle")
    {
        if (match(rotations.convention, "z", "y'", "x''"))
        {
            return euler_angles<INTRINSIC, CHANGING_ANGLE_ORDER, 3, 2, 1>(R);
        }
        std::stringstream ss;
        ss << "Rotation convention '" << rotations.convention.at(0) << "," << rotations.convention.at(1) << "," << rotations.convention.at(2) << "' is not currently supported.";
        THROW(__PRETTY_FUNCTION__, ssc::exception_handling::Exception, ss.str());
    }
    else
    {
        THROW(__PRETTY_FUNCTION__, ssc::exception_handling::Exception, std::string("Ordering rotations by '") + rotations.order_by + "' is not currently supported");
    }
    return EulerAngles();
}

ssc::kinematics::EulerAngles BodyStates::convert(const ssc::kinematics::RotationMatrix& R)
{
	return convert(R, convention);
}

ssc::kinematics::EulerAngles BodyStates::convert(const std::tuple<double,double,double,double>& quat, const YamlRotation& rotations)
{
    using namespace ssc::kinematics;
    const ssc::kinematics::RotationMatrix R = Eigen::Quaternion<double>(std::get<0>(quat),std::get<1>(quat),std::get<2>(quat),std::get<3>(quat)).matrix();
    return convert(R,rotations);
}

ssc::kinematics::EulerAngles BodyStates::convert(const std::tuple<double,double,double,double>& quat)
{
	return convert(quat, convention);
}

std::tuple<double,double,double,double> BodyStates::convert(const ssc::kinematics::EulerAngles& angles, const YamlRotation& rotations)
{
    using namespace ssc::kinematics;
    if (rotations.order_by == "angle")
    {
        if (match(rotations.convention, "z", "y'", "x''"))
        {
            Eigen::Quaternion<double> quat(rotation_matrix<INTRINSIC, CHANGING_ANGLE_ORDER, 3, 2, 1>(angles));
            std::tuple<double,double,double,double> ret;
            std::get<0>(ret) = quat.w();
            std::get<1>(ret) = quat.x();
            std::get<2>(ret) = quat.y();
            std::get<3>(ret) = quat.z();
            return ret;
        }

        std::stringstream ss;
        ss << "Rotation convention '" << rotations.convention.at(0) << "," << rotations.convention.at(1) << "," << rotations.convention.at(2) << "' is not currently supported.";
        THROW(__PRETTY_FUNCTION__, ssc::exception_handling::Exception, ss.str());
    }
    else
    {
        THROW(__PRETTY_FUNCTION__, ssc::exception_handling::Exception, std::string("Ordering rotations by '") + rotations.order_by + "' is not currently supported");
    }
    return std::make_tuple(0.,0.,0.,0.);
}

std::tuple<double,double,double,double> BodyStates::convert(const ssc::kinematics::EulerAngles& R)
{
	return convert(R, convention);
}

void BodyStates::record(double t, double x_, double y_, double z_, double u_, double v_, double w_, double p_, double q_, double r_, double qr_, double qi_, double qj_, double qk_)
{
	x.record(t, x_);
	y.record(t, y_);
	z.record(t, z_);
	u.record(t, u_);
	v.record(t, v_);
	w.record(t, w_);
	p.record(t, p_);
	q.record(t, q_);
	r.record(t, r_);
	qr.record(t, qr_);
	qi.record(t, qi_);
	qj.record(t, qj_);
	qk.record(t, qk_);
	low_frequency_velocity.update(t,u_,v_,w_,p_,q_,r_);
}

void BodyStates::reset()
{
	x.reset();
	y.reset();
	z.reset();
	u.reset();
	v.reset();
	w.reset();
	p.reset();
	q.reset();
	r.reset();
	qr.reset();
	qi.reset();
	qj.reset();
	qk.reset();
	low_frequency_velocity.reset();
}

ssc::kinematics::EulerAngles BodyStates::get_angles() const
{
    return convert(get_rot_from_ned_to_body(),convention);
}

ssc::kinematics::EulerAngles BodyStates::get_angles(const YamlRotation& c) const
{
    return convert(get_rot_from_ned_to_body(),c);
}

ssc::kinematics::EulerAngles BodyStates::get_angles(const StateType& all_states, const size_t idx, const YamlRotation& c) const
{
    return convert(get_rot_from_ned_to(all_states,idx),c);
}

ssc::kinematics::RotationMatrix BodyStates::get_rot_from_ned_to_body() const
{
    return Eigen::Quaternion<double>(qr(),qi(),qj(),qk()).matrix();
}

ssc::kinematics::RotationMatrix BodyStates::get_rot_from_ned_to(const StateType& x, const size_t idx) const
{
    return Eigen::Quaternion<double>(*_QR(x,idx),*_QI(x,idx),*_QJ(x,idx),*_QK(x,idx)).matrix();
}

std::vector<double> BodyStates::get_current_state_values(const size_t idx) const
{
    std::vector<double> s(13, 0);
    s[XIDX(idx)] = x();
    s[YIDX(idx)] = y();
    s[ZIDX(idx)] = z();
    s[UIDX(idx)] = u();
    s[VIDX(idx)] = v();
    s[WIDX(idx)] = w();
    s[PIDX(idx)] = p();
    s[QIDX(idx)] = q();
    s[RIDX(idx)] = r();
    s[QRIDX(idx)] = qr();
    s[QIIDX(idx)] = qi();
    s[QJIDX(idx)] = qj();
    s[QKIDX(idx)] = qk();
    return s;
}

const History& BodyStates::get_speed(const size_t idx) const
{
  switch(idx)
  {
    case 0:
      return u;
    case 1:
      return v;
    case 2:
      return w;
    case 3:
      return p;
    case 4:
      return q;
    case 5:
      return r;
    default:
      std::cerr << "Requested speed from body states is out of bounds [0,5], returning u instead." << std::endl;
      return u;
  }
}

Eigen::Matrix<double, 6, 1> BodyStates::get_generalized_speed() const
{
	Eigen::Matrix<double, 6, 1> U;
	U << u(),v(),w(),p(),q(),r();
	return U;
}

Eigen::Vector3d BodyStates::get_speed() const
{
	Eigen::Vector3d speed;
	speed << u(),v(),w();
	return speed;
}

Eigen::Matrix<double, 6, 1> BodyStates::get_mean_generalized_speed(const double T) const
{
	Eigen::Matrix<double, 6, 1> U;
	U << u.average(T),v.average(T),w.average(T),p.average(T),q.average(T),r.average(T);
	return U;
}

std::pair<double,double> BodyStates::get_phi_bounds_over_period(const double T) const
{
	std::vector<Eigen::Quaternion<double>> quat;
	const auto QR=qr.get_values(T);
	const auto QI=qi.get_values(T);
	const auto QJ=qj.get_values(T);
	const auto QK=qk.get_values(T);
	for(size_t i=0 ; i<QR.size() ; i++)
	{
		quat.push_back(Eigen::Quaternion<double>(QR.at(i),QI.at(i),QJ.at(i),QK.at(i)));
	}
	std::pair<double,double> phi_bounds;
	auto lambda = [this](Eigen::Quaternion<double>& a,Eigen::Quaternion<double>& b){return convert(a.matrix(),convention).phi<convert(b.matrix(),convention).phi;};
	phi_bounds.first=convert(min_element(quat.begin(),quat.end(),lambda)->matrix(),convention).phi;
	phi_bounds.second=convert(max_element(quat.begin(),quat.end(),lambda)->matrix(),convention).phi;
	return phi_bounds;
}

void BodyStates::set_Tmax(const double Tmax_)
{
	AbstractStates<History>::set_Tmax(Tmax_);
	Tmax = Tmax_;
}

