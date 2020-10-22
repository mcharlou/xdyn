/*
 * BodyStates.hpp
 *
 *  Created on: Jan 8, 2015
 *      Author: cady
 */

#ifndef BODYSTATES_HPP_
#define BODYSTATES_HPP_

#include <tuple>

#include <ssc/macros.hpp>
#include <memory>
#include <memory>

#include <Eigen/Dense>

#include <ssc/kinematics.hpp>
#include "GeometricTypes3d.hpp"
#include "MeshIntersector.hpp"
#include "StateMacros.hpp"
#include "History.hpp"
#include "YamlRotation.hpp"
#include "AbstractStates.hpp"
#include "VelocityMovingAverage.hpp"

struct YamlRotation;

class Mesh;
typedef std::shared_ptr<Mesh> MeshPtr;
typedef std::shared_ptr<Eigen::Matrix<double,6,6> > MatrixPtr;


struct BodyStates : AbstractStates<History>
{
    BodyStates(const double Tmax=0 //!< Defines how much history we store
              );
    BodyStates& operator=(const AbstractStates<History>& rhs);
    std::string name;                                              //!< Body's name
    double Tmax;												   //!< Maximum history length
    ssc::kinematics::Point G;                                      //!< Position of the ship's centre of gravity in the ship's frame
    MeshPtr mesh;                                                  //!< Vertices & edges of the body's mesh
    MatrixPtr total_inertia;                                       //!< 6x6 matrix corresponding to the sum of the rigid body inertia + added mass expressed in the body frame
    MatrixPtr solid_body_inertia;                                  //!< 6x6 rigid body inertia matrix (i.e. without added mass) in the body frame
    MatrixPtr inverse_of_the_total_inertia;
    double x_relative_to_mesh;                                     //!< Position of the body frame relative to the mesh frame, along the x-axis, in meters
    double y_relative_to_mesh;                                     //!< Position of the body frame relative to the mesh frame, along the y-axis, in meters
    double z_relative_to_mesh;                                     //!< Position of the body frame relative to the mesh frame, along the z-axis, in meters
    ssc::kinematics::RotationMatrix mesh_to_body;                  //!< Rotation matrix from mesh to body
    ssc::kinematics::PointMatrixPtr M;                             //!< For the wave model (just the points in the mesh)
    MeshIntersectorPtr intersector;                                //!< Allows us to iterate on all emerged or immersed facets
    EPoint g_in_mesh_frame;                                        //!< Unit vertical vector, expressed in the body's mesh frame
    ssc::kinematics::Point hydrodynamic_forces_calculation_point;  //!< Point of expression of hydrodynamic forces (except Froude-Krylov & hydrostatic)

    VelocityMovingAverage low_frequency_velocity;				   //!< Low frequency part of the velocities, for use in some force model

	void record(double t, double x_, double y_, double z_, double u_, double v_, double w_, double p_, double q_, double r_, double qr_, double qi_, double qj_, double qk_);
    void reset();
	ssc::kinematics::EulerAngles get_angles() const;
    ssc::kinematics::EulerAngles get_angles(const YamlRotation& c) const;
    ssc::kinematics::EulerAngles get_angles(const StateType& all_states, const size_t idx, const YamlRotation& c) const;
    static ssc::kinematics::EulerAngles convert(const ssc::kinematics::RotationMatrix& R, const YamlRotation& rotations);
    ssc::kinematics::EulerAngles convert(const ssc::kinematics::RotationMatrix& R);
    static ssc::kinematics::EulerAngles convert(const std::tuple<double,double,double,double>& quat, const YamlRotation& rotations);
    ssc::kinematics::EulerAngles convert(const std::tuple<double,double,double,double>& quat);
    static std::tuple<double,double,double,double> convert(const ssc::kinematics::EulerAngles& R, const YamlRotation& rotations);
    std::tuple<double,double,double,double> convert(const ssc::kinematics::EulerAngles& R);
    ssc::kinematics::RotationMatrix get_rot_from_ned_to_body() const;
    ssc::kinematics::RotationMatrix get_rot_from_ned_to(const StateType& x, const size_t idx) const;
    YamlRotation convention;  //!< Rotation convention
    std::vector<double> get_current_state_values(const size_t idx) const;
    const History& get_speed(const size_t idx) const;
    Eigen::Matrix<double, 6, 1> get_generalized_speed() const;
    Eigen::Vector3d get_speed() const;
    Eigen::Matrix<double, 6, 1> get_mean_generalized_speed(const double T) const;
    std::pair<double,double> get_phi_bounds_over_period(const double T) const;
    void set_Tmax(const double Tmax) override;
};

#endif /* BODYSTATES_HPP_ */
