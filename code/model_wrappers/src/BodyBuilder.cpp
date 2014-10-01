/*
 * BodyBuilder.cpp
 *
 *  Created on: Jun 17, 2014
 *      Author: cady
 */

#include "BodyBuilder.hpp"
#include "BodyBuilderException.hpp"
#include "MeshBuilder.hpp"
#include <ssc/kinematics.hpp>
#include "YamlBody.hpp"

BodyBuilder::BodyBuilder(const YamlRotation& convention) : rotations(convention)
{
}

void BodyBuilder::change_mesh_ref_frame(Body& body, const VectorOfVectorOfPoints& mesh) const
{
    ssc::kinematics::Point translation(body.name, body.x_relative_to_mesh, body.y_relative_to_mesh, body.z_relative_to_mesh);
    ssc::kinematics::Transform transform(translation, body.mesh_to_body, "mesh("+body.name+")");
    body.mesh = MeshPtr(new Mesh(MeshBuilder(mesh).build()));
    const auto T = transform.inverse();
    body.mesh->nodes = (T*ssc::kinematics::PointMatrix(body.mesh->nodes, "mesh("+body.name+")")).m;
    body.mesh->all_nodes = (T*ssc::kinematics::PointMatrix(body.mesh->all_nodes, "mesh("+body.name+")")).m;
    for (size_t i = 0 ; i < body.mesh->facets.size() ; ++i)
    {
        body.mesh->facets[i].barycenter = T*body.mesh->facets[i].barycenter;
        body.mesh->facets[i].unit_normal = T.get_rot()*body.mesh->facets[i].unit_normal;
    }
    body.M = PointMatrixPtr(new ssc::kinematics::PointMatrix(body.mesh->nodes, body.name));
}

Body BodyBuilder::build(const YamlBody& input, const VectorOfVectorOfPoints& mesh) const
{
    Body ret;
    ret.name = input.name;
    ret.G = ssc::kinematics::Point(input.dynamics.centre_of_inertia.frame,
                                   input.dynamics.centre_of_inertia.x,
                                   input.dynamics.centre_of_inertia.y,
                                   input.dynamics.centre_of_inertia.z);
    ret.m = input.dynamics.mass;

    ret.x_relative_to_mesh = input.position_of_body_frame_relative_to_mesh.coordinates.x;
    ret.y_relative_to_mesh = input.position_of_body_frame_relative_to_mesh.coordinates.y;
    ret.z_relative_to_mesh = input.position_of_body_frame_relative_to_mesh.coordinates.z;
    ret.mesh_to_body = angle2matrix(input.position_of_body_frame_relative_to_mesh.angle);
    change_mesh_ref_frame(ret, mesh);
    add_inertia(ret, input.dynamics.rigid_body_inertia, input.dynamics.added_mass);
    ret.u = input.initial_velocity_of_body_frame_relative_to_NED_projected_in_body.u;
    ret.v = input.initial_velocity_of_body_frame_relative_to_NED_projected_in_body.v;
    ret.w = input.initial_velocity_of_body_frame_relative_to_NED_projected_in_body.w;
    ret.p = input.initial_velocity_of_body_frame_relative_to_NED_projected_in_body.p;
    ret.q = input.initial_velocity_of_body_frame_relative_to_NED_projected_in_body.q;
    ret.r = input.initial_velocity_of_body_frame_relative_to_NED_projected_in_body.r;
    return ret;
}

ssc::kinematics::RotationMatrix BodyBuilder::angle2matrix(const YamlAngle& a) const
{
    const ssc::kinematics::EulerAngles e(a.phi, a.theta, a.psi);

    if (rotations.order_by == "angle")
    {
        if (match(rotations.convention, "z", "y'", "x''"))
            return ssc::kinematics::rotation_matrix<ssc::kinematics::INTRINSIC,
                                                    ssc::kinematics::CHANGING_ANGLE_ORDER,
                                                    ssc::kinematics::CARDAN, 3, 2, 1>(e);
        std::stringstream ss;
        ss << "Rotation convention '" << rotations.convention.at(0) << "," << rotations.convention.at(1) << "," << rotations.convention.at(2) << "' is not currently supported.";
        THROW(__PRETTY_FUNCTION__, BodyBuilderException, ss.str());
    }
    else
    {
        THROW(__PRETTY_FUNCTION__, BodyBuilderException, std::string("Ordering rotations by '") + rotations.order_by + "' is not currently supported");
    }
    return ssc::kinematics::RotationMatrix();
}

bool BodyBuilder::match(const std::vector<std::string>& convention, const std::string& first, const std::string& second, const std::string& third) const
{
    return (convention.at(0) == first) and (convention.at(1) == second) and (convention.at(2) == third);
}

void BodyBuilder::add_inertia(Body& body, const YamlInertiaMatrix& rigid_body_inertia, const YamlInertiaMatrix& added_mass) const
{
    Eigen::Matrix<double,6,6> Mrb = convert(rigid_body_inertia);
    Eigen::Matrix<double,6,6> Ma = convert(added_mass);
    if (fabs((Mrb+Ma).determinant())<1E-10)
    {
        std::stringstream ss;
        ss << "Unable to compute the inverse of the total inertia matrix (rigid body inertia + added mass): " << std::endl
           << "Mrb = " << std::endl
           << Mrb << std::endl
           << "Ma = " << std::endl
           << Ma << std::endl
           << "Mrb+Ma = " << std::endl
           << Mrb+Ma << std::endl;
        THROW(__PRETTY_FUNCTION__, BodyBuilderException, ss.str());
    }
    Eigen::Matrix<double,6,6> M_inv = (Mrb+Ma).inverse();
    body.inverse_of_the_total_inertia = MatrixPtr(new Eigen::Matrix<double,6,6>(M_inv));
    body.solid_body_inertia = MatrixPtr(new Eigen::Matrix<double,6,6>(Mrb));
    body.total_inertia = MatrixPtr(new Eigen::Matrix<double,6,6>(Mrb+Ma));
}

Eigen::Matrix<double,6,6> BodyBuilder::convert(const YamlInertiaMatrix& M) const
{
    Eigen::Matrix<double,6,6> ret;
    for (size_t j = 0 ; j < 6 ; ++j)
    {
        ret(0,(int)j) = M.row_1.at(j);
        ret(1,(int)j) = M.row_2.at(j);
        ret(2,(int)j) = M.row_3.at(j);
        ret(3,(int)j) = M.row_4.at(j);
        ret(4,(int)j) = M.row_5.at(j);
        ret(5,(int)j) = M.row_6.at(j);
    }
    return ret;
}

Body BodyBuilder::build(const std::string& name, const VectorOfVectorOfPoints& mesh) const
{
    YamlBody input;
    input.name = name;
    input.dynamics.centre_of_inertia.frame = name;
    input.dynamics.rigid_body_inertia.frame = name;
    input.dynamics.rigid_body_inertia.row_1 = {1,0,0,0,0,0};
    input.dynamics.rigid_body_inertia.row_2 = {0,1,0,0,0,0};
    input.dynamics.rigid_body_inertia.row_3 = {0,0,1,0,0,0};
    input.dynamics.rigid_body_inertia.row_4 = {0,0,0,1,0,0};
    input.dynamics.rigid_body_inertia.row_5 = {0,0,0,0,1,0};
    input.dynamics.rigid_body_inertia.row_6 = {0,0,0,0,0,1};
    input.dynamics.added_mass = input.dynamics.rigid_body_inertia;
    return build(input, mesh);
}
