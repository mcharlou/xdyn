/*
 * BodyBuilder.cpp
 *
 *  Created on: Jun 17, 2014
 *      Author: cady
 */

#include <ssc/kinematics.hpp>
#include <ssc/text_file_reader.hpp>

#include "InvalidInputException.hpp"
#include "BodyWithSurfaceForces.hpp"
#include "BodyWithoutSurfaceForces.hpp"
#include "HDBParser.hpp"
#include "MeshBuilder.hpp"
#include "YamlBody.hpp"
#include "yaml2eigen.hpp"
#include "Body.hpp"
#include "BodyStates.hpp"

#include "BodyBuilder.hpp"

bool isSymmetric(const Eigen::MatrixXd& m)
{
    const double tol = 1e-10;
    if (m.rows()!=m.cols()) return false;
    const size_t n = m.rows();
    for (size_t i = 0;i<n;++i)
        for (size_t j = i+1;j<n;++j)
            if (std::abs(m(i,j)-m(j,i))>tol) return false;
    return true;
}
/**
 * \note is based on Sylvester criterion
 */
bool isSymmetricDefinitePositive(const Eigen::MatrixXd& m)
{
    if (!isSymmetric(m)) return false;
    const size_t n = m.rows();
    for (size_t i = 1;i<=n;++i)
    {
        if (m.block(0, 0, i, i).determinant()<=0.0)
        {
            return false;
        }
    }
    return true;
}

BodyBuilder::BodyBuilder(const YamlRotation& convention) : rotations(convention), force_parsers()
{}

void BodyBuilder::change_mesh_ref_frame(BodyStates& states, const VectorOfVectorOfPoints& mesh) const
{
    const ssc::kinematics::Point translation(states.name, states.x_relative_to_mesh, states.y_relative_to_mesh, states.z_relative_to_mesh);
    const ssc::kinematics::Transform transform(translation, states.mesh_to_body, "mesh("+states.name+")");
    states.mesh = MeshPtr(new Mesh(MeshBuilder(mesh).build()));
    const auto T = transform.inverse();
    states.mesh->nodes = (T*ssc::kinematics::PointMatrix(states.mesh->nodes, "mesh("+states.name+")")).m;
    states.mesh->all_nodes = (T*ssc::kinematics::PointMatrix(states.mesh->all_nodes, "mesh("+states.name+")")).m;
    for (size_t i = 0 ; i < states.mesh->facets.size() ; ++i)
    {
        states.mesh->facets[i].centre_of_gravity = T*states.mesh->facets[i].centre_of_gravity;
        states.mesh->facets[i].unit_normal = T.get_rot()*states.mesh->facets[i].unit_normal;
    }
    states.M = ssc::kinematics::PointMatrixPtr(new ssc::kinematics::PointMatrix(states.mesh->nodes, states.name));
}

BodyStates BodyBuilder::get_initial_states(const YamlBody& input, const VectorOfVectorOfPoints& mesh, const double t0) const
{
	BodyStates states(0.);
    states.name = input.name;
    states.G = make_point(input.dynamics.centre_of_inertia);
    states.hydrodynamic_forces_calculation_point = make_point(input.dynamics.hydrodynamic_forces_calculation_point_in_body_frame, input.name);
    states.x_relative_to_mesh = input.position_of_body_frame_relative_to_mesh.coordinates.x;
    states.y_relative_to_mesh = input.position_of_body_frame_relative_to_mesh.coordinates.y;
    states.z_relative_to_mesh = input.position_of_body_frame_relative_to_mesh.coordinates.z;
    states.mesh_to_body = angle2matrix(input.position_of_body_frame_relative_to_mesh.angle, rotations);
    change_mesh_ref_frame(states, mesh);
    add_inertia(states, input.dynamics.rigid_body_inertia, input.dynamics.added_mass);
    states.u.record(t0, input.initial_velocity_of_body_frame_relative_to_NED_projected_in_body.u);
    states.v.record(t0, input.initial_velocity_of_body_frame_relative_to_NED_projected_in_body.v);
    states.w.record(t0, input.initial_velocity_of_body_frame_relative_to_NED_projected_in_body.w);
    states.p.record(t0, input.initial_velocity_of_body_frame_relative_to_NED_projected_in_body.p);
    states.q.record(t0, input.initial_velocity_of_body_frame_relative_to_NED_projected_in_body.q);
    states.r.record(t0, input.initial_velocity_of_body_frame_relative_to_NED_projected_in_body.r);
    states.low_frequency_velocity.update(states);
    states.intersector = MeshIntersectorPtr(new MeshIntersector(states.mesh));
    states.convention = rotations;
    return states;
}

BodyPtr BodyBuilder::build(const YamlBody& input, const VectorOfVectorOfPoints& mesh, const size_t idx, const double t0, const EnvironmentAndFrames& env) const
{
	BodyStates states = get_initial_states(input, mesh, t0);

    const BlockedDOF blocked_states(input.blocked_dof,idx);
    BodyPtr body(new Body(states,idx,blocked_states));

    std::vector<ForcePtr> forces = get_forces(input, body->get_name(), env, force_parsers);
    double Tmax = get_max_history_length(forces);
    body->set_Tmax(Tmax);
    body->set_forces(forces);

    return body;
}

void BodyBuilder::add_inertia(BodyStates& states, const YamlDynamics6x6Matrix& rigid_body_inertia, const YamlDynamics6x6Matrix& added_mass) const
{
    const Eigen::Matrix<double,6,6> Mrb = convert(rigid_body_inertia);
    if(!isSymmetricDefinitePositive(Mrb))
    {
        THROW(__PRETTY_FUNCTION__, InvalidInputException,
                "The rigid body inertia mass matrix is not symmetric definite positive "
                << "for body '" << states.name << "': " << std::endl
                << "Mrb = " << std::endl
                << Mrb << std::endl);
    }
    Eigen::Matrix<double,6,6> Ma;
    if (added_mass.read_from_file)
    {
        const std::string hdb = ssc::text_file_reader::TextFileReader(std::vector<std::string>(1,added_mass.hdb_filename)).get_contents();
        Ma = HDBParser(hdb).get_added_mass();
    }
    else
    {
        Ma = convert(added_mass);
    }
    if(!isSymmetric(Ma))
    {
        THROW(__PRETTY_FUNCTION__, InvalidInputException,
                "The input added mass is not symmetric"
                << " for body '" << states.name << "': " << std::endl
                << "Ma = " << std::endl
                << Ma << std::endl);
    }
    const Eigen::Matrix<double,6,6> Mt = Mrb + Ma;
    if(!isSymmetricDefinitePositive(Mt))
    {
        THROW(__PRETTY_FUNCTION__, InvalidInputException,
                "The total inertia matrix (rigid body inertia + added mass) is not symmetric definite positive"
                << " for body '" << states.name << "': " << std::endl
                << "Mrb = " << std::endl
                << Mrb << std::endl
                << "Ma = " << std::endl
                << Ma << std::endl
                << "Mrb+Ma = " << std::endl
                << Mt << std::endl);
    }
    Eigen::Matrix<double,6,6> M_inv = Mt.inverse();
    states.inverse_of_the_total_inertia = MatrixPtr(new Eigen::Matrix<double,6,6>(M_inv));
    states.solid_body_inertia = MatrixPtr(new Eigen::Matrix<double,6,6>(Mrb));
    states.total_inertia = MatrixPtr(new Eigen::Matrix<double,6,6>(Mt));
}

Eigen::Matrix<double,6,6> BodyBuilder::convert(const YamlDynamics6x6Matrix& M) const
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

ForcePtr BodyBuilder::parse_force(const YamlModel& model, const std::string body_name, const EnvironmentAndFrames& env, const std::vector<ForceParser>& force_parsers) const
{
	ForcePtr ret;
	bool parsed = false;
	for (auto try_to_parse:force_parsers)
	{
		boost::optional<ForcePtr> f = try_to_parse(model, body_name, env);
		if (f)
		{
			parsed = true;
			return f.get();
			break;
		}
	}

	if (not(parsed))
	{
		THROW(__PRETTY_FUNCTION__, InvalidInputException, "Simulator does not know model '" << model.model << "': maybe the name is misspelt or you are using an outdated version of this simulator.");
	}
    return ret;
}

std::vector<ForcePtr> BodyBuilder::get_forces(const YamlBody& input, const std::string body_name, const EnvironmentAndFrames& env, const std::vector<ForceParser>& force_parsers) const
{
    std::vector<ForcePtr> forces;
    for (const auto force:input.external_forces)
    {
        forces.push_back(parse_force(force, body_name, env, force_parsers));
    }
    for (const auto force:input.controlled_forces)
    {
    	forces.push_back(parse_force(force, body_name, env, force_parsers));
    }
    return forces;
}

double BodyBuilder::get_max_history_length(const std::vector<ForcePtr>& forces) const
{
	double Tmax = 0;
	for (const auto force:forces) Tmax = std::max(Tmax, force->get_Tmax());
	return Tmax;
}

BodyPtr BodyBuilder::build(const std::string& name, const VectorOfVectorOfPoints& mesh, const size_t idx, const double t0, const bool) const
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
    BodyStates states = get_initial_states(input, mesh, t0);

    const BlockedDOF blocked_states(input.blocked_dof,idx);

    return BodyPtr(new Body(states,idx,blocked_states));
}

BodyPtr BodyBuilder::build(const YamlBody& input, const VectorOfVectorOfPoints& mesh, const size_t idx, const double t0, const bool) const
{
	BodyStates states = get_initial_states(input, mesh, t0);
	const BlockedDOF blocked_states(input.blocked_dof,idx);
	return BodyPtr(new Body(states,idx,blocked_states));
}
