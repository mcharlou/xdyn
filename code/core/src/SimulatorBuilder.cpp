/*
 * SimulatorBuilder.cpp
 *
 *  Created on: Jun 16, 2014
 *      Author: cady
 */

#include <vector>
#include <ssc/text_file_reader.hpp>
#include <ssc/data_source/DataSourceModule.hpp>

#include "InternalErrorException.hpp"
#include "update_kinematics.hpp"
#include "stl_reader.hpp"
#include "BodyStatesModule.hpp"
#include "BodyBuilder.hpp"
#include "EnvironmentAndFrames.hpp"

#include "SimulatorBuilder.hpp"

SimulatorBuilder::SimulatorBuilder(const YamlSimulatorInput& input_, const double t0_, const ssc::data_source::DataSource& command_listener_) :
                                        input(input_),
                                        body_builder(std::shared_ptr<BodyBuilder>(new  BodyBuilder(input.rotations))),
                                        surface_elevation_parsers(),
                                        wave_parsers(std::shared_ptr<std::vector<WaveModelBuilderPtr> >(new std::vector<WaveModelBuilderPtr>())),
										directional_spreading_parsers(std::shared_ptr<std::vector<DirectionalSpreadingBuilderPtr> >(new std::vector<DirectionalSpreadingBuilderPtr>())),
										spectrum_parsers(std::shared_ptr<std::vector<SpectrumBuilderPtr> >(new std::vector<SpectrumBuilderPtr>())),
										wind_parsers(),
										wind_mean_velocity_parsers(std::shared_ptr<std::vector<WindMeanVelocityProfileBuilderPtr>>(new std::vector<WindMeanVelocityProfileBuilderPtr>())),
										wind_turbulence_model_parsers(std::shared_ptr<std::vector<WindTurbulenceModelBuilderPtr>>(new std::vector<WindTurbulenceModelBuilderPtr>())),
										command_listener(command_listener_),
										controller_parsers(),
										t0(t0_)
{}

std::vector<BodyPtr> SimulatorBuilder::get_bodies(const MeshMap& meshes, const EnvironmentAndFrames& env) const
{
    std::vector<BodyPtr> ret;
    size_t i = 0;
    for (const auto body:input.bodies)
    {
        const auto that_mesh = meshes.find(body.name);
        if (that_mesh != meshes.end())
        {
            ret.push_back(body_builder->build(body, that_mesh->second, i,t0, env));
            i++;
        }
        else
        {
            THROW(__PRETTY_FUNCTION__, InternalErrorException, "Unable to find mesh for '" << body.name << "' in map.");
        }
    }
    return ret;
}

void SimulatorBuilder::add_initial_transforms(const std::vector<BodyPtr>& bodies, ssc::kinematics::KinematicsPtr& k) const
{
    const StateType x = ::get_initial_states(input.rotations, input.bodies);
    for (size_t i = 0; i < bodies.size(); ++i)
    {
        k->add(bodies.at(i)->get_transform_from_mesh_to_body());
        k->add(bodies.at(i)->get_transform_from_ned_to_body(x));
        k->add(bodies.at(i)->get_transform_from_ned_to_local_ned(x));
    }
}

EnvironmentAndFrames SimulatorBuilder::get_environment() const
{
    EnvironmentAndFrames env;
    env.g = input.environmental_constants.g;
    env.rho = input.environmental_constants.rho;
    env.nu = input.environmental_constants.nu;
    env.air_density = input.environmental_constants.air_density;
    env.rot = input.rotations;
    env.w = get_wave();
    env.k = ssc::kinematics::KinematicsPtr(new ssc::kinematics::Kinematics());
    env.wind = get_wind();
    return env;
}

SurfaceElevationPtr SimulatorBuilder::get_wave() const
{
    if (surface_elevation_parsers.empty())
    {
        THROW(__PRETTY_FUNCTION__, InternalErrorException, "No wave parser defined. Need to call SimulatorBuilder::can_parse<T> with e.g. T=DefaultWaveModel");
    }
    SurfaceElevationPtr ret;
    for (auto that_model=input.environment.begin() ; that_model != input.environment.end() ; ++that_model)
    {
        //bool wave_model_successfully_parsed = false;
        for (auto that_parser=surface_elevation_parsers.begin() ; that_parser != surface_elevation_parsers.end() ; ++that_parser)
        {
            boost::optional<SurfaceElevationPtr> w = (*that_parser)->try_to_parse(that_model->model, that_model->yaml);
            if (w)
            {
                if (ret.use_count())
                {
                    THROW(__PRETTY_FUNCTION__, InternalErrorException, "More than one wave model was defined.");
                }
                ret = w.get();
                //wave_model_successfully_parsed = true;
            }
        }
        /*if (not(wave_model_successfully_parsed))
        {
            THROW(__PRETTY_FUNCTION__, InvalidInputException, "Simulator does not understand wave model '" << that_model->model << "'");
        }*/ // The environment model can also be a wind model ! TODO: Add a way to detect that the input under "environment models" is neither a wave nor a wind model
    }
    return ret;
}

WindModelPtr SimulatorBuilder::get_wind() const
{
    if (wind_parsers.empty())
    {
        THROW(__PRETTY_FUNCTION__, InternalErrorException, "No wind parser defined. Need to call SimulatorBuilder::can_parse<T> with e.g. T=DefaultWindModel");
    }
    WindModelPtr ret;
    for (auto that_model=input.environment.begin() ; that_model != input.environment.end() ; ++that_model)
    {
        //bool wind_model_successfully_parsed = false;
        for (auto that_parser=wind_parsers.begin() ; that_parser != wind_parsers.end() ; ++that_parser)
        {
            boost::optional<WindModelPtr> w = (*that_parser)->try_to_parse(that_model->model, that_model->yaml);
            if (w)
            {
                if (ret.use_count())
                {
                    THROW(__PRETTY_FUNCTION__, InternalErrorException, "More than one wind model was defined.");
                }
                ret = w.get();
                //wind_model_successfully_parsed = true;
            }
        }
        /*if (not(wind_model_successfully_parsed))
        {
            THROW(__PRETTY_FUNCTION__, InvalidInputException, "Simulator does not understand wind model '" << that_model->model << "'");
        }*/
    }
    return ret;
}

void SimulatorBuilder::get_controllers(ssc::data_source::DataSource* const data_source, const std::vector<BodyPtr> bodies) const
{
	if(input.controllers.size()>0)
	{
		data_source->check_in(__PRETTY_FUNCTION__);
		for (BodyPtr body:bodies)
		{
			data_source->add(BodyStatesModule(data_source,body));
		}
		data_source->check_out();
		for (auto that_controller=input.controllers.begin() ; that_controller != input.controllers.end() ; ++that_controller)
		{
			add_controller(*that_controller, data_source);
		}
		/*std::cout << "Dependencies graph: " << std::endl;
		std::cout << data_source->draw() << std::endl;*/
	}
}

template <typename T> bool could_parse(const std::vector<T>& parsers, const YamlModel& model, const std::string& body_name, const EnvironmentAndFrames& env)
{
    try
    {
        for (auto try_to_parse:parsers)
        {
            auto f = try_to_parse(model, body_name, env);
            if (f)
            {
                return true;
            }
        }
    }
    catch (const InvalidInputException&)
    {
        return true;
    }
    catch (...)
    {
        return false;
    }
    return false;
}

void SimulatorBuilder::add_controller(const YamlModel& model, ssc::data_source::DataSource* const data_source) const
{
    bool parsed = false;
    data_source->check_in(__PRETTY_FUNCTION__);
    for (auto try_to_parse:controller_parsers)
    {
        boost::optional<ControllerPtr> f = try_to_parse(model, data_source);
        if (f)
        {
            data_source->add(*(f.get()));
            parsed = true;
            break;
        }
    }
    if (not(parsed))
    {
        THROW(__PRETTY_FUNCTION__, InvalidInputException, "Simulator does not know controller model '" << model.model << "': maybe the name is misspelt or you are using an outdated version of this simulator.");
    }
    data_source->check_out();
}

Sim SimulatorBuilder::build(const MeshMap& meshes)
{
    auto env = get_environment();
    const auto bodies = get_bodies(meshes, env);
    add_initial_transforms(bodies, env.k);
    get_controllers(&command_listener,bodies);
    return Sim(bodies, env, get_initial_states(), command_listener);
}

StateType SimulatorBuilder::get_initial_states() const
{
    return ::get_initial_states(input.rotations, input.bodies);
}

Sim SimulatorBuilder::build()
{
    return build(make_mesh_map());
}

YamlSimulatorInput SimulatorBuilder::get_parsed_yaml() const
{
    return input;
}

MeshMap SimulatorBuilder::make_mesh_map() const
{
    MeshMap ret;
    for (auto that_body = input.bodies.begin() ; that_body != input.bodies.end() ; ++that_body)
    {
        ret[that_body->name] = get_mesh(*that_body);
    }
    return ret;
}

VectorOfVectorOfPoints SimulatorBuilder::get_mesh(const YamlBody& body) const
{
    if (not(body.mesh.empty()))
    {
        const ssc::text_file_reader::TextFileReader reader(body.mesh);
        return read_stl(reader.get_contents());
    }
    return VectorOfVectorOfPoints();
}
