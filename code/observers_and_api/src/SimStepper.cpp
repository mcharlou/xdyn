#include <functional>

#include <ssc/kinematics.hpp>
#include "InvalidInputException.hpp"
#include "SimServerInputs.hpp"
#include "SimStepper.hpp"
#include "simulator_api.hpp"

SimStepper::SimStepper(const ConfBuilder& builder, const std::string& solver, const double dt)
    : sim(builder.sim)
    , solver(solver)
    , dt(dt)
	, current_time(0.)
{
}

YamlState convert_without_angles(const Res& res);
YamlState convert_without_angles(const Res& res)
{
    YamlState ret;
    ret.t     = res.t;
    ret.x     = res.x[0];
    ret.y     = res.x[1];
    ret.z     = res.x[2];
    ret.u     = res.x[3];
    ret.v     = res.x[4];
    ret.w     = res.x[5];
    ret.p     = res.x[6];
    ret.q     = res.x[7];
    ret.r     = res.x[8];
    ret.qr    = res.x[9];
    ret.qi    = res.x[10];
    ret.qj    = res.x[11];
    ret.qk    = res.x[12];
    ret.phi   = 0;
    ret.theta = 0;
    ret.psi   = 0;
    ret.extra_observations = res.extra_observations;
    return ret;
}

std::function<YamlState(const Res&)> convert_with_angles(const BodyPtr& body);
std::function<YamlState(const Res&)> convert_with_angles(const BodyPtr& body)
{
    return [&body](const Res& res)
            {
                YamlState ret = convert_without_angles(res);
                /*const State new_state(ret,0);
                body->set_states_history(new_state);
                const auto angles = body->get_states().get_angles();*/
                std::tuple<double,double,double,double> quat(ret.qr,ret.qi,ret.qj,ret.qk);
                ssc::kinematics::EulerAngles angles = body->get_states().convert(quat);
                ret.phi = angles.phi;
                ret.theta = angles.theta;
                ret.psi = angles.psi;
                return ret;
            };
}

std::vector<YamlState> SimStepper::step(const SimServerInputs& infos, double Dt)
{
    double tstart;
    const std::vector<State> states = {infos.full_state_history};
    if(infos.reset_history)
    {
    	sim.reset_history();
    	sim.set_bodystates(states);
    	tstart = infos.t;
    }
    else
    {
    	tstart = current_time;
    }
    sim.set_command_listener(infos.commands);
    sim.update_forces(tstart);
    std::vector<Res> results;

    if(solver == "euler")
    {
        results = simulate<ssc::solver::EulerStepper>(sim, tstart, tstart+Dt, dt);
    }
    else if (solver == "rk4")
    {
        results = simulate<ssc::solver::RK4Stepper>(sim, tstart, tstart+Dt, dt);
    }
    else if (solver == "rkck")
    {
        results = simulate<ssc::solver::RKCK>(sim, tstart, tstart+Dt, dt);
    }
    else
    {
        THROW(__PRETTY_FUNCTION__, InvalidInputException, "unknown solver");
    }

    /*std::vector<double> x;
    for(auto state:results){
    	x.push_back(state.x[0]);
    }*/

    current_time = results.back().t;

    std::vector<YamlState> ret(results.size());
    if (not(sim.get_bodies().empty()))
    {
        std::transform(results.begin(), results.end(), ret.begin(), convert_with_angles(sim.get_bodies().at(0)));
    }
    else
    {
        std::transform(results.begin(), results.end(), ret.begin(), convert_without_angles);
    }

    return ret;
}

YamlState get_Tmax(const Sim& sim);
YamlState get_Tmax(const Sim& sim)
{
	YamlState ret;
	ret.extra_observations["Tmax"] = sim.get_bodies().at(0)->get_Tmax();
	return ret;
}

YamlState euler2quaternion(const Sim& sim, const std::map<std::string, double>& commands);
YamlState euler2quaternion(const Sim& sim, const std::map<std::string, double>& commands)
{
	YamlState ret;
	if(commands.find("phi")!=commands.end() and commands.find("theta")!=commands.end() and commands.find("psi")!=commands.end())
	{
		ssc::kinematics::EulerAngles euler_angles(commands.at("phi"),commands.at("theta"),commands.at("psi"));
		std::tuple<double,double,double,double> quat = sim.get_bodies()[0]->get_states().convert(euler_angles,sim.get_bodies()[0]->get_states().convention);
		ret.qr = std::get<0>(quat);
		ret.qi = std::get<1>(quat);
		ret.qj = std::get<2>(quat);
		ret.qk = std::get<3>(quat);
		ret.extra_observations["qr"] = std::get<0>(quat);
		ret.extra_observations["qi"] = std::get<1>(quat);
		ret.extra_observations["qj"] = std::get<2>(quat);
		ret.extra_observations["qk"] = std::get<3>(quat);
	}
	else
	{
		THROW(__PRETTY_FUNCTION__, InvalidInputException, "The threes values 'phi', 'theta' and 'psi' must be present in 'commands' in order to convert them to a quaternion.");
	}
	return ret;
}

YamlState quaternion2euler(const Sim& sim, const StateType& states);
YamlState quaternion2euler(const Sim& sim, const StateType& states)
{
	YamlState ret;
	std::tuple<double,double,double,double> quat(states[9],states[10],states[11],states[12]);
	ssc::kinematics::EulerAngles euler_angles = sim.get_bodies()[0]->get_states().convert(quat,sim.get_bodies()[0]->get_states().convention);
	ret.phi = euler_angles.phi;
	ret.theta = euler_angles.theta;
	ret.psi = euler_angles.psi;
	ret.extra_observations["phi"] = euler_angles.phi;
	ret.extra_observations["theta"] = euler_angles.theta;
	ret.extra_observations["psi"] = euler_angles.psi;
	return ret;
}

YamlState force_signals(Sim& sim, const std::map<std::string, double>& commands);
YamlState force_signals(Sim& sim, const std::map<std::string, double>& commands)
{
	sim.force_commands(commands);
	YamlState ret;
	return ret;
}

std::vector<YamlState> SimStepper::handle_request(const SimServerInputs& input)
{
	switch(input.request)
	{
		case Request::get_Tmax:
			return std::vector<YamlState>({get_Tmax(sim)});
		case Request::euler2quaternion:
			return std::vector<YamlState>({euler2quaternion(sim,input.commands)});
		case Request::quaternion2euler:
			return std::vector<YamlState>({quaternion2euler(sim,input.state_at_t)});
		case Request::force_signals:
			return std::vector<YamlState>({force_signals(sim,input.commands)});
		default:
			THROW(__PRETTY_FUNCTION__, InvalidInputException, "unknown request")
	}
}
