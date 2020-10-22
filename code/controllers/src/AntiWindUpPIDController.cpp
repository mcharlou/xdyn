/*
 * AntiWindUpPIDController.cpp
 *
 *  Created on: 21 févr. 2020
 *      Author: mcharlou2016
 */

#include <iostream>
#include "yaml.h"
#include <ssc/yaml_parser.hpp>
#include <ssc/data_source.hpp>

#include "History.hpp"

#include "AntiWindUpPIDController.hpp"

std::string AntiWindUpPIDController::model_name() {return "PID w/ saturation & anti wind-up";}

AntiWindUpPIDController::Yaml::Yaml(): name(), command(), output(), probe(), Kp(), tau_i(), tau_d(), upper_bound(), lower_bound(), velocity_algorithm() {}

AntiWindUpPIDController::Yaml AntiWindUpPIDController::parse(const std::string& yaml)
{
	std::stringstream stream(yaml);
	YAML::Parser parser(stream);
	YAML::Node node;
	parser.GetNextDocument(node);
	Yaml ret;

	node["name"]    >> ret.name;
	node["command"] >> ret.command;
	node["output"]  >> ret.output;
	node["probe"]   >> ret.probe;
	node["velocity algorithm"] >> ret.velocity_algorithm;
	ssc::yaml_parser::parse_uv(node["Kp"], ret.Kp);
	node["tau_i"]      >> ret.tau_i;
	if(ret.tau_i==0)
	{
		std::cout << "WARNING: Setting tau_i=0 in the PID controller results in an infinite integral action. This is most likely unwanted." << std::endl;
	}
	node["tau_d"]      >> ret.tau_d;
	ssc::yaml_parser::parse_uv(node["upper bound"], ret.upper_bound);
	ssc::yaml_parser::parse_uv(node["lower bound"], ret.lower_bound);

	return ret;
}

class AntiWindUpPIDController::Impl
{
	public:
		Impl(const Yaml& input): name(input.name), command(input.command), output(input.output), probe(input.probe),
				Kp(input.Kp), Ki(input.Kp/input.tau_i), IntE(0), Kd(input.Kp*input.tau_d), t_prev(0), e_prev(0), e_prev_prev(0), control_prev(0),
				velocity_algorithm(input.velocity_algorithm), upper_bound(input.upper_bound),lower_bound(input.lower_bound),
				saturated(false)
		{}

		std::string namify(const std::string& name, const std::string& output) const
		{
			return name+"("+output+")";
		}

		void update(ssc::data_source::DataSource* const data_source)
		{
			double t = data_source->get<double>("t");
			double signal_in = data_source->get<double>(command);
			double signal_out = data_source->get<double>(probe);
			double e = signal_in-signal_out;
			double control;
			double T0=t-t_prev;


			if(!velocity_algorithm)
			{
				if(!saturated) IntE += (e+e_prev)*T0/2 ;
				double I = IntE;
				double D = (t>0 ? (e-e_prev)/(t-t_prev) : 0);
				control = Kp*e+Ki*I+Kd*D;
				//std::cout << "P/I/D: " << Kp*e/control << "/" << Ki*I/control << "/" << Kd*D/control << std::endl;
			}
			else
			{
				//std::cout << "T0: " << T0 << ", e(k-2): " << e_prev_prev << ", e(k-1): " << e_prev << ", e(k): " << e << std::endl;
				double P = e-e_prev;
				double I = e_prev*T0;
				double D = (t-T0>0 ? (e-2*e_prev+e_prev_prev)/T0 : (T0>0 ? (e-e_prev)/T0 : 0));
				double d_control = Kp*P + Kd*D;
				if(!saturated) d_control += Ki*I;
				control = control_prev + d_control;
				//std::cout << "P/I/D: " << Kp*P/d_control << "/" << Ki*I/d_control << "/" << Kd*D/d_control << std::endl;
				//std::cout << "u(k-1): " << control_prev << ", q0: " << q0 << ", q1: " << q1 << ", q2: " << q2 << std::endl;
			}
			control_prev=control;
			if(control>upper_bound)
			{
				control=upper_bound;
				saturated=true;
			}
			else if(control<lower_bound)
			{
				control=lower_bound;
				saturated=true;
			}
			else saturated=false;
			//std::cout << "Error: " << e << ", command: " << control << ", saturated: " << saturated << std::endl;
			data_source->set<double>(namify(name,output),control);
			t_prev=t;
			e_prev_prev=e_prev;
			e_prev=e;
		}

		std::string get_output_signal_name() const
		{
			return namify(name,output);
		}

		std::string get_name() const
		{
			return namify(name,output)+"_AntiWindUpPIDController";
		}

	private:
		Impl();
		const std::string name;
		const std::string command;
		const std::string output;
		const std::string probe;
		const double Kp;
		const double Ki;
		double IntE;
		const double Kd;
		double t_prev;
		double e_prev;
		double e_prev_prev;
		double control_prev;
		const bool velocity_algorithm;
		const double upper_bound;
		const double lower_bound;
		bool saturated;
};

AntiWindUpPIDController::AntiWindUpPIDController(const Yaml& input, ssc::data_source::DataSource* const data_source) : AbstractController(data_source, input.name+"("+input.output+")"+"_AntiWindUpPIDController"), pimpl(new Impl(input))
{}

void AntiWindUpPIDController::update() const
{
	pimpl->update(ds);
}

std::string AntiWindUpPIDController::get_name() const
{
	return pimpl->get_name();
}

std::string AntiWindUpPIDController::get_signal_name() const
{
	return pimpl->get_output_signal_name();
}

