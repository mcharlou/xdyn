/*
 * PIDController.cpp
 *
 *  Created on: 20 févr. 2020
 *      Author: mcharlou2016
 */



#include <iostream>
#include "yaml.h"
#include <ssc/yaml_parser.hpp>
#include <ssc/data_source.hpp>

#include "History.hpp"

#include "PIDController.hpp"

std::string PIDController::model_name() {return "PID";}

PIDController::Yaml::Yaml(): name(), command(), output(), probe(), Kp(), Ki(), Tmax(), Kd(), velocity_algorithm() {}

PIDController::Yaml PIDController::parse(const std::string& yaml)
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
	node["Kp"]		>> ret.Kp;
	node["Ki"]      >> ret.Ki;
	if(!ret.velocity_algorithm) node["Tmax"]    >> ret.Tmax;
	node["Kd"]      >> ret.Kd;


	return ret;
}

class PIDController::Impl
{
	public:
		Impl(const Yaml& input): name(input.name), command(input.command), output(input.output), probe(input.probe),
				Kp(input.Kp), Ki(input.Ki), Tmax(input.Tmax), eHistory(input.Tmax), Kd(input.Kd), t_prev(0), e_prev(0),
				e_prev_prev(0), control_prev(0), velocity_algorithm(input.velocity_algorithm)
		{}

		std::string namify(const std::string& name, const std::string& output) const
		{
			return name+"("+output+")";
		}

		void update(ssc::data_source::DataSource* const data_source)
		{
			double t = data_source->get<double>("t");// So the DataSource knows that this modules requires an update when t changes.
			double signal_in = data_source->get<double>(command);
			double signal_out = data_source->get<double>(probe);
			double e = signal_in-signal_out;
			double control;

			if(!velocity_algorithm)
			{
				eHistory.record(t,e);
				double I = eHistory.integrate_over_duration();
				double D = (t>0 ? (e-e_prev)/(t-t_prev) : 0);
				control = Kp*e+Ki*I+Kd*D;
				//std::cout << "P/I/D: " << Kp*e/control << "/" << Ki*I/control << "/" << Kd*D/control << std::endl;
			}
			else
			{
				double T0=t-t_prev;
				/*double q0=(t>0 ? Kp+Kd/T0 : Kp);
				double q1=(t>0 ? -Kp-2*Kd/T0+Ki*T0 : 0);
				double q2=(t>0 ? Kd/T0 : 0);
				control = control_prev + q0*e + q1*e_prev + q2*e_prev_prev;*/
				//std::cout << "T0: " << T0 << ", e(k-2): " << e_prev_prev << ", e(k-1): " << e_prev << ", e(k): " << e << std::endl;
				double P = e-e_prev;
				double I = e_prev*T0;
				double D = (t-T0>0 ? (e-2*e_prev+e_prev_prev)/T0 : (T0>0 ? (e-e_prev)/T0 : 0));
				double d_control = Kp*P + Ki*I + Kd*D;
				control = control_prev + d_control;
				//std::cout << "P/I/D: " << Kp*P/d_control << "/" << Ki*I/d_control << "/" << Kd*D/d_control << std::endl;
				//std::cout << "u(k-1): " << control_prev << ", q0: " << q0 << ", q1: " << q1 << ", q2: " << q2 << std::endl;
			}
			//std::cout << "Error: " << e << ", command: " << control << std::endl;
			data_source->set<double>(namify(name,output),control);
			t_prev=t;
			e_prev_prev=e_prev;
			e_prev=e;
			control_prev=control;
		}

		std::string get_output_signal_name() const
		{
			return namify(name,output);
		}

		std::string get_name() const
		{
			return name+"_IntegralController";
		}

	private:
		Impl();
		std::string name;
		std::string command;
		std::string output;
		std::string probe;
		double Kp;
		double Ki;
		double Tmax;
		History eHistory;
		double Kd;
		double t_prev;
		double e_prev;
		double e_prev_prev;
		double control_prev;
		bool velocity_algorithm;
};

PIDController::PIDController(const Yaml& input, ssc::data_source::DataSource* const data_source) : AbstractController(data_source, input.name+"_IntegralController"), pimpl(new Impl(input))
{}

void PIDController::update() const
{
	pimpl->update(ds);
}

std::string PIDController::get_name() const
{
	return pimpl->get_name();
}

std::string PIDController::get_signal_name() const
{
	return pimpl->get_output_signal_name();
}
