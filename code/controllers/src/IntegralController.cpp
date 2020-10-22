/*
 * IntegralController.cpp
 *
 *  Created on: 20 févr. 2020
 *      Author: mcharlou2016
 */

#include <iostream>
#include "yaml.h"
#include <ssc/yaml_parser.hpp>
#include <ssc/data_source.hpp>

#include "History.hpp"

#include "IntegralController.hpp"

std::string IntegralController::model_name() {return "integral";}

IntegralController::Yaml::Yaml(): name(), command(), output(), probe(), Ki(), Tmax() {}

IntegralController::Yaml IntegralController::parse(const std::string& yaml)
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
	node["Ki"]      >> ret.Ki;
	node["Tmax"]    >> ret.Tmax;

	return ret;
}

class IntegralController::Impl
{
	public:
		Impl(const Yaml& input): name(input.name), command(input.command), output(input.output), probe(input.probe), Ki(input.Ki), Tmax(input.Tmax), eHistory(input.Tmax)
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
			eHistory.record(t,e);
			double control = Ki*eHistory.integrate_over_duration();
			data_source->set<double>(namify(name,output),control);
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
		double Ki;
		double Tmax;
		History eHistory;
};

IntegralController::IntegralController(const Yaml& input, ssc::data_source::DataSource* const data_source) : AbstractController(data_source, input.name+"_IntegralController"), pimpl(new Impl(input))
{}

void IntegralController::update() const
{
	pimpl->update(ds);
}

std::string IntegralController::get_name() const
{
	return pimpl->get_name();
}

std::string IntegralController::get_signal_name() const
{
	return pimpl->get_output_signal_name();
}
