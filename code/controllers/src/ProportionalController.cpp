/*
 * ProportionalController.cpp
 *
 *  Created on: 18 févr. 2020
 *      Author: mcharlou2016
 */

#include <iostream>
#include "yaml.h"
#include <ssc/yaml_parser.hpp>
#include <ssc/data_source.hpp>

#include "ProportionalController.hpp"

std::string ProportionalController::model_name() {return "proportional";}

ProportionalController::Yaml::Yaml(): name(), command(), output(), probe(), Kp() {}

ProportionalController::Yaml ProportionalController::parse(const std::string& yaml)
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
	node["Kp"]      >> ret.Kp;

	return ret;
}

class ProportionalController::Impl
{
	public:
		Impl(const Yaml& input): name(input.name), command(input.command), output(input.output), probe(input.probe), Kp(input.Kp)
		{}

		std::string namify(const std::string& name, const std::string& output) const
		{
			return name+"("+output+")";
		}

		void update(ssc::data_source::DataSource* const data_source) const
		{
			data_source->get<double>("t");// So the DataSource knows that this modules requires an update when t changes.
			double signal_in = data_source->get<double>(command);
			double signal_out = data_source->get<double>(probe);
			double control = Kp*(signal_in-signal_out);
			//std::cout << "Updating ProportionalController with e=" << signal_in << "-" << signal_out << "=" << signal_in-signal_out << " and control=" << control << std::endl;
			data_source->set<double>(namify(name,output),control);
		}

		std::string get_output_signal_name() const
		{
			return namify(name,output);
		}

		std::string get_name() const
		{
			return name+"_ProportionalController";
		}

	private:
		Impl();
		std::string name;
		std::string command;
		std::string output;
		std::string probe;
		double Kp;
};

ProportionalController::ProportionalController(const Yaml& input, ssc::data_source::DataSource* const data_source) : AbstractController(data_source, input.name+"_ProportionalController"), pimpl(new Impl(input))
{}

void ProportionalController::update() const
{
	pimpl->update(ds);
}

std::string ProportionalController::get_name() const
{
	return pimpl->get_name();
}

std::string ProportionalController::get_signal_name() const
{
	return pimpl->get_output_signal_name();
}
