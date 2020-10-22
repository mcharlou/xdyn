/*
 * SaturationController.cpp
 *
 *  Created on: 17 févr. 2020
 *      Author: mcharlou2016
 */

#include <iostream>
#include "yaml.h"
#include <ssc/yaml_parser.hpp>
#include <ssc/data_source.hpp>

#include "SaturationController.hpp"

std::string SaturationController::model_name() {return "saturation";}

SaturationController::Yaml::Yaml(): name(), input(), output(), upper_bound(), lower_bound() {}

SaturationController::Yaml SaturationController::parse(const std::string& yaml)
{
	std::stringstream stream(yaml);
	YAML::Parser parser(stream);
	YAML::Node node;
	parser.GetNextDocument(node);
	Yaml ret;

	node["name"]        >> ret.name;
	node["input signal"]>> ret.input;
	node["output"]>> ret.output;
	ssc::yaml_parser::parse_uv(node["upper bound"], ret.upper_bound);
	ssc::yaml_parser::parse_uv(node["lower bound"], ret.lower_bound);

	return ret;
}

class SaturationController::Impl
{
	public:
		Impl(const Yaml& input): name(input.name),input(input.input), output(input.output), upper_bound(input.upper_bound),lower_bound(input.lower_bound)
		{}

		std::string namify(const std::string& name, const std::string& output) const
		{
			return name+"("+output+")";
		}

		void update(ssc::data_source::DataSource* const data_source) const
		{
			data_source->get<double>("t");// So the DataSource knows that this modules requires an update when t changes.
			double signal_in = data_source->get<double>(input);
			double signal_out = signal_in;
			if(signal_in>upper_bound) signal_out=upper_bound;
			else if(signal_in<lower_bound) signal_out=lower_bound;
			//std::cout << "Updating SaturationController with in: " << signal_in << " and out: " << signal_out << std::endl;
			data_source->set<double>(namify(name,output),signal_out);
		}

		std::string get_output_signal_name() const
		{
			return namify(name,output);
		}

		std::string get_name() const
		{
			return name+"_SaturationController";
		}

	private:
		Impl();
		std::string name;
		std::string input;
		std::string output;
		double upper_bound;
		double lower_bound;
};

SaturationController::SaturationController(const Yaml& input, ssc::data_source::DataSource* const data_source) : AbstractController(data_source, input.name+"_SaturationController"), pimpl(new Impl(input))
{}

void SaturationController::update() const
{
	pimpl->update(ds);
}

std::string SaturationController::get_name() const
{
	return pimpl->get_name();
}

std::string SaturationController::get_signal_name() const
{
	return pimpl->get_output_signal_name();
}
