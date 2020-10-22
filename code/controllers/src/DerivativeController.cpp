/*
 * DerivativeController.cpp
 *
 *  Created on: 20 févr. 2020
 *      Author: mcharlou2016
 */

#include <iostream>
#include "yaml.h"
#include <ssc/yaml_parser.hpp>
#include <ssc/data_source.hpp>

#include "DerivativeController.hpp"

std::string DerivativeController::model_name() {return "derivative";}

DerivativeController::Yaml::Yaml(): name(), command(), output(), probe(), Kd() {}

DerivativeController::Yaml DerivativeController::parse(const std::string& yaml)
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
	node["Kd"]      >> ret.Kd;

	return ret;
}

class DerivativeController::Impl
{
	public:
		Impl(const Yaml& input): name(input.name), command(input.command), output(input.output), probe(input.probe), Kd(input.Kd), t_prev(), e_prev()
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
			double D = ((t-t_prev)>0 ? (e-e_prev)/(t-t_prev) : 0);
			double control = Kd*D;
			data_source->set<double>(namify(name,output),control);
			t_prev=t;
			e_prev=e;
		}

		std::string get_output_signal_name() const
		{
			return namify(name,output);
		}

		std::string get_name() const
		{
			return name+"_DerivativeController";
		}

	private:
		Impl();
		std::string name;
		std::string command;
		std::string output;
		std::string probe;
		double Kd;
		double t_prev;
		double e_prev;
};

DerivativeController::DerivativeController(const Yaml& input, ssc::data_source::DataSource* const data_source) : AbstractController(data_source, input.name+"_DerivativeController"), pimpl(new Impl(input))
{}

void DerivativeController::update() const
{
	pimpl->update(ds);
}

std::string DerivativeController::get_name() const
{
	return pimpl->get_name();
}

std::string DerivativeController::get_signal_name() const
{
	return pimpl->get_output_signal_name();
}
