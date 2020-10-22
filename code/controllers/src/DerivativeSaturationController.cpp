/*
 * DerivativeSaturationController.cpp
 *
 *  Created on: 5 mai 2020
 *      Author: mcharlou2016
 */

#include <iostream>
#include "yaml.h"
#include <ssc/yaml_parser.hpp>
#include <ssc/data_source.hpp>

#include "DerivativeSaturationController.hpp"

std::string DerivativeSaturationController::model_name() {return "derivative saturation";}

DerivativeSaturationController::Yaml::Yaml(): name(), input(), output(), upper_bound(), lower_bound() {}

DerivativeSaturationController::Yaml DerivativeSaturationController::parse(const std::string& yaml)
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

class DerivativeSaturationController::Impl
{
	public:
		Impl(const Yaml& input): name(input.name),input(input.input), output(input.output), upper_bound(input.upper_bound),lower_bound(input.lower_bound), t_prev(), signal_prev()
		{}

		std::string namify(const std::string& name, const std::string& output) const
		{
			return name+"("+output+")";
		}

		void update(ssc::data_source::DataSource* const data_source)
		{
			double t = data_source->get<double>("t");
			double signal_in = data_source->get<double>(input);
			double signal_out;
			double Dt = t - t_prev;
			if(Dt>0)
			{
				double derivate = (signal_in-signal_prev)/Dt;
				if(derivate>upper_bound) signal_out = signal_prev + upper_bound*Dt;
				else if(derivate<lower_bound) signal_out = signal_prev + lower_bound*Dt;
				else signal_out = signal_in;
			}
			else
			{
				signal_out = signal_prev;
			}
			//std::cout << "Updating SaturationController with in: " << signal_in << " and out: " << signal_out << std::endl;
			data_source->set<double>(namify(name,output),signal_out);
			t_prev=t;
			signal_prev=signal_out;
		}

		std::string get_output_signal_name() const
		{
			return namify(name,output);
		}

		std::string get_name() const
		{
			return name+"("+output+")"+"_DerivativeSaturationController";
		}

	private:
		Impl();
		std::string name;
		std::string input;
		std::string output;
		double upper_bound;
		double lower_bound;
		double t_prev;
		double signal_prev;
};

DerivativeSaturationController::DerivativeSaturationController(const Yaml& input, ssc::data_source::DataSource* const data_source) : AbstractController(data_source, input.name+"("+input.output+")"+"_DerivativeSaturationController"), pimpl(new Impl(input))
{}

void DerivativeSaturationController::update() const
{
	pimpl->update(ds);
}

std::string DerivativeSaturationController::get_name() const
{
	return pimpl->get_name();
}

std::string DerivativeSaturationController::get_signal_name() const
{
	return pimpl->get_output_signal_name();
}
