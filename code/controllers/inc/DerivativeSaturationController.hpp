/*
 * DerivativeSaturationController.hpp
 *
 *  Created on: 5 mai 2020
 *      Author: mcharlou2016
 */

#ifndef CONTROLLERS_INC_DERIVATIVESATURATIONCONTROLLER_HPP_
#define CONTROLLERS_INC_DERIVATIVESATURATIONCONTROLLER_HPP_

#include <string>

#include "AbstractController.hpp"

class DerivativeSaturationController : public AbstractController
{
public:
	struct Yaml
	{
		Yaml();
		virtual ~Yaml(){}
		std::string name;
		std::string input;
		std::string output;
		double upper_bound;
		double lower_bound;
	};
	static Yaml parse(const std::string& yaml);
	static std::string model_name();
	DerivativeSaturationController(const Yaml& input, ssc::data_source::DataSource* const data_source);
	DerivativeSaturationController(const DerivativeSaturationController& rhs, DataSource* const data_source) : AbstractController(rhs, data_source), pimpl(rhs.pimpl)
	{}

	DataSourceModule* clone() const
	{
		return new DerivativeSaturationController(*this);
	}

	DataSourceModule* clone(DataSource* const data_source) const
	{
		return new DerivativeSaturationController(*this, data_source);
	}

	virtual void update() const;
	virtual std::string get_name() const;
	virtual std::string get_signal_name() const;

private:
	DerivativeSaturationController(); // Default constructor is disabled
	class Impl;
	std::shared_ptr<Impl> pimpl;

};

#endif /* CONTROLLERS_INC_DERIVATIVESATURATIONCONTROLLER_HPP_ */
