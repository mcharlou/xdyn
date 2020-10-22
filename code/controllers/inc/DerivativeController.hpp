/*
 * DerivativeController.hpp
 *
 *  Created on: 20 févr. 2020
 *      Author: mcharlou2016
 */

#ifndef CONTROLLERS_INC_DERIVATIVECONTROLLER_HPP_
#define CONTROLLERS_INC_DERIVATIVECONTROLLER_HPP_

#include "AbstractController.hpp"

class DerivativeController : public AbstractController
{
public:
	struct Yaml
	{
		Yaml();
		virtual ~Yaml(){}
		std::string name;
		std::string command;
		std::string output;
		std::string probe;
		double Kd;
	};
	static Yaml parse(const std::string& yaml);
	static std::string model_name();
	DerivativeController(const Yaml& input, ssc::data_source::DataSource* const data_source);
	DerivativeController(const DerivativeController& rhs, DataSource* const data_source) : AbstractController(rhs, data_source), pimpl(rhs.pimpl)
	{}

	DataSourceModule* clone() const
	{
		return new DerivativeController(*this);
	}

	DataSourceModule* clone(DataSource* const data_source) const
	{
		return new DerivativeController(*this, data_source);
	}

	virtual void update() const;
	virtual std::string get_name() const;
	virtual std::string get_signal_name() const;

private:
	DerivativeController(); // Default constructor is disabled
	class Impl;
	std::shared_ptr<Impl> pimpl;
};

#endif /* CONTROLLERS_INC_DERIVATIVECONTROLLER_HPP_ */
