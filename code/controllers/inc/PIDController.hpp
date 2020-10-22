/*
 * PIDController.hpp
 *
 *  Created on: 20 févr. 2020
 *      Author: mcharlou2016
 */

#ifndef CONTROLLERS_INC_PIDCONTROLLER_HPP_
#define CONTROLLERS_INC_PIDCONTROLLER_HPP_

#include "AbstractController.hpp"

class PIDController : public AbstractController
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
		double Kp;
		double Ki;
		double Tmax;
		double Kd;
		bool velocity_algorithm;
	};
	static Yaml parse(const std::string& yaml);
	static std::string model_name();
	PIDController(const Yaml& input, ssc::data_source::DataSource* const data_source);
	PIDController(const PIDController& rhs, DataSource* const data_source) : AbstractController(rhs, data_source), pimpl(rhs.pimpl)
	{}

	DataSourceModule* clone() const
	{
		return new PIDController(*this);
	}

	DataSourceModule* clone(DataSource* const data_source) const
	{
		return new PIDController(*this, data_source);
	}

	virtual void update() const;
	virtual std::string get_name() const;
	virtual std::string get_signal_name() const;

private:
	PIDController(); // Default constructor is disabled
	class Impl;
	std::shared_ptr<Impl> pimpl;
};

#endif /* CONTROLLERS_INC_PIDCONTROLLER_HPP_ */
