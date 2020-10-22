/*
 * AntiWindUpPIDController.hpp
 *
 *  Created on: 21 févr. 2020
 *      Author: mcharlou2016
 */

#ifndef CONTROLLERS_INC_ANTIWINDUPPIDCONTROLLER_HPP_
#define CONTROLLERS_INC_ANTIWINDUPPIDCONTROLLER_HPP_

#include "AbstractController.hpp"

class AntiWindUpPIDController : public AbstractController
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
		double tau_i;
		double tau_d;
		double upper_bound;
		double lower_bound;
		bool velocity_algorithm;
	};
	static Yaml parse(const std::string& yaml);
	static std::string model_name();
	AntiWindUpPIDController(const Yaml& input, ssc::data_source::DataSource* const data_source);
	AntiWindUpPIDController(const AntiWindUpPIDController& rhs, DataSource* const data_source) : AbstractController(rhs, data_source), pimpl(rhs.pimpl)
	{}

	DataSourceModule* clone() const
	{
		return new AntiWindUpPIDController(*this);
	}

	DataSourceModule* clone(DataSource* const data_source) const
	{
		return new AntiWindUpPIDController(*this, data_source);
	}

	virtual void update() const;
	virtual std::string get_name() const;
	virtual std::string get_signal_name() const;

private:
	AntiWindUpPIDController(); // Default constructor is disabled
	class Impl;
	std::shared_ptr<Impl> pimpl;
};

#endif /* CONTROLLERS_INC_ANTIWINDUPPIDCONTROLLER_HPP_ */
