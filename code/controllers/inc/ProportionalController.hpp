/*
 * ProportionalController.hpp
 *
 *  Created on: 18 févr. 2020
 *      Author: mcharlou2016
 */

#ifndef CONTROLLERS_PROPORTIONALCONTROLLER_HPP_
#define CONTROLLERS_PROPORTIONALCONTROLLER_HPP_

#include <string>

#include "AbstractController.hpp"

class ProportionalController : public AbstractController
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
	};
	static Yaml parse(const std::string& yaml);
	static std::string model_name();
	ProportionalController(const Yaml& input, ssc::data_source::DataSource* const data_source);
	ProportionalController(const ProportionalController& rhs, DataSource* const data_source) : AbstractController(rhs, data_source), pimpl(rhs.pimpl)
	{}

	DataSourceModule* clone() const
	{
		return new ProportionalController(*this);
	}

	DataSourceModule* clone(DataSource* const data_source) const
	{
		return new ProportionalController(*this, data_source);
	}

	virtual void update() const;
	virtual std::string get_name() const;
	virtual std::string get_signal_name() const;

private:
	ProportionalController (); // Default constructor is disabled
	class Impl;
	std::shared_ptr<Impl> pimpl;
};

#endif /* CONTROLLERS_PROPORTIONALCONTROLLER_HPP_ */
