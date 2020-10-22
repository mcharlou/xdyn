/*
 * BodySatesModule.hpp
 *
 *  Created on: 18 févr. 2020
 *      Author: mcharlou2016
 */

#ifndef CONTROLLERS_INC_BODYSTATESMODULE_HPP_
#define CONTROLLERS_INC_BODYSTATESMODULE_HPP_

#include <memory>
#include <ssc/data_source/DataSourceModule.hpp>
#include <ssc/data_source/DataSource.hpp>

#include "Body.hpp"

typedef ssc::data_source::DataSource DataSource;
typedef ssc::data_source::DataSourceModule DataSourceModule;

class BodyStatesModule : public DataSourceModule
{
public:
	BodyStatesModule(DataSource* const data_source, const BodyPtr body_);

	BodyStatesModule(const BodyStatesModule& rhs, DataSource* const data_source);

	virtual std::string get_name() const;

	DataSourceModule* clone() const
	{
		return new BodyStatesModule(*this);
	}

	DataSourceModule* clone(DataSource* const data_source) const
	{
		return new BodyStatesModule(*this, data_source);
	}

	virtual void update() const;
	std::string namify(const std::string& bodyName, const std::string& variableName) const;

private:
	const BodyPtr body;
};

#endif /* CONTROLLERS_INC_BODYSTATESMODULE_HPP_ */
