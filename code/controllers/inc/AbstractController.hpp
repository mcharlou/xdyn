/*
 * AbstractController.hpp
 *
 *  Created on: 14 févr. 2020
 *      Author: mcharlou2016
 */

#ifndef PARSER_EXTENSIONS_INC_ABSTRACTCONTROLLER_HPP_
#define PARSER_EXTENSIONS_INC_ABSTRACTCONTROLLER_HPP_

#include <memory>
#include <functional>
#include <vector>
#include <string>
#include <boost/optional/optional.hpp>
#include <ssc/data_source.hpp>
#include <ssc/data_source/DataSourceModule.hpp>

#include "yaml-cpp/exceptions.h"
#include "InvalidInputException.hpp"
#include "YamlModel.hpp"

#include "BodyStates.hpp"

class AbstractController;
typedef std::shared_ptr<AbstractController> ControllerPtr;
typedef std::vector<ControllerPtr> ListOfControllers;
typedef std::function<boost::optional<ControllerPtr>(const YamlModel&, ssc::data_source::DataSource* const data_source)> ControllerParser;
typedef ssc::data_source::DataSource DataSource;
typedef ssc::data_source::DataSourceModule DataSourceModule;

template <typename ControllerType>
using ControllerTypePtr = std::shared_ptr<ControllerType>;

class AbstractController : public DataSourceModule
{
public:
	AbstractController(DataSource* const data_source, const std::string& module_name) : DataSourceModule(data_source, module_name)
	{}

	AbstractController(const AbstractController& rhs, DataSource* const data_source) : DataSourceModule(rhs, data_source)
	{}

	virtual std::string get_name() const=0;
	virtual std::string get_signal_name() const=0;

	/*DataSourceModule* clone() const=0;
	{
		return new AbstractController(*this);
	}

	DataSourceModule* clone(DataSource* const data_source) const=0;
	{ // to be implemented in non-abstract children classes
		return new AbstractController(*this, data_source);
	}

	virtual void update() const=0;*/


	template <typename ControllerType>
	static ControllerParser build_parser()
	{
		auto parser = [](const YamlModel& yaml, DataSource* const data_source) -> boost::optional<ControllerPtr>
		{
			boost::optional<ControllerPtr> ret;
			if (yaml.model == ControllerType::model_name())
			{
				std::string context = "Invalid input data for model '" + ControllerType::model_name() + "'.";
				try
				{
					ret.reset(ControllerPtr(new ControllerType(ControllerType::parse(yaml.yaml),data_source)));
				}
				catch (const InvalidInputException& exception)
				{
					THROW(__PRETTY_FUNCTION__, InvalidInputException, context << std::endl << exception.get_message() << std::endl << "Model containing error is defined line "
						  << yaml.index_of_first_line_in_global_yaml << " of the YAML file." << std::endl);
				}
				catch (const YAML::Exception& exception)
				{
					const size_t line_number = yaml.index_of_first_line_in_global_yaml;
					THROW(__PRETTY_FUNCTION__, InvalidInputException, context << std::endl << "Model containing error is defined line "
						  << line_number << " of the YAML file." << std::endl << "The error was: " << exception.msg);
				}
			}
			return ret;
		};
		return parser;
	}

};



#endif /* PARSER_EXTENSIONS_INC_ABSTRACTCONTROLLER_HPP_ */
