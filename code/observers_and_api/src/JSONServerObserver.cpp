/*
 * JSONServerObserver.cpp
 *
 *  Created on: 26 juin 2020
 *      Author: mcharlou2016
 */

#include "rapidjson/document.h"
#include "NumericalErrorException.hpp"

#include "JSONServerObserver.hpp"

JSONServerObserver::JSONServerObserver(const std::vector<std::string>& data):Observer(data),document(rapidjson::kObjectType){}

JSONServerObserver::JSONServerObserver():Observer(std::vector<std::string>({})),document(rapidjson::kObjectType){}

std::function<void()> JSONServerObserver::get_serializer(const double val, const DataAddressing& addressing)
{
	return [this,val,addressing]()
		{
			if(std::isnan(val) || std::isinf(val))
			{
				THROW(__PRETTY_FUNCTION__, NumericalErrorException, "Field '" << addressing.name << "' has a NaN or Inf value, which is not supported by JSON.");
			}
			build_path(addressing.address,document).PushBack(val,document.GetAllocator());
		};
}

std::function<void()> JSONServerObserver::get_initializer(const double , const DataAddressing& addressing)
{
	return [this,addressing]()
		{
			build_path(addressing.address,document);
		};
}

void JSONServerObserver::observe(const Sim& sys, const double t)
{
	Observer::observe_everything(sys,t);
}

rapidjson::Document JSONServerObserver::get()
{
	rapidjson::Document ret;
	ret.CopyFrom(document,ret.GetAllocator());
	return ret;
}

void JSONServerObserver::flush_after_initialization()
{
}

void JSONServerObserver::flush_after_write()
{
}

void JSONServerObserver::flush_value_during_write()
{
}
