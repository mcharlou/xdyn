/*
 * JSONServerObserver.hpp
 *
 *  Created on: 26 juin 2020
 *      Author: mcharlou2016
 */

#ifndef OBSERVERS_AND_API_INC_JSONSERVEROBSERVER_HPP_
#define OBSERVERS_AND_API_INC_JSONSERVEROBSERVER_HPP_

#include <vector>
#include <memory>

#include <ssc/json.hpp>
#include "rapidjson/document.h"
#include "Observer.hpp"


class JSONServerObserver : public Observer
{
public:
	JSONServerObserver(const std::vector<std::string>& data);
	JSONServerObserver();
	void observe(const Sim& sys, const double t);
	rapidjson::Document get();

private:
	using Observer::get_serializer;
	using Observer::get_initializer;

	std::function<void()> get_serializer(const double val, const DataAddressing& address);
	std::function<void()> get_initializer(const double val, const DataAddressing& address);

	void flush_after_initialization();
	void flush_after_write();
	void flush_value_during_write();

	rapidjson::Document document;

	// recursive function to navigate the data tree
	template<typename EncodingT, typename AllocatorT>
	rapidjson::Value& build_path(const std::vector<std::string>& address, rapidjson::GenericValue<EncodingT,AllocatorT>& root)
	{
		//std::cout << "Calling 'build_path' for address: " << address  << std::endl;
		if(address.size()>1) // 'address' is a path, not the final location yet, we stack the recursion
		{
			if(not(root.HasMember(address[0].c_str()))) // node corresponding to the root of 'address' does not already exists, we create it
			{
				rapidjson::Value val(rapidjson::kObjectType);
				//root.AddMember(address[0].c_str(),val,document.GetAllocator());
				rapidjson::Value name(address[0].c_str(),document.GetAllocator());
				root.AddMember(name,val,document.GetAllocator());
			}
			auto new_address = std::vector<std::string>(address.begin()+1,address.end()); // next we travel further in the tree
			return build_path(new_address,root[address[0].c_str()]);
		}
		else // 'address' is the final location of the path (dead end of the recursion)
		{
			if(root.HasMember(address[0].c_str())) // if the node (containing an array of doubles) exists, we return it
			{
				return root[address[0].c_str()];
			}
			else // if it does not exist, we create it and return it
			{
				root.AddMember(rapidjson::Value(address[0].c_str(),document.GetAllocator()),rapidjson::Value(rapidjson::kArrayType),document.GetAllocator());
				return root[address[0].c_str()];
			}
		}
	}
};

#endif /* OBSERVERS_AND_API_INC_JSONSERVEROBSERVER_HPP_ */
