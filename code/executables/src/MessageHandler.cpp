/*
 * MessageHandler.cpp
 *
 *  Created on: 11 juin 2020
 *      Author: mcharlou2016
 */

#include <iostream>
#include <memory>

#include <ssc/json.hpp>
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "parse_history.hpp"
#include "ConfBuilder.hpp"

#include "MessageHandler.hpp"
#include "ServerRequest.hpp"

MessageHandler::MessageHandler(const std::string& yaml_model, const std::string& solver_, const double dt_):sim(ConfBuilder(yaml_model).sim),dt(dt_),solver(solver_),current_time(0.),current_request()
{}

std::string MessageHandler::operator()(const std::string& raw_server_input)
{
	set_current_request(raw_server_input);
	current_request->execute(sim);
	rapidjson::Document doc = current_request->response();
	rapidjson::StringBuffer buffer;
	rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
	doc.Accept(writer);
	return buffer.GetString();
}

void MessageHandler::set_current_request(const std::string& json_input)
{
	rapidjson::Document document;
	ssc::json::parse(json_input, document);
	if (not(document.IsObject()))
	{
		THROW(__PRETTY_FUNCTION__, ssc::json::Exception, "JSON should be an object (i.e. within curly braces), but it's not (it's a " << ssc::json::print_type(document) << "). The JSON we're looking at was: " << ssc::json::dump(document));
	}
	if (document.HasMember("request"))
	{
		const rapidjson::Value& req = document["request"];
		std::string request = ssc::json::dump(req);
		if(request == "\"compute\"")
		{
			const rapidjson::Value& input = document["input"];
			current_request.reset(new ComputeRequest(input,dt,solver));
		}
		else if(request == "\"get_Tmax\"")
		{
			current_request.reset(new GetTmaxRequest());
		}
		else if(request == "\"quaternion2euler\"")
		{
			const rapidjson::Value& input = document["input"];
			current_request.reset(new Quaternion2EulerRequest(input));
		}
		else if(request == "\"euler2quaternion\"")
		{
			const rapidjson::Value& input = document["input"];
			current_request.reset(new Euler2QuaternionRequest(input));
		}
		else if(request == "\"set_signals\"")
		{
			const rapidjson::Value& input = document["input"];
			current_request.reset(new SetSignalsRequest(input));
		}
		else if(request == "\"set_states\"")
		{
			const rapidjson::Value& input = document["input"];
			current_request.reset(new SetStatesHistoryRequest(input,sim.get_bodies().at(0)->get_Tmax()));
		}
		else
		{
			THROW(__PRETTY_FUNCTION__, ssc::json::Exception, "Value '"+request+"' of key 'request' in json input is unknown.")
		}
	}
	else
	{
		THROW(__PRETTY_FUNCTION__, ssc::json::Exception, "Input JSON should have a 'request' key but none was found.")
	}
}
