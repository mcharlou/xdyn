/*
 * MessageHandler.hpp
 *
 *  Created on: 11 juin 2020
 *      Author: mcharlou2016
 */

#ifndef EXECUTABLES_INC_MESSAGEHANDLER_HPP_
#define EXECUTABLES_INC_MESSAGEHANDLER_HPP_

#include <memory>
#include <zmq.hpp>
#include <string>

#include "ServerRequest.hpp"
#include "Sim.hpp"

class MessageHandler
{
public:
	MessageHandler(const std::string& yaml_model,
				   const std::string& solver,
				   const double dt);
	std::string operator()(const std::string& msg);

private:
	void set_current_request(const std::string& json_input);
	Sim sim;
	const double dt;
	const std::string solver;
	double current_time;
	std::shared_ptr<ServerRequest> current_request;
};

#endif /* EXECUTABLES_INC_MESSAGEHANDLER_HPP_ */
