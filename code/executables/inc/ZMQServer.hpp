/*
 * ZMQServer.hpp
 *
 *  Created on: 11 juin 2020
 *      Author: mcharlou2016
 */

#ifndef EXECUTABLES_INC_ZMQSERVER_HPP_
#define EXECUTABLES_INC_ZMQSERVER_HPP_

#include <zmq.hpp>
// For handling Ctrl+C
#include <csignal>
#include <string>

#include "MessageHandler.hpp"

class ZMQServer
{
public:
	ZMQServer(short unsigned int port, bool docker_localhost);
	void run(MessageHandler& handler, const bool verbose);

private:
	zmq::context_t context;
	zmq::socket_t socket;
	std::string localhost_address;
	short unsigned int port;

	std::string receive_string();
	void send_string(const std::string& message);
};

#endif /* EXECUTABLES_INC_ZMQSERVER_HPP_ */
