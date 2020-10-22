/*
 * ZMQServer.cpp
 *
 *  Created on: 11 juin 2020
 *      Author: mcharlou2016
 */

#include <functional>
#include <iostream>
#include <string>
#include <boost/algorithm/string/replace.hpp>

// For handling Ctrl+C
#include <unistd.h>
#include <cstdio>

#include "report_xdyn_exceptions_to_user.hpp"

#include "ZMQServer.hpp"

ZMQServer::ZMQServer(short unsigned int port_, bool docker_localhost):context(1),socket(context, ZMQ_REP),localhost_address(),port(port_)
{
//	std::cout << "Binding socket to address: " << "'tcp://"+std::string(ADDRESS)+":"+std::to_string(port)+"'" << std::endl;
	if(docker_localhost)
	{
		localhost_address = "0.0.0.0";
	}
	else
	{
		localhost_address = "127.0.0.1";
	}
	socket.bind("tcp://"+localhost_address+":"+std::to_string(port));
//	std::cout << "Socket bound!" << std::endl;
}

volatile sig_atomic_t stop;

void inthand(int);
void inthand(int)
{
	stop = 1;
}

std::string replace_newlines_by_spaces(std::string str);
std::string replace_newlines_by_spaces(std::string str)
{
    boost::replace_all(str, "\n", " ");
    return str;
}

void ZMQServer::run(MessageHandler& handler, const bool verbose)
{
	std::cout << "Starting ZMQ server on " << localhost_address << ":" << std::to_string(port) << " (press Ctrl+C to terminate)" << std::endl;
	signal(SIGINT, inthand);
	while(!stop){
		//  Wait for next request from client
//		std::cout << "Waiting for request from client..." << std::endl;
		const std::string input_json = receive_string();
//		std::cout << "Request received!" << std::endl;

		// Convert ZMQ message to string
		if (verbose)
		{
			std::cout << current_date_time() << " Received: " << input_json << std::endl;
		}

		//zmq::message_t reply;

		// Deal with request and send back reply
		const std::function<void(const std::string&)> quiet_error_outputter = [this](const std::string& what) {this->send_string(replace_newlines_by_spaces(std::string("{\"error\": \"") + what + "\"}"));};
		const std::function<void(const std::string&)> verbose_error_outputter = [this](const std::string& what) {std::cerr << current_date_time() << " Error: " << what << std::endl; this->send_string(replace_newlines_by_spaces(std::string("{\"error\": \"") + what + "\"}"));};
		const auto error_outputter = verbose ? verbose_error_outputter : quiet_error_outputter;

		const std::function<void(void)> quiet_f = [&handler, &input_json, this]() {this->send_string(handler(input_json));};
		const std::function<void(void)> verbose_f = [&handler, &input_json, this]() {const std::string json = handler(input_json); std::cout << current_date_time() << " Sending: " << json << std::endl; this->send_string(json);};
		const std::function<void(void)> f = verbose ? verbose_f : quiet_f;

		report_xdyn_exceptions_to_user(f, error_outputter);

		//  Send reply back to client
		//socket.send (reply);
	}
	std::cout << std::endl << "Gracefully stopping the websocket server..." << std::endl;
}

std::string ZMQServer::receive_string()
{
	zmq::message_t request;
	//auto flag = zmq::recv_flags::none;
	socket.recv(&request);
	return std::string(static_cast<char*>(request.data()), request.size());
}

void ZMQServer::send_string(const std::string& str)
{
	zmq::message_t message(str.size());
	std::memcpy (message.data(), str.data(), str.size());
	socket.send(message);
}

