/*
 * XdynServer.hpp
 *
 *  Created on: 19 juin 2020
 *      Author: mcharlou2016
 */

#ifndef EXECUTABLES_INC_XDYNSERVER_HPP_
#define EXECUTABLES_INC_XDYNSERVER_HPP_

// Library inclusions
#include <string>

// Project inclusions
#include "Sim.hpp"

class XdynServer
{
public:
	XdynServer(const std::string& yaml_model,
			   const std::string& solver,
			   const double dt);

private:
	XdynServer();

	Sim sim;
	const std::string solver;
	const double dt;
	double current_time;
};

#endif /* EXECUTABLES_INC_XDYNSERVER_HPP_ */
