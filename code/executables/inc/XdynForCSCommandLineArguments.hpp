/*
 * XdynForCSCommandLineArguments.hpp
 *
 *  Created on: Nov 14, 2018
 *      Author: cady
 */

#ifndef EXECUTABLES_INC_XDYNFORCSCOMMANDLINEARGUMENTS_HPP_
#define EXECUTABLES_INC_XDYNFORCSCOMMANDLINEARGUMENTS_HPP_

#include <string>
#include <vector>

struct XdynForCSCommandLineArguments
{
    XdynForCSCommandLineArguments();
    bool empty() const;
    std::vector<std::string> yaml_filenames;
    std::string solver;
    double initial_timestep;
    bool catch_exceptions;
    bool docker_localhost;
    short unsigned int port;
    bool verbose;
    bool show_help;
    bool inline_yaml;
    bool show_websocket_debug_information;
};

#endif /* EXECUTABLES_INC_XDYNFORCSCOMMANDLINEARGUMENTS_HPP_ */
