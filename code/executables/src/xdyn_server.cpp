/*
 * xdyn-server.cpp
 *
 *  Created on: 10 juin 2020
 *      Author: mcharlou2016
 */

#include <ssc/text_file_reader.hpp>
#include <ssc/macros.hpp>
#include <memory>
#include <sstream>
#include <string>

#include "XdynForCS.hpp"
#include "parse_history.hpp"
#include "report_xdyn_exceptions_to_user.hpp"
#include "parse_XdynForCSCommandLineArguments.hpp"
#include "ZMQServer.hpp"
#include "MessageHandler.hpp"

#include <ssc/check_ssc_version.hpp>
CHECK_SSC_VERSION(8,0)

void start_server(const XdynForCSCommandLineArguments& input_data);
void start_server(const XdynForCSCommandLineArguments& input_data)
{
	std::string yaml;
	if(input_data.inline_yaml){
		for(std::string str:input_data.yaml_filenames){
			yaml.append("\n\n");
			yaml.append(str);
		}
	}
	else{
		yaml = ssc::text_file_reader::TextFileReader(input_data.yaml_filenames).get_contents();
	}

    ZMQServer server(input_data.port, input_data.docker_localhost);
    MessageHandler handler(yaml, input_data.solver, input_data.initial_timestep);
    server.run(handler, input_data.verbose);
}

int main(int argc, char** argv)
{
    XdynForCSCommandLineArguments input_data;
    if (argc==1) return display_help(argv[0], input_data);
    int error = 0;
    report_xdyn_exceptions_to_user([&error,&argc,&argv,&input_data]{error = get_input_data(argc, argv, input_data);}, [](const std::string& s){std::cerr << s;});
    if (error)
    {
        return error;
    }
    if (input_data.empty() || input_data.show_help) return EXIT_SUCCESS;
    const auto run = [input_data](){
    {
        start_server(input_data);
    }};
    if (input_data.catch_exceptions)
    {

        report_xdyn_exceptions_to_user(run, [](const std::string& s){std::cerr << s;});
    }
    else
    {
        run();
    }
    return error;
}


