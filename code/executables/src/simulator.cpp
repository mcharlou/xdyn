/*
 * simulator.cpp
 *
 *  Created on: 17 avr. 2014
 *      Author: cady
 */

#include <ssc/text_file_reader.hpp>
#include "InputData.hpp"
#include "utilities_for_InputData.hpp"

#include "listeners.hpp"
#include "simulator_api.hpp"
#include "SimCsvObserver.hpp"
#include <ssc/solver.hpp>
#include <ssc/check_ssc_version.hpp>
#include <string>
#include <fstream>


CHECK_SSC_VERSION(0x1b9c9e4bb06ea4d5)

int main(int argc, char** argv)
{
    InputData input_data;
    const int error = get_input_data(argc, argv, input_data);
    if (not(error))
    {
        const ssc::text_file_reader::TextFileReader yaml_reader(input_data.yaml_filenames);
        ssc::data_source::DataSource command_listener;
        if (not(input_data.command_file.empty()))
        {
            command_listener = listen_to_file(ssc::text_file_reader::TextFileReader(std::vector<std::string>(1,input_data.command_file)).get_contents());
        }
        auto sys = get_system(yaml_reader.get_contents(),command_listener);
        std::ofstream ws(input_data.wave_output.c_str());
        if (input_data.wave_output.empty()) copy_stream(dev_null_buffer, ws);
        SimCsvObserver observer(std::cout, ws);
        if (input_data.solver=="euler")
        {
            ssc::solver::quicksolve<ssc::solver::EulerStepper>(sys, input_data.tstart, input_data.tend, input_data.initial_timestep, observer);
        }
        else if (input_data.solver=="rk4")
        {
            ssc::solver::quicksolve<ssc::solver::RK4Stepper>(sys, input_data.tstart, input_data.tend, input_data.initial_timestep, observer);
        }
        else if (input_data.solver=="rkck")
        {
            ssc::solver::quicksolve<ssc::solver::RKCK>(sys, input_data.tstart, input_data.tend, input_data.initial_timestep, observer);
        }
        else
        {
            ssc::solver::quicksolve<ssc::solver::EulerStepper>(sys, input_data.tstart, input_data.tend, input_data.initial_timestep, observer);
        }
    }
    return error;
}
