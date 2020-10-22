/*
 * parse_output.cpp
 *
 *  Created on: Jan 13, 2015
 *      Author: cady
 */

#include <boost/algorithm/string/predicate.hpp>
#include "yaml.h"
#include "InvalidInputException.hpp"
#include "parse_address.hpp"
#include "parse_output.hpp"

struct s_body
{
	std::string name="";
	std::vector<std::string> forces=std::vector<std::string>();
};

void operator >> (const YAML::Node& node, YamlOutput& f);
std::string customize(const std::string& var_name, const std::string& body_name);
std::string customize2(const std::string& var_name,const std::string& body_name, const std::string& frame_name, const std::string& force_name);
void fill(YamlOutput& out, const s_body& body);
std::vector<s_body> get_body_names(const std::string yaml);

void operator >> (const YAML::Node& node, YamlOutput& f)
{
    if(const YAML::Node *pName = node.FindValue("filename"))
    {
        *pName >> f.filename;
    }
    if(const YAML::Node *pName = node.FindValue("address"))
    {
        *pName >> f.address;
    }
    if(const YAML::Node *pName = node.FindValue("port"))
    {
        *pName >> f.port;
    }
    node["format"]   >> f.format;
    node["data"]     >> f.data;
}

std::vector<YamlOutput> parse_output(const std::string& yaml)
{
    std::vector<YamlOutput> ret;
    std::stringstream stream(yaml);
    YAML::Parser parser(stream);
    YAML::Node node;
    parser.GetNextDocument(node);
    try
    {
        node["output"] >> ret;
    }
    catch(std::exception& ) // Nothing to do: 'output' section is not mandatory
    {
    }
    return ret;
}

std::string customize(const std::string& var_name, const std::string& body_name)
{
    return var_name + "(" + body_name + ")";
}

std::string customize2(const std::string& var_name,const std::string& body_name, const std::string& frame_name, const std::string& force_name)
{
	return var_name + "(" + force_name + "," + body_name + "," + frame_name + ")";
}

void fill(YamlOutput& out, const s_body& body)
{
    out.data.push_back(customize("x", body.name));
    out.data.push_back(customize("y", body.name));
    out.data.push_back(customize("z", body.name));
    out.data.push_back(customize("u", body.name));
    out.data.push_back(customize("v", body.name));
    out.data.push_back(customize("w", body.name));
    out.data.push_back(customize("p", body.name));
    out.data.push_back(customize("q", body.name));
    out.data.push_back(customize("r", body.name));
    out.data.push_back(customize("qr", body.name));
    out.data.push_back(customize("qi", body.name));
    out.data.push_back(customize("qj", body.name));
    out.data.push_back(customize("qk", body.name));

    out.data.push_back(customize("phi", body.name));
    out.data.push_back(customize("theta", body.name));
    out.data.push_back(customize("psi", body.name));

    for(auto force:body.forces){
    	out.data.push_back(customize2("Fx",body.name,body.name,force));
    	out.data.push_back(customize2("Fy",body.name,body.name,force));
    	out.data.push_back(customize2("Fz",body.name,body.name,force));
    	out.data.push_back(customize2("Mx",body.name,body.name,force));
    	out.data.push_back(customize2("My",body.name,body.name,force));
    	out.data.push_back(customize2("Mz",body.name,body.name,force));
    	out.data.push_back(customize2("Fx",body.name,"NED",force));
    	out.data.push_back(customize2("Fy",body.name,"NED",force));
    	out.data.push_back(customize2("Fz",body.name,"NED",force));
    	out.data.push_back(customize2("Mx",body.name,"NED",force));
    	out.data.push_back(customize2("My",body.name,"NED",force));
    	out.data.push_back(customize2("Mz",body.name,"NED",force));
    }
}

std::vector<s_body> get_body_names(const std::string yaml)
{
    std::vector<s_body> out;
    std::stringstream stream(yaml);
    YAML::Parser parser(stream);
    YAML::Node node;
    parser.GetNextDocument(node);
    const YAML::Node *parameter = node.FindValue("bodies");
    if (parameter)
    {
        for (size_t i = 0 ; i < parameter->size() ; ++i)
        {
            s_body body;
            (*parameter)[i]["name"] >> body.name;
            const YAML::Node *param = (*parameter)[i].FindValue("external forces");
            if (param){
            	for(size_t j=0;j<param->size();++j){
            		std::string model;
            		(*param)[j]["model"] >> model;
            		body.forces.push_back(model);
            	}
            }
            param = (*parameter)[i].FindValue("controlled forces");
			if (param){
				for(size_t j=0;j<param->size();++j){
					std::string name;
					(*param)[j]["name"] >> name;// Controlled forces are identified by their names, not their models
					body.forces.push_back(name);
				}
			}
            out.push_back(body);
        }
    }
    return out;
}

std::string get_format_for_wave_observer(const std::string& filename)
{
    const size_t n = filename.size();
    if (n<=3)
    {
        THROW(__PRETTY_FUNCTION__, InvalidInputException, "Invalid file format for wave output file. Expected string should be at least 4 characters");
    }
    if (filename.substr(n-3,3)==".h5")   return "hdf5";
    if (filename.substr(n-5,5)==".hdf5") return "hdf5";
    if (filename.substr(n-3,3)==".H5")   return "hdf5";
    if (filename.substr(n-5,5)==".HDF5") return "hdf5";
    {
        THROW(__PRETTY_FUNCTION__, InvalidInputException, "Invalid file format for wave output file. Expected filename extensions are h5, hdf5");
    }
}

std::string get_format(const std::string& filename)
{
    const size_t n = filename.size();
    if (!n)                              return "tsv";
    if (n<=3)
    {
        THROW(__PRETTY_FUNCTION__, InvalidInputException, "Invalid file format for output file. E");
    }
    if (filename.substr(n-3,3)==".h5")   return "hdf5";
    if (filename.substr(n-5,5)==".hdf5") return "hdf5";
    if (filename.substr(n-4,4)==".csv")  return "csv";
    if (filename.substr(n-4,4)==".tsv")  return "tsv";
    if (filename.substr(n-5,5)==".json") return "json";
    {
        THROW(__PRETTY_FUNCTION__, InvalidInputException, "Could not recognize the format of specified output file '" << filename << "': expected filename extensions are .tsv, .csv, .h5, .hdf5 or .json");
    }
}

YamlOutput build_YamlOutput_from_filename(const std::string& filename);
YamlOutput build_YamlOutput_from_filename(const std::string& filename)
{
    YamlOutput out;
    if ((filename.empty()) or (filename=="tsv"))
    {
        out.format = "tsv";
        out.filename = "";
    }
    else if (filename=="csv")
    {
        out.format = "csv";
        out.filename = "";
    }
    else if (filename=="json")
    {
        out.format = "json";
        out.filename = "";
    }
    else if ((filename=="h5") or (filename=="hdf5"))
    {
        THROW(__PRETTY_FUNCTION__, InvalidInputException, "Need to specify an input filename for HDF5 export from command line");
    }
    else if (boost::algorithm::starts_with(filename,"ws") or
             boost::algorithm::starts_with(filename,"wss"))
    {
        out = build_YamlOutput_from_WS_URL(filename);
    }
    else
    {
        out.format = get_format(filename);
        out.filename = filename;
    }
    return out;
}

YamlOutput generate_default_outputter_with_all_states_in_it(const std::string& yaml, const std::string& filename)
{
    auto out = build_YamlOutput_from_filename(filename);
    out.data.push_back("t");
    const auto bodies = get_body_names(yaml);
    for (auto body:bodies) fill(out, body);
    return out;
}
