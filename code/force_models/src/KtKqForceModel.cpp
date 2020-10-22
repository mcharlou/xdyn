/*
 * KtKqForceModel.cpp
 *
 *  Created on: Jun 27, 2015
 *      Author: cady
 */
#include <ssc/yaml_parser.hpp>
#include <ssc/interpolation.hpp>
#include "external_data_structures_parsers.hpp"

#include "KtKqForceModel.hpp"
#include "yaml.h"

std::string KtKqForceModel::model_name() {return "Kt(J) & Kq(J)";}

class KtKqForceModel::Impl
{
    public:
        Impl(const std::vector<double>&J, const std::vector<double>& Kt_, const std::vector<double>& Kq_) :
            Kt(J, Kt_),
            Kq(J, Kq_)
        {}

        ssc::interpolation::SplineVariableStep Kt;
        ssc::interpolation::SplineVariableStep Kq;

    private:
        Impl();
};

KtKqForceModel::Yaml::Yaml() :
            AbstractWageningen::Yaml(),
            J(),
            Kt(),
            Kq()
{}

KtKqForceModel::Yaml::Yaml(const AbstractWageningen::Yaml& y) :
        AbstractWageningen::Yaml(y),
        J(),
        Kt(),
        Kq()
{}

KtKqForceModel::Yaml KtKqForceModel::parse(const std::string& yaml)
{
    std::stringstream stream(yaml);
    YAML::Parser parser(stream);
    YAML::Node node;
    parser.GetNextDocument(node);
    Yaml ret = AbstractWageningen::parse(yaml);;
    node["J"]  >> ret.J;
    node["Kt"] >> ret.Kt;
    node["Kq"] >> ret.Kq;

    return ret;
}

KtKqForceModel::KtKqForceModel(const Yaml& input, const std::string& body_name, const EnvironmentAndFrames& env) :
            AbstractWageningen(input, body_name, env), pimpl(new Impl(input.J, input.Kt, input.Kq))
{}

double KtKqForceModel::get_Kt(const std::map<std::string,double>&, const double J) const
{
	double kt;
	try{kt = pimpl->Kt.f(J);}
    catch(const ssc::exception_handling::Exception& e)
    {
    	kt=0;
    	//THROW(__PRETTY_FUNCTION__, ssc::exception_handling::Exception, "Error while trying to retrieve the thrust coefficient from the input with J=" << J << ":\n" << e.get_message());
    }
    return kt;
}

double KtKqForceModel::get_Kq(const std::map<std::string,double>&, const double J) const
{
	double kq;
	try{kq = pimpl->Kq.f(J);}
	catch(const ssc::exception_handling::Exception& e)
	{
		kq=0;
		//THROW(__PRETTY_FUNCTION__, ssc::exception_handling::Exception, "Error while trying to retrieve the torque coefficient from the input with J=" << J << ":\n" << e.get_message());
	}
	return kq;
}
