/*
 * RadiationDampingForceModel.cpp
 *
 *  Created on: Oct 2, 2014
 *      Author: jacquenot
 */

#include <iostream>
#include <functional>
#include <Eigen/Dense>
#include "FunctionSupportForEigen.hpp"

#include "RadiationDampingForceModel.hpp"

#include "Body.hpp"
#include "HDBParser.hpp"
#include "History.hpp"
#include "InvalidInputException.hpp"
#include "RadiationDampingBuilder.hpp"
#include "external_data_structures_parsers.hpp"

#include <ssc/macros.hpp>
#include <ssc/text_file_reader.hpp>
#include <ssc/integrate.hpp>

#include <ssc/yaml_parser.hpp>

#include "yaml.h"

#include <cassert>
#include <array>

#define _USE_MATH_DEFINE
#include <cmath>
#define PI M_PI

typedef Eigen::Matrix<std::function<double(double)>, 6, 1> Vector6funct;
typedef Eigen::Matrix<std::function<double(double)>, 3, 1> Vector3funct;

std::string RadiationDampingForceModel::model_name() {return "radiation damping";}

class CSVWriter
{
    public:
        CSVWriter(std::ostream& os_, const std::string& x_name_, const std::vector<double>& x) : os(os_), x_name(x_name_), xs(x), functions(), function_names()
        {
        }
        CSVWriter& operator=(const CSVWriter& rhs);
        ~CSVWriter(){};
        CSVWriter(const CSVWriter&);

        void add(const std::string& name, const std::function<double(const double)>& f, const size_t i, const size_t j)
        {
            std::stringstream ss;
            ss << name << '_' << i << j;
            function_names.push_back(ss.str());
            functions.push_back(f);
        }

        void print()
        {
            print_title();
            print_values();

        }

    private:
        CSVWriter();

        void print_title()
        {
            os << x_name;
            for (auto f:function_names)
            {
                os << ',' << f;
            }
            os << std::endl;
        }

        void print_values()
        {
            for (auto x:xs)
            {
                os << x;
                for (auto f:functions)
                {
                    os << ',' << f(x);
                }
                os << std::endl;
            }
        }
        std::ostream& os;
        std::string x_name;
        std::vector<double> xs;
        std::vector<std::function<double(const double)> > functions;
        std::vector<std::string> function_names;

};

class RadiationDampingForceModel::Impl
{
    public:
        Impl(const std::shared_ptr<HDBParser>& parser, const YamlRadiationDamping& yaml) : hdb{parser}, builder(RadiationDampingBuilder(yaml.type_of_quadrature_for_convolution, yaml.type_of_quadrature_for_cos_transform)),
		forward_speed_correction(yaml.forward_speed_correction), suppress_constant_part(yaml.suppress_constant_part), K0(), K1(), Ainf(),
        omega(parser->get_radiation_damping_angular_frequencies()), taus(), n(yaml.nb_of_points_for_retardation_function_discretization), Tmin(yaml.tau_min), Tmax(yaml.tau_max)
        {
            CSVWriter omega_writer(std::cerr, "omega", omega);
            taus = builder.build_regular_intervals(Tmin,Tmax,n);
            CSVWriter tau_writer(std::cerr, "tau", taus);
            CSVWriter tau_writer2(std::cerr, "tau", taus);

            for (size_t i = 0 ; i < 6 ; ++i)
            {
        	    for (size_t j = 0 ; j < 6 ; ++j)
        	    {
        		    const auto Br = get_Br(i,j);
        		    K0(i,j) = get_K(Br);
        		    if (yaml.output_Br_and_K)
        		    {
        			    omega_writer.add("Br",Br,i+1,j+1);
        			    tau_writer.add("K0",K0(i,j),i+1,j+1);
        		    }
        	    }
            }

            for (size_t i = 0 ; i < 6 ; ++i)
            {
            	for (size_t j = 0 ; j < 3 ; ++j)
            	{
            		Ainf(i,j) = hdb->get_infinite_frequency_added_mass_coeff(i,j,K0(i,j),Tmin,Tmax);
            		K1(i,j) = get_K(get_A(i,j)-Ainf(i,j));
            		if (yaml.output_Br_and_K)
            		{
            			tau_writer2.add("K1",K1(i,j),i+1,j+1);
            		}
            	}
            }

            if (yaml.output_Br_and_K)
            {
                std::cerr << "Debugging information for damping functions Br:" << std::endl;
                omega_writer.print();
                std::cerr << std::endl << "Debugging information for retardation functions K0 and K1:" << std::endl;
                tau_writer.print();
                tau_writer2.print();
            }
        }

        std::function<double(double)> get_Br(const size_t i, const size_t j) const
        {
            return builder.build_interpolator(omega,hdb->get_radiation_damping_coeff(i,j));
        }

        std::function<double(double)> get_A(const size_t i, const size_t j) const
        {
        	return builder.build_interpolator(omega,hdb->get_added_mass_coeff(i,j));
        }

        std::function<double(double)> get_K(const std::function<double(double)>& Br) const
        {
            return builder.build_retardation_function(Br,taus,1E-3,omega.front(),omega.back());
        }

        /*Eigen::Matrix<double, 6, 6> get_Ls(const BodyStates& states, const Eigen::Matrix<double, 6, 1>& Ubar) const
		{
        	Eigen::Matrix<double, 6, 6> Ls=Eigen::Matrix<double, 6, 6>::Zero();
        	Ls(1,5)=Ubar(0);
        	Ls(2,4)=-Ubar(0);
        	Ls(0,5)=-Ubar(1);
        	Ls(2,3)=Ubar(1);
        	Ls(0,4)=Ubar(2);
        	Ls(1,3)=-Ubar(2);
        	return Ls;
		}*/

        Eigen::Matrix<double, 6, 1> get_convolution(const Eigen::Matrix<std::function<double(double)>, 6, 6>& K, const Vector6funct& X_dot) const
        {
        	// !!! X_dot must already be X_dot(t-tau)
        	Vector6d conv;
        	for(size_t i=0 ; i<6 ; i++)
        	{
        		double K_X_dot = 0;
        		for (size_t k = 0 ; k < 6 ; ++k)
        		{
        			const double co = builder.convolution(X_dot(k), K(i,k), Tmin, Tmax);
        			K_X_dot += co;
        		}
        		conv(i)=K_X_dot;
        	}
        	return conv;
        }

        Eigen::Matrix<double, 6, 1> get_half_convolution(const Eigen::Matrix<std::function<double(double)>, 6, 3>& K, const Vector3funct& Ls_by_X_dot) const
                		{
        	// !!! X_dot must already be X_dot(t-tau)
        	Vector6d conv;
        	for(size_t i=0 ; i<6 ; i++)
        	{
        		double K_X_dot = 0;
        		for (size_t k = 0 ; k < 3 ; ++k)
        		{
        			const double co = builder.convolution(Ls_by_X_dot(k), K(i,k), Tmin, Tmax);
        			K_X_dot += co;
        		}
        		conv(i)=K_X_dot;
        	}
        	return conv;
                		}

        /*Eigen::Matrix<std::function<double(double)>, 6, 6> dedicated_product(Eigen::Matrix<std::function<double(double)>, 6, 6>& K1,Eigen::Matrix<double, 6, 6>& Ls) const
		{
        	Eigen::Matrix<std::function<double(double)>, 6, 6> product;
        	for(size_t i=0 ; i<6 ; i++)
        	{
        		for(size_t j=0 ; j<3 ; j++)
        		{
        			product(i,j)=[](double x){return 0;};
        		}
        		product(i,3)=Ls(1,3)*K1(i,1)+Ls(2,3)*K1(i,2);
        		product(i,4)=Ls(0,4)*K1(i,0)+Ls(2,4)*K1(i,2);
        		product(i,5)=Ls(0,5)*K1(i,0)+Ls(1,5)*K1(i,1);
        	}
        	return product;
		}*/

        Vector6funct get_X_dot(const BodyStates& states)
        {
        	Vector6funct ret;
        	for(size_t i=0 ; i<6 ; i++)
        	{
        		if(suppress_constant_part)
        		{
        			ret(i) = [&states,i](double tau){return states.get_speed(i)(tau) - states.low_frequency_velocity.get(i)(tau);};
        		}
        		else
        		{
        			ret(i) = [&states,i](double tau){return states.get_speed(i)(tau);};
        		}
        	}
        	return ret;
        }

        Vector3funct get_Ls_by_X_dot(const Vector6funct& X_dot,const BodyStates& states)
        {
        	Vector3funct ret;
        	ret(0) = [&X_dot,&states](double tau){return -states.low_frequency_velocity.V(tau)*X_dot(5)(tau);};
        	ret(1) = [&X_dot,&states](double tau){return states.low_frequency_velocity.U(tau)*X_dot(5)(tau);};
        	ret(2) = [&X_dot,&states](double tau){return states.low_frequency_velocity.V(tau)*X_dot(3)(tau)-states.low_frequency_velocity.U(tau)*X_dot(4)(tau);};
        	return ret;
        }

        Eigen::Vector3d get_at_t(const Vector3funct& Ls_by_X_dot)
        {
        	Eigen::Vector3d ret;
        	for (size_t k = 0 ; k < 3 ; ++k)
        	{
        		ret(k) = Ls_by_X_dot(k)(0.);
        	}
        	return ret;
        }

        Vector6d get_force(const BodyStates& states, double t)
        {
            Vector6d W = Vector6d::Zero();
            Vector6d Ubar = states.get_mean_generalized_speed(Tmax/2.);

            Vector6funct X_dot = get_X_dot(states);

            W -= get_convolution(K0,X_dot);

            if(forward_speed_correction)
            {
            	Vector3funct Ls_by_X_dot = get_Ls_by_X_dot(X_dot,states);

            	W += get_half_convolution(K1,Ls_by_X_dot);

            	Eigen::Vector3d Ls_by_X_dot_at_t = get_at_t(Ls_by_X_dot);

            	W += Ainf*Ls_by_X_dot_at_t;
            }

            return W;
        }

        double get_Tmax() const
        {
            return Tmax;
        }

    private:
        Impl();
        std::shared_ptr<HDBParser> hdb;
        RadiationDampingBuilder builder;
        bool forward_speed_correction;
        bool suppress_constant_part;
        Eigen::Matrix<std::function<double(double)>, 6, 6> K0;
        Eigen::Matrix<std::function<double(double)>, 6, 3> K1;
        Eigen::Matrix<double, 6, 3> Ainf;
        std::vector<double> omega;
        std::vector<double> taus;
        size_t n;
        double Tmin;
        double Tmax;
};


RadiationDampingForceModel::RadiationDampingForceModel(const RadiationDampingForceModel::Input& input, const std::string& body_name, const EnvironmentAndFrames& env) : ForceModel("radiation damping", body_name, env, YamlPosition(input.yaml.calculation_point_in_body_frame,body_name)),
pimpl(new Impl(input.hdb, input.yaml))
{}

double RadiationDampingForceModel::get_Tmax() const
{
    return pimpl->get_Tmax();
}

Vector6d RadiationDampingForceModel::get_force(const BodyStates& states, const double t, const EnvironmentAndFrames& env, const std::map<std::string,double>&) const
{
    return pimpl->get_force(states, t);
}

TypeOfQuadrature parse_type_of_quadrature_(const std::string& s);
TypeOfQuadrature parse_type_of_quadrature_(const std::string& s)
{
    if      (s == "gauss-kronrod")   return TypeOfQuadrature::GAUSS_KRONROD;
    else if (s == "rectangle")       return TypeOfQuadrature::RECTANGLE;
    else if (s == "simpson")         return TypeOfQuadrature::SIMPSON;
    else if (s == "trapezoidal")     return TypeOfQuadrature::TRAPEZOIDAL;
    else if (s == "burcher")         return TypeOfQuadrature::BURCHER;
    else if (s == "clenshaw-curtis") return TypeOfQuadrature::CLENSHAW_CURTIS;
    else if (s == "filon")           return TypeOfQuadrature::FILON;
    else
    {
        THROW(__PRETTY_FUNCTION__, InvalidInputException, "Unkown quadrature type: " << s << ". Should be one of 'gauss-kronrod', 'rectangle', ' simpson', 'trapezoidal', 'burcher', 'clenshaw-curtis' or 'filon'.";);
    }
    return TypeOfQuadrature::FILON;
}

RadiationDampingForceModel::Input RadiationDampingForceModel::parse(const std::string& yaml, const bool parse_hdb)
{
    RadiationDampingForceModel::Input ret;
    std::stringstream stream(yaml);
    std::stringstream ss;
    YAML::Parser parser(stream);
    YAML::Node node;
    parser.GetNextDocument(node);
    YamlRadiationDamping input;
    node["hdb"] >> input.hdb_filename;
    std::string s;
    node["type of quadrature for cos transform"] >> s;
    input.type_of_quadrature_for_cos_transform = parse_type_of_quadrature_(s);
    node["type of quadrature for convolution"] >> s;
    input.type_of_quadrature_for_convolution = parse_type_of_quadrature_(s);
    node["nb of points for retardation function discretization"] >> input.nb_of_points_for_retardation_function_discretization;
    ssc::yaml_parser::parse_uv(node["omega min"], input.omega_min);
    ssc::yaml_parser::parse_uv(node["omega max"], input.omega_max);
    ssc::yaml_parser::parse_uv(node["tau min"], input.tau_min);
    ssc::yaml_parser::parse_uv(node["tau max"], input.tau_max);
    node["output Br and K"] >> input.output_Br_and_K;
    if(node.FindValue("forward speed correction"))
    {
    	node["forward speed correction"] >> input.forward_speed_correction;
    }
    else
    {
    	input.forward_speed_correction=false;
    }
    if(node.FindValue("suppress constant part"))
    {
    	node["suppress constant part"] >> input.suppress_constant_part;
    }
    else
    {
    	input.suppress_constant_part=false;
    }
    node["calculation point in body frame"] >> input.calculation_point_in_body_frame;
    if (parse_hdb)
    {
        const std::shared_ptr<HDBParser> hdb(new HDBParser(ssc::text_file_reader::TextFileReader(std::vector<std::string>(1,input.hdb_filename)).get_contents()));
        ret.hdb = hdb;
    }
    ret.yaml = input;
    return ret;
}
