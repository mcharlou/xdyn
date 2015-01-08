/*
 * DampingForceModel.cpp
 *
 *  Created on: Oct 2, 2014
 *      Author: jacquenot
 */

#include <array>

#define _USE_MATH_DEFINE
#include <cmath>
#define PI M_PI

#include "Body.hpp"
#include "HDBParser.hpp"
#include "History.hpp"
#include "RadiationDampingBuilder.hpp"
#include "RadiationDampingForceModel.hpp"
#include "RadiationDampingForceModelException.hpp"

#include <ssc/macros.hpp>
#include <ssc/text_file_reader.hpp>
#include <cassert>
#include <sstream>

#include "yaml.h"
#include "external_data_structures_parsers.hpp"
#include "parse_unit_value.hpp"

const std::string RadiationDampingForceModel::model_name = "radiation damping";

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
        Impl(const TR1(shared_ptr)<HDBParser>& parser, const YamlRadiationDamping& yaml) : hdb{parser}, h(), builder(RadiationDampingBuilder(yaml.type_of_quadrature_for_convolution, yaml.type_of_quadrature_for_cos_transform)), K(),
        omega(parser->get_radiation_damping_angular_frequencies()), taus(), n(yaml.nb_of_points_for_retardation_function_discretization), Tmin(yaml.tau_min), Tmax(yaml.tau_max),
        H0(yaml.calculation_point_in_body_frame.x,yaml.calculation_point_in_body_frame.y,yaml.calculation_point_in_body_frame.y)
        {
            CSVWriter omega_writer(std::cerr, "omega", omega);
            taus = builder.build_regular_intervals(Tmin,Tmax,n);
            CSVWriter tau_writer(std::cerr, "tau", taus);

            for (size_t i = 0 ; i < 6 ; ++i)
            {
                for (size_t j = 0 ; j < 6 ; ++j)
                {
                    const auto Br = get_Br(i,j);
                    K[i][j] = get_K(Br);
                    if (yaml.output_Br_and_K)
                    {
                        omega_writer.add("Br",Br,i+1,j+1);
                        tau_writer.add("K",K[i][j],i+1,j+1);
                    }
                }
                h.at(i) = History(Tmax);
            }
            if (yaml.output_Br_and_K)
            {
                std::cerr << "Debugging information for damping functions Br:" << std::endl;
                omega_writer.print();
                std::cerr << std::endl << "Debugging information for retardation functions K:" << std::endl;
                tau_writer.print();
            }
        }

        std::function<double(double)> get_Br(const size_t i, const size_t j) const
        {
            return builder.build_interpolator(omega,hdb->get_radiation_damping_coeff(i,j));
        }

        std::function<double(double)> get_K(const std::function<double(double)>& Br) const
        {
            return builder.build_retardation_function(Br,taus,1E-3,omega.front(),omega.back());
        }

        double get_convolution_for_axis(const size_t i) const
        {
            double K_X_dot = 0;
            for (size_t k = 0 ; k < 6 ; ++k)
            {
                if (h[k].get_length() >= Tmin)
                {
                    // Integrate up to Tmax if possible, but never exceed the history length
                    const double co = builder.convolution(h[k], K[i][k], Tmin, std::min(Tmax, h[k].get_length()));
                    K_X_dot += co;
                }
            }
            return K_X_dot;
        }

        ssc::kinematics::Wrench get_wrench(const Body& b, const double t)
        {
            save_state(b, t);
            ssc::kinematics::Vector6d W;
            const ssc::kinematics::Point H(b.name,H0);

            W(0) = get_convolution_for_axis(0);
            W(1) = get_convolution_for_axis(1);
            W(2) = get_convolution_for_axis(2);
            W(3) = get_convolution_for_axis(3);
            W(4) = get_convolution_for_axis(4);
            W(5) = get_convolution_for_axis(5);
            return ssc::kinematics::Wrench(b.name,W);
        }

        void save_state(const Body& b, const double t)
        {
            h[0].record(t,b.u);
            h[1].record(t,b.v);
            h[2].record(t,b.w);
            h[3].record(t,b.p);
            h[4].record(t,b.q);
            h[5].record(t,b.r);
        }

    private:
        Impl();
        TR1(shared_ptr)<HDBParser> hdb;
        std::array<History, 6> h;
        RadiationDampingBuilder builder;
        std::array<std::array<std::function<double(double)>,6>, 6> K;
        std::vector<double> omega;
        std::vector<double> taus;
        size_t n;
        double Tmin;
        double Tmax;
        Eigen::Vector3d H0;
};


RadiationDampingForceModel::RadiationDampingForceModel(const RadiationDampingForceModel::Input& input, const EnvironmentAndFrames& ) : ForceModel("radiation damping"),
pimpl(new Impl(input.hdb, input.yaml))
{
}

ssc::kinematics::Wrench RadiationDampingForceModel::operator()(const Body& b, const double t) const
{
    return pimpl->get_wrench(b, t);
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
        std::stringstream ss;
        ss << "Unkown quadrature type: " << s << ". Should be one of 'gauss-kronrod', 'rectangle', ' simpson', 'trapezoidal', 'burcher', 'clenshaw-curtis' or 'filon'.";
        THROW(__PRETTY_FUNCTION__, RadiationDampingForceModelException, ss.str());
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
    parse_uv(node["omega min"], input.omega_min);
    parse_uv(node["omega max"], input.omega_max);
    parse_uv(node["tau min"], input.tau_min);
    parse_uv(node["tau max"], input.tau_max);
    node["output Br and K"] >> input.output_Br_and_K;
    node["calculation point in body frame"] >> input.calculation_point_in_body_frame;
    if (parse_hdb)
    {
        const TR1(shared_ptr)<HDBParser> hdb(new HDBParser(ssc::text_file_reader::TextFileReader(std::vector<std::string>(1,input.hdb_filename)).get_contents()));
        ret.hdb = hdb;
    }
    ret.yaml = input;
    return ret;
}
