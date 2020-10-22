/*
 * DiffractionForceModel.cpp
 *
 *  Created on: Oct 24, 2014
 *      Author: cady
 */

#include <cmath>
#include <array>
#include <ssc/interpolation.hpp>
#include <ssc/text_file_reader.hpp>

#include "Body.hpp"
#include "DiffractionInterpolator.hpp"
#include "HDBParser.hpp"
#include "InvalidInputException.hpp"
#include "SurfaceElevationInterface.hpp"
#include "yaml.h"
#include "external_data_structures_parsers.hpp"
#include "yaml2eigen.hpp"

#include "DiffractionForceModel.hpp"

#define TWOPI 6.283185307179586232
#define PI M_PI

std::string DiffractionForceModel::model_name() {return "diffraction";}

HDBParser hdb_from_file(const std::string& filename);
HDBParser hdb_from_file(const std::string& filename)
{
    return HDBParser(ssc::text_file_reader::TextFileReader(filename).get_contents());
}

void check_all_omegas_are_within_bounds(const double min_bound, const std::vector<std::vector<double> >& vector_to_check, const double max_bound);
void check_all_omegas_are_within_bounds(const double min_bound, const std::vector<std::vector<double> >& vector_to_check, const double max_bound)
{
    const double eps = 0.01; // We don't care if we're above or below the bounds by 0.01 s: those are wave frequencies so not very precise.
    for (auto t:vector_to_check)
    {
        for (auto Tp:t)
        {
            if (Tp<(min_bound-eps))
            {
                THROW(__PRETTY_FUNCTION__, InvalidInputException, "HDB used by DiffractionForceModel only defines the RAO for wave period Tp within [" << min_bound << "," << max_bound << "]"
                     << " s, but wave spectrum discretization contains Tp = " << Tp
                     << " s which is below the min bound by " << min_bound-Tp << " s: you need to modify the section 'environment models/model: waves/discretization' in the YAML file or the 'spectrum' section or change the HDB file ")
                ;
            }
            if (Tp>(max_bound+eps))
            {
                THROW(__PRETTY_FUNCTION__, InvalidInputException, "HDB used by DiffractionForceModel only defines the RAO for wave period Tp within [" << min_bound << "," << max_bound << "]"
                     << " s, but wave spectrum discretization contains Tp = " << Tp
                     << " s which is above the max bound by " << Tp-max_bound << " s: you need to modify the section 'environment models/model: waves/discretization' in the YAML file or the 'spectrum' section or change the HDB file ")
                ;
            }
        }
    }
}

std::vector<std::vector<double> > convert_to_periods(std::vector<std::vector<double> > angular_frequencies);
std::vector<std::vector<double> > convert_to_periods(std::vector<std::vector<double> > angular_frequencies)
{
    for (auto& omegas:angular_frequencies)
    {
        for (auto& omega:omegas)
        {
            omega = TWOPI/omega;
        }
    }
    return angular_frequencies;
}


class DiffractionForceModel::Impl
{
    public:

        Impl(const YamlDiffraction& data, const EnvironmentAndFrames& env, const HDBParser& hdb, const std::string& body_name)
          : initialized(false), use_encounter_frequency(data.use_encounter_frequency),
        rao(DiffractionInterpolator(hdb,std::vector<double>(),std::vector<double>(),data.mirror)),
		calculation_point_in_body_frame(body_name,data.calculation_point.x,data.calculation_point.y,data.calculation_point.z),
        periods_for_each_direction(),
        psis()
        {
            if (env.w.use_count()>0)
            {
                // For each directional spectrum (i.e. for each direction), the wave angular frequencies the spectrum was discretized at.
                // periods[direction][omega]
                try
                {
                    periods_for_each_direction = convert_to_periods(env.w->get_wave_angular_frequency_for_each_model());
                }
                catch (const ssc::exception_handling::Exception& e)
                {
                    THROW(__PRETTY_FUNCTION__, ssc::exception_handling::Exception, "This simulation uses the diffraction force model which uses the spectral discretization (in angular frequency) of the wave models. When querying the wave model for this discretization, the following problem occurred:\n" << e.get_message());
                }
                const auto hdb_periods = hdb.get_diffraction_module_periods();
                if (not(hdb_periods.empty()))
                {
                    check_all_omegas_are_within_bounds(hdb_periods.front(), periods_for_each_direction, hdb_periods.back());
                }
                try
                {
                    psis = env.w->get_wave_directions_for_each_model();
                }
                catch (const ssc::exception_handling::Exception& e)
                {
                    THROW(__PRETTY_FUNCTION__, ssc::exception_handling::Exception, "This simulation uses the diffraction force model which uses the spatial discretization (in incidence) of the wave models. When querying the wave model for this discretization, the following problem occurred:\n" << e.get_message());
                }
            }
            else
            {
                THROW(__PRETTY_FUNCTION__, InvalidInputException, "Force model '" << DiffractionForceModel::model_name << "' needs a wave model, even if it's 'no waves'");
            }
        }

        Vector6d evaluate(const std::string& body_name, const double t, const BodyStates& states, const EnvironmentAndFrames& env)
        {
            Vector6d w;
            auto T = env.k->get("NED", body_name);
            T.swap();
            // TODO: check transformation
            const ssc::kinematics::Point position_in_ned_for_the_wave_model = T*calculation_point_in_body_frame; //The known frame is always the body frame as per the input definition
            std::array<std::vector<std::vector<double> >, 6 > rao_modules;
            std::array<std::vector<std::vector<double> >, 6 > rao_phases;
            if (env.w.use_count()>0)
            {
                const size_t nb_of_spectra = periods_for_each_direction.size();
                if (not(periods_for_each_direction.empty()))
                {
                    // Resize for each degree of freedom
                    for (size_t k = 0 ; k < 6 ; ++k)
                    {
                        rao_modules[k].resize(nb_of_spectra);
                        rao_phases[k].resize(nb_of_spectra);
                    }
                }
                for (size_t degree_of_freedom_idx = 0 ; degree_of_freedom_idx < 6 ; ++degree_of_freedom_idx) // For each degree of freedom (X, Y, Z, K, M, N)
                {
                    for (size_t spectrum_idx = 0 ; spectrum_idx < nb_of_spectra ; ++spectrum_idx) // For each directional spectrum
                    {
                        const size_t nb_of_period_incidence_pairs = periods_for_each_direction[spectrum_idx].size();
                        rao_modules[degree_of_freedom_idx][spectrum_idx].resize(nb_of_period_incidence_pairs);
                        rao_phases[degree_of_freedom_idx][spectrum_idx].resize(nb_of_period_incidence_pairs);
                        for (size_t omega_beta_idx = 0 ; omega_beta_idx < nb_of_period_incidence_pairs ; ++omega_beta_idx) // For each incidence and each period (omega[i[omega_beta_idx]], beta[j[omega_beta_idx]])
                        {
                            // Wave incidence
                        	const double psi_w = psis.at(spectrum_idx).at(omega_beta_idx);
                            const double beta = states.get_angles().psi - psi_w;
                            // Encounter angle
                            Eigen::Vector3d Vs_NED = env.k->get("NED", states.name).get_rot()*states.low_frequency_velocity.get_speed();
                            Eigen::Vector3d Vs_plan(Vs_NED(0),Vs_NED(1),0.);
                            Eigen::Vector3d d_w(cos(psi_w),sin(psi_w),0);
                            // Encounter period
                            double Tw = periods_for_each_direction[spectrum_idx][omega_beta_idx];
                            if(use_encounter_frequency) Tw = Tw/(1-2*PI*Vs_plan.dot(d_w)/Tw/env.g); // using encounter frequency instead of just wave frequency

                            // Interpolate RAO module for this axis, period and incidence
                            rao_modules[degree_of_freedom_idx][spectrum_idx][omega_beta_idx] = rao.interpolate_module(degree_of_freedom_idx, Tw, beta);
                            // Interpolate RAO phase for this axis, period and incidence
                            rao_phases[degree_of_freedom_idx][spectrum_idx][omega_beta_idx] = -rao.interpolate_phase(degree_of_freedom_idx, Tw, beta);
                        }
                    }
                    try
                    {
                        w((int)degree_of_freedom_idx) = env.w->evaluate_rao(position_in_ned_for_the_wave_model.x(),
                                                        position_in_ned_for_the_wave_model.y(),
                                                        t,
                                                        rao_modules[degree_of_freedom_idx],
                                                        rao_phases[degree_of_freedom_idx]);
                    }
                    catch (const ssc::exception_handling::Exception& e)
                    {
                        THROW(__PRETTY_FUNCTION__, ssc::exception_handling::Exception, "This simulation uses the diffraction force model which evaluates a Response Amplitude Operator using a wave model. During this evaluation, the following problem occurred:\n" << e.get_message());
                    }
                }
            }
            const auto ww = express_aquaplus_wrench_in_xdyn_coordinates(w);
            return ww;
        }

        Vector6d express_aquaplus_wrench_in_xdyn_coordinates(Vector6d v) const
        {
            v(0) *= -1;
            v(3) *= -1;
            return v;
        }

    private:
        Impl();
        bool initialized;
        bool use_encounter_frequency;
        DiffractionInterpolator rao;
        ssc::kinematics::Point calculation_point_in_body_frame;
        std::vector<std::vector<double> > periods_for_each_direction;
        std::vector<std::vector<double> > psis;

};

DiffractionForceModel::DiffractionForceModel(const Input& data, const std::string& body_name, const EnvironmentAndFrames& env)
: ForceModel(DiffractionForceModel::model_name(), body_name, env, YamlPosition(data.calculation_point,body_name)), pimpl(new Impl(data,env,hdb_from_file(data.hdb_filename), body_name))
{
}

DiffractionForceModel::DiffractionForceModel(const Input& data, const std::string& body_name, const EnvironmentAndFrames& env, const std::string& hdb_file_contents)
 : ForceModel(DiffractionForceModel::model_name(), body_name, env, YamlPosition(data.calculation_point,body_name)), pimpl(new Impl(data,env,HDBParser(hdb_file_contents), body_name))
{
}

Vector6d DiffractionForceModel::get_force(const BodyStates& states, const double t, const EnvironmentAndFrames& env, const std::map<std::string,double>&) const
{
    return pimpl->evaluate(states.name, t, states, env);
}

DiffractionForceModel::Input DiffractionForceModel::parse(const std::string& yaml)
{
    std::stringstream stream(yaml);
    YAML::Parser parser(stream);
    YAML::Node node;
    parser.GetNextDocument(node);
    YamlDiffraction ret;
    node["hdb"]                             >> ret.hdb_filename;
    node["calculation point in body frame"] >> ret.calculation_point;
    node["mirror for 180 to 360"]           >> ret.mirror;
    if(node.FindValue("use encounter frequency")) node["use encounter frequency"] >> ret.use_encounter_frequency;
    return ret;
}
