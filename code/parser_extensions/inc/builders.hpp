/*
 * builders.hpp
 *
 *  Created on: Aug 12, 2014
 *      Author: cady
 */

#ifndef BUILDERS_HPP_
#define BUILDERS_HPP_

#include "ActualWindModel.hpp"
#include "DefaultSurfaceElevation.hpp"
#include "SurfaceElevationBuilder.hpp"
#include "DiracDirectionalSpreading.hpp"
#include "DiscreteDirectionalWaveSpectrum.hpp"
#include "SurfaceElevationFromWaves.hpp"
#include "Airy.hpp"
#include "BretschneiderSpectrum.hpp"
#include "Cos2sDirectionalSpreading.hpp"
#include "DiracSpectralDensity.hpp"
#include "JonswapSpectrum.hpp"
#include "PiersonMoskowitzSpectrum.hpp"
#include "WindModelInterface.hpp"
#include "WindModelBuilder.hpp"
#include "DefaultWindModel.hpp"
#include "WindMeanVelocityProfile.hpp"
#include "WindTurbulenceModel.hpp"
#include "UniformWindProfile.hpp"
#include "NoWindTurbulence.hpp"

typedef std::shared_ptr<SurfaceElevationInterface> SurfaceElevationInterfacePtr;
typedef std::shared_ptr<WaveSpectralDensity> WaveSpectralDensityPtr;
typedef std::shared_ptr<WaveDirectionalSpreading> WaveDirectionalSpreadingPtr;

typedef std::shared_ptr<WindModelInterface> WindModelInterfacePtr;
/*typedef std::shared_ptr<WindMeanVelocityProfile> WindMeanVelocityProfileePtr;
typedef std::shared_ptr<WindTurbulenceModel> WindTurbulenceModelPtr;*/

template <>
class SurfaceElevationBuilder<DefaultSurfaceElevation> : public SurfaceElevationBuilderInterface
{
    public:
        SurfaceElevationBuilder(const std::shared_ptr<std::vector<DirectionalSpreadingBuilderPtr> >& directional_spreading_parsers_,
                                const std::shared_ptr<std::vector<SpectrumBuilderPtr> >& spectrum_parsers_);
        boost::optional<SurfaceElevationInterfacePtr> try_to_parse(const std::string& model, const std::string& yaml) const;
};

struct YamlDiscretization;
struct YamlSpectra;

template <>
class SurfaceElevationBuilder<SurfaceElevationFromWaves> : public SurfaceElevationBuilderInterface
{
    public:
        SurfaceElevationBuilder(const std::shared_ptr<std::vector<DirectionalSpreadingBuilderPtr> >& directional_spreading_parsers_,
                                const std::shared_ptr<std::vector<SpectrumBuilderPtr> >& spectrum_parsers_);
        boost::optional<SurfaceElevationInterfacePtr> try_to_parse(const std::string& model, const std::string& yaml) const;

    private:
        SurfaceElevationBuilder();
        WaveModelPtr parse_wave_model(const YamlDiscretization& discretization, const YamlSpectra& spectrum) const;
        DiscreteDirectionalWaveSpectrum parse_directional_spectrum(const YamlDiscretization& discretization, const YamlSpectra& spectrum) const;
        WaveSpectralDensityPtr parse_spectral_density(const YamlSpectra& spectrum) const;
        WaveDirectionalSpreadingPtr parse_directional_spreading(const YamlSpectra& spectrum) const;
};

class SurfaceElevationFromGRPC;
template <>
class SurfaceElevationBuilder<SurfaceElevationFromGRPC> : public SurfaceElevationBuilderInterface
{
    public:
        SurfaceElevationBuilder();
        boost::optional<SurfaceElevationInterfacePtr> try_to_parse(const std::string& model, const std::string& yaml) const;
};

template <>
class WaveModelBuilder<Airy> : public WaveModelBuilderInterface
{
    public:
        WaveModelBuilder();
        boost::optional<WaveModelPtr> try_to_parse(const std::string& model, const DiscreteDirectionalWaveSpectrum& spectrum, const std::string& yaml) const;
};

template <>
class SpectrumBuilder<BretschneiderSpectrum> : public SpectrumBuilderInterface
{
    public:
        SpectrumBuilder();
        boost::optional<WaveSpectralDensityPtr> try_to_parse(const std::string& model, const std::string& yaml) const;
};

template <>
class SpectrumBuilder<JonswapSpectrum> : public SpectrumBuilderInterface
{
    public:
        SpectrumBuilder();
        boost::optional<WaveSpectralDensityPtr> try_to_parse(const std::string& model, const std::string& yaml) const;
};

template <>
class SpectrumBuilder<PiersonMoskowitzSpectrum> : public SpectrumBuilderInterface
{
    public:
        SpectrumBuilder();
        boost::optional<WaveSpectralDensityPtr> try_to_parse(const std::string& model, const std::string& yaml) const;
};

template <>
class SpectrumBuilder<DiracSpectralDensity> : public SpectrumBuilderInterface
{
    public:
        SpectrumBuilder();
        boost::optional<WaveSpectralDensityPtr> try_to_parse(const std::string& model, const std::string& yaml) const;
};

template <>
class DirectionalSpreadingBuilder<DiracDirectionalSpreading> : public DirectionalSpreadingBuilderInterface
{
    public:
        DirectionalSpreadingBuilder() : DirectionalSpreadingBuilderInterface(){}
        boost::optional<WaveDirectionalSpreadingPtr> try_to_parse(const std::string& model, const std::string& yaml) const;
};

template <>
class DirectionalSpreadingBuilder<Cos2sDirectionalSpreading> : public DirectionalSpreadingBuilderInterface
{
    public:
        DirectionalSpreadingBuilder() : DirectionalSpreadingBuilderInterface(){}
        boost::optional<WaveDirectionalSpreadingPtr> try_to_parse(const std::string& model, const std::string& yaml) const;
};

template <>
class WindModelBuilder<DefaultWindModel> : public WindModelBuilderInterface
{
    public:
        WindModelBuilder(const std::shared_ptr<std::vector<WindMeanVelocityProfileBuilderPtr> >& velocity_profile_parsers_,
                                const std::shared_ptr<std::vector<WindTurbulenceModelBuilderPtr> >& turbulence_model_parsers_);
        boost::optional<WindModelInterfacePtr> try_to_parse(const std::string& model, const std::string& yaml) const;
};

struct YamlWindProfile;
struct YamlWindTurbulence;

template <>
class WindModelBuilder<ActualWindModel> : public WindModelBuilderInterface
{
    public:
        WindModelBuilder(const std::shared_ptr<std::vector<WindMeanVelocityProfileBuilderPtr> >& velocity_profile_parsers_,
                                const std::shared_ptr<std::vector<WindTurbulenceModelBuilderPtr> >& turbulence_model_parsers_);
        boost::optional<WindModelInterfacePtr> try_to_parse(const std::string& model, const std::string& yaml) const;

    private:
        WindModelBuilder();
        WindMeanVelocityProfilePtr parse_wind_velocity_profile(const YamlWindProfile& velocity_profile) const;
        WindTurbulenceModelPtr parse_wind_turbulence_model(const YamlWindTurbulence& turbulence_model) const;
};

template <>
class WindMeanVelocityProfileBuilder<UniformWindProfile> : public WindMeanVelocityProfileBuilderInterface
{
    public:
	WindMeanVelocityProfileBuilder();
        boost::optional<WindMeanVelocityProfilePtr> try_to_parse(const std::string& model, const std::string& yaml) const;
};

template <>
class WindTurbulenceModelBuilder<NoWindTurbulence> : public WindTurbulenceModelBuilderInterface
{
    public:
	WindTurbulenceModelBuilder();
        boost::optional<WindTurbulenceModelPtr> try_to_parse(const std::string& model, const std::string& yaml) const;
};



#endif /* BUILDERS_HPP_ */
