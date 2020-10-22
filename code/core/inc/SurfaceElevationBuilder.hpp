/*
 * SurfaceElevationBuilder.hpp
 *
 *  Created on: Jun 17, 2014
 *      Author: cady
 */

#ifndef SURFACEELEVATIONBUILDER_HPP_
#define SURFACEELEVATIONBUILDER_HPP_

#include <vector>

#include <ssc/kinematics.hpp>

#include <boost/optional/optional.hpp>
#include <ssc/macros.hpp>
#include <memory>

#include "DirectionalSpreadingBuilder.hpp"
#include "SpectrumBuilder.hpp"
#include "WaveModelBuilder.hpp"

class SurfaceElevationInterface;
struct YamlWaveOutput;

class SurfaceElevationBuilderInterface
{
    public:
        SurfaceElevationBuilderInterface(const std::shared_ptr<std::vector<WaveModelBuilderPtr> >& wave_parsers_,
                                         const std::shared_ptr<std::vector<DirectionalSpreadingBuilderPtr> >& directional_spreading_parsers_,
                                         const std::shared_ptr<std::vector<SpectrumBuilderPtr> >& spectrum_parsers_) :
                             wave_parsers(wave_parsers_),
                             directional_spreading_parsers(directional_spreading_parsers_),
                             spectrum_parsers(spectrum_parsers_)
        {}
        virtual ~SurfaceElevationBuilderInterface();
        static ssc::kinematics::PointMatrixPtr make_wave_mesh(const YamlWaveOutput& output);
        std::pair<std::size_t,std::size_t> get_wave_mesh_size(const YamlWaveOutput& output) const;
        virtual boost::optional<std::shared_ptr<SurfaceElevationInterface> > try_to_parse(const std::string& model, const std::string& yaml) const = 0;

    protected:
        SurfaceElevationBuilderInterface();
        std::shared_ptr<std::vector<WaveModelBuilderPtr> > wave_parsers;
        std::shared_ptr<std::vector<DirectionalSpreadingBuilderPtr> > directional_spreading_parsers;
        std::shared_ptr<std::vector<SpectrumBuilderPtr> > spectrum_parsers;
};

template <typename T>
class SurfaceElevationBuilder : public SurfaceElevationBuilderInterface
{
    public:
        SurfaceElevationBuilder(const std::shared_ptr<std::vector<WaveModelBuilderPtr> >& wave_parsers_,
                                const std::shared_ptr<std::vector<DirectionalSpreadingBuilderPtr> >& directional_spreading_parsers_,
                                const std::shared_ptr<std::vector<SpectrumBuilderPtr> >& spectrum_parsers_) : SurfaceElevationBuilderInterface(wave_parsers_,directional_spreading_parsers_,spectrum_parsers_)
        {}
        boost::optional<std::shared_ptr<SurfaceElevationInterface> > try_to_parse(const std::string& model, const std::string& yaml) const;

    private:
        SurfaceElevationBuilder(); // Disabled
};


typedef std::shared_ptr<SurfaceElevationBuilderInterface> SurfaceElevationBuilderPtr;
typedef std::shared_ptr<SurfaceElevationInterface> SurfaceElevationPtr;


#endif /* SURFACEELEVATIONBUILDER_HPP_ */
