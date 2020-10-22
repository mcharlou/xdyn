/*
 * WaveModelBuilder.hpp
 *
 *  Created on: Aug 12, 2014
 *      Author: cady
 */

#ifndef WAVEMODELBUILDER_HPP_
#define WAVEMODELBUILDER_HPP_

#include <ssc/macros.hpp>
#include <memory>
#include <boost/optional/optional.hpp>
#include <string>
#include <vector>

class WaveModel;
struct DiscreteDirectionalWaveSpectrum;

/** \brief Interface to all WaveModelBuilder<T>. Used to build, eg. Airy
 *  \details Allows us to store WaveModelBuilders with different template
 *  parameters inside a same collection (eg. vector<WaveModelBuilderPtr>)
 *  \addtogroup simulator
 *  \ingroup simulator
 */
class WaveModelBuilderInterface
{
    public:
        WaveModelBuilderInterface(const std::shared_ptr<std::vector<DirectionalSpreadingBuilderPtr> >& directional_spreading_parsers_,
                                  const std::shared_ptr<std::vector<SpectrumBuilderPtr> >& spectrum_parsers_) : directional_spreading_parsers(directional_spreading_parsers_),
                                  spectrum_parsers(spectrum_parsers_)
                                  {}
        virtual ~WaveModelBuilderInterface() {}
        virtual boost::optional<std::shared_ptr<WaveModel> > try_to_parse(const std::string& model, const DiscreteDirectionalWaveSpectrum& spectrum, const std::string& yaml) const = 0;

    protected:
        std::shared_ptr<std::vector<DirectionalSpreadingBuilderPtr> > directional_spreading_parsers;
        std::shared_ptr<std::vector<SpectrumBuilderPtr> > spectrum_parsers;

};

template <typename T> class WaveModelBuilder : public WaveModelBuilderInterface
{
    public:
        WaveModelBuilder(const std::shared_ptr<std::vector<DirectionalSpreadingBuilderPtr> >& directional_spreading_parsers_,
                         const std::shared_ptr<std::vector<SpectrumBuilderPtr> >& spectrum_parsers_) : WaveModelBuilderInterface(directional_spreading_parsers_,spectrum_parsers_) {}
        ~WaveModelBuilder() {}
        boost::optional<std::shared_ptr<WaveModel> > try_to_parse(const std::string& model, const DiscreteDirectionalWaveSpectrum& spectrum, const std::string& yaml) const;
};

typedef std::shared_ptr<WaveModelBuilderInterface> WaveModelBuilderPtr;

#endif /* WAVEMODELBUILDER_HPP_ */
