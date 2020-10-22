/*
 * DirectionalSpreadingBuilder.hpp
 *
 *  Created on: Aug 7, 2014
 *      Author: cady
 */

#ifndef SPECTRUMBUILDER_HPP_
#define SPECTRUMBUILDER_HPP_

#include <boost/optional/optional.hpp>

#include <string>
#include <ssc/macros.hpp>
#include <memory>

class WaveSpectralDensity;

class SpectrumBuilderInterface
{
    public:
        SpectrumBuilderInterface() {}
        virtual ~SpectrumBuilderInterface(){}
        virtual boost::optional<std::shared_ptr<WaveSpectralDensity> > try_to_parse(const std::string& model, const std::string& yaml) const = 0;
};

template <typename T>
class SpectrumBuilder : public SpectrumBuilderInterface
{
    public:
        SpectrumBuilder() : SpectrumBuilderInterface(){}
        boost::optional<std::shared_ptr<WaveSpectralDensity> > try_to_parse(const std::string& model, const std::string& yaml) const;
};

typedef std::shared_ptr<SpectrumBuilderInterface> SpectrumBuilderPtr;

#endif /* SPECTRUMBUILDER_HPP_ */
