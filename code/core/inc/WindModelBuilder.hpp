/*
 * WindModelBuilder.h
 *
 *  Created on: 8 janv. 2020
 *      Author: mcharlou2016
 */

#ifndef CORE_INC_WINDMODELBUILDER_HPP_
#define CORE_INC_WINDMODELBUILDER_HPP_

#include <boost/optional/optional.hpp>
#include <string>
#include <vector>

#include "WindMeanVelocityProfileBuilder.hpp"
#include "WindTurbulenceModelBuilder.hpp"

#include <ssc/macros.hpp>
#include <memory>

class WindModelInterface;

class WindModelBuilderInterface
{
    public:
        WindModelBuilderInterface(const std::shared_ptr<std::vector<WindMeanVelocityProfileBuilderPtr> >& velocity_profile_parsers_,
                                  const std::shared_ptr<std::vector<WindTurbulenceModelBuilderPtr> >& turbulence_model_parsers_) :
                                  velocity_profile_parsers(velocity_profile_parsers_),
                                  turbulence_model_parsers(turbulence_model_parsers_)
                                  {}
        virtual ~WindModelBuilderInterface() {}
        virtual boost::optional<std::shared_ptr<WindModelInterface> > try_to_parse(const std::string& model, const std::string& yaml) const = 0;

    protected:
        WindModelBuilderInterface();
        std::shared_ptr<std::vector<WindMeanVelocityProfileBuilderPtr> > velocity_profile_parsers;
        std::shared_ptr<std::vector<WindTurbulenceModelBuilderPtr> > turbulence_model_parsers;
};

template <typename T> class WindModelBuilder : public WindModelBuilderInterface
{
public:
	WindModelBuilder(const std::shared_ptr<std::vector<WindMeanVelocityProfileBuilderPtr> >& velocity_profile_parsers_,
                         const std::shared_ptr<std::vector<WindTurbulenceModelBuilderPtr> >& turbulence_model_parsers_) :
                         WindModelBuilderInterface(velocity_profile_parsers_,turbulence_model_parsers_)
                         {}
	boost::optional<std::shared_ptr<WindModelInterface> > try_to_parse(const std::string& model, const std::string& yaml) const;

private:
	WindModelBuilder();
};

typedef std::shared_ptr<WindModelBuilderInterface> WindModelBuilderPtr;
typedef std::shared_ptr<WindModelInterface> WindModelPtr;

#endif /* CORE_INC_WINDMODELBUILDER_HPP_ */
