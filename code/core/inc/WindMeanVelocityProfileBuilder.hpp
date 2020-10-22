/*
 * WindMeanVelocityProfileBuilder.hpp
 *
 *  Created on: 8 janv. 2020
 *      Author: mcharlou2016
 */

#ifndef CORE_INC_WINDMEANVELOCITYPROFILEBUILDER_HPP_
#define CORE_INC_WINDMEANVELOCITYPROFILEBUILDER_HPP_

#include <boost/optional/optional.hpp>

#include <string>

#include <ssc/macros.hpp>
#include <memory>

class WindMeanVelocityProfile;

class WindMeanVelocityProfileBuilderInterface
{
    public:
        WindMeanVelocityProfileBuilderInterface() {}
        virtual ~WindMeanVelocityProfileBuilderInterface(){}
        virtual boost::optional<std::shared_ptr<WindMeanVelocityProfile> > try_to_parse(const std::string& model, const std::string& yaml) const = 0;
};

template <typename T>
class WindMeanVelocityProfileBuilder : public WindMeanVelocityProfileBuilderInterface
{
    public:
        WindMeanVelocityProfileBuilder() : WindMeanVelocityProfileBuilderInterface(){}
        boost::optional<std::shared_ptr<WindMeanVelocityProfile> > try_to_parse(const std::string& model, const std::string& yaml) const;
};

typedef std::shared_ptr<WindMeanVelocityProfileBuilderInterface> WindMeanVelocityProfileBuilderPtr;

#endif /* CORE_INC_WINDMEANVELOCITYPROFILEBUILDER_HPP_ */
