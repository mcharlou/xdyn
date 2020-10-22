/*
 * WindTurbulenceModelBuilder.hpp
 *
 *  Created on: 8 janv. 2020
 *      Author: mcharlou2016
 */

#ifndef CORE_INC_WINDTURBULENCEMODELBUILDER_HPP_
#define CORE_INC_WINDTURBULENCEMODELBUILDER_HPP_

#include <boost/optional/optional.hpp>

#include <string>
#include <ssc/macros.hpp>
#include <memory>

class WindTurbulenceModel;

class WindTurbulenceModelBuilderInterface
{
    public:
        WindTurbulenceModelBuilderInterface() {}
        virtual ~WindTurbulenceModelBuilderInterface(){}
        virtual boost::optional<std::shared_ptr<WindTurbulenceModel> > try_to_parse(const std::string& model, const std::string& yaml) const = 0;
};

template <typename T>
class WindTurbulenceModelBuilder : public WindTurbulenceModelBuilderInterface
{
    public:
        WindTurbulenceModelBuilder() : WindTurbulenceModelBuilderInterface(){}
        boost::optional<std::shared_ptr<WindTurbulenceModel> > try_to_parse(const std::string& model, const std::string& yaml) const;
};

typedef std::shared_ptr<WindTurbulenceModelBuilderInterface> WindTurbulenceModelBuilderPtr;

#endif /* CORE_INC_WINDTURBULENCEMODELBUILDER_HPP_ */
