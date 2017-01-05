/*
 * Stretching.cpp
 *
 *  Created on: Dec 20, 2016
 *      Author: cady
 */

#include "YamlWaveModelInput.hpp"
#include "Stretching.hpp"
#include <ssc/macros.hpp>
Stretching::Stretching(const YamlStretching& input //!< Usually read from YAML
                  )
: delta(input.delta)
, h(input.h)
{}

double Stretching::rescaled_z(const double z, //!< z value we wish to rescale (in meters)
                              const double ksi //!< Wave height (in meters), z being oriented downwards
                             ) const
{
    if (h == 0) return z; // No stretching if h = 0
    if (z > h)  return z;
                return (z-h)*(delta*ksi-h)/(ksi-h)+h;
}