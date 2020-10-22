/*
 * YamlRadiationDamping.cpp
 *
 *  Created on: Nov 27, 2014
 *      Author: cady
 */

#include "YamlRadiationDamping.hpp"

YamlRadiationDamping::YamlRadiationDamping() : hdb_filename(),
                                               type_of_quadrature_for_cos_transform(),
                                               type_of_quadrature_for_convolution(),
                                               nb_of_points_for_retardation_function_discretization(0),
                                               omega_min(0),
                                               omega_max(0),
                                               tau_min(0),
                                               tau_max(0),
											   forward_speed_correction(),
											   suppress_constant_part(),
                                               output_Br_and_K(),
                                               calculation_point_in_body_frame()
{
}
