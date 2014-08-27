/*
 * YamlWaveModelInput.hpp
 *
 *  Created on: Aug 7, 2014
 *      Author: cady
 */

#ifndef YAMLWAVEMODELINPUT_HPP_
#define YAMLWAVEMODELINPUT_HPP_

#include <cstdlib> // For size_t
#include <string>
#include <vector>

struct YamlDiscretization
{
    YamlDiscretization();
    size_t n;         //!< Nb of frequencies in the discretization
    double omega_min; //!< First angular frequency (in rad/s)
    double omega_max; //!< Last angular frequency (in rad/s)
};

struct YamlSpectra
{
    YamlSpectra();
    std::string model;                      //!< Wave model (eg. airy)
    std::string model_yaml;                 //!< Model data in YAML format
    std::string directional_spreading_type; //!< Spreading type (eg. cos2s)
    std::string directional_spreading_yaml; //!< Model data in YAML format
    std::string spectral_density_type;      //!< Type of spectrum (eg. jonswap)
    std::string spectral_density_yaml;      //!< Spectral model parameters in YAML format
    double depth;                           //!< Water depth (in meters): 0 for infinite depth
};

struct YamlWaveOutput
{
    YamlWaveOutput();
    std::string format;              //!< Format of the output (eg. YAML, OBJ, HDF5...)
    std::string frame_of_reference;  //!< Name of the frame of reference the x & y coordinates are expressed in
    std::vector<double> x;           //!< x-coordinate of the points we wish to observe (in frame 'frame_of_reference')
    std::vector<double> y;           //!< y-coordinate of the points we wish to observe (in frame 'frame_of_reference')
};

struct YamlWaveModel
{
    YamlWaveModel();
    YamlDiscretization discretization; //!< Spectral discretization parameters
    std::vector<YamlSpectra> spectra;  //!< Wave spectra to generate
    YamlWaveOutput output;             //!< Defines what wave data is outputted during the simulation & how it is generated
};

struct YamlJonswap
{
    YamlJonswap();
    double Hs;    //!< Significant wave height (in meters)
    double Tp;    //!< Mean wave period (in seconds)
    double gamma; //!< Non-dimensional peak shape parameter
};

struct YamlBretschneider
{
    YamlBretschneider();
    double Hs;    //!< Significant wave height (in meters)
    double Tp;    //!< Mean wave period (in seconds)
};

struct YamlCos2s
{
    YamlCos2s();
    double psi0; //!< Primary wave direction in radians.
    double s;    //!< Exponent
};

struct YamlDiracDirection
{
    YamlDiracDirection();
    double psi0; //!< Primary wave direction in radians.
};

struct YamlDiracSpectrum
{
    YamlDiracSpectrum();
    double omega0; //!< Angular frequency (\f$2\pi f\f$) in rad/s of the significant wave height
    double Hs;     //!< Significant wave height (in meters)
};

struct YamlPiersonMoskowitz
{
    YamlPiersonMoskowitz();
    double Hs;     //!< Significant wave height (in meters)
    double Tp;    //!< Mean wave period (in seconds)
};

struct YamlDefaultWaveModel
{
    YamlDefaultWaveModel();
    double zwave;          //!< Constant free surface elevation (in meters)
    YamlWaveOutput output; //!< Defines what wave data is outputted during the simulation & how it is generated
};

#endif /* YAMLWAVEMODELINPUT_HPP_ */
