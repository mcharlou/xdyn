/*
 * RadiationDampingBuilder.hpp
 *
 *  Created on: Nov 27, 2014
 *      Author: cady
 */

#ifndef RADIATIONDAMPINGBUILDER_HPP_
#define RADIATIONDAMPINGBUILDER_HPP_


#include <vector>
#include <functional>

#include "YamlRadiationDamping.hpp"

class History;

/** \brief Creates interpolation functions for the damping matrix
 *  \details Used to create the retardation functions
 *  \addtogroup hdb_interpolators
 *  \ingroup hdb_interpolators
 */
class RadiationDampingBuilder
{
    public:
        RadiationDampingBuilder(
                const TypeOfQuadrature& type_of_quadrature_for_convolution, //!< Gauss-Kronrod, rectangle, Simpson, trapezoidal, Burcher, Clenshaw-Curtis or Filon
                const TypeOfQuadrature& type_of_quadrature_for_cos_transform //!< Gauss-Kronrod, rectangle, Simpson, trapezoidal, Burcher, Clenshaw-Curtis or Filon
                );

        /**  \brief Build a continuous function from discrete (x,y) points (using interpolation type defined in constructor)
          */
        std::function<double(double)> build_interpolator(const std::vector<double>& x, const std::vector<double>& y) const;

        /**  \brief Use radiation damping function to compute retardation function
          *  \details Radiation damping can come, eg. from the build_interpolator method. This method evaluates the
          *  integral \f$K(\tau) := \frac{2}{\pi} int_{\omega_{\mbox{min}}}^{\omega_{\mbox{max}}} B_r(\omega)\cos(\omega\cdot \tau)d\tau\f$
          *  for n different values of \f$\tau\f$ between \f$\frac{2\pi}{\omega{\mbox{max}}} and \frac{2\pi}{\omega{\mbox{min}}}\f$.
          *  It then uses "build_interpolator" to build a function (lambda).
          */
        std::function<double(double)> build_retardation_function(
                const std::function<double(double)>& Br,    //!< Radiation damping function
                const std::vector<double>& taus,            //!<
                const double eps,                           //!< When to truncate (0 for no truncation)
                const double omega_min,
                double omega_max
                ) const;

        std::function<double(double)> build_sin_transform(
		const std::function<double(double)>& K,
		const std::vector<double>& omegas,
		const double eps,
		const double tau_min,
		double tau_max
		) const;

        /**  \brief Computes the convolution of a function with state history, over a certain time
          *  \returns \f$\int_0^T h(t-\tau)*f(\tau) d\tau\f$
          *  \snippet hdb_interpolators/unit_tests/src/RadiationDampingBuilderTest.cpp RadiationDampingBuilderTest method_example
          */
        double convolution(const std::function<double(double)>& h, //!< State history
                           const std::function<double(double)>& f, //!< Function to convolute with
                           const double Tmin, //!< Beginning of the convolution (because retardation function may not be defined for T=0)
                           const double Tmax  //!< End of the convolution
                           ) const;

        /**  \brief Computes the intergal of a function over a certain time, using the convolution method
         *  \returns \f$\int_Tmin^Tmax f(\tau) d\tau\f$
         */
        double integral(const std::function<double(double)>& f, //!< Function to integrate
						const double Tmin, //!< Beginning of the integral
						const double Tmax  //!< End of the integral
        ) const;


        /**  \brief Build a vector of n regularly incremented doubles from xmin to xmax. First value is xmin last is xmax.
          */
        std::vector<double> build_regular_intervals(
                const double first, //!< First value in vector
                const double last,  //!< Last value in vector
                const size_t n      //!< Number of values to return
                ) const;

        /**  \brief Build a vector of n exponentially incremented doubles from xmin to xmax.
          *  \details First value is xmin last is xmax. Spacing is small near xmin and large near xmax
          */
        std::vector<double> build_exponential_intervals(
                const double first, //!< First value in vector
                const double last,  //!< Last value in vector
                const size_t n      //!< Number of values to return
                ) const;

        /**  \brief Build a vector of n exponentially incremented doubles from xmin to xmax.
          *  \details First value is xmin last is xmax. Spacing is large near xmin and small near xmax
          */
        std::vector<double> build_exponential_intervals_reversed(
                const double first, //!< First value in vector
                const double last,  //!< Last value in vector
                const size_t n      //!< Number of values to return
                ) const;

        /**  \brief Find bound representing a significant amount of the integral
          *  \details Uses TOMS Algorithm 748 to compute the minimum bound \f$\omega_0\f$ such that
          *  \f$int_{\omega_{\mbox{min}}}^{\omega_0} f(\omega) d\omega = (1-eps) int_{\omega_{\mbox{min}}}^{\omega_{\mbox{max}}}\f$
          *  \returns Upper integration bound
          *  \snippet hdb_interpolators/unit_tests/src/RadiationDampingBuilderTest.cpp RadiationDampingBuilderTest find_integration_bound_example
          */
        double find_integration_bound(
                const std::function<double(double)>& f, //!< Function to integrate
                const double omega_min, //!< Lower bound of the integration (returned omega is necessarily greater than omega_min)
                const double omega_max, //!< Upper bound of the integration (returned omega is necessarily lower than omega_min)
                const double eps        //!< Integration error (compared to full integration from omega_min up to omega_max)
                ) const;

        double find_r_bound(
                const std::function<double(double)>& f, //!< Function to integrate
                const double omega_min,                 //!< Lower bound of the integration (returned omega is necessarily greater than omega_min)
                const double omega_max,                 //!< Upper bound of the integration (returned omega is necessarily lower than omega_min)
                const double r                          //!< How much of the total integral between omega_min & omega_max do we wish to represent?
                ) const;

        double cos_transform(const std::function<double(double)>& Br, const double omega_min, const double omega_max, const double tau) const;

    private:
        RadiationDampingBuilder();

        TypeOfQuadrature type_of_quadrature_for_convolution;
        TypeOfQuadrature type_of_quadrature_for_cos_transform;
};

#endif /* RADIATIONDAMPINGBUILDER_HPP_ */
