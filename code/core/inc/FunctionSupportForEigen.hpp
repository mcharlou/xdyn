/*
 * FunctionSupportForEigen.hpp
 *
 *  Created on: 27 janv. 2020
 *      Author: mcharlou2016
 */

#ifndef CORE_INC_FUNCTIONSUPPORTFOREIGEN_HPP_
#define CORE_INC_FUNCTIONSUPPORTFOREIGEN_HPP_

#include <functional>
#include <Eigen/Core>

namespace Eigen {

template<>
struct NumTraits<std::function<double(double)>> : GenericNumTraits<std::function<double(double)>>
{
		typedef std::function<double(double)> Real;
		typedef std::function<double(double)> NonInteger;
		typedef std::function<double(double)> Nested;
		typedef std::function<double(double)> Literal;

		  enum {
		    IsComplex = 0,
		    IsInteger = 0,
		    IsSigned = 0,
		    RequireInitialization = 1,
		    ReadCost = 1,
		    AddCost = 2,
		    MulCost = 3
		  };
};

}

namespace std {

function<double(double)> operator+(const function<double(double)>& f1, const function<double(double)>& f2)
{
	return [f1,f2](double x){return f1(x)+f2(x);};
}

function<double(double)> operator-(const function<double(double)>& f1, const function<double(double)>& f2)
{
	return [f1,f2](double x){return f1(x)-f2(x);};
}

function<double(double)> operator*(const function<double(double)>& f1, const function<double(double)>& f2)
{
	return [f1,f2](double x){return f1(x)*f2(x);};
}

function<double(double)> operator/(const function<double(double)>& f1, const function<double(double)>& f2)
{
	return [f1,f2](double x){return f1(x)/f2(x);};
}

function<double(double)> operator+(const function<double(double)>& f, const double a)
{
	return [a,f](double x){return f(x)+a;};
}

function<double(double)> operator+(const double a, const function<double(double)>& f)
{
	return [a,f](double x){return a+f(x);};
}

function<double(double)> operator-(const function<double(double)>& f, const double a)
{
	return [a,f](double x){return f(x)-a;};
}

function<double(double)> operator-(const double a, const function<double(double)>& f)
{
	return [a,f](double x){return a-f(x);};
}

function<double(double)> operator*(const function<double(double)>& f, const double a)
{
	return [a,f](double x){return f(x)*a;};
}

function<double(double)> operator*(const double a, const function<double(double)>& f)
{
	return [a,f](double x){return a*f(x);};
}

function<double(double)> operator/(const function<double(double)>& f, const double a)
{
	return [a,f](double x){return f(x)/a;};
}

function<double(double)> operator/(const double a, const function<double(double)>& f)
{
	return [a,f](double x){return a/f(x);};
}

}

#endif /* CORE_INC_FUNCTIONSUPPORTFOREIGEN_HPP_ */
