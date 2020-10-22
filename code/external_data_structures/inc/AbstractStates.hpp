/*
 * AbstractStates.hpp
 *
 *  Created on: Nov 13, 2015
 *      Author: cady
 */


#ifndef CORE_INC_ABSTRACTSTATES_HPP_
#define CORE_INC_ABSTRACTSTATES_HPP_

#include "History.hpp"

#define OP(x,op,y) (x op y)

#define EQUAL(x) OP(lhs.x,==,rhs.x)

#define ASSIGN(x) OP(x,=,rhs.x) // To prevent typos (eg. rhs & lhs don't match in assigment)

template <typename T> struct AbstractStates
{
    AbstractStates(const T& x_
                  ,const T& y_
                  ,const T& z_
                  ,const T& u_
                  ,const T& v_
                  ,const T& w_
                  ,const T& p_
                  ,const T& q_
                  ,const T& r_
                  ,const T& qr_
                  ,const T& qi_
                  ,const T& qj_
                  ,const T& qk_
                  ) : x(x_),y(y_),z(z_),u(u_),v(v_),w(w_),p(p_),q(q_),r(r_),qr(qr_),qi(qi_),qj(qj_),qk(qk_)
    {}

    virtual ~AbstractStates(){}

    AbstractStates(const double Tmax=0) : x(Tmax),y(Tmax),z(Tmax),u(Tmax),v(Tmax),w(Tmax),p(Tmax),q(Tmax),r(Tmax),qr(Tmax),qi(Tmax),qj(Tmax),qk(Tmax) {}
    AbstractStates<T>& operator=(const AbstractStates<T>& rhs)
    {
        ASSIGN(x);
        ASSIGN(y);
        ASSIGN(z);
        ASSIGN(u);
        ASSIGN(v);
        ASSIGN(w);
        ASSIGN(p);
        ASSIGN(q);
        ASSIGN(r);
        ASSIGN(qr);
        ASSIGN(qi);
        ASSIGN(qj);
        ASSIGN(qk);
        return *this;
    }

    T x;  //!< x-coordinate of the body's center of gravity in the NED frame (in m)
    T y;  //!< y-coordinate of the body's center of gravity in the NED frame (in m)
    T z;  //!< z-coordinate of the body's center of gravity in the NED frame (in m)
    T u;  //!< Projection of the body's translation speed (relative to NED) along the body's X-axis (in m/s)
    T v;  //!< Projection of the body's translation speed (relative to NED) along the body's Y-axis (in m/s)
    T w;  //!< Projection of the body's translation speed (relative to NED) along the body's Z-axis (in m/s)
    T p;  //!< Projection of the body's rotational speed (relative to NED) along the body's X-axis (in rad/s)
    T q;  //!< Projection of the body's rotational speed (relative to NED) along the body's Y-axis (in rad/s)
    T r;  //!< Projection of the body's rotational speed (relative to NED) along the body's Z-axis (in rad/s)
    T qr; //!< Real part of the quaternion (of the rotation from NED to body)
    T qi; //!< Imaginary part of the quaternion (of the rotation from NED to body)
    T qj; //!< Imaginary part of the quaternion (of the rotation from NED to body)
    T qk; //!< Imaginary part of the quaternion (of the rotation from NED to body)
};

template <> struct AbstractStates<History>
{
    AbstractStates(const History& x_
                  ,const History& y_
                  ,const History& z_
                  ,const History& u_
                  ,const History& v_
                  ,const History& w_
                  ,const History& p_
                  ,const History& q_
                  ,const History& r_
                  ,const History& qr_
                  ,const History& qi_
                  ,const History& qj_
                  ,const History& qk_
                  ) : x(x_),y(y_),z(z_),u(u_),v(v_),w(w_),p(p_),q(q_),r(r_),qr(qr_),qi(qi_),qj(qj_),qk(qk_)
    {}

    virtual ~AbstractStates(){}

    AbstractStates(const double Tmax=0) : x(Tmax),y(Tmax),z(Tmax),u(Tmax),v(Tmax),w(Tmax),p(Tmax),q(Tmax),r(Tmax),qr(Tmax),qi(Tmax),qj(Tmax),qk(Tmax) {}
    AbstractStates<History>& operator=(const AbstractStates<History>& rhs)
    {
        ASSIGN(x);
        ASSIGN(y);
        ASSIGN(z);
        ASSIGN(u);
        ASSIGN(v);
        ASSIGN(w);
        ASSIGN(p);
        ASSIGN(q);
        ASSIGN(r);
        ASSIGN(qr);
        ASSIGN(qi);
        ASSIGN(qj);
        ASSIGN(qk);
        return *this;
    }

    History x;  //!< x-coordinate of the body's center of gravity in the NED frame (in m)
    History y;  //!< y-coordinate of the body's center of gravity in the NED frame (in m)
    History z;  //!< z-coordinate of the body's center of gravity in the NED frame (in m)
    History u;  //!< Projection of the body's translation speed (relative to NED) along the body's X-axis (in m/s)
    History v;  //!< Projection of the body's translation speed (relative to NED) along the body's Y-axis (in m/s)
    History w;  //!< Projection of the body's translation speed (relative to NED) along the body's Z-axis (in m/s)
    History p;  //!< Projection of the body's rotational speed (relative to NED) along the body's X-axis (in rad/s)
    History q;  //!< Projection of the body's rotational speed (relative to NED) along the body's Y-axis (in rad/s)
    History r;  //!< Projection of the body's rotational speed (relative to NED) along the body's Z-axis (in rad/s)
    History qr; //!< Real part of the quaternion (of the rotation from NED to body)
    History qi; //!< Imaginary part of the quaternion (of the rotation from NED to body)
    History qj; //!< Imaginary part of the quaternion (of the rotation from NED to body)
    History qk; //!< Imaginary part of the quaternion (of the rotation from NED to body)

	virtual void set_Tmax(const double Tmax)
	{
		x.set_Tmax(Tmax);
		y.set_Tmax(Tmax);
		z.set_Tmax(Tmax);
		u.set_Tmax(Tmax);
		v.set_Tmax(Tmax);
		w.set_Tmax(Tmax);
		p.set_Tmax(Tmax);
		q.set_Tmax(Tmax);
		r.set_Tmax(Tmax);
		qr.set_Tmax(Tmax);
		qi.set_Tmax(Tmax);
		qj.set_Tmax(Tmax);
		qk.set_Tmax(Tmax);
	}
};

template <typename T> bool operator==(const AbstractStates<T>& lhs, const AbstractStates<T>& rhs)
{
    return EQUAL(x)
       and EQUAL(y)
       and EQUAL(z)
       and EQUAL(u)
       and EQUAL(v)
       and EQUAL(w)
       and EQUAL(p)
       and EQUAL(q)
       and EQUAL(r)
       and EQUAL(qr)
       and EQUAL(qi)
       and EQUAL(qj)
       and EQUAL(qk);
}

#endif  /* CORE_INC_ABSTRACTSTATES_HPP_ */
