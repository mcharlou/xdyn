/*
 * DampingForceModelTest.cpp
 *
 *  Created on: Oct 2, 2014
 *      Author: jacquenot
 */

#include "QuadraticDampingForceModel.hpp"
#include "QuadraticDampingForceModelTest.hpp"
#include "generate_body_for_tests.hpp"
#include "BodyStates.hpp"
#include "EnvironmentAndFrames.hpp"

#include <ssc/kinematics.hpp>

#define BODY "body 1"

QuadraticDampingForceModelTest::QuadraticDampingForceModelTest() : a(ssc::random_data_generator::DataGenerator(666))
{
}

QuadraticDampingForceModelTest::~QuadraticDampingForceModelTest()
{
}

void QuadraticDampingForceModelTest::SetUp()
{
}

void QuadraticDampingForceModelTest::TearDown()
{
}

namespace ssc
{
    namespace random_data_generator
    {
        template <> Eigen::Matrix<double,6,6> TypedScalarDataGenerator<Eigen::Matrix<double,6,6> >::get() const
        {
            Eigen::Matrix<double,6,6> ret;
            for (int i = 0 ; i < 6 ; ++i)
            {
                for (int j = 0 ; j < 6 ; ++j)
                {
                    ret(i,j) = random<double>();
                }
            }
            return ret;
        }
    }
}

TEST_F(QuadraticDampingForceModelTest, example_with_null_velocities)
{
//! [DampingForceModelTest example]
	EnvironmentAndFrames env;
	ssc::data_source::DataSource ds;
    QuadraticDampingForceModel F(a.random<Eigen::Matrix<double,6,6> >(), BODY, env);
    ASSERT_EQ("quadratic damping", F.model_name());
    const BodyStates states = get_body(BODY)->get_states();
    const double t = a.random<double>();
    const ssc::kinematics::Wrench f = F(states, t, env, ds);
//! [DampingForceModelTest example]
//! [DampingForceModelTest expected output]
    ASSERT_EQ(BODY, f.get_frame());
    ASSERT_DOUBLE_EQ(0, f.K());
    ASSERT_DOUBLE_EQ(0, f.M());
    ASSERT_DOUBLE_EQ(0, f.N());
    ASSERT_DOUBLE_EQ(0, f.X());
    ASSERT_DOUBLE_EQ(0, f.Y());
    ASSERT_DOUBLE_EQ(0, f.Z());
//! [DampingForceModelTest expected output]
}

TEST_F(QuadraticDampingForceModelTest, example_with_random_positive_velocities_and_identity_damping_matrix)
{
    const double EPS = 1e-11;
    const Eigen::Matrix<double,6,6> D = Eigen::Matrix<double,6,6>::Identity();
    const EnvironmentAndFrames env;
    QuadraticDampingForceModel F(D, BODY, env);
    double u,v,w,p,q,r;
    BodyStates states = get_body(BODY)->get_states();
    ssc::data_source::DataSource ds;
    for (size_t i=0;i<100;++i)
    {
        states.u.record(0, u = a.random<double>().greater_than(0.0));
        states.v.record(0, v = a.random<double>().greater_than(0.0));
        states.w.record(0, w = a.random<double>().greater_than(0.0));
        states.p.record(0, p = a.random<double>().greater_than(0.0));
        states.q.record(0, q = a.random<double>().greater_than(0.0));
        states.r.record(0, r = a.random<double>().greater_than(0.0));
        const ssc::kinematics::Wrench f = F(states, a.random<double>(), env, ds);
        ASSERT_EQ(BODY, f.get_frame());
        ASSERT_NEAR(-u*u, f.X(),EPS);
        ASSERT_NEAR(-v*v, f.Y(),EPS);
        ASSERT_NEAR(-w*w, f.Z(),EPS);
        ASSERT_NEAR(-p*p, f.K(),EPS);
        ASSERT_NEAR(-q*q, f.M(),EPS);
        ASSERT_NEAR(-r*r, f.N(),EPS);
    }
}

TEST_F(QuadraticDampingForceModelTest, example_with_dense_damping_matrix)
{
	EnvironmentAndFrames env;
	ssc::data_source::DataSource ds;
    const double EPS = 1e-9;
    Eigen::Matrix<double,6,6> D;
    double u,v,w,p,q,r;
    double uu,vv,ww,pp,qq,rr;
    BodyStates states = get_body(BODY)->get_states();
    D <<  2,   3,   5,   7,  11,  13,
         17,  19,  23,  29,  31,  37,
         41,  43,  47,  53,  59,  61,
         67,  71,  73,  79,  83,  89,
         97, 101, 103, 107, 109, 113,
        127, 131, 137, 139, 149, 151;
    QuadraticDampingForceModel F(D, BODY, env);
    for (int i=0;i<100;++i)
    {
        states.u.record((double)(i+1), u = a.random<double>().between(-10.0,+10.0));
        states.v.record((double)(i+1), v = a.random<double>().between(-10.0,+10.0));
        states.w.record((double)(i+1), w = a.random<double>().between(-10.0,+10.0));
        states.p.record((double)(i+1), p = a.random<double>().between(-10.0,+10.0));
        states.q.record((double)(i+1), q = a.random<double>().between(-10.0,+10.0));
        states.r.record((double)(i+1), r = a.random<double>().between(-10.0,+10.0));
        uu = fabs(u)*u;
        vv = fabs(v)*v;
        ww = fabs(w)*w;
        pp = fabs(p)*p;
        qq = fabs(q)*q;
        rr = fabs(r)*r;
        const ssc::kinematics::Wrench f = F(states,a.random<double>(), env, ds);
        ASSERT_EQ(BODY, f.get_frame());
        for (int j=0;j<3;++j)
        {
            const int k = j+3;
            ASSERT_NEAR(D(j,0)*uu+D(j,1)*vv+D(j,2)*ww+D(j,3)*pp+D(j,4)*qq+D(j,5)*rr,-f.force[j],EPS)<<" row: "<<i << ", col:"<<j;
            ASSERT_NEAR(D(k,0)*uu+D(k,1)*vv+D(k,2)*ww+D(k,3)*pp+D(k,4)*qq+D(k,5)*rr,-f.torque[j],EPS)<<" row: "<<i << ", col:"<<k;
        }
    }
}

