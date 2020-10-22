/*
 * BodyBuilderTest.cpp
 *
 *  Created on: Jun 18, 2014
 *      Author: cady
 */

#include "BodyBuilder.hpp"
#include "BodyBuilderTest.hpp"
#include "Mesh.hpp"
#include "SimulatorYamlParser.hpp"
#include "stl_data.hpp"
#include "stl_reader.hpp"
#include "yaml_data.hpp"
#include <ssc/kinematics.hpp>

const BodyPtr BodyBuilderTest::body = BodyBuilderTest::build_body();

BodyBuilderTest::BodyBuilderTest() : a(ssc::random_data_generator::DataGenerator(1218221))
{
}

BodyBuilderTest::~BodyBuilderTest()
{
}

void BodyBuilderTest::SetUp()
{
}

void BodyBuilderTest::TearDown()
{
}

BodyPtr BodyBuilderTest::build_body(size_t idx)
{
    const auto yaml = SimulatorYamlParser(test_data::full_example()).parse();
    const auto mesh = read_stl(test_data::cube());
    BodyBuilder builder(yaml.rotations);
    return builder.build(yaml.bodies.front(), mesh, idx, 0);
}

TEST_F(BodyBuilderTest, name_should_be_correct)
{
    ASSERT_EQ("body 1", body->get_name());
}
TEST_F(BodyBuilderTest, centre_of_gravity_should_be_computed_properly)
{
    const auto G = body->get_states().G;
    ASSERT_EQ("body 1", G.get_frame());
    ASSERT_DOUBLE_EQ(4, G.x());
    ASSERT_DOUBLE_EQ(7, G.y());
    ASSERT_DOUBLE_EQ(-10, G.z());
}

TEST_F(BodyBuilderTest, should_be_able_to_detect_no_symmetric_inertia_matrix)
{
    const size_t n = 3;
    Eigen::MatrixXd m = Eigen::MatrixXd::Zero(n,n);
    m(0,0) = m(1,1) = m(2,2) = 2;
    m(0,1) = m(1,0) = m(2,1) = m(1,2) = -1;
    ASSERT_TRUE(isSymmetric(m));
    m(2,1) = -2;
    ASSERT_FALSE(isSymmetric(m));
}

TEST_F(BodyBuilderTest, should_be_able_to_detect_inconsistent_inertia_matrix)
{
    const size_t n = 3;
    Eigen::MatrixXd m = Eigen::MatrixXd::Zero(n,n);
    m(0,0) = m(1,1) = m(2,2) = 2;
    m(0,1) = m(1,0) = m(2,1) = m(1,2) = -1;
    ASSERT_TRUE(isSymmetricDefinitePositive(m));
    m(0,0) = 0.0;
    ASSERT_FALSE(isSymmetricDefinitePositive(m));
}


/**
 * \code
    import scipy.linalg
    M = scipy.linalg.pascal(6)
    for i in range(6):
        for j in range(6):
            print('    m({0},{1}) = {2};'.format(i,j,M[i,j]))
 * \endcode
 */
TEST_F(BodyBuilderTest, should_be_able_to_detect_inconsistent_inertia_matrix_pascal6)
{
    const size_t n = 6;
    Eigen::MatrixXd m = Eigen::MatrixXd::Zero(n,n);
    m(0,0) = 1;
    m(0,1) = 1;
    m(0,2) = 1;
    m(0,3) = 1;
    m(0,4) = 1;
    m(0,5) = 1;
    m(1,0) = 1;
    m(1,1) = 2;
    m(1,2) = 3;
    m(1,3) = 4;
    m(1,4) = 5;
    m(1,5) = 6;
    m(2,0) = 1;
    m(2,1) = 3;
    m(2,2) = 6;
    m(2,3) = 10;
    m(2,4) = 15;
    m(2,5) = 21;
    m(3,0) = 1;
    m(3,1) = 4;
    m(3,2) = 10;
    m(3,3) = 20;
    m(3,4) = 35;
    m(3,5) = 56;
    m(4,0) = 1;
    m(4,1) = 5;
    m(4,2) = 15;
    m(4,3) = 35;
    m(4,4) = 70;
    m(4,5) = 126;
    m(5,0) = 1;
    m(5,1) = 6;
    m(5,2) = 21;
    m(5,3) = 56;
    m(5,4) = 126;
    m(5,5) = 252;
    ASSERT_TRUE(isSymmetricDefinitePositive(m));
}

TEST_F(BodyBuilderTest, relative_position_should_be_correct)
{
    ASSERT_DOUBLE_EQ(10,body->get_states().x_relative_to_mesh);
    ASSERT_DOUBLE_EQ(0.21,body->get_states().y_relative_to_mesh);
    ASSERT_DOUBLE_EQ(33E3,body->get_states().z_relative_to_mesh);
}


/**
 * \brief Unit test where instructions where generated with MatLab
 * \code
 *
    % Data
    euler = [1 3 2];
    translation = [10,21e-2,33e3]';

    unit_normal = ...
     [0.0   0.0   1.0
      0.0   0.0   1.0
      0.0   0.0  -1.0
      0.0   0.0  -1.0
     -1.0   0.0   0.0
     -1.0   0.0   0.0
      1.0   0.0   0.0
      1.0   0.0   0.0
      0.0  -1.0   0.0
      0.0  -1.0   0.0
      0.0   1.0   0.0
      0.0   1.0   0.0]';

    % Evaluation of the ctm matrix
    cc=cos(euler);
    ss=sin(euler);
    cphi=cc(1);cteta=cc(2);cpsi=cc(3);
    sphi=ss(1);steta=ss(2);spsi=ss(3);
    ctm = [cteta*cpsi                , cteta*spsi                , -steta    ;
           sphi*steta*cpsi-cphi*spsi , sphi*steta*spsi+cphi*cpsi , sphi*cteta;
           cphi*steta*cpsi+sphi*spsi , cphi*steta*spsi-sphi*cpsi , cphi*cteta];

    % Begin of computations for unit_normal
    n = size(unit_normal,2);
    body_unit_normal = ctm * unit_normal;

    for i=1:n
        fprintf('ASSERT_NEAR(%18.12f,(double)body.states.mesh->facets.at(%d).unit_normal(0),EPS);\n',body_unit_normal(1,i),i-1);
        fprintf('ASSERT_NEAR(%18.12f,(double)body.states.mesh->facets.at(%d).unit_normal(1),EPS);\n',body_unit_normal(2,i),i-1);
        fprintf('ASSERT_NEAR(%18.12f,(double)body.states.mesh->facets.at(%d).unit_normal(2),EPS);\n',body_unit_normal(3,i),i-1);
        fprintf('\n');
    end
    % Data

    % Begin of computations for unit_normal
    vertices = ...
    [-0.5,-0.5,+1.0
     +0.5,-0.5,+1.0
     +0.5,+0.5,+1.0
     -0.5,-0.5,+1.0
     +0.5,+0.5,+1.0
     -0.5,+0.5,+1.0
     +0.5,-0.5,+0.0
     -0.5,-0.5,+0.0
     -0.5,+0.5,+0.0
     +0.5,-0.5,+0.0
     -0.5,+0.5,+0.0
     +0.5,+0.5,+0.0
     -0.5,-0.5,+0.0
     -0.5,-0.5,+1.0
     -0.5,+0.5,+1.0
     -0.5,-0.5,+0.0
     -0.5,+0.5,+1.0
     -0.5,+0.5,+0.0
     +0.5,-0.5,+1.0
     +0.5,-0.5,+0.0
     +0.5,+0.5,+0.0
     +0.5,-0.5,+1.0
     +0.5,+0.5,+0.0
     +0.5,+0.5,+1.0
     -0.5,-0.5,+0.0
     +0.5,-0.5,+0.0
     +0.5,-0.5,+1.0
     -0.5,-0.5,+0.0
     +0.5,-0.5,+1.0
     -0.5,-0.5,+1.0
     -0.5,+0.5,+1.0
     +0.5,+0.5,+1.0
     +0.5,+0.5,+0.0
     -0.5,+0.5,+1.0
     +0.5,+0.5,+0.0
     -0.5,+0.5,+0.0]';

    % Begin of computations for barycenter
    n=size(vertices,2)/3;
    for i=1:n
        bary = sum(vertices(:,1+3*(i-1):3+3*(i-1)),2)/3;
        body_bary = ctm * bary -ctm*translation;
        fprintf('ASSERT_NEAR(%18.12f,(double)body.states.mesh->facets.at(%d).barycenter(0),EPS);\n',body_bary(1),i-1);
        fprintf('ASSERT_NEAR(%18.12f,(double)body.states.mesh->facets.at(%d).barycenter(1),EPS);\n',body_bary(2),i-1);
        fprintf('ASSERT_NEAR(%18.12f,(double)body.states.mesh->facets.at(%d).barycenter(2),EPS);\n',body_bary(3),i-1);
        fprintf('\n');
    end

   \endcode
 */
TEST_F(BodyBuilderTest, mesh_is_correct)
{
    const double EPS = 1E-11;
    const auto states = body->get_states();
    ASSERT_EQ(12,states.mesh->facets.size());
    for (size_t i = 0 ; i < 12 ; ++i)
    {
        ASSERT_DOUBLE_EQ(0.5,states.mesh->facets.at(i).area) << std::endl << "Facet: " << i << " (starting at 0)";
    }
    ASSERT_NEAR(   -0.141120008060,(double)states.mesh->facets.at(0).unit_normal(0),EPS);
    ASSERT_NEAR(   -0.833049961067,(double)states.mesh->facets.at(0).unit_normal(1),EPS);
    ASSERT_NEAR(   -0.534895228705,(double)states.mesh->facets.at(0).unit_normal(2),EPS);

    ASSERT_NEAR(   -0.141120008060,(double)states.mesh->facets.at(1).unit_normal(0),EPS);
    ASSERT_NEAR(   -0.833049961067,(double)states.mesh->facets.at(1).unit_normal(1),EPS);
    ASSERT_NEAR(   -0.534895228705,(double)states.mesh->facets.at(1).unit_normal(2),EPS);

    ASSERT_NEAR(    0.141120008060,(double)states.mesh->facets.at(2).unit_normal(0),EPS);
    ASSERT_NEAR(    0.833049961067,(double)states.mesh->facets.at(2).unit_normal(1),EPS);
    ASSERT_NEAR(    0.534895228705,(double)states.mesh->facets.at(2).unit_normal(2),EPS);

    ASSERT_NEAR(    0.141120008060,(double)states.mesh->facets.at(3).unit_normal(0),EPS);
    ASSERT_NEAR(    0.833049961067,(double)states.mesh->facets.at(3).unit_normal(1),EPS);
    ASSERT_NEAR(    0.534895228705,(double)states.mesh->facets.at(3).unit_normal(2),EPS);

    ASSERT_NEAR(   -0.411982245666,(double)states.mesh->facets.at(4).unit_normal(0),EPS);
    ASSERT_NEAR(    0.540712264176,(double)states.mesh->facets.at(4).unit_normal(1),EPS);
    ASSERT_NEAR(   -0.733417259564,(double)states.mesh->facets.at(4).unit_normal(2),EPS);

    ASSERT_NEAR(   -0.411982245666,(double)states.mesh->facets.at(5).unit_normal(0),EPS);
    ASSERT_NEAR(    0.540712264176,(double)states.mesh->facets.at(5).unit_normal(1),EPS);
    ASSERT_NEAR(   -0.733417259564,(double)states.mesh->facets.at(5).unit_normal(2),EPS);

    ASSERT_NEAR(    0.411982245666,(double)states.mesh->facets.at(6).unit_normal(0),EPS);
    ASSERT_NEAR(   -0.540712264176,(double)states.mesh->facets.at(6).unit_normal(1),EPS);
    ASSERT_NEAR(    0.733417259564,(double)states.mesh->facets.at(6).unit_normal(2),EPS);

    ASSERT_NEAR(    0.411982245666,(double)states.mesh->facets.at(7).unit_normal(0),EPS);
    ASSERT_NEAR(   -0.540712264176,(double)states.mesh->facets.at(7).unit_normal(1),EPS);
    ASSERT_NEAR(    0.733417259564,(double)states.mesh->facets.at(7).unit_normal(2),EPS);

    ASSERT_NEAR(    0.900197629736,(double)states.mesh->facets.at(8).unit_normal(0),EPS);
    ASSERT_NEAR(    0.116867487937,(double)states.mesh->facets.at(8).unit_normal(1),EPS);
    ASSERT_NEAR(   -0.419507112791,(double)states.mesh->facets.at(8).unit_normal(2),EPS);

    ASSERT_NEAR(    0.900197629736,(double)states.mesh->facets.at(9).unit_normal(0),EPS);
    ASSERT_NEAR(    0.116867487937,(double)states.mesh->facets.at(9).unit_normal(1),EPS);
    ASSERT_NEAR(   -0.419507112791,(double)states.mesh->facets.at(9).unit_normal(2),EPS);

    ASSERT_NEAR(   -0.900197629736,(double)states.mesh->facets.at(10).unit_normal(0),EPS);
    ASSERT_NEAR(   -0.116867487937,(double)states.mesh->facets.at(10).unit_normal(1),EPS);
    ASSERT_NEAR(    0.419507112791,(double)states.mesh->facets.at(10).unit_normal(2),EPS);

    ASSERT_NEAR(   -0.900197629736,(double)states.mesh->facets.at(11).unit_normal(0),EPS);
    ASSERT_NEAR(   -0.116867487937,(double)states.mesh->facets.at(11).unit_normal(1),EPS);
    ASSERT_NEAR(    0.419507112791,(double)states.mesh->facets.at(11).unit_normal(2),EPS);

    ASSERT_NEAR( 4653.107061659046,(double)states.mesh->facets.at(0).centre_of_gravity(0),EPS);
    ASSERT_NEAR(27495.176689261680,(double)states.mesh->facets.at(0).centre_of_gravity(1),EPS);
    ASSERT_NEAR(17643.637701317213,(double)states.mesh->facets.at(0).centre_of_gravity(2),EPS);

    ASSERT_NEAR( 4652.669668367245,(double)states.mesh->facets.at(1).centre_of_gravity(0),EPS);
    ASSERT_NEAR(27495.317970853757,(double)states.mesh->facets.at(1).centre_of_gravity(1),EPS);
    ASSERT_NEAR(17643.533064601623,(double)states.mesh->facets.at(1).centre_of_gravity(2),EPS);

    ASSERT_NEAR( 4653.110854251884,(double)states.mesh->facets.at(2).centre_of_gravity(0),EPS);
    ASSERT_NEAR(27496.189976644138,(double)states.mesh->facets.at(2).centre_of_gravity(1),EPS);
    ASSERT_NEAR(17643.928124126061,(double)states.mesh->facets.at(2).centre_of_gravity(2),EPS);

    ASSERT_NEAR( 4652.948115790528,(double)states.mesh->facets.at(3).centre_of_gravity(0),EPS);
    ASSERT_NEAR(27495.970783393434,(double)states.mesh->facets.at(3).centre_of_gravity(1),EPS);
    ASSERT_NEAR(17644.312432250183,(double)states.mesh->facets.at(3).centre_of_gravity(2),EPS);

    ASSERT_NEAR( 4652.879446831289,(double)states.mesh->facets.at(4).centre_of_gravity(0),EPS);
    ASSERT_NEAR(27495.814847424819,(double)states.mesh->facets.at(4).centre_of_gravity(1),EPS);
    ASSERT_NEAR(17643.327054887071,(double)states.mesh->facets.at(4).centre_of_gravity(2),EPS);

    ASSERT_NEAR( 4652.626420957397,(double)states.mesh->facets.at(5).centre_of_gravity(0),EPS);
    ASSERT_NEAR(27496.053574915863,(double)states.mesh->facets.at(5).centre_of_gravity(1),EPS);
    ASSERT_NEAR(17643.645189000905,(double)states.mesh->facets.at(5).centre_of_gravity(2),EPS);

    ASSERT_NEAR( 4653.338469079641,(double)states.mesh->facets.at(6).centre_of_gravity(0),EPS);
    ASSERT_NEAR(27495.551818480999,(double)states.mesh->facets.at(6).centre_of_gravity(1),EPS);
    ASSERT_NEAR(17644.238770556203,(double)states.mesh->facets.at(6).centre_of_gravity(2),EPS);

    ASSERT_NEAR( 4652.991363200376,(double)states.mesh->facets.at(7).centre_of_gravity(0),EPS);
    ASSERT_NEAR(27495.235179331332,(double)states.mesh->facets.at(7).centre_of_gravity(1),EPS);
    ASSERT_NEAR(17644.200307850901,(double)states.mesh->facets.at(7).centre_of_gravity(2),EPS);

    ASSERT_NEAR( 4653.501207540998,(double)states.mesh->facets.at(8).centre_of_gravity(0),EPS);
    ASSERT_NEAR(27495.771011731704,(double)states.mesh->facets.at(8).centre_of_gravity(1),EPS);
    ASSERT_NEAR(17643.854462432086,(double)states.mesh->facets.at(8).centre_of_gravity(2),EPS);

    ASSERT_NEAR( 4653.316840123090,(double)states.mesh->facets.at(9).centre_of_gravity(0),EPS);
    ASSERT_NEAR(27495.673565832738,(double)states.mesh->facets.at(9).centre_of_gravity(1),EPS);
    ASSERT_NEAR(17643.431691602662,(double)states.mesh->facets.at(9).centre_of_gravity(2),EPS);

    ASSERT_NEAR( 4652.553969908576,(double)states.mesh->facets.at(10).centre_of_gravity(0),EPS);
    ASSERT_NEAR(27495.376460923409,(double)states.mesh->facets.at(10).centre_of_gravity(1),EPS);
    ASSERT_NEAR(17644.095671135306,(double)states.mesh->facets.at(10).centre_of_gravity(2),EPS);

    ASSERT_NEAR( 4652.463682496041,(double)states.mesh->facets.at(11).centre_of_gravity(0),EPS);
    ASSERT_NEAR(27495.834381665158,(double)states.mesh->facets.at(11).centre_of_gravity(1),EPS);
    ASSERT_NEAR(17644.029497125022,(double)states.mesh->facets.at(11).centre_of_gravity(2),EPS);

    ASSERT_EQ(8,states.mesh->nodes.cols());
    ASSERT_EQ(3,states.mesh->nodes.rows());
    ASSERT_NEAR( 4653.132472705181,(double)states.mesh->nodes.col(0)(0),EPS);
    ASSERT_NEAR(27495.576119933776,(double)states.mesh->nodes.col(0)(1),EPS);
    ASSERT_NEAR(17643.008920773238,(double)states.mesh->nodes.col(0)(2),EPS);
}

TEST_F(BodyBuilderTest, mesh_to_body_is_correct)
{
    const auto states = body->get_states();
    ASSERT_DOUBLE_EQ(0.41198224566568298,     (double)states.mesh_to_body(0,0));
    ASSERT_DOUBLE_EQ(-0.90019762973551742391, (double)states.mesh_to_body(1,0));
    ASSERT_DOUBLE_EQ(-0.14112000805986721352, (double)states.mesh_to_body(2,0));
    ASSERT_DOUBLE_EQ(-0.54071226417559081767, (double)states.mesh_to_body(0,1));
    ASSERT_DOUBLE_EQ(-0.11686748793698308047, (double)states.mesh_to_body(1,1));
    ASSERT_DOUBLE_EQ(-0.83304996106680495593, (double)states.mesh_to_body(2,1));
    ASSERT_DOUBLE_EQ(0.73341725956399950181,  (double)states.mesh_to_body(0,2));
    ASSERT_DOUBLE_EQ(0.41950711279054053726,  (double)states.mesh_to_body(1,2));
    ASSERT_DOUBLE_EQ(-0.53489522870537720145, (double)states.mesh_to_body(2,2));
}

/**
 * \code
    import scipy.linalg
    M = scipy.linalg.pascal(6)
    Mrb = M + 10
    Ma = M
    Mt = Ma+Mrb
    for i in range(6):
        for j in range(6):
            print('    ASSERT_DOUBLE_EQ({2},(double)states.solid_body_inertia->operator()({0},{1}));'.format(i,j,Mrb[i,j]))
 * \endcode
 */
TEST_F(BodyBuilderTest, rigid_body_inertia_is_correct)
{
    const auto states = body->get_states();
    ASSERT_DOUBLE_EQ(11,(double)states.solid_body_inertia->operator()(0,0));
    ASSERT_DOUBLE_EQ(11,(double)states.solid_body_inertia->operator()(0,1));
    ASSERT_DOUBLE_EQ(11,(double)states.solid_body_inertia->operator()(0,2));
    ASSERT_DOUBLE_EQ(11,(double)states.solid_body_inertia->operator()(0,3));
    ASSERT_DOUBLE_EQ(11,(double)states.solid_body_inertia->operator()(0,4));
    ASSERT_DOUBLE_EQ(11,(double)states.solid_body_inertia->operator()(0,5));
    ASSERT_DOUBLE_EQ(11,(double)states.solid_body_inertia->operator()(1,0));
    ASSERT_DOUBLE_EQ(12,(double)states.solid_body_inertia->operator()(1,1));
    ASSERT_DOUBLE_EQ(13,(double)states.solid_body_inertia->operator()(1,2));
    ASSERT_DOUBLE_EQ(14,(double)states.solid_body_inertia->operator()(1,3));
    ASSERT_DOUBLE_EQ(15,(double)states.solid_body_inertia->operator()(1,4));
    ASSERT_DOUBLE_EQ(16,(double)states.solid_body_inertia->operator()(1,5));
    ASSERT_DOUBLE_EQ(11,(double)states.solid_body_inertia->operator()(2,0));
    ASSERT_DOUBLE_EQ(13,(double)states.solid_body_inertia->operator()(2,1));
    ASSERT_DOUBLE_EQ(16,(double)states.solid_body_inertia->operator()(2,2));
    ASSERT_DOUBLE_EQ(20,(double)states.solid_body_inertia->operator()(2,3));
    ASSERT_DOUBLE_EQ(25,(double)states.solid_body_inertia->operator()(2,4));
    ASSERT_DOUBLE_EQ(31,(double)states.solid_body_inertia->operator()(2,5));
    ASSERT_DOUBLE_EQ(11,(double)states.solid_body_inertia->operator()(3,0));
    ASSERT_DOUBLE_EQ(14,(double)states.solid_body_inertia->operator()(3,1));
    ASSERT_DOUBLE_EQ(20,(double)states.solid_body_inertia->operator()(3,2));
    ASSERT_DOUBLE_EQ(30,(double)states.solid_body_inertia->operator()(3,3));
    ASSERT_DOUBLE_EQ(45,(double)states.solid_body_inertia->operator()(3,4));
    ASSERT_DOUBLE_EQ(66,(double)states.solid_body_inertia->operator()(3,5));
    ASSERT_DOUBLE_EQ(11,(double)states.solid_body_inertia->operator()(4,0));
    ASSERT_DOUBLE_EQ(15,(double)states.solid_body_inertia->operator()(4,1));
    ASSERT_DOUBLE_EQ(25,(double)states.solid_body_inertia->operator()(4,2));
    ASSERT_DOUBLE_EQ(45,(double)states.solid_body_inertia->operator()(4,3));
    ASSERT_DOUBLE_EQ(80,(double)states.solid_body_inertia->operator()(4,4));
    ASSERT_DOUBLE_EQ(136,(double)states.solid_body_inertia->operator()(4,5));
    ASSERT_DOUBLE_EQ(11,(double)states.solid_body_inertia->operator()(5,0));
    ASSERT_DOUBLE_EQ(16,(double)states.solid_body_inertia->operator()(5,1));
    ASSERT_DOUBLE_EQ(31,(double)states.solid_body_inertia->operator()(5,2));
    ASSERT_DOUBLE_EQ(66,(double)states.solid_body_inertia->operator()(5,3));
    ASSERT_DOUBLE_EQ(136,(double)states.solid_body_inertia->operator()(5,4));
    ASSERT_DOUBLE_EQ(262,(double)states.solid_body_inertia->operator()(5,5));
}

/**
 * \code
    import scipy.linalg
    M = scipy.linalg.pascal(6)
    Mrb = M + 10
    Ma = M
    Mt = Ma+Mrb
    for i in range(6):
        for j in range(6):
            print('    ASSERT_DOUBLE_EQ({2},(double)states.total_inertia->operator()({0},{1}));'.format(i,j,Mt[i,j]))
 * \endcode
 */
TEST_F(BodyBuilderTest, total_inertia_is_correct)
{
    const auto states = body->get_states();
    ASSERT_DOUBLE_EQ(12,(double)states.total_inertia->operator()(0,0));
    ASSERT_DOUBLE_EQ(12,(double)states.total_inertia->operator()(0,1));
    ASSERT_DOUBLE_EQ(12,(double)states.total_inertia->operator()(0,2));
    ASSERT_DOUBLE_EQ(12,(double)states.total_inertia->operator()(0,3));
    ASSERT_DOUBLE_EQ(12,(double)states.total_inertia->operator()(0,4));
    ASSERT_DOUBLE_EQ(12,(double)states.total_inertia->operator()(0,5));
    ASSERT_DOUBLE_EQ(12,(double)states.total_inertia->operator()(1,0));
    ASSERT_DOUBLE_EQ(14,(double)states.total_inertia->operator()(1,1));
    ASSERT_DOUBLE_EQ(16,(double)states.total_inertia->operator()(1,2));
    ASSERT_DOUBLE_EQ(18,(double)states.total_inertia->operator()(1,3));
    ASSERT_DOUBLE_EQ(20,(double)states.total_inertia->operator()(1,4));
    ASSERT_DOUBLE_EQ(22,(double)states.total_inertia->operator()(1,5));
    ASSERT_DOUBLE_EQ(12,(double)states.total_inertia->operator()(2,0));
    ASSERT_DOUBLE_EQ(16,(double)states.total_inertia->operator()(2,1));
    ASSERT_DOUBLE_EQ(22,(double)states.total_inertia->operator()(2,2));
    ASSERT_DOUBLE_EQ(30,(double)states.total_inertia->operator()(2,3));
    ASSERT_DOUBLE_EQ(40,(double)states.total_inertia->operator()(2,4));
    ASSERT_DOUBLE_EQ(52,(double)states.total_inertia->operator()(2,5));
    ASSERT_DOUBLE_EQ(12,(double)states.total_inertia->operator()(3,0));
    ASSERT_DOUBLE_EQ(18,(double)states.total_inertia->operator()(3,1));
    ASSERT_DOUBLE_EQ(30,(double)states.total_inertia->operator()(3,2));
    ASSERT_DOUBLE_EQ(50,(double)states.total_inertia->operator()(3,3));
    ASSERT_DOUBLE_EQ(80,(double)states.total_inertia->operator()(3,4));
    ASSERT_DOUBLE_EQ(122,(double)states.total_inertia->operator()(3,5));
    ASSERT_DOUBLE_EQ(12,(double)states.total_inertia->operator()(4,0));
    ASSERT_DOUBLE_EQ(20,(double)states.total_inertia->operator()(4,1));
    ASSERT_DOUBLE_EQ(40,(double)states.total_inertia->operator()(4,2));
    ASSERT_DOUBLE_EQ(80,(double)states.total_inertia->operator()(4,3));
    ASSERT_DOUBLE_EQ(150,(double)states.total_inertia->operator()(4,4));
    ASSERT_DOUBLE_EQ(262,(double)states.total_inertia->operator()(4,5));
    ASSERT_DOUBLE_EQ(12,(double)states.total_inertia->operator()(5,0));
    ASSERT_DOUBLE_EQ(22,(double)states.total_inertia->operator()(5,1));
    ASSERT_DOUBLE_EQ(52,(double)states.total_inertia->operator()(5,2));
    ASSERT_DOUBLE_EQ(122,(double)states.total_inertia->operator()(5,3));
    ASSERT_DOUBLE_EQ(262,(double)states.total_inertia->operator()(5,4));
    ASSERT_DOUBLE_EQ(514,(double)states.total_inertia->operator()(5,5));
}

/**
 * \brief Unit test where instructions were generated with MatLab
 * \code
    % Data
    raw_mesh_points = ...
    [[-0.5, 0.5, 0.5,-0.5, 0.5, -0.5, -0.5, 0.5];
     [-0.5,-0.5, 0.5, 0.5,-0.5, -0.5,  0.5, 0.5];
     [   1,   1,   1,   1,   0,    0,    0,   0]];
    translation = [10,21e-2,33e3]';
    euler = [1 3 2];
    % Begin of computations
    cc=cos(euler);
    ss=sin(euler);
    cphi=cc(1);cteta=cc(2);cpsi=cc(3);
    sphi=ss(1);steta=ss(2);spsi=ss(3);
    ctm = [cteta*cpsi                , cteta*spsi                , -steta    ;
           sphi*steta*cpsi-cphi*spsi , sphi*steta*spsi+cphi*cpsi , sphi*cteta;
           cphi*steta*cpsi+sphi*spsi , cphi*steta*spsi-sphi*cpsi , cphi*cteta];


    n = size(raw_mesh_points,2);
    body_mesh_points = ctm * raw_mesh_points -repmat(ctm*translation,1,n);

    for i=1:n
        fprintf('ASSERT_NEAR(%18.12f,(double)body->states.M->m(0,%d),EPS);\n',body_mesh_points(1,i),i-1);
        fprintf('ASSERT_NEAR(%18.12f,(double)body->states.M->m(1,%d),EPS);\n',body_mesh_points(2,i),i-1);
        fprintf('ASSERT_NEAR(%18.12f,(double)body->states.M->m(2,%d),EPS);\n',body_mesh_points(3,i),i-1);
        fprintf('\n');
    end
   \endcode
 */
TEST_F(BodyBuilderTest, mesh_should_be_correct)
{
    const double EPS = 1E-11;
    const auto states = body->get_states();
    ASSERT_EQ("body 1", states.M->get_frame());
    ASSERT_NEAR( 4653.132472705181,(double)states.M->m(0,0),EPS);
    ASSERT_NEAR(27495.576119933776,(double)states.M->m(1,0),EPS);
    ASSERT_NEAR(17643.008920773238,(double)states.M->m(2,0),EPS);

    ASSERT_NEAR( 4653.544454950847,(double)states.M->m(0,1),EPS);
    ASSERT_NEAR(27495.035407669598,(double)states.M->m(1,1),EPS);
    ASSERT_NEAR(17643.742338032804,(double)states.M->m(2,1),EPS);

    ASSERT_NEAR( 4652.644257321111,(double)states.M->m(0,2),EPS);
    ASSERT_NEAR(27494.918540181661,(double)states.M->m(1,2),EPS);
    ASSERT_NEAR(17644.161845145594,(double)states.M->m(2,2),EPS);

    ASSERT_NEAR( 4652.232275075446,(double)states.M->m(0,3),EPS);
    ASSERT_NEAR(27495.459252445839,(double)states.M->m(1,3),EPS);
    ASSERT_NEAR(17643.428427886029,(double)states.M->m(2,3),EPS);

    ASSERT_NEAR( 4653.685574958907,(double)states.M->m(0,4),EPS);
    ASSERT_NEAR(27495.868457630666,(double)states.M->m(1,4),EPS);
    ASSERT_NEAR(17644.277233261510,(double)states.M->m(2,4),EPS);

    ASSERT_NEAR( 4653.273592713241,(double)states.M->m(0,5),EPS);
    ASSERT_NEAR(27496.409169894843,(double)states.M->m(1,5),EPS);
    ASSERT_NEAR(17643.543816001944,(double)states.M->m(2,5),EPS);

    ASSERT_NEAR( 4652.373395083505,(double)states.M->m(0,6),EPS);
    ASSERT_NEAR(27496.292302406906,(double)states.M->m(1,6),EPS);
    ASSERT_NEAR(17643.963323114735,(double)states.M->m(2,6),EPS);

    ASSERT_NEAR( 4652.785377329171,(double)states.M->m(0,7),EPS);
    ASSERT_NEAR(27495.751590142729,(double)states.M->m(1,7),EPS);
    ASSERT_NEAR(17644.696740374300,(double)states.M->m(2,7),EPS);
}

TEST_F(BodyBuilderTest, hydrodynamic_forces_calculation_point_in_body_frame)
{
    const auto states = body->get_states();
    const auto P = states.hydrodynamic_forces_calculation_point;
    ASSERT_EQ("body 1", P.get_frame());
    ASSERT_DOUBLE_EQ(0.696, P.x());
    ASSERT_DOUBLE_EQ(0, P.y());
    ASSERT_DOUBLE_EQ(1.418, P.z());
}

/* TODO: Adapt this test from SimulatorBuilderTest to BodyBuilderTest (the responsibility of building forces was moved from the former to the latter)
TEST_F(SimulatorBuilderTest, get_forces_should_throw_if_there_is_anything_it_cannot_parse)
{
    builder.can_parse<DefaultSurfaceElevation>();
    MeshMap m;
    const std::string name = input.bodies.front().name;
    m[name] = two_triangles();
    const EnvironmentAndFrames env = builder.get_environment();
    const auto bodies = builder.get_bodies(m, env);
    ASSERT_THROW(builder.get_forces(env), InvalidInputException);
}*/
