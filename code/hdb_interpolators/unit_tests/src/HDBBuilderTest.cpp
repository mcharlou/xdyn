/*
 * HDBBuilderTest.cpp
 *
 *  Created on: Nov 13, 2014
 *      Author: cady
 */

#include "HDBParser.hpp"
#include "HDBParserTest.hpp"
#include "hdb_data.hpp"

HDBBuilderTest::HDBBuilderTest() : a(ssc::random_data_generator::DataGenerator(833332))
{
}

HDBBuilderTest::~HDBBuilderTest()
{
}

void HDBBuilderTest::SetUp()
{
}

void HDBBuilderTest::TearDown()
{
}

TEST_F(HDBBuilderTest, can_get_added_mass)
{
    const HDBParser builder(test_data::anthineas_hdb());
    const auto Ma = builder.get_added_mass();
    ASSERT_EQ(6,Ma.size());
    ASSERT_DOUBLE_EQ(1,Ma.at(0).first);
    ASSERT_EQ(6,Ma.at(0).second.size());
    ASSERT_DOUBLE_EQ(1.097184E+04,Ma.at(0).second.at(0).at(0));
    ASSERT_DOUBLE_EQ(4.443533E+01,Ma.at(0).second.at(3).at(4));

    ASSERT_DOUBLE_EQ(2,Ma.at(1).first);
    ASSERT_EQ(6,Ma.at(1).second.size());
    ASSERT_DOUBLE_EQ(2.102286E+04,Ma.at(1).second.at(1).at(3));
    ASSERT_DOUBLE_EQ(2.702315E+05,Ma.at(1).second.at(2).at(4));

    ASSERT_DOUBLE_EQ(3,Ma.at(2).first);
    ASSERT_EQ(6,Ma.at(2).second.size());
    ASSERT_DOUBLE_EQ(7.183531E+01,Ma.at(2).second.at(5).at(0));
    ASSERT_DOUBLE_EQ(-8.938050E+01,Ma.at(2).second.at(2).at(1));

    ASSERT_DOUBLE_EQ(3.5,Ma.at(3).first);
    ASSERT_EQ(6,Ma.at(3).second.size());

    ASSERT_DOUBLE_EQ(3.8,Ma.at(4).first);
    ASSERT_EQ(6,Ma.at(4).second.size());

    ASSERT_DOUBLE_EQ(4,Ma.at(5).first);
    ASSERT_EQ(6,Ma.at(5).second.size());
}

TEST_F(HDBBuilderTest, can_retrieve_radiation_damping)
{
    const HDBParser builder(test_data::anthineas_hdb());
    const auto Br = builder.get_radiation_damping();
    ASSERT_EQ(6,Br.size());
    ASSERT_DOUBLE_EQ(1,  Br.at(0).first);
    ASSERT_DOUBLE_EQ(2,  Br.at(1).first);
    ASSERT_DOUBLE_EQ(3,  Br.at(2).first);
    ASSERT_DOUBLE_EQ(3.5,Br.at(3).first);
    ASSERT_DOUBLE_EQ(3.8,Br.at(4).first);
    ASSERT_DOUBLE_EQ(4,  Br.at(5).first);

    ASSERT_DOUBLE_EQ(6.771553E+03, Br.at(0).second.at(0).at(0));
    ASSERT_DOUBLE_EQ(2.194728E+05, Br.at(1).second.at(1).at(1));
    ASSERT_DOUBLE_EQ(1.488785E+05, Br.at(2).second.at(2).at(2));
    ASSERT_DOUBLE_EQ(8.694864E+04, Br.at(3).second.at(3).at(3));
    ASSERT_DOUBLE_EQ(5.476829E+06, Br.at(4).second.at(4).at(4));
    ASSERT_DOUBLE_EQ(4.374308E+06, Br.at(5).second.at(5).at(5));
}

