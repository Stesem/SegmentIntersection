#include "pch.h"
#include "Vector3D.hpp"


TEST(Vector3DTest, Constructor)
{
    EXPECT_NO_THROW(Vector3D v1(1.0, 1.0, 1.0));
    EXPECT_NO_THROW(Vector3D v2(0.0, 0.0, 0.0));
}

TEST(Vector3DTest, Getters)
{
    Vector3D v(1.0, 2.0, 3.0);

    EXPECT_DOUBLE_EQ(v.X(), 1.0);
    EXPECT_DOUBLE_EQ(v.Y(), 2.0);
    EXPECT_DOUBLE_EQ(v.Z(), 3.0);
}

TEST(Vector3DTest, ArithmeticOperators)
{
    Vector3D v1(1.0, 2.0, 3.0);
    Vector3D v2(4.0, 5.0, 6.0);

    Vector3D sum = v1 + v2;
    EXPECT_DOUBLE_EQ(sum.X(), 5.0);
    EXPECT_DOUBLE_EQ(sum.Y(), 7.0);
    EXPECT_DOUBLE_EQ(sum.Z(), 9.0);

    Vector3D diff = v2 - v1;
    EXPECT_DOUBLE_EQ(diff.X(), 3.0);
    EXPECT_DOUBLE_EQ(diff.Y(), 3.0);
    EXPECT_DOUBLE_EQ(diff.Z(), 3.0);

    Vector3D scaled = v1 * 2.0;
    EXPECT_DOUBLE_EQ(scaled.X(), 2.0);
    EXPECT_DOUBLE_EQ(scaled.Y(), 4.0);
    EXPECT_DOUBLE_EQ(scaled.Z(), 6.0);

    Vector3D divided = v2 / 2.0;
    EXPECT_DOUBLE_EQ(divided.X(), 2.0);
    EXPECT_DOUBLE_EQ(divided.Y(), 2.5);
    EXPECT_DOUBLE_EQ(divided.Z(), 3.0);
}