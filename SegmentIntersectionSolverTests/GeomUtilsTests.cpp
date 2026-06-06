#include "pch.h"
#include "GeomUtils.hpp"


class GeomUtilsTest : public ::testing::Test {
protected:
    static constexpr double eps = 1e-9;
};

TEST_F(GeomUtilsTest, DotProduct_OrthogonalVectors_ReturnsZero)
{
    Vector3D x(1.0, 0.0, 0.0);
    Vector3D y(0.0, 1.0, 0.0);

    EXPECT_NEAR(GeomUtils::Dot(x, y), 0.0, eps);
}

TEST_F(GeomUtilsTest, DotProduct_ParallelVectors_ReturnsProductOfLengths)
{
    Vector3D v1(2.0, 0.0, 0.0);
    Vector3D v2(3.0, 0.0, 0.0);

    EXPECT_NEAR(GeomUtils::Dot(v1, v2), 6.0, eps);
}

TEST_F(GeomUtilsTest, DotProduct_AntiParallelVectors_ReturnsNegativeProduct)
{
    Vector3D v1(2.0, 0.0, 0.0);
    Vector3D v2(-3.0, 0.0, 0.0);

    EXPECT_NEAR(GeomUtils::Dot(v1, v2), -6.0, eps);
}

TEST_F(GeomUtilsTest, DotProduct_ZeroVector_ReturnsZero)
{
    Vector3D zero_v(0.0, 0.0, 0.0);
    Vector3D v(1.0, 2.0, 3.0);

    EXPECT_DOUBLE_EQ(GeomUtils::Dot(zero_v, v), 0.0);
}

TEST_F(GeomUtilsTest, CrossProduct_OrthogonalVectors_RightHandRule)
{
    Vector3D x(1.0, 0.0, 0.0);
    Vector3D y(0.0, 1.0, 0.0);
    Vector3D z(0.0, 0.0, 1.0);

    Vector3D result = GeomUtils::Cross(x, y);

    EXPECT_NEAR(result.X(), z.X(), eps);
    EXPECT_NEAR(result.Y(), z.Y(), eps);
    EXPECT_NEAR(result.Z(), z.Z(), eps);
}

TEST_F(GeomUtilsTest, CrossProduct_CollinearVectors_ReturnsZeroVector)
{
    Vector3D v1(1.0, 2.0, 3.0);
    Vector3D v2(2.0, 4.0, 6.0);

    EXPECT_NEAR(GeomUtils::SquaredModule(GeomUtils::Cross(v1, v2)), 0.0, eps);
}

TEST_F(GeomUtilsTest, CrossProduct_Anticommutativity)
{
    Vector3D v1(1.0, 2.0, 3.0);
    Vector3D v2(4.0, 5.0, 6.0);

    Vector3D v1v2 = GeomUtils::Cross(v1, v2);
    Vector3D v2v1 = GeomUtils::Cross(v2, v1);

    EXPECT_NEAR(v1v2.X(), -v2v1.X(), eps);
    EXPECT_NEAR(v1v2.Y(), -v2v1.Y(), eps);
    EXPECT_NEAR(v1v2.Z(), -v2v1.Z(), eps);
}

TEST_F(GeomUtilsTest, CrossProduct_Result_IsOrthogonalToInputs)
{
    Vector3D v1(1.0, 2.0, 3.0);
    Vector3D v2(4.0, 5.0, 6.0);

    Vector3D v3 = GeomUtils::Cross(v1, v2);

    EXPECT_NEAR(GeomUtils::Dot(v3, v1), 0.0, eps);
    EXPECT_NEAR(GeomUtils::Dot(v3, v2), 0.0, eps);
}

TEST_F(GeomUtilsTest, SquaredModule_UnitVector_ReturnsOne)
{
    Vector3D v(1.0, 0.0, 0.0);

    EXPECT_NEAR(GeomUtils::SquaredModule(v), 1.0, eps);
}

TEST_F(GeomUtilsTest, SquaredModule_ZeroVector_ReturnsZero)
{
    Vector3D zero(0.0, 0.0, 0.0);

    EXPECT_DOUBLE_EQ(GeomUtils::SquaredModule(zero), 0.0);
}

TEST_F(GeomUtilsTest, SquaredModule_GeneralVector_ReturnsSumOfSquares)
{
    Vector3D v(1.0, 2.0, 3.0);

    EXPECT_NEAR(GeomUtils::SquaredModule(v), 14.0, eps);
}