#include "pch.h"
#include "Segment3D.hpp"


TEST(Segment3DTest, Constructors)
{
    Vector3D start1(0.0, 0.0, 0.0);
    Vector3D end1(1.0, 1.0, 1.0);

    EXPECT_NO_THROW(Segment3D seg1(start1, end1));
    EXPECT_NO_THROW(Segment3D seg2(start1, start1));
}

TEST(Segment3DTest, Accessors)
{
    Vector3D start(0.0, 0.0, 0.0);
    Vector3D end(1.0, 1.0, 1.0);
    Segment3D seg(start, end);

    EXPECT_DOUBLE_EQ(seg.Start().X(), 0.0);
    EXPECT_DOUBLE_EQ(seg.End().Z(), 1.0);

    Vector3D dir = seg.Direction();

    EXPECT_DOUBLE_EQ(dir.X(), 1.0);
    EXPECT_DOUBLE_EQ(dir.Y(), 1.0);
    EXPECT_DOUBLE_EQ(dir.Z(), 1.0);
}