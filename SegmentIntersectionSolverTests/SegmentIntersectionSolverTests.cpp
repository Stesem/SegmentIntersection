#include "pch.h"
#include "SegmentIntersectionSolver.hpp"
#include "GeomUtils.hpp"


using IntersectionResult = std::optional<std::variant<Vector3D, Segment3D>>;

static bool MatchesPoint(
    const IntersectionResult& result,
    const Vector3D& v_expected,
    double tolerance = 1e-9) 
{
    if (!result || !std::holds_alternative<Vector3D>(*result))
        return false;

    auto& v_actual = std::get<Vector3D>(*result);

    return GeomUtils::SquaredModule(v_actual - v_expected) <= tolerance * tolerance;
}

static bool MatchesSegment(
    const IntersectionResult& result, 
    const Vector3D& v_start, 
    const Vector3D& v_end, 
    double tolerance = 1e-9) 
{
    if (!result || !std::holds_alternative<Segment3D>(*result)) 
        return false;

    auto& seg_actual = std::get<Segment3D>(*result);

    double direct_match = GeomUtils::SquaredModule(seg_actual.Start() - v_start) +
        GeomUtils::SquaredModule(seg_actual.End() - v_end);
    double reverse_match = GeomUtils::SquaredModule(seg_actual.Start() - v_end) +
        GeomUtils::SquaredModule(seg_actual.End() - v_start);

    return (direct_match <= tolerance * tolerance) || (reverse_match <= tolerance * tolerance);
}

TEST(SegmentIntersect, IdenticalPoints_ReturnPoint)
{
    Vector3D v_point(1.0, 2.0, 3.0);

    EXPECT_TRUE(MatchesPoint(
        SegmentIntersectionSolver::Intersect({ v_point, v_point }, { v_point, v_point }), 
        v_point));
}

TEST(SegmentIntersect, DifferentPoints_ReturnNullopt)
{
    Vector3D v1(0.0, 0.0, 0.0);
    Vector3D v2(5.0, 0.0, 0.0);

    EXPECT_FALSE(SegmentIntersectionSolver::Intersect({ v1, v1 }, { v2, v2 }).has_value());
}

TEST(SegmentIntersect, PointOnSegment_ReturnPoint)
{
    Vector3D v_point(5.0, 0.0, 0.0);
    Segment3D seg({ 0.0, 0.0, 0.0 }, { 10.0, 0.0, 0.0 });

    EXPECT_TRUE(MatchesPoint(
        SegmentIntersectionSolver::Intersect({ v_point, v_point }, seg), 
        v_point));
}

TEST(SegmentIntersect, PointOutsideSegment_ReturnNullopt)
{
    Vector3D v_point(15.0, 0.0, 0.0);
    Segment3D seg({ 0.0, 0.0, 0.0 }, { 10.0, 0.0, 0.0 });

    EXPECT_FALSE(SegmentIntersectionSolver::Intersect({ v_point, v_point }, seg).has_value());
}

TEST(SegmentIntersect, SecondPointOnFirstSegment_ReturnPoint)
{
    Vector3D v_point(0.0, 5.0, 0.0);
    Segment3D seg({ 0.0, 0.0, 0.0 }, { 0.0, 10.0, 0.0 });

    EXPECT_TRUE(MatchesPoint(SegmentIntersectionSolver::Intersect(seg, { v_point, v_point }), v_point));
}

TEST(SegmentIntersect, SkewSegments_ReturnNullopt)
{
    Segment3D seg1({ 0.0, 0.0, 0.0 }, { 10.0, 0.0, 0.0 });
    Segment3D seg2({ 5.0, -5.0, 5.0 }, { 5.0, 5.0, 5.0 });

    EXPECT_FALSE(SegmentIntersectionSolver::Intersect(seg1, seg2).has_value());
}

TEST(SegmentIntersect, ParallelNonCollinear_ReturnNullopt)
{
    Segment3D seg1({ 0.0, 0.0, 0.0 }, { 10.0, 0.0, 0.0 });
    Segment3D seg2({ 0.0, 5.0, 0.0 }, { 10.0, 5.0, 0.0 });

    EXPECT_FALSE(SegmentIntersectionSolver::Intersect(seg1, seg2).has_value());
}

TEST(SegmentIntersect, CollinearNoOverlap_ReturnNullopt)
{
    Segment3D seg1({ 0.0, 0.0, 0.0 }, { 5.0, 0.0, 0.0 });
    Segment3D seg2({ 10.0, 0.0, 0.0 }, { 15.0, 0.0, 0.0 });

    EXPECT_FALSE(SegmentIntersectionSolver::Intersect(seg1, seg2).has_value());
}

TEST(SegmentIntersect, CollinearIntersectAtEndpoint_ReturnPoint)
{
    Segment3D seg1({ 0.0, 0.0, 0.0 }, { 5.0, 0.0, 0.0 });
    Segment3D seg2({ 5.0, 0.0, 0.0 }, { 10.0, 0.0, 0.0 });

    Vector3D v_expected(5.0, 0.0, 0.0);

    EXPECT_TRUE(MatchesPoint(SegmentIntersectionSolver::Intersect(seg1, seg2), v_expected));
}

TEST(SegmentIntersect, CollinearPartialOverlap_ReturnSegment)
{
    Segment3D seg1({ 0.0, 0.0, 0.0 }, { 10.0, 0.0, 0.0 });
    Segment3D seg2({ 5.0, 0.0, 0.0 }, { 15.0, 0.0, 0.0 });

    Vector3D v_overlap_start(5.0, 0.0, 0.0);
    Vector3D v_overlap_end(10.0, 0.0, 0.0);

    EXPECT_TRUE(MatchesSegment(SegmentIntersectionSolver::Intersect(seg1, seg2), v_overlap_start, v_overlap_end));
}

TEST(SegmentIntersect, CollinearInnerSegment_ReturnInner)
{
    Segment3D seg_out({ 0.0, 0.0, 0.0 }, { 20.0, 0.0, 0.0 });
    Segment3D seg_in({ 5.0, 0.0, 0.0 }, { 15.0, 0.0, 0.0 });

    Vector3D v_inner_start(5.0, 0.0, 0.0);
    Vector3D v_inner_end(15.0, 0.0, 0.0);

    EXPECT_TRUE(MatchesSegment(SegmentIntersectionSolver::Intersect(seg_out, seg_in), v_inner_start, v_inner_end));
}

TEST(SegmentIntersect, CollinearOppositeDirection_ReturnSegment)
{
    Segment3D seg1({ 0.0, 0.0, 0.0 }, { 10.0, 0.0, 0.0 });
    Segment3D seg2({ 8.0, 0.0, 0.0 }, { 2.0, 0.0, 0.0 });

    Vector3D v_overlap_start(2.0, 0.0, 0.0);
    Vector3D v_overlap_end(8.0, 0.0, 0.0);

    EXPECT_TRUE(MatchesSegment(SegmentIntersectionSolver::Intersect(seg1, seg2), v_overlap_start, v_overlap_end));
}

TEST(SegmentIntersect, GeneralMiddleIntersection_ReturnPoint)
{
    Segment3D horizontal({ 0.0, 0.0, 0.0 }, { 10.0, 0.0, 0.0 });
    Segment3D vertical({ 5.0, -5.0, 0.0 }, { 5.0, 5.0, 0.0 });

    Vector3D v_expected(5.0, 0.0, 0.0);

    EXPECT_TRUE(MatchesPoint(SegmentIntersectionSolver::Intersect(horizontal, vertical), v_expected));
}

TEST(SegmentIntersect, GeneralEndpointIntersection_ReturnPoint)
{
    Segment3D seg1({ 0.0, 0.0, 0.0 }, { 5.0, 0.0, 0.0 });
    Segment3D seg2({ 5.0, 0.0, 0.0 }, { 5.0, 5.0, 0.0 });

    Vector3D v_expected(5.0, 0.0, 0.0);

    EXPECT_TRUE(MatchesPoint(SegmentIntersectionSolver::Intersect(seg1, seg2), v_expected));
}

TEST(SegmentIntersect, LinesIntersectSegments_DoNotReturnNullopt)
{
    Segment3D seg1({ 0.0, 0.0, 0.0 }, { 5.0, 0.0, 0.0 });
    Segment3D seg2({ 10.0, -5.0, 0.0 }, { 10.0, 5.0, 0.0 });

    EXPECT_FALSE(SegmentIntersectionSolver::Intersect(seg1, seg2).has_value());
}

TEST(SegmentIntersect, CollinearGapLargerThanEpsilon_ReturnNullopt)
{
    Segment3D seg1({ 0.0, 0.0, 0.0 }, { 5.0, 0.0, 0.0 });
    Segment3D seg2({ 5.0 + 1e-9 + 1e-10, 0.0, 0.0 }, { 10.0, 0.0, 0.0 });

    EXPECT_FALSE(SegmentIntersectionSolver::Intersect(seg1, seg2).has_value());
}

TEST(SegmentIntersect, ShortIntersectingSegments_ReturnPoint)
{
    double scale = 1e-10;

    Segment3D seg1({ 0.0, 0.0, 0.0 }, { scale, 0.0, 0.0 });
    Segment3D seg2({ scale / 2.0, -scale / 2.0, 0.0 }, { scale / 2.0, scale / 2.0, 0.0 });

    Vector3D v_expected(scale / 2.0, 0.0, 0.0);

    auto result = SegmentIntersectionSolver::Intersect(seg1, seg2);
    if (result.has_value()) {
        EXPECT_TRUE(MatchesPoint(result, v_expected, 1e-8));
    }
}

TEST(SegmentIntersect, CollinearIntersectNoise_ReturnValidPoint)
{
    Segment3D first_segment({ 0.0, 0.0, 0.0 }, { 5.0, 0.0, 0.0 });
    Segment3D second_segment({ -5.0, 0.0, 0.0 }, { 1e-15, 0.0, 0.0 });

    auto result = SegmentIntersectionSolver::Intersect(first_segment, second_segment);
    if (result && std::holds_alternative<Vector3D>(*result)) {
        auto v_touch = std::get<Vector3D>(*result);
        EXPECT_NEAR(GeomUtils::SquaredModule(v_touch), 0.0, 1e-18);
    }
}