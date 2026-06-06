#include "SegmentIntersectionSolver.hpp"
#include "GeomUtils.hpp"
#include <algorithm>


namespace {
    constexpr double COORD_EPS = 1e-9;
    constexpr double GEOM_EPS = 1e-18;
}

std::optional<Vector3D> SegmentIntersectionSolver::IntersectPointWithSegment(
    const Vector3D& point,
    const Vector3D& seg_start,
    const Vector3D& seg_direction,
    double seg_squared_len) noexcept
{
    Vector3D to_point = point - seg_start;

    double projection = GeomUtils::Dot(to_point, seg_direction);
    double t = projection / seg_squared_len;

    if (t < -COORD_EPS || t > 1.0 + COORD_EPS)
        return std::nullopt;

    Vector3D closest_on_seg = seg_start + seg_direction * t;

    if (GeomUtils::SquaredModule(point - closest_on_seg) <= GEOM_EPS)
        return point;

    return std::nullopt;
}

std::optional<std::variant<Vector3D, Segment3D>> SegmentIntersectionSolver::Intersect(
    const Segment3D& first_seg,
    const Segment3D& second_seg) noexcept
{
    Vector3D start1 = first_seg.Start();
    Vector3D d1 = first_seg.Direction();
    Vector3D start2 = second_seg.Start();
    Vector3D d2 = second_seg.Direction();

    Vector3D r = start1 - start2;

    double d1d1 = GeomUtils::Dot(d1, d1);
    double d2d2 = GeomUtils::Dot(d2, d2);

    bool is_first_point = (d1d1 <= GEOM_EPS);
    bool is_second_point = (d2d2 <= GEOM_EPS);

    if (is_first_point && is_second_point) 
    {
        if (GeomUtils::SquaredModule(r) <= GEOM_EPS)
            return start1;

        return std::nullopt;
    }

    if (is_first_point)
        return IntersectPointWithSegment(start1, start2, d2, d2d2);

    if (is_second_point)
        return IntersectPointWithSegment(start2, start1, d1, d1d1);

    double d1d2 = GeomUtils::Dot(d1, d2);
    double d1r = GeomUtils::Dot(d1, r);
    double d2r = GeomUtils::Dot(d2, r);

    Vector3D cross_d1_d2 = GeomUtils::Cross(d1, d2);
    double mixed_product = GeomUtils::Dot(cross_d1_d2, r);

    if (std::abs(mixed_product) > GEOM_EPS)
        return std::nullopt;

    double det = d1d1 * d2d2 - d1d2 * d1d2;

    if (std::abs(det) <= GEOM_EPS) 
    {
        Vector3D cross_r_d1 = GeomUtils::Cross(r, d1);
        if (GeomUtils::SquaredModule(cross_r_d1) > GEOM_EPS)
            return std::nullopt;

        double t_start = -d1r / d1d1;

        double t_end = (-d1r + d1d2) / d1d1;

        double t_min = std::min(t_start, t_end);
        double t_max = std::max(t_start, t_end);

        double overlap_start = std::max(0.0, t_min);
        double overlap_end = std::min(1.0, t_max);

        if (overlap_start - overlap_end > COORD_EPS)
            return std::nullopt;

        if (std::abs(overlap_start - overlap_end) <= COORD_EPS)
            return start1 + d1 * overlap_start;

        Vector3D seg_start = start1 + d1 * overlap_start;
        Vector3D seg_end = start1 + d1 * overlap_end;
        return Segment3D(seg_start, seg_end);
    }

    double t = (d1d2 * d2r - d1r * d2d2) / det;
    double u = (d1d1 * d2r - d1d2 * d1r) / det;

    if (t < -COORD_EPS || t > 1.0 + COORD_EPS ||
        u < -COORD_EPS || u > 1.0 + COORD_EPS) 
        return std::nullopt;

    Vector3D point_on_first = start1 + d1 * t;
    Vector3D point_on_second = start2 + d2 * u;

    if (GeomUtils::SquaredModule(point_on_first - point_on_second) > GEOM_EPS) 
        return std::nullopt;

    return point_on_first;
}