#pragma once
#include "Segment3D.hpp"
#include <optional>
#include <variant>


static class SegmentIntersectionSolver
{
public:
	static std::optional<std::variant<Vector3D, Segment3D>> Intersect(
		const Segment3D& first_seg,
		const Segment3D& second_seg) noexcept;

private:
	static std::optional<Vector3D> IntersectPointWithSegment(
		const Vector3D& point,
		const Vector3D& seg_start,
		const Vector3D& seg_dir,
		double seg_squared_len) noexcept;
};

