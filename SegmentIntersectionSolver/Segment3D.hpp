#pragma once
#include "Vector3D.hpp"


class Segment3D final
{
private:
	Vector3D start_, end_;

public:
	Segment3D(const Vector3D& start, const Vector3D& end) noexcept : start_(start), end_(end) {}

	Vector3D Direction() const { return end_ - start_; }
	Vector3D Start() const { return start_; }
	Vector3D End() const { return end_; }
};
