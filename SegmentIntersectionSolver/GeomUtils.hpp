#pragma once
#include "Vector3D.hpp"


static class GeomUtils
{
public:
	static Vector3D Cross(const Vector3D& a, const Vector3D& b);
	static double Dot(const Vector3D& a, const Vector3D& b);
	static double SquaredModule(const Vector3D& v);
};
