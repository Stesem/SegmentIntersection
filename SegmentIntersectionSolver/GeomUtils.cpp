#include "GeomUtils.hpp"

Vector3D GeomUtils::Cross(const Vector3D& a, const Vector3D& b)
{
    return Vector3D(
        a.Y() * b.Z() - a.Z() * b.Y(),
        a.Z() * b.X() - a.X() * b.Z(),
        a.X() * b.Y() - a.Y() * b.X()
    );
}

double GeomUtils::Dot(const Vector3D& a, const Vector3D& b)
{
    return a.X() * b.X() + a.Y() * b.Y() + a.Z() * b.Z();
}

double GeomUtils::SquaredModule(const Vector3D& v)
{
    return Dot(v, v);
}
