#pragma once


class Vector3D final
{
private:
	double x_, y_, z_;

public:
	Vector3D(double x, double y, double z) noexcept: x_(x), y_(y), z_(z) {}

	double X() const { return x_; }
	double Y() const { return y_; }
	double Z() const { return z_; }

	Vector3D operator+(const Vector3D& v) const 
	{ 
		return Vector3D(x_ + v.x_, y_ + v.y_, z_ + v.z_); 
	}
	Vector3D operator-(const Vector3D& v) const 
	{ 
		return Vector3D(x_ - v.x_, y_ - v.y_, z_ - v.z_); 
	}
	Vector3D operator*(double c) const 
	{ 
		return Vector3D(x_ * c, y_ * c, z_ * c); 
	}
	Vector3D operator/(double c) const 
	{ 
		return Vector3D(x_ / c, y_ / c, z_ / c); 
	}
};
