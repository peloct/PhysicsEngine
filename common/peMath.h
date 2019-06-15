#pragma once

#include"peSettings.h"
#include<float.h>

class Vector3;
class Matrix3x3;
class Quaternion;

const float64 PI = 3.14159265358979323846;
const float32 MaxFloat32 = FLT_MAX;

float64 peAbs(float64 a);
float32 peAbsf(float32 a);
float32 peMinf(float32 a, float32 b);
float32 peMaxf(float32 a, float32 b);
int32 peMax(int32 a, int32 b);
float32 peSqrtf(float32 a);
float64 peCos(float64 rad);
float64 peSin(float64 rad);
float64 peTan(float64 rad);
float32 peCosf(float32 rad);
float32 peSinf(float32 rad);
float32 peTanDegf(float32 deg);
float32 peTanf(float32 rad);
float64 peAsin(float64 val);
float64 peAtan2(float64 y, float64 x);
float64 peCopysign(float64 number, float64 sign);
float32 peClamp(float32 val, float32 min, float32 max);
float32 peFmod(float32 val, float32 mod);

class Vector2
{
public:
	float32 x;
	float32 y;

	Vector2() : x(0), y(0) {}
	Vector2(float32 x, float32 y) : x(x), y(y) {}

	Vector2 operator-() const
	{
		return Vector2(-x, -y);
	}

	Vector2 operator+(const Vector2& rhs) const
	{
		return Vector2(x + rhs.x, y + rhs.y);
	}

	Vector2 operator-(const Vector2& rhs) const
	{
		return Vector2(x - rhs.x, y - rhs.y);
	}

	Vector2 operator*(const float32 rhs) const
	{
		return Vector2(x * rhs, y * rhs);
	}

	Vector2 operator/(const float32 rhs) const
	{
		return Vector2(x / rhs, y / rhs);
	}

	float32 magnitude() const
	{
		return peSqrtf(x * x + y * y);
	}

	void setZero()
	{
		x = 0;
		y = 0;
	}
};

static Vector2 operator*(const float32 lhs, const Vector2 rhs)
{
	return Vector2(rhs.x * lhs, rhs.y * lhs);
}

class Matrix2x2
{
public:
	float32 m00;
	float32 m10;
	float32 m01;
	float32 m11;

	Matrix2x2() : m00(0), m01(0), m10(0), m11(0) {}

	void setZero()
	{
		m00 = 0;
		m01 = 0;
		m10 = 0;
		m11 = 0;
	}

	void inverse()
	{
		Matrix2x2 newM;

		float32 invDet = 1 / (m00 * m11 - m01 * m10);
		newM.m00 = m11 * invDet;
		newM.m11 = m00 * invDet;
		newM.m01 = -m01 * invDet;
		newM.m10 = -m10 * invDet;

		(*this) = newM;
	}

	Vector2 operator*(const Vector2& rhs) const
	{
		Vector2 ret;
		ret.x = m00 * rhs.x + m01 * rhs.y;
		ret.y = m10 * rhs.x + m11 * rhs.y;
		return ret;
	}
};

class Vector3
{
public:
	float32 x;
	float32 y;
	float32 z;

	Vector3() : x(0), y(0), z(0) {}
	Vector3(float32 x, float32 y, float32 z) : x(x), y(y), z(z) {}

	Vector3 operator/ (const float32 rhs) const
	{
		return Vector3(x / rhs, y / rhs, z / rhs);
	}

	Vector3 operator* (const float32 rhs) const
	{
		return Vector3(x * rhs, y * rhs, z * rhs);
	}

	Vector3 operator+ (const Vector3& rhs) const
	{
		return Vector3(x + rhs.x, y + rhs.y, z + rhs.z);
	}

	Vector3 operator- (const Vector3& rhs) const
	{
		return Vector3(x - rhs.x, y - rhs.y, z - rhs.z);
	}

	void operator+= (const Vector3& rhs)
	{
		x += rhs.x;
		y += rhs.y;
		z += rhs.z;
	}

	void operator-= (const Vector3& rhs)
	{
		x -= rhs.x;
		y -= rhs.y;
		z -= rhs.z;
	}

	void operator*= (float32 rhs)
	{
		x *= rhs;
		y *= rhs;
		z *= rhs;
	}

	void operator/= (float32 rhs)
	{
		x /= rhs;
		y /= rhs;
		z /= rhs;
	}

	Vector3 operator- () const
	{
		return Vector3(-x, -y, -z);
	}

	float32 sqrMagnitude() const
	{
		return x * x + y * y + z * z;
	}

	float32 magnitude() const
	{
		return peSqrtf(x * x + y * y + z * z);
	}

	void normalize()
	{
		float32 magInv = 1 / magnitude();
		(*this) = (*this) * magInv;
	}

	static float32 dot(const Vector3& a, const Vector3& b)
	{
		return a.x * b.x + a.y * b.y + a.z * b.z;
	}

	float32 dot(const Vector3& rhs) const
	{
		return x * rhs.x + y * rhs.y + z * rhs.z;
	}

	static Vector3 cross(const Vector3& a, const Vector3& b)
	{
		return Vector3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
	}

	Vector3 cross(const Vector3& rhs) const
	{
		return Vector3(y * rhs.z - z * rhs.y, z * rhs.x - x * rhs.z, x * rhs.y - y * rhs.x);
	}

	void addScaledVector(const Vector3& vector, float32 scale)
	{
		(*this) += (vector * scale);
	}

	void set(float32 x, float32 y, float32 z)
	{
		this->x = x;
		this->y = y;
		this->z = z;
	}

	void setZero()
	{
		this->x = 0.0f;
		this->y = 0.0f;
		this->z = 0.0f;
	}

	void getCrossMatrix(Matrix3x3* m);
};

static Vector3 operator* (const float32 lhs, const Vector3& rhs)
{
	return Vector3(lhs * rhs.x, lhs * rhs.y, lhs * rhs.z);
}

class Vector4
{
public:
	float32 x;
	float32 y;
	float32 z;
	float32 w;

	Vector4() : x(0), y(0), z(0), w(0) {}
	Vector4(float32 x, float32 y, float32 z, float32 w) : x(x), y(y), z(z), w(w) {}
};

class Matrix3x3
{
public:
	float32 m00;
	float32 m10;
	float32 m20;
	float32 m01;
	float32 m11;
	float32 m21;
	float32 m02;
	float32 m12;
	float32 m22;

	Matrix3x3() : 
		m00(0.0f), m01(0.0f), m02(0.0f),
		m10(0.0f), m11(0.0f), m12(0.0f),
		m20(0.0f), m21(0.0f), m22(0.0f)
	{}

	void identity()
	{
		m00 = 1.0f;
		m01 = 0.0f;
		m02 = 0.0f;
		m10 = 0.0f;
		m11 = 1.0f;
		m12 = 0.0f;
		m20 = 0.0f;
		m21 = 0.0f;
		m22 = 1.0f;
	}

	void setZero()
	{
		m00 = 0.0f;
		m01 = 0.0f;
		m02 = 0.0f;
		m10 = 0.0f;
		m11 = 0.0f;
		m12 = 0.0f;
		m20 = 0.0f;
		m21 = 0.0f;
		m22 = 0.0f;
	}

	// eulerAngle : deg
	static Matrix3x3 getEulerAngleMatrix(Vector3 eulerAngle)
	{
		eulerAngle = eulerAngle * (PI / 180.0f);
		Matrix3x3 xRot;
		Matrix3x3 yRot;
		Matrix3x3 zRot;

		float32 cx = peCosf(eulerAngle.x);
		float32 sx = peSinf(eulerAngle.x);
		float32 cy = peCosf(eulerAngle.y);
		float32 sy = peSinf(eulerAngle.y);
		float32 cz = peCosf(eulerAngle.z);
		float32 sz = peSinf(eulerAngle.z);

		xRot.m00 = 1.0f;
		xRot.m11 = cx;
		xRot.m12 = -sx;
		xRot.m21 = sx;
		xRot.m22 = cx;

		yRot.m00 = cy;
		yRot.m02 = sy;
		yRot.m11 = 1.0f;
		yRot.m20 = -sy;
		yRot.m22 = cy;

		zRot.m00 = cz;
		zRot.m01 = -sz;
		zRot.m10 = sz;
		zRot.m11 = cz;
		zRot.m22 = 1.0f;

		return zRot * yRot* xRot;
	}

	float32 getDeterminant()
	{
		float32 ret = m00 * (m11 * m22 - m12 * m21) - m01 * (m10 * m22 - m12 * m20) + m02 * (m10 * m21 - m11 * m20);
		return ret;
	}

	Matrix3x3 getInverse()
	{
		float32 det = getDeterminant();
		float32 invDet = 1 / det;

		Matrix3x3 ret;
		ret.m00 = (m11 * m22 - m12 * m21) * invDet;
		ret.m01 = -(m01 * m22 - m02 * m21) * invDet;
		ret.m02 = (m01 * m12 - m02 * m11) * invDet;
		ret.m10 = -(m10 * m22 - m12 * m20) * invDet;
		ret.m11 = (m00 * m22 - m02 * m20) * invDet;
		ret.m12 = -(m00 * m12 - m02 * m10) * invDet;
		ret.m20 = (m10 * m21 - m11 * m20) * invDet;
		ret.m21 = -(m00 * m21 - m01 * m20) * invDet;
		ret.m22 = (m00 * m11 - m01 * m10) * invDet;

		return ret;
	}

	void transpose() 
	{
		Matrix3x3 ret;
		ret.m00 = m00;
		ret.m01 = m10;
		ret.m02 = m20;
		ret.m10 = m01;
		ret.m11 = m11;
		ret.m12 = m21;
		ret.m20 = m02;
		ret.m21 = m12;
		ret.m22 = m22;
		(*this) = ret;
	}

	Matrix3x3 getTranspose() const
	{
		Matrix3x3 ret;
		ret.m00 = m00;
		ret.m01 = m10;
		ret.m02 = m20;
		ret.m10 = m01;
		ret.m11 = m11;
		ret.m12 = m21;
		ret.m20 = m02;
		ret.m21 = m12;
		ret.m22 = m22;
		return ret;
	}

	Matrix3x3 operator* (const Matrix3x3& rhs) const
	{
		Matrix3x3 ret;
		ret.m00 = m00 * rhs.m00 + m01 * rhs.m10 + m02 * rhs.m20;
		ret.m01 = m00 * rhs.m01 + m01 * rhs.m11 + m02 * rhs.m21;
		ret.m02 = m00 * rhs.m02 + m01 * rhs.m12 + m02 * rhs.m22;
		ret.m10 = m10 * rhs.m00 + m11 * rhs.m10 + m12 * rhs.m20;
		ret.m11 = m10 * rhs.m01 + m11 * rhs.m11 + m12 * rhs.m21;
		ret.m12 = m10 * rhs.m02 + m11 * rhs.m12 + m12 * rhs.m22;
		ret.m20 = m20 * rhs.m00 + m21 * rhs.m10 + m22 * rhs.m20;
		ret.m21 = m20 * rhs.m01 + m21 * rhs.m11 + m22 * rhs.m21;
		ret.m22 = m20 * rhs.m02 + m21 * rhs.m12 + m22 * rhs.m22;
		return ret;
	}

	void setValues(const Vector3& a, const Vector3& b, const Vector3& c)
	{
		m00 = a.x;
		m10 = a.y;
		m20 = a.z;

		m01 = b.x;
		m11 = b.y;
		m21 = b.z;

		m02 = c.x;
		m12 = c.y;
		m22 = c.z;
	}

	Matrix3x3 operator- (const Matrix3x3& rhs) const
	{
		Matrix3x3 ret;
		ret.m00 = m00 - rhs.m00;
		ret.m01 = m01 - rhs.m01;
		ret.m02 = m02 - rhs.m02;
		ret.m10 = m10 - rhs.m10;
		ret.m11 = m11 - rhs.m11;
		ret.m12 = m12 - rhs.m12;
		ret.m20 = m20 - rhs.m20;
		ret.m21 = m21 - rhs.m21;
		ret.m22 = m22 - rhs.m22;
		return ret;
	}

	Matrix3x3 operator+ (const Matrix3x3& rhs) const
	{
		Matrix3x3 ret;
		ret.m00 = m00 + rhs.m00;
		ret.m01 = m01 + rhs.m01;
		ret.m02 = m02 + rhs.m02;
		ret.m10 = m10 + rhs.m10;
		ret.m11 = m11 + rhs.m11;
		ret.m12 = m12 + rhs.m12;
		ret.m20 = m20 + rhs.m20;
		ret.m21 = m21 + rhs.m21;
		ret.m22 = m22 + rhs.m22;
		return ret;
	}

	Matrix3x3 operator* (const float32 rhs) const
	{
		Matrix3x3 ret;
		ret.m00 = m00 * rhs;
		ret.m01 = m01 * rhs;
		ret.m02 = m02 * rhs;
		ret.m10 = m10 * rhs;
		ret.m11 = m11 * rhs;
		ret.m12 = m12 * rhs;
		ret.m20 = m20 * rhs;
		ret.m21 = m21 * rhs;
		ret.m22 = m22 * rhs;
		return ret;
	}

	Vector3 operator* (const Vector3& rhs) const
	{
		Vector3 ret;
		ret.x = m00 * rhs.x + m01 * rhs.y + m02 * rhs.z;
		ret.y = m10 * rhs.x + m11 * rhs.y + m12 * rhs.z;
		ret.z = m20 * rhs.x + m21 * rhs.y + m22 * rhs.z;
		return ret;
	}

	Vector3 transform(const Vector3& rhs) const
	{
		Vector3 ret;
		ret.x = m00 * rhs.x + m01 * rhs.y + m02 * rhs.z;
		ret.y = m10 * rhs.x + m11 * rhs.y + m12 * rhs.z;
		ret.z = m20 * rhs.x + m21 * rhs.y + m22 * rhs.z;
		return ret;
	}

	Vector3 transposeMuliply(const Vector3& rhs) const
	{
		Vector3 ret;
		ret.x = m00 * rhs.x + m10 * rhs.y + m20 * rhs.z;
		ret.y = m01 * rhs.x + m11 * rhs.y + m21 * rhs.z;
		ret.z = m02 * rhs.x + m12 * rhs.y + m22 * rhs.z;
		return ret;
	}
};

class Matrix4x4
{
public:
	float32 m00;
	float32 m10;
	float32 m20;
	float32 m30;
	float32 m01;
	float32 m11;
	float32 m21;
	float32 m31;
	float32 m02;
	float32 m12;
	float32 m22;
	float32 m32;
	float32 m03;
	float32 m13;
	float32 m23;
	float32 m33;

	Matrix4x4() :
		m00(0.0f), m01(0.0f), m02(0.0f), m03(0.0f),
		m10(0.0f), m11(0.0f), m12(0.0f), m13(0.0f),
		m20(0.0f), m21(0.0f), m22(0.0f), m23(0.0f),
		m30(0.0f), m31(0.0f), m32(0.0f), m33(0.0f)
	{
	}

	Matrix4x4(const Matrix3x3& rotation, const Vector3& translation)
	{
		m00 = rotation.m00;
		m01 = rotation.m01;
		m02 = rotation.m02;
		m03 = translation.x;

		m10 = rotation.m10;
		m11 = rotation.m11;
		m12 = rotation.m12;
		m13 = translation.y;

		m20 = rotation.m20;
		m21 = rotation.m21;
		m22 = rotation.m22;
		m23 = translation.z;

		m30 = 0.0f;
		m31 = 0.0f;
		m32 = 0.0f;
		m33 = 1.0f;
	}

	// fov : frustrum 의 세로 각도 (deg)
	// aspect : height / width
	// near, far : 양의 값으로 된 가장 앞, 먼 거리
	static Matrix4x4 getProjectionMatrix(float32 fov, float32 aspect, float32 n, float32 f)
	{
		Matrix4x4 ret;
		float32 yRat = peTanDegf(fov * 0.5f);
		float32 xRat = yRat * aspect;
		ret.m00 = 1 / xRat;
		ret.m11 = 1 / yRat;
		ret.m22 = -(f + n) / (f - n);
		ret.m23 = -2 * f * n / (f - n);
		ret.m32 = -1;
		return ret;
	}

	void setTransform(const Matrix3x3& rotation, const Vector3& translation)
	{
		m00 = rotation.m00;
		m01 = rotation.m01;
		m02 = rotation.m02;
		m03 = translation.x;

		m10 = rotation.m10;
		m11 = rotation.m11;
		m12 = rotation.m12;
		m13 = translation.y;

		m20 = rotation.m20;
		m21 = rotation.m21;
		m22 = rotation.m22;
		m23 = translation.z;

		m30 = 0.0f;
		m31 = 0.0f;
		m32 = 0.0f;
		m33 = 1.0f;
	}

	void setInverseTransform(const Matrix3x3& rotation, const Vector3& translation)
	{
		m00 = rotation.m00;
		m10 = rotation.m01;
		m20 = rotation.m02;
		m01 = rotation.m10;
		m11 = rotation.m11;
		m21 = rotation.m12;
		m02 = rotation.m20;
		m12 = rotation.m21;
		m22 = rotation.m22;

		m03 = -(rotation.m00 * translation.x + rotation.m10 * translation.y + rotation.m20 * translation.z);
		m13 = -(rotation.m01 * translation.x + rotation.m11 * translation.y + rotation.m21 * translation.z);
		m23 = -(rotation.m02 * translation.x + rotation.m12 * translation.y + rotation.m22 * translation.z);
		m33 = 1.0f;
	}

	Matrix4x4 operator* (const Matrix4x4& rhs) const
	{
		Matrix4x4 ret;
		ret.m00 = m00 * rhs.m00 + m01 * rhs.m10 + m02 * rhs.m20 + m03 * rhs.m30;
		ret.m01 = m00 * rhs.m01 + m01 * rhs.m11 + m02 * rhs.m21 + m03 * rhs.m31;
		ret.m02 = m00 * rhs.m02 + m01 * rhs.m12 + m02 * rhs.m22 + m03 * rhs.m32;
		ret.m03 = m00 * rhs.m03 + m01 * rhs.m13 + m02 * rhs.m23 + m03 * rhs.m33;

		ret.m10 = m10 * rhs.m00 + m11 * rhs.m10 + m12 * rhs.m20 + m13 * rhs.m30;
		ret.m11 = m10 * rhs.m01 + m11 * rhs.m11 + m12 * rhs.m21 + m13 * rhs.m31;
		ret.m12 = m10 * rhs.m02 + m11 * rhs.m12 + m12 * rhs.m22 + m13 * rhs.m32;
		ret.m13 = m10 * rhs.m03 + m11 * rhs.m13 + m12 * rhs.m23 + m13 * rhs.m33;

		ret.m20 = m20 * rhs.m00 + m21 * rhs.m10 + m22 * rhs.m20 + m23 * rhs.m30;
		ret.m21 = m20 * rhs.m01 + m21 * rhs.m11 + m22 * rhs.m21 + m23 * rhs.m31;
		ret.m22 = m20 * rhs.m02 + m21 * rhs.m12 + m22 * rhs.m22 + m23 * rhs.m32;
		ret.m23 = m20 * rhs.m03 + m21 * rhs.m13 + m22 * rhs.m23 + m23 * rhs.m33;

		ret.m30 = m30 * rhs.m00 + m31 * rhs.m10 + m32 * rhs.m20 + m33 * rhs.m30;
		ret.m31 = m30 * rhs.m01 + m31 * rhs.m11 + m32 * rhs.m21 + m33 * rhs.m31;
		ret.m32 = m30 * rhs.m02 + m31 * rhs.m12 + m32 * rhs.m22 + m33 * rhs.m32;
		ret.m33 = m30 * rhs.m03 + m31 * rhs.m13 + m32 * rhs.m23 + m33 * rhs.m33;

		return ret;
	}

	Matrix4x4 getInverseTransform()
	{
		Matrix4x4 ret;
		ret.m00 = m00;
		ret.m10 = m01;
		ret.m20 = m02;
		ret.m01 = m10;
		ret.m11 = m11;
		ret.m21 = m12;
		ret.m02 = m20;
		ret.m12 = m21;
		ret.m22 = m22;
		ret.m03 = -(m00 * m03 + m10 * m13 + m20 * m23);
		ret.m13 = -(m01 * m03 + m11 * m13 + m21 * m23);
		ret.m23 = -(m02 * m03 + m12 * m13 + m22 * m23);
		ret.m33 = 1.0f;
		return ret;
	}

	Vector3 transformPoint(const Vector3& pos) const
	{
		Vector3 ret;
		ret.x = m00 * pos.x + m01 * pos.y + m02 * pos.z + m03;
		ret.y = m10 * pos.x + m11 * pos.y + m12 * pos.z + m13;
		ret.z = m20 * pos.x + m21 * pos.y + m22 * pos.z + m23;
		return ret;
	}

	Vector3 transformDirection(const Vector3& dir) const
	{
		Vector3 ret;
		ret.x = m00 * dir.x + m01 * dir.y + m02 * dir.z;
		ret.y = m10 * dir.x + m11 * dir.y + m12 * dir.z;
		ret.z = m20 * dir.x + m21 * dir.y + m22 * dir.z;
		return ret;
	}
};

class Quaternion
{
public:
	float32 s;
	Vector3 u;

	Quaternion() : s(1), u(0, 0, 0) {}
	Quaternion(Vector3 v) : s(0), u(v) {}

	Quaternion operator* (const Quaternion& rhs) const
	{
		Quaternion ret;
		ret.s = s * rhs.s - u.dot(rhs.u);
		ret.u = rhs.u * s + u * rhs.s + u.cross(rhs.u);
		return ret;
	}

	Vector3 operator* (const Vector3& rhs) const
	{
		return 2.0f * Vector3::dot(u, rhs) * u
			+ (s * s - Vector3::dot(u, u)) * rhs
			+ 2.0f * s * Vector3::cross(u, rhs);
	}

	static Quaternion getEuler(const Vector3& eulerAngle)
	{
		float64 cy = peCos(eulerAngle.z * 0.5);
		float64 sy = peSin(eulerAngle.z * 0.5);
		float64 cp = peCos(eulerAngle.y * 0.5);
		float64 sp = peSin(eulerAngle.y * 0.5);
		float64 cr = peCos(eulerAngle.x * 0.5);
		float64 sr = peSin(eulerAngle.x * 0.5);
	
		Quaternion q;
		q.s = cy * cp * cr + sy * sp * sr;
		q.u.x = cy * cp * sr - sy * sp * cr;
		q.u.y = sy * cp * sr + cy * sp * cr;
		q.u.z = sy * cp * cr - cy * sp * sr;
		return q;
	}

	Vector3 toEulerAngle() const
	{
		Vector3 ret;
	
		float64 w = s;
		float64 x = u.x;
		float64 y = u.y;
		float64 z = u.z;
	
		// roll (x-axis rotation)
		float64 sinr_cosp = +2.0 * (w * x + y * z);
		float64 cosr_cosp = +1.0 - 2.0 * (x * x + y * y);
		ret.x = peAtan2(sinr_cosp, cosr_cosp);
	
		// pitch (y-axis rotation)
		float64 sinp = +2.0 * (w * y - z * x);
		if (peAbs(sinp) >= 1)
			ret.y = peCopysign(PI / 2, sinp); // use 90 degrees if out of range
		else
			ret.y = peAsin(sinp);
	
		// yaw (z-axis rotation)
		float64 siny_cosp = +2.0 * (w * z + x * y);
		float64 cosy_cosp = +1.0 - 2.0 * (y * y + z * z);
		ret.z = peAtan2(siny_cosp, cosy_cosp);
	
		return ret;
	}

	void identity()
	{
		s = 1;
		u.set(0.0f, 0.0f, 0.0f);
	}

	void normailize()
	{
		float32 length = peSqrtf(s * s + u.x * u.x + u.y * u.y + u.z * u.z);
		s = s / length;
		u = u / length;
	}

	Quaternion normalized() const
	{
		Quaternion ret;
		ret.s = s;
		ret.u = u;
		ret.normailize();
		return ret;
	}

	void addScaledVector(const Vector3& vector, float32 scale)
	{
		Quaternion q(vector * scale);
		q = q * (*this);
		s += q.s * ((float32)0.5);
		u.x += q.u.x * ((float32)0.5);
		u.y += q.u.y * ((float32)0.5);
		u.z += q.u.z * ((float32)0.5);
	}

	void getOrientationMatrix(Matrix3x3* m);
};