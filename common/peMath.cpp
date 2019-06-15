#include"peMath.h"
#include<math.h>

float64 peAbs(float64 a)
{
	return a < 0 ? -a : a;
}

float32 peAbsf(float32 a)
{
	return a < 0 ? -a : a;
}

float32 peMinf(float32 a, float32 b)
{
	return a < b ? a : b;
}

float32 peMaxf(float32 a, float32 b)
{
	return a < b ? b : a;
}

int32 peMax(int32 a, int32 b)
{
	return a < b ? b : a;
}

float32 peSqrtf(float32 a)
{
	return sqrtf(a);
}

float64 peCos(float64 rad)
{
	return cos(rad);
}

float64 peSin(float64 rad)
{
	return sin(rad);
}

float64 peTan(float64 rad)
{
	return tan(rad);
}

float32 peCosf(float32 rad)
{
	return cosf(rad);
}

float32 peSinf(float32 rad)
{
	return sinf(rad);
}

float32 peTanDegf(float32 deg)
{
	return tanf(deg * PI / 180.0f);
}

float32 peTanf(float32 rad)
{
	return tanf(rad);
}

float64 peAsin(float64 val)
{
	return asin(val);
}

float64 peAtan2(float64 y, float64 x)
{
	return atan2(y, x);
}

float64 peCopysign(float64 number, float64 sign)
{
	return copysign(number, sign);
}

float32 peClamp(float32 val, float32 min, float32 max)
{
	if (val < min)
		return min;
	if (val > max)
		return max;
	return val;
}

float32 peFmod(float32 val, float32 mod)
{
	return fmod(val, mod);
}

void Vector3::getCrossMatrix(Matrix3x3* m)
{
	m->m00 = 0.0f;
	m->m01 = -z;
	m->m02 = y;
	m->m10 = z;
	m->m11 = 0.0f;
	m->m12 = -x;
	m->m20 = -y;
	m->m21 = x;
	m->m22 = 0.0f;
}

void Quaternion::getOrientationMatrix(Matrix3x3* m)
{
	float32 xx = u.x * u.x;
	float32 xy = u.x * u.y;
	float32 xz = u.x * u.z;
	float32 yy = u.y * u.y;
	float32 yz = u.y * u.z;
	float32 zz = u.z * u.z;
	float32 sx = s * u.x;
	float32 sy = s * u.y;
	float32 sz = s * u.z;

	m->m00 = 1 - 2 * (yy + zz);
	m->m01 = 2 * (xy - sz);
	m->m02 = 2 * (xz + sy);
	m->m10 = 2 * (xy + sz);
	m->m11 = 1 - 2 * (xx + zz);
	m->m12 = 2 * (yz - sx);
	m->m20 = 2 * (xz - sy);
	m->m21 = 2 * (yz + sx);
	m->m22 = 1 - 2 * (xx + yy);
}