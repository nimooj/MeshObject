#include "pch.h"
#include "mjType.h"

#include <cassert>

#define PI 3.14159265


//////////////////////////////////////////////////////////////////////////////////
//								non-Member functions							//
//////////////////////////////////////////////////////////////////////////////////

/////////////// Addition
mjPos2 operator +(const mjPos2 &p, const mjPos2 &q) {
	return mjPos2(p.x + q.x, p.y + q.y);
}

mjVec2 operator +(const mjPos2 &p, const mjVec2 &q) {
	return mjVec2(p.x + q.x, p.y + q.y);
}


mjPos3 operator +(const mjPos3 &p, const mjPos3 &q) {
	return mjPos3(p.x + q.x, p.y + q.y, p.z + q.z);
}

mjVec3 operator +(const mjPos3 &p, const mjVec3 &q) {
	return mjVec3(p.x + q.x, p.y + q.y, p.z + q.z);
}


mjVec2 operator +(const mjVec2 &p, const mjVec2 &q) {
	return mjVec2(p.x + q.x, p.y + q.y);
}


mjVec3 operator +(const mjVec3 &p, const mjVec3 &q) {
	return mjVec3(p.x + q.x, p.y + q.y, p.z + q.z);
}


////////////// Subtraction
mjVec2 operator -(const mjPos2 &p, const mjVec2 &q) {
	return mjVec2(q.x - p.x, q.y - p.y);
}


mjVec2 operator -(const mjVec2 &p, const mjVec2 &q) {
	return mjVec2(q.x - p.x, q.y - p.y);
}

mjVec2 operator -(const mjVec2 &p, const mjPos2 &q) {
	return mjVec2(q.x - p.x, q.y - p.y);
}


mjVec3 operator -(const mjVec3 &p, const mjVec3 &q) {
	return mjVec3(q.x - p.x, q.y - p.y, q.z - p.z);
}

mjVec3 operator -(const mjVec3 &p, const mjPos3 &q) {
	return mjVec3(q.x - p.x, q.y - p.y, q.z - p.z);
}

mjVec3 operator -(const mjPos3 &p, const mjPos3 &q) {
	return mjVec3(q.x - p.x, q.y - p.y, q.z - p.z);
}


///////////// Multiplication

mjPos3 operator *(float s, const mjPos3 &p) {
	return mjPos3(s * p.x, s * p.y, s * p.z);
}

mjVec2 operator *(const float s, const mjVec2 &q) {
	return mjVec2(s * q.x, s * q.y);
}

mjVec2 operator *(const mjVec2 &p, const mjVec2 &q) {
	return mjVec2(p.x * q.x, p.y * q.y);
}


mjVec3 operator *(const float s, const mjVec3 &q) {
	return mjVec3(s * q.x, s * q.y, s * q.z);
}

mjVec3 operator *(const mjVec3 &p, const mjVec3 &q) {
	return mjVec3(p.x * q.x, p.y * q.y, p.z * q.z);
}


mjPos3 operator *(const mjQuaternion &q, const mjPos3 &v) {
	mjPos3 u;
	// 0 : w, 
	// 1 : x, 
	// 2 : y, 
	// 3 : z
	u.x = (pow(q.x, 2) + pow(q.w, 2) - pow(q.y, 2) - pow(q.z, 2)) * v.x + (2 * q.x * q.y - 2 * q.w * q.z) * v.y + (2 * q.x * q.z + 2 * q.w * q.y) * v.z;
	u.y = (2 * q.x * q.y + 2 * q.w * q.z) * v.x + (pow(q.y, 2) + pow(q.w, 2) - pow(q.x, 2) - pow(q.z, 2)) * v.y + (2 * q.y * q.z - 2 * q.w * q.x) * v.z;
	u.z = (2 * q.x * q.z - 2 * q.w * q.y) * v.x + (2 * q.y * q.z + 2 * q.w * q.x) * v.y + (pow(q.w, 2) + pow(q.z, 2) - pow(q.x, 2) - pow(q.y, 2)) * v.z;

	return u;
}



///////////// Division
mjPos3 operator /(const mjPos3 &p, float s) {
	if (s == 0) {
		assert("Division by 0!");
		return p;
	}

	return mjPos3(p.x / s, p.y / s, p.z / s);
}

mjVec2 operator /(const mjVec2 &p, const float d) {
	if (d == 0) {
		assert("Division by 0!");
		return mjVec2();
	}
	return mjVec2(p.x / d, p.y / d);
}

mjVec3 operator /(const mjVec3 &p, const float d) {
	if (d == 0) {
		assert("Division by 0!");
		return mjVec3();
	}
	return mjVec3(p.x / d, p.y / d, p.z / d);
}



///////////// Cross product
mjVec3 operator ^(const mjVec3 &p, const mjVec3 &q) {
	return mjVec3(
		p.y * q.z - p.z * q.y,
		p.z * q.x - p.x * q.z,
		p.x * q.y - p.y * q.x
	);
}


///////////// Boolean
bool operator == (const mjPos2 &p, const mjPos2 &q) {
	if (p.x == q.x && p.y == q.y)
		return true;
	else 
		return false;
}

bool operator != (const mjPos2 &p, const mjPos2 &q) {
	return (p == q) ? false : true;
}



bool operator == (const mjPos3 &p, const mjPos3 &q) {
	if (p.x == q.x && p.y == q.y && p.z == q.z)
		return true;
	else 
		return false;
}

bool operator != (const mjPos3 &p, const mjPos3 &q) {
	return (p == q) ? false : true;
}


//////////////////////////////////////////////////////////////////////////////////
//										mjPos2									// 
//////////////////////////////////////////////////////////////////////////////////
mjPos2::mjPos2(float _x, float _y) {
	x = _x;
	y = _y;
}

mjPos2::mjPos2(const mjPos2 &cpy) {
	x = cpy.x;
	y = cpy.y;
}

mjPos2::~mjPos2() {

}

mjPos2& mjPos2::operator= (const mjPos2 &rhs) {
	x = rhs.x;
	y = rhs.y;
	return *this;
}

mjPos2& mjPos2::operator+= (const mjPos2 &rhs) {
	x += rhs.x;
	y += rhs.y;
	return *this;
}

mjPos2& mjPos2::operator-= (const mjPos2 &rhs) {
	x -= rhs.x;
	y -= rhs.y;
	return *this;
}

//////////////////////////////////////////////////////////////////////////////////
//										mjPos3									// 
//////////////////////////////////////////////////////////////////////////////////

mjPos3::mjPos3(float _x, float _y, float _z) {
	x = _x;
	y = _y;
	z = _z;
}

mjPos3::mjPos3(mjVec3 v) {
	x = v.x;
	y = v.y;
	z = v.z;
}

mjPos3::mjPos3(const mjPos3 &cpy) {
	x = cpy.x;
	y = cpy.y;
	z = cpy.z;
}

mjPos3::~mjPos3() {
	
}


mjPos3& mjPos3::operator =(const mjPos3 &rhs) {
	x = rhs.x;
	y = rhs.y;
	z = rhs.z;
	return *this;
}

mjPos3& mjPos3::operator +=(const mjPos3 &rhs) {
	x += rhs.x;
	y += rhs.y;
	z += rhs.z;
	return *this;
}

mjPos3& mjPos3::operator -=(const mjPos3 &rhs) {
	x -= rhs.x;
	y -= rhs.y;
	z -= rhs.z;
	return *this;
}



//////////////////////////////////////////////////////////////////////////////////
//										mjVec2									// 
//////////////////////////////////////////////////////////////////////////////////
mjVec2::mjVec2(float _x, float _y) {
	x = _x;
	y = _y;

	m_Pos = new mjPos2(x, y);
}

mjVec2::mjVec2(const mjVec2 &cpy) {
	x = cpy.x;
	y = cpy.y;

	m_Pos = cpy.m_Pos;
}

mjVec2::~mjVec2() {

}

mjVec2& mjVec2::set(float _x, float _y) {
	x = _x;
	y = _y;
	m_Pos = new mjPos2(x, y);
	return *this;
}

mjVec2& mjVec2::normalize() {
	float s = length();
	x /= s;
	y /= s;
	m_Pos->x /= s;
	m_Pos->y /= s;
	return *this;
}

float mjVec2::length() {
	return sqrt(pow(x, 2) + pow(y, 2));
}

mjVec2& mjVec2::operator =(const mjVec2 &rhs) {
	x = rhs.x;
	y = rhs.y;
	m_Pos->x = x;
	m_Pos->y = y;
	return *this;
}

mjVec2& mjVec2::operator +=(const mjVec2 &rhs) {
	x += rhs.x;
	y += rhs.y;
	m_Pos->x = x;
	m_Pos->y = y;
	return *this;
}

mjVec2& mjVec2::operator -=(const mjVec2 &rhs) {
	x -= rhs.x;
	y -= rhs.y;
	m_Pos->x = x;
	m_Pos->y = y;
	return *this;
}

mjVec2& mjVec2::operator *=(const float &rhs) {
	x *= rhs;
	y *= rhs;
	m_Pos->x = x;
	m_Pos->y = y;
	return *this;
}

mjVec2& mjVec2::operator /=(const float &rhs) {
	if (rhs == 0) {
		assert("Division by 0!");
		return *this;
	}
	x /= rhs;
	y /= rhs;
	m_Pos->x = x;
	m_Pos->y = y;
	return *this;
}


//////////////////////////////////////////////////////////////////////////////////
//										mjVec3									// 
//////////////////////////////////////////////////////////////////////////////////

mjVec3::mjVec3(float _x, float _y, float _z) {
	x = _x;
	y = _y;
	z = _z;
	m_Pos = new mjPos3(x, y, z);
}

mjVec3::mjVec3(mjPos3 *start, mjPos3 *end) {
	x = end->x - start->x;
	y = end->y - start->y;
	z = end->z - start->z;
	m_Pos = new mjPos3(x, y, z);
}

mjVec3::mjVec3(mjPos3 p) {
	x = p.x;
	y = p.y;
	z = p.z;

	m_Pos = new mjPos3(p);
}

mjVec3::mjVec3(const mjVec3 &cpy) {
	x = cpy.x;
	y = cpy.y;
	z = cpy.z;
	m_Pos = cpy.m_Pos;
}

mjVec3::~mjVec3() {

}

mjVec3& mjVec3::set(float _x, float _y, float _z) {
	x = _x;
	y = _y;
	z = _z;
	m_Pos->x = x;
	m_Pos->y = y;
	m_Pos->z = z;
	return *this;
}

mjVec3& mjVec3::normalize() {
	float s = length();
	x /= s;
	y /= s;
	z /= s;
	m_Pos->x = x;
	m_Pos->y = y;
	m_Pos->z = z;
	return *this;
}

float mjVec3::length() {
	return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
}

float mjVec3::sum() {
	return (x + y + z);
}

mjVec3& mjVec3::operator =(const mjVec3 &rhs) {
	x = rhs.x;
	y = rhs.y;
	z = rhs.z;
	m_Pos->x = x;
	m_Pos->y = y;
	m_Pos->z = z;
	return *this;
}

mjVec3& mjVec3::operator +=(const mjVec3 &rhs) {
	x += rhs.x;
	y += rhs.y;
	z += rhs.z;
	m_Pos->x = x;
	m_Pos->y = y;
	m_Pos->z = z;
	return *this;
}

mjVec3& mjVec3::operator -=(const mjVec3 &rhs) {
	x -= rhs.x;
	y -= rhs.y;
	z -= rhs.z;
	m_Pos->x = x;
	m_Pos->y = y;
	m_Pos->z = z;
	return *this;
}

mjVec3& mjVec3::operator *=(const float &rhs) {
	x *= rhs;
	y *= rhs;
	z *= rhs;
	m_Pos->x = x;
	m_Pos->y = y;
	m_Pos->z = z;
	return *this;
}

mjVec3& mjVec3::operator /=(const float &rhs) {
	if (rhs == 0) {
		assert("Division by 0!");
		return *this;
	}
	x /= rhs;
	y /= rhs;
	z /= rhs;
	m_Pos->x = x;
	m_Pos->y = y;
	m_Pos->z = z;
	return *this;
}

//////////////////////////////////////////////////////////////////////////////////
//	 								 mjLine3									// 
//////////////////////////////////////////////////////////////////////////////////
mjLine::mjLine() {
	m_Dir = mjVec3(0, 1, 0);
	m_Pos = mjPos3();
}

mjLine::mjLine(mjVec3 dir, mjPos3 pos) {
	m_Dir = dir;
	m_Pos = pos;
}

mjLine::mjLine(mjPos3 p, mjPos3 q) {
	m_Dir = q - p;
	m_Pos = p;
}

mjLine::mjLine(const mjLine &cpy) {
	m_Dir = cpy.m_Dir;
	m_Pos = cpy.m_Pos;
}


mjLine::~mjLine() {

}


//////////////////////////////////////////////////////////////////////////////////
//	 								 mjPlane									// 
//////////////////////////////////////////////////////////////////////////////////
mjPlane::mjPlane() {
	m_Pos = mjPos3();
	m_Normal = mjVec3(0, 1, 0);
	d = 0.0;
}

mjPlane::mjPlane(mjPos3 p, mjPos3 q, mjPos3 r) {
	m_Pos = p;
	m_Normal = (q - p) ^ (r - p);
	d = -(m_Normal.x * m_Pos.x + m_Normal.y * m_Pos.y + m_Normal.z * m_Pos.z);
}

mjPlane::mjPlane(const mjPlane& pln) {
	m_Pos = pln.m_Pos;
	m_Normal = pln.m_Normal;
	d = pln.d;
}

mjPlane::~mjPlane() {

}

bool mjPlane::IsAbove(mjPos3 p) {
	// Normal vector客 m_Pos~p客狼 阿 < 90
	mjVec3 v = p - m_Pos;
	mjVec3 dot = m_Normal * v;

	float angle = acos((dot.x + dot.y + dot.z) / (m_Normal.length() * v.length()));
	// Conversion to degree
	angle = angle * 180.0 / PI;

	if (angle > 90)
		return false;

	return true;
}

bool mjPlane::IsBelow(mjPos3 p) {
	// Normal vector客 m_Pos~p客狼 阿 < 90
	mjVec3 v = p - m_Pos;
	mjVec3 dot = m_Normal * v;

	float angle = acos((dot.x + dot.y + dot.z) / (m_Normal.length() * v.length()));
	// Conversion to degree
	angle = angle * 180.0 / PI;

	if (angle > 90)
		return false;

	return true;
}

//////////////////////////////////////////////////////////////////////////////////
//	 								mjMatrix4x4									// 
//////////////////////////////////////////////////////////////////////////////////
mjMatrix4x4::mjMatrix4x4() {
	for (int i = 0; i < 16; i++)
		value[i] = 0;
}

mjMatrix4x4::mjMatrix4x4(float *v) {
	for (int i = 0; i < 16; i++) {
		value[i] = v[i];
	}
}

mjMatrix4x4::~mjMatrix4x4() {

}

mjQuaternion* mjMatrix4x4::MatrixToQuaternion() {
	double tr, s;
	int i, j, k;
	int nxt[3] = { 1, 2, 0 };

	float q[4];

	float w, x, y, z;

	tr = value[0] + value[5] + value[10];
	if (tr > 0.0) {
		s = sqrt(tr + 1.0);
		w = s * 0.5;
		s = 0.5 / s;
		x = (value[9] - value[6]) * s;
		y = (value[2] - value[8]) * s;
		z = (value[4] - value[1]) * s;
	}
	else {
		i = 0;
		if (value[5] > value[0]) {
			i = 1;
		}
		if (value[10] > value[i * 4 + i]) {
			i = 2;
		}
		j = nxt[i];
		k = nxt[j];
		s = sqrt((value[i * 4 + i] - (value[j * 4 + j] + value[k * 4 + k])) + 1.0);
		q[i] = s * 0.5;
		s = 0.5 / s;
		w = (value[k * 4 + j] - value[j * 4 + k]) * s;
		q[j] = (value[j * 4 + i] + value[i * 4 + j]) * s;
		q[k] = (value[k * 4 + i] + value[i * 4 + k]) * s;
		x = q[0];
		y = q[1];
		z = q[2];
	}

	return new mjQuaternion(w, x, y, z);
}


//////////////////////////////////////////////////////////////////////////////////
//	 								mjQuaternion								// 
//////////////////////////////////////////////////////////////////////////////////

mjQuaternion::mjQuaternion(float _w, float _x, float _y, float _z) {
	w = _w;
	x = _x;
	y = _y;
	z = _z;

	norm = this->Length();
}

mjQuaternion::mjQuaternion(const mjQuaternion &cpy) {
	w = cpy.w;
	x = cpy.x;
	y = cpy.y;
	z = cpy.z;

	norm = cpy.norm;
}

mjQuaternion::~mjQuaternion() {

}

float mjQuaternion::Length() {
	return sqrt(pow(w, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2));
}

mjQuaternion* mjQuaternion::Normalize() {
	if (norm != 0) {
		w /= norm;
		x /= norm;
		y /= norm;
		z /= norm;
	}
	else {
		assert("Division by 0!\n");
	}

	return this;
}


mjQuaternion* mjQuaternion::Conjugate() {
	return new mjQuaternion(w, -x, -y, -z);
}


mjPos3* mjQuaternion::Rotate(mjPos3 *v) {
	float nx, ny, nz;

	nx = (1 - 2 * pow(y, 2) - 2 * pow(z, 2)) * v->x
		+ (2 * x * y - 2 * w * z) * v->y
		+ (2 * x * z + 2 * w * y) * v->z;

	ny = (2 * x * y + 2 * w * z) * v->x
		+ (1 - 2 * pow(x, 2) - 2 * pow(z, 2)) * v->y
		+ (2 * y * z - 2 * w * x) * v->z;

	nz = (2 * x * z - 2 * w * y) * v->x
		+ (2 * y * z + 2 * w * x) * v->y
		+ (1 - 2 * pow(x, 2) - 2 * pow(y, 2)) * v->z;

	v->x = nx;
	v->y = ny;
	v->z = nz;

	return v;
}
