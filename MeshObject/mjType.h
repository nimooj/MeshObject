#pragma once


#include <iostream>
#include <vector>


// Using glm types
#include <glm/glm.hpp>

#define EPSILON 1.192092896e-7
#define INFINITY ((float)(_HUGE_ENUF * _HUGE_ENUF))

#define ABS(x) ((x < 0.0) ? -(x) : x)
#define DEGREE2RADIAN(x) (x * M_PI / 180.0)
#define RADIAN2DEGREE(x) (x * 180.0 / M_PI)
#define EQUAL(x, y) (ABS((x) - (y)) < EPSILON)

// Class names
class mjPos2;
class mjPos3;

class mjVec2;
class mjVec3;

class mjLine;
class mjPlane;

class mjMatrix;
class mjQuaternion;



// Class declaration
class mjPos2 {
public:
	//////////////////// Member Variables
	float x, y;

public:
	//////////////////// Constructor & Deconstructor
	mjPos2(float _x = 0.0f, float _y = 0.0f);
	mjPos2(const mjPos2 &cpy);
	~mjPos2();

	//////////////////// Member Functions
	mjPos2 &operator =(const mjPos2 &rhs);
	mjPos2 &operator +=(const mjPos2 &rhs);
	mjPos2 &operator -=(const mjPos2 &rhs);

	// Addition
	friend mjPos2 operator +(const mjPos2 &p, const mjPos2 &q);
	friend mjVec2 operator +(const mjPos2 &p, const mjVec2 &q);
	// Subtration
	friend mjPos2 operator -(const mjPos2 &p, const mjPos2 &q);
	friend mjVec2 operator -(const mjPos2 &p, const mjVec2 &q);
	// Boolean
	friend bool operator == (const mjPos2 &p, const mjPos2 &q);
	friend bool operator != (const mjPos2 &p, const mjPos2 &q);
};


class mjPos3 {
public:
	//////////////////// Member Variables
	float x, y, z;
public:
	//////////////////// Constructor & Deconstructor
	mjPos3(float _x = 0.0f, float _y = 0.0f, float _z = 0.0f);
	mjPos3(mjVec3 v);
	mjPos3(const mjPos3 &cpy);
	~mjPos3();


	//////////////////// Member Functions
	mjPos3 &operator =(const mjPos3 &rhs);
	mjPos3 &operator +=(const mjPos3 &rhs);
	mjPos3 &operator -=(const mjPos3 &rhs);


	// Addition
	friend mjPos3 operator +(const mjPos3 &p, const mjPos3 &q);
	friend mjPos3 operator *(float s, const mjPos3 &p);
	friend mjPos3 operator /(const mjPos3 &p, float s);
	friend mjVec3 operator +(const mjPos3 &p, const mjVec3 &q);

	friend mjVec3 operator -(const mjPos3 &p, const mjPos3 &q);

	// Boolean
	friend bool operator == (const mjPos3 &p, const mjPos3 &q);
	friend bool operator != (const mjPos3 &p, const mjPos3 &q);
};


class mjVec2 {
public:
	//////////////////// Member Variables
	float x, y;
	mjPos2 *m_Pos;


public:
	//////////////////// Constructor & Deconstructor
	mjVec2(float _x = 0.0f, float _y = 0.0f);
	mjVec2(const mjVec2 &cpy);
	~mjVec2();


	//////////////////// Member Functions
	mjVec2 &set(float _x, float _y);
	mjVec2 &normalize();
	float length();


	//////////////////// Operator overload
	mjVec2 &operator =(const mjVec2 &rhs);
	mjVec2 &operator +=(const mjVec2 &rhs);
	mjVec2 &operator -=(const mjVec2 &rhs);
	mjVec2 &operator *=(const float &rhs);
	mjVec2 &operator /=(const float &rhs);


	friend mjVec2 operator +(const mjVec2 &p, const mjVec2 &q);
	friend mjVec2 operator -(const mjVec2 &p, const mjVec2 &q);
	friend mjVec2 operator -(const mjVec2 &p, const mjPos2 &q);
	friend mjVec2 operator *(const float s, const mjVec2 &q);
	friend mjVec2 operator *(const mjVec2 &p, const mjVec2 &q); // Dot product
	friend mjVec2 operator /(const mjVec2 &p, const float d);
};


class mjVec3 {
public:
	//////////////////// Member Variables
	float x, y, z;
	mjPos3 *m_Pos;

public:
	//////////////////// Constructor & Deconstructor
	mjVec3(float _x = 0.0f, float _y = 0.0f, float _z = 0.0f);
	mjVec3(mjPos3 *start, mjPos3 *end);
	mjVec3(mjPos3 p);
	mjVec3(const mjVec3 &cpy);
	~mjVec3();


	//////////////////// Member Functions
	mjVec3 &set(float _x, float _y, float _z);
	mjVec3 &normalize();
	float length();
	float sum();


	//////////////////// Operator overload
	mjVec3 &operator =(const mjVec3 &rhs);
	mjVec3 &operator +=(const mjVec3 &rhs);
	mjVec3 &operator -=(const mjVec3 &rhs);
	mjVec3 &operator *=(const float &rhs);
	mjVec3 &operator /=(const float &rhs);

	friend mjVec3 operator +(const mjVec3 &p, const mjVec3 &q);
	friend mjVec3 operator -(const mjVec3 &p, const mjVec3 &q);
	friend mjVec3 operator -(const mjVec3 &p, const mjPos3 &q);
	friend mjVec3 operator -(const mjPos3 &p, const mjPos3 &q);
	friend mjVec3 operator *(const float s, const mjVec3 &q);
	friend mjVec3 operator *(const mjVec3 &p, const mjVec3 &q); // Dot product
	friend mjVec3 operator /(const mjVec3 &p, const float d);
	friend mjVec3 operator ^(const mjVec3 &p, const mjVec3 &q);
};


class mjLine {
public:
	mjLine();
	mjLine(mjVec3 dir, mjPos3 pos);
	mjLine(mjPos3 p, mjPos3 q);
	mjLine(const mjLine &cpy);
	~mjLine();
	
public:
	// 직선의 방향
	mjVec3 m_Dir;

	// 직선 위 임의의 한 점
	mjPos3 m_Pos; 
};


class mjPlane {
public: 
	mjPos3 m_Pos;
	mjVec3 m_Normal; // Not an unit vector

	float d;

public:
	mjPlane();
	mjPlane(mjPos3 p, mjPos3 q, mjPos3 r);
	mjPlane(const mjPlane& pln);
	~mjPlane();

	bool IsAbove(mjPos3);
	bool IsBelow(mjPos3);
};


// 4 x 4 matrix
class mjMatrix4x4 {
public:
	//////////////////// Member Variables
	float value[16];
	
public:
	//////////////////// Constructor & Deconstructor
	mjMatrix4x4();
	mjMatrix4x4(float *v);
	~mjMatrix4x4();

	//////////////////// Member Functions
	mjQuaternion* MatrixToQuaternion();

};


class mjQuaternion {
public:
	//////////////////// Member Variables
	// 실수부
	float w;
	// 허수부
	float x, y, z;

	float norm;

public:
	//////////////////// Constructor & Deconstructor
	mjQuaternion(float _w = 0.0, float _x = 0.0, float _y = 0.0, float _z = 0.0);
	mjQuaternion(const mjQuaternion &cpy);
	~mjQuaternion();

	//////////////////// Member Functions
	float Length();
	mjQuaternion* Normalize();
	mjQuaternion* Conjugate();
	mjPos3* Rotate(mjPos3 *v);

	friend mjPos3 operator *(const mjQuaternion &q, const mjPos3 &v); // Dot product
};


//////////////////////////////////////////////////////

