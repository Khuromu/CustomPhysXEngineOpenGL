#ifndef _MATHS_H_
#define _MATHS_H_

#define _USE_MATH_DEFINES
#include <math.h>


#define RAD2DEG(x) ((x)*(180.0f/(float)M_PI))
#define DEG2RAD(x) ((x)*((float)M_PI/180.0f))

template<typename T>
T Select(bool condition, T a, T b)
{
	return ((T)condition) * a + (1 - ((T)condition)) * b;
}

template<typename T>
T Min(T a, T b)
{
	return Select(a < b, a, b);
}

template<typename T>
T Max(T a, T b)
{
	return Select(a > b, a, b);
}

template<typename T>
T Clamp(T val, T min, T max)
{
	return Min(Max(val, min), max);
}

float Sign(float a);

float Random(float from, float to);

struct Vec2
{
	float x, y;

	Vec2() : x(0.0f), y(0.0f){}

	Vec2(float _x, float _y) : x(_x), y(_y){}

	Vec2 operator+(const Vec2& rhs) const
	{
		return Vec2(x + rhs.x, y + rhs.y);
	}

	Vec2& operator+=(const Vec2& rhs)
	{
		x += rhs.x;
		y += rhs.y;
		return *this;
	}

	Vec2 operator-(const Vec2& rhs) const
	{
		return Vec2(x - rhs.x, y - rhs.y);
	}

	Vec2& operator-=(const Vec2& rhs)
	{
		x -= rhs.x;
		y -= rhs.y;
		return *this;
	}

	Vec2 operator*(float factor) const
	{
		return Vec2(x * factor, y * factor);
	}

	Vec2& operator*=(float factor)
	{
		*this = Vec2(x * factor, y * factor);
		return *this;
	}

	Vec2 operator/(float factor) const
	{
		return Vec2(x / factor, y / factor);
	}

	Vec2& operator/=(float factor)
	{
		*this = Vec2(x / factor, y / factor);
		return *this;
	}

	bool operator!=(Vec2& other)
	{
		return x != other.x || y != other.y;
	}

	float operator|(const Vec2& rhs) const
	{
		return x * rhs.x + y * rhs.y;
	}

	float operator^(const Vec2& rhs) const
	{
		return x * rhs.y - y * rhs.x;
	}

	bool operator==(const Vec2& other) const
	{
		return x == other.x && y == other.y;
	}

	static Vec2 Zero()
	{
		Vec2 temp = Vec2(0, 0);
		return temp;
	}

	float GetLength() const
	{
		return sqrtf(x*x + y*y);
	}

	float GetSqrLength() const
	{
		return x*x + y*y;
	}

	void	Normalize()
	{
		float length = GetLength();
		x /= length;
		y /= length;
	}

	Vec2	Normalized() const
	{
		Vec2 res = *this;
		res.Normalize();
		return res;
	}

	void Reflect(Vec2 normal, float elasticity = 1.0f)
	{
		*this = *this - normal * (1.0f + elasticity) * (*this | normal);
	}

	Vec2 GetNormal() const
	{
		return Vec2(-y, x);
	}

	float Angle(const Vec2& to)
	{
		float cosAngle = Clamp(Normalized() | to.Normalized(), -1.0f, 1.0f);
		float angle = RAD2DEG(acosf(cosAngle)) * Sign(*this ^ to);
		return angle;
	}

	static float ComputeArea(Vec2& firstPnt, Vec2& secondPoint, Vec2& thirdPnt)
	{
		return ((secondPoint - firstPnt) ^ (thirdPnt - firstPnt)) / 2;
	}
};


struct Mat2
{
	Vec2 X, Y;

	Mat2() : X(1.0f, 0.0f), Y(0.0f, 1.0f){}

	Mat2(float a, float b, float c, float d) : X(a, c), Y(b, d){}

	Mat2	GetInverse() const
	{
		return Mat2(X.x, X.y, Y.x, Y.y);
	}

	float	GetAngle() const
	{
		return Vec2(1.0f, 0.0f).Angle(X);
	}

	void	SetAngle(float angle)
	{
		float c = cosf(angle * ((float)M_PI / 180.0f));
		float s = sinf(angle * ((float)M_PI / 180.0f));

		X.x = c; X.y = s;
		Y.x = -s; Y.y = c;
	}

	void Rotate(float angle)
	{
		Mat2 matRot;
		matRot.SetAngle(angle);
		*this = *this * matRot;
	}

	Mat2 operator*(const Mat2& rhs) const
	{
		return Mat2(X.x*rhs.X.x + Y.x*rhs.X.y, X.x*rhs.Y.x + Y.x*rhs.Y.y, X.y*rhs.X.x + Y.y*rhs.X.y, X.y*rhs.Y.x + Y.y*rhs.Y.y);
	}

	Vec2 operator*(const Vec2& vec) const
	{
		return Vec2(X.x*vec.x + Y.x*vec.y, X.y*vec.x + Y.y*vec.y);
	}
};

struct Line
{
	Vec2 point, dir;

	Line() = default;
	Line(Vec2 _point, Vec2 _dir) : point(_point), dir(_dir){}

	Vec2	GetNormal() const
	{
		return dir.GetNormal();
	}

	// positive value means point above line, negative means point is under line
	float	GetPointDist(const Vec2& pt) const
	{
		return (pt - point) | GetNormal();
	}

	Line	Transform(const Mat2& rotation, const Vec2& position) const
	{
		return Line(position + rotation * point, rotation * dir);
	}

	Vec2	Project(const Vec2& pt) const
	{
		return point + dir * ((pt - point) | dir);
	}
};

struct Simplex
{
	Vec2 vertexArray[3];
	int count;

	Vec2 ClosestPoint(Vec2& firstPoint, Vec2& secondPoint, Vec2& otherPoint)
	{
		Vec2 closestPoint = Vec2(0, 0);
		Vec2 n = (secondPoint - firstPoint).Normalized();

		float u = ((otherPoint - firstPoint) | n / (secondPoint - firstPoint).GetLength());
		float v = ((secondPoint - otherPoint) | n / (secondPoint - firstPoint).GetLength());

		if (u <= 0)
			return closestPoint = firstPoint;

		else if (v <= 0)
			return closestPoint = secondPoint;

		else
			return closestPoint = (firstPoint * v) + (secondPoint * u);
	}

	Vec2 ClosestPoint(Vec2& firstPnt, Vec2& secondPnt, Vec2& thirdPnt, Vec2& externalPnt)
	{
		float segmentLengthAB = (secondPnt - firstPnt).GetLength();
		float segmentLengthBC = (thirdPnt - secondPnt).GetLength();
		float segmentLengthCA = (firstPnt - thirdPnt).GetLength();
		Vec2 otherAB = (secondPnt - firstPnt) / segmentLengthAB;
		Vec2 otherBC = (thirdPnt - secondPnt) / segmentLengthBC;
		Vec2 otherCA = (firstPnt - thirdPnt) / segmentLengthCA;

		float uAB = (secondPnt - externalPnt) | otherAB / segmentLengthAB;
		float vAB = (externalPnt - firstPnt) | otherAB / segmentLengthAB;

		float uBC = (thirdPnt - externalPnt) | otherBC / segmentLengthBC;
		float vBC = (externalPnt - secondPnt) | otherBC / segmentLengthBC;

		float uCA = (firstPnt - externalPnt) | otherCA / segmentLengthCA;
		float vCA = (externalPnt - thirdPnt) | otherCA / segmentLengthCA;

		float uABC = Vec2::ComputeArea(externalPnt, secondPnt, thirdPnt) / Vec2::ComputeArea(firstPnt, secondPnt, thirdPnt);
		float vABC = Vec2::ComputeArea(externalPnt, thirdPnt, firstPnt) / Vec2::ComputeArea(firstPnt, secondPnt, thirdPnt);
		float wABC = Vec2::ComputeArea(externalPnt, firstPnt, secondPnt) / Vec2::ComputeArea(firstPnt, secondPnt, thirdPnt);

		Vec2 p;
		// Vertex
		if (uCA <= 0 && vAB <= 0)
			return p = firstPnt;

		else if (uAB <= 0 && vBC <= 0)
			return p = secondPnt;

		else if (uBC <= 0 && vCA <= 0)
			return p = thirdPnt;

		// Segments
		else if (uAB > 0 && vAB > 0 && wABC <= 0)
			return p = firstPnt * uAB + secondPnt * vAB;

		else if (uBC > 0 && vBC > 0 && uABC <= 0)
			return p = firstPnt * uBC + secondPnt * vBC;

		else if (uCA > 0 && vCA > 0 && vABC <= 0)
			return p = firstPnt * uCA + secondPnt * vCA;

		// Inside the Triangle
		else
			return p = externalPnt;
	}

	Vec2 GetClosestPoint(Vec2 point)
	{
		switch (count)
		{
			case 0:
				return Vec2::Zero();

			case 1:
				return vertexArray[0];

			case 2:
				return ClosestPoint(vertexArray[0], vertexArray[1], point);

			case 3:
				return ClosestPoint(vertexArray[0], vertexArray[1], vertexArray[2], point);
		}
	}

	void Culling(Vec2 point)
	{
		Vec2 n = (vertexArray[1] - vertexArray[0]).Normalized();
		switch (count)
		{
			case 0 || 1:
				return;

			case 2:
			{
				float simplexLength = (vertexArray[1] - vertexArray[0]).GetLength();

				float weightA = ((point - vertexArray[0]) | n / simplexLength);
				float weightB = ((vertexArray[1] - point) | n / simplexLength);

				if (weightA <= 0)
					RemovePoint(vertexArray[0]);
				else if (weightB <= 0)
					RemovePoint(vertexArray[1]);
				return;
			}

			case 3:
			{
				float simplexArea = Vec2::ComputeArea(vertexArray[0], vertexArray[1], vertexArray[2]);

				float weightA = Vec2::ComputeArea(point, vertexArray[1], vertexArray[2]) / simplexArea;
				float weightB = Vec2::ComputeArea(point, vertexArray[2], vertexArray[0]) / simplexArea;
				float weightC = Vec2::ComputeArea(point, vertexArray[0], vertexArray[1]) / simplexArea;

				if (weightA <= 0)
					RemovePoint(vertexArray[0]);
				if (weightB <= 0)
					RemovePoint(vertexArray[1]);
				if (weightC <= 0)
					RemovePoint(vertexArray[2]);
				return;
			}
		}
	}

	bool ComparePoints(Vec2 projectedPoint, Vec2 originPoint)
	{
		if (projectedPoint == originPoint)
			return true;

		return false;
	}

	void AddPoint(Vec2 point)
	{
		if (count < 3)
		{
			vertexArray[count] = point;
			count++;
		}
	}

	void RemovePoint(Vec2 point)
	{
		Vec2 tempVertexArray[3]{ Vec2::Zero(), Vec2::Zero(), Vec2::Zero() };
		int tempIndex = 0;

		for (int i = 0; i < count; i++)
		{
			if (vertexArray[i] != point)
			{
				tempVertexArray[tempIndex] = vertexArray[i];
				tempIndex++;
			}

			else
				count--;
		}
		
		for (int i = 0; i < 3; i++)
			vertexArray[i] = tempVertexArray[i];
	}
};

#endif