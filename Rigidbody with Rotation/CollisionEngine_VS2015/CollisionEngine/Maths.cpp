#include "Maths.h"

#include <stdlib.h>
#include <cmath>

#include "GlobalVariables.h"
#include "Renderer.h"

float Sign(float a)
{
	return Select(a >= 0.0f, 1.0f, -1.0f);
}

float Random(float from, float to)
{
	return from + (to - from) * (((float)rand()) / ((float)RAND_MAX));
}


float ClampAngleRadians(float angle)
{
	return std::fmod(angle + (float)M_PI, (float)M_PI) - (float)M_PI;
}

float Abs(float value)
{
	if (value < 0)
		value *= -1.f;

	return value;
}

Vec2 minv(const Vec2& a, const Vec2& b)
{
	return Vec2(Min(a.x, b.x), Min(a.y, b.y));
}

Vec2 maxv(const Vec2& a, const Vec2& b)
{
	return Vec2(Max(a.x, b.x), Max(a.y, b.y));
}

bool Clip(const Vec2& center, const Vec2& normal, Vec2& pt1, Vec2& pt2)
{
	float dist1 = (pt1 - center) | normal;
	float dist2 = (pt2 - center) | normal;

	if (dist1 * dist2 >= 0.0f)
	{
		return false;
	}

	Vec2 *pIn, *pOut;
	if (dist1 >= 0.0f)
	{
		pOut = &pt1;
		pIn = &pt2;
	}
	else
	{
		pOut = &pt2;
		pIn = &pt1;
	}

	float k = ((center - *pIn) | normal) / ((*pOut - *pIn) | normal);
	*pIn += (*pOut - *pIn) * k;

	return true;
}

// 2D Analytic LCP solver (find exact solution)
bool Solve2DLCP(const Mat2& A, const Mat2& invA, const Vec2& b, Vec2& x)
{
	// - We want :
	// y = A*x + b >= 0, transpose(x) * y = 0   
	//  x = (x1 x2) are the (new) accumulated impulses for contacts 1 and 2
	//	y = (y1 y2) are the contact relative velocity (minus the target/bias)
	// So we have x1 * y1 = x2 * y2 = 0  (for each contact, y > 0 means x = 0 (why impulse would participate if speed already higher than expected ?)
	//		and x > 0 means y = 0 (if impulse participate, it does it only to the extent needed))
	// - Therefore there 4 cases : x1 = x2 = 0, y1 = y2 = 0, x1 = y2 = 0, x2 = y1 = 0
	// - We compute each case and check if A*x + b >= 0, if so the LCP is solved
	// - We aim for stability, so we want the y to be 0 as much as possible, therefore we test first y1 = y2 = 0, then y1 = 0 and then y2 = 0
	Vec2 y;

	// case 1 : y =  A * x + b = (0 0)
	// x = inv(A) * (-b)
	x = invA * (b * -1.0f);
	if (x.x >= 0.0f && x.y >= 0.0f)
	{
		// x >= 0, all conditions are satisfied, we keep this impulse
		return true;
	}

	// case 2 : y1 = 0, x2 = 0 
	// A * (x1 0) + b = (0 y2)
	// a11 * x1 + b1 = 0   &&   a21 * x1 + b2 = y2
	// x1 = -b1 / a11
	// y2 = a21 * x1 + b2
	if (A.X.x != 0.0f)
	{
		x.x = -b.x / A.X.x;
		y.y = A.X.y * x.x + b.y;
		if (x.x >= 0.0f && y.y >= 0.0f)
		{
			x.y = 0.0f;
			return true;
		}
	}

	// case 2 : y2 = 0, x1 = 0 
	// A * (0 x2) + b = (y1 0)
	// a12 * x2 + b1 = y1  &&   a22 * x2 + b2 = 0
	// x2 = -b2 / a22
	// y1 = a12 * x2 + b1
	if (A.Y.x != 0.0f)
	{
		x.y = -b.y / A.Y.y;
		y.x = A.Y.x * x.y + b.x;
		if (x.x >= 0.0f && y.y >= 0.0f)
		{
			x.x = 0.0f;
			return true;
		}
	}

	// case 3 : x = (0 0) (no need extra testing, its the last possible case...)
	x.x = x.y = 0.0f;
	y = b;
	return (y.x >= 0.0f && y.y >= 0.0f);
}


float KernelDefault(float r, float h)
{
	float h2 = h * h;
	float h4 = h2 * h2;
	float kernel = h2 - r * r;
	return (kernel * kernel * kernel) * (4.0f / (((float)M_PI) * h4 * h4));
}

float KernelSpikyGradientFactorNorm(float r, float h)
{
	float h2 = h * h;
	float h5 = h2 * h2 * h;
	float kernel = h - r;
	return kernel * kernel * (-15.0f / ((float)M_PI * h5));
}

float KernelSpikyGradientFactor(float r, float h)
{
	float h2 = h * h;
	float h5 = h2 * h2 * h;
	float kernel = h - r;
	return kernel * kernel * (-15.0f / ((float)M_PI * h5 * r));
}

float KernelViscosityLaplacian(float r, float h)
{
	float h2 = h * h;
	float kernel = h - r;
	return kernel * (30.0f / ((float)M_PI * h2 * h2 * h));
}

float KernelPoly6hGradientFactor(float r, float h)
{
	float h2 = h * h;
	float kernel = h2 - r * r;
	return kernel * kernel * (24.0f / ((float)M_PI * h2 * h2 * h2 * h * r));
}

void AABB::RenderBoundingBox()
{
	if (bIsColliding == true)
	{
		gVars->pRenderer->DrawLine(min, Vec2(max.x, min.y), 0.5f, 0.9f, 1);
		gVars->pRenderer->DrawLine(Vec2(max.x, min.y), max, 0.5f, 0.9f, 1);
		gVars->pRenderer->DrawLine(max, Vec2(min.x, max.y), 0.5f, 0.9f, 1);
		gVars->pRenderer->DrawLine(Vec2(min.x, max.y), min, 0.5f, 0.9f, 1);
	}
	
	else
	{
		gVars->pRenderer->DrawLine(min, Vec2(max.x, min.y), 0.3f, 0, 1);
		gVars->pRenderer->DrawLine(Vec2(max.x, min.y), max, 0.3f, 0, 1);
		gVars->pRenderer->DrawLine(max, Vec2(min.x, max.y), 0.3f, 0, 1);
		gVars->pRenderer->DrawLine(Vec2(min.x, max.y), min, 0.3f, 0, 1);
	}
}

void Simplex::Draw()
{
	if (count == 3)
	{
		gVars->pRenderer->DrawLine(vertexArray[0], vertexArray[1], 0.4f, 0.1f, 0.7f);
		gVars->pRenderer->DrawLine(vertexArray[1], vertexArray[2], 0.4f, 0.1f, 0.7f);
		gVars->pRenderer->DrawLine(vertexArray[2], vertexArray[0], 0.4f, 0.1f, 0.7f);
	}
}

Vec2 Simplex::ClosestPoint(Vec2& firstPnt, Vec2& secondPnt, Vec2& thirdPnt, Vec2& externalPnt)
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

	float pointsArea = Vec2::ComputeArea(firstPnt, secondPnt, thirdPnt);
	float tempPointsArea = Abs(pointsArea);

	float a1 = Vec2::ComputeArea(externalPnt, secondPnt, thirdPnt); // pointsArea;
	float a2 = Vec2::ComputeArea(externalPnt, thirdPnt, firstPnt);  // pointsArea;
	float a3 = Vec2::ComputeArea(externalPnt, firstPnt, secondPnt); // pointsArea;

	float uABC = a1 / pointsArea;
	float vABC = a2 / pointsArea;
	float wABC = a3 / pointsArea;

	float tempA1 = Abs(a1);
	float tempA2 = Abs(a2);
	float tempA3 = Abs(a3);

	float tempTest = tempA1 + tempA2 + tempA3;

	Vec2 p;

	// Vertex
	if (uCA <= 0 && vAB <= 0)
		return p = firstPnt;

	if (uAB <= 0 && vBC <= 0)
		return p = secondPnt;

	if (uBC <= 0 && vCA <= 0)
		return p = thirdPnt;

	// Segments
	if (uAB > 0 && vAB > 0 && wABC <= 0)
		return p = (firstPnt * uAB) + (secondPnt * vAB);

	if (uBC > 0 && vBC > 0 && uABC <= 0)
		return p = (firstPnt * uBC) + (secondPnt * vBC);

	if (uCA > 0 && vCA > 0 && vABC <= 0)
		return p = (firstPnt * uCA) + (secondPnt * vCA);

	// Inside the Triangle
	if (uABC > 0.f && vABC > 0.f && wABC > 0.f)
	{
		gVars->pRenderer->DrawLine(firstPnt, secondPnt, 0.5f, 0.8f, 0.5f);
		gVars->pRenderer->DrawLine(secondPnt, thirdPnt, 0.5f, 0.8f, 0.5f);
		gVars->pRenderer->DrawLine(thirdPnt, firstPnt, 0.5f, 0.8f, 0.5f);

		return p = externalPnt;
	}

	else
		return Vec2(1.f, 1.f);
}