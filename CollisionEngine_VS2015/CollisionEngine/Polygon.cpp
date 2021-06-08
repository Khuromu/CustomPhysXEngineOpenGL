#include "Polygon.h"
#include <GL/glu.h>

#include "PhysicEngine.h"
#include "GlobalVariables.h"
#include "Renderer.h"

#include <iostream>

CPolygon::CPolygon(size_t index)
	: m_vertexBufferId(0), m_index(index), density(0.1f)
{
	aabb = new AABB();
}

CPolygon::~CPolygon()
{
	DestroyBuffers();
	delete aabb;
}

void CPolygon::Build()
{
	m_lines.clear();

	CreateBuffers();
	BuildLines();
}

void CPolygon::Draw()
{
	glColor3f(0.7f, 0.7f, 0.7f);
	// Set transforms (assuming model view mode is set)
	float transfMat[16] = {	rotation.X.x, rotation.X.y, 0.0f, 0.0f,
							rotation.Y.x, rotation.Y.y, 0.0f, 0.0f,
							0.0f, 0.0f, 0.0f, 1.0f,
							position.x, position.y, -1.0f, 1.0f };
	glPushMatrix();
	glMultMatrixf(transfMat);

	// Draw vertices
	BindBuffers();
	glDrawArrays(GL_LINE_LOOP, 0, points.size());
	glDisableClientState(GL_VERTEX_ARRAY);

	glPopMatrix();

	DrawCenterOfGravity();
	UpdateAABB();
}

size_t	CPolygon::GetIndex() const
{
	return m_index;
}

AABB*	CPolygon::GetAABB()
{
	return aabb;
}

Vec2	CPolygon::TransformPoint(const Vec2& point) const
{
	return position + rotation * point;
}

Vec2	CPolygon::InverseTransformPoint(const Vec2& point) const
{
	return rotation.GetInverse() * (point - position);
}

Vec2	CPolygon::GetCenterOfGravity()
{
	Vec2 firstPoint = points[0];
	float centerX = 0;
	float centerY = 0;

	// The area is needed twice to be properly set in the center of gravity
	float area = 0;
	float offset;

	for (unsigned int i = 0; i < points.size() -1; i++)
	{
		// two points of the segment
		Vec2 a = points[i];
		Vec2 b = points[i + 1];

		// formula (x1 * yi+1 - xi+1 * yi) => gives the Area * 2
		offset = (a.x - firstPoint.x) * (b.y - firstPoint.y) - (b.x - firstPoint.x) * (a.y - firstPoint.y);
		
		// offset / 2 because Area * 2 above
		area += offset / 2;

		// formula (xi + xi+1) / (yi + yi+1)
		centerX += (a.x + b.x - 2 * firstPoint.x) * offset;
		centerY += (a.y + b.y - 2 * firstPoint.y) * offset;

	}
	offset = area * 6;

	return TransformPoint(Vec2(centerX / offset + firstPoint.x, centerY / offset + firstPoint.y));
}

void	CPolygon::DrawCenterOfGravity()
{
	Vec2 g = GetCenterOfGravity();

	if (aabb->GetIsDisplayingAABB())
	{
		gVars->pRenderer->DrawLine(Vec2(g.x - 0.1f, g.y), Vec2(g.x + 0.1f, g.y), 1, 0.8f, 0);
		gVars->pRenderer->DrawLine(Vec2(g.x, g.y - 0.1f), Vec2(g.x, g.y + 0.1f), 1, 0.8f, 0);
	}
}

CPolygon* CPolygon::MinkowskiDiff(const CPolygon& otherPoly) const
{
	// -1 cannot be in the index
	CPolygon* poly = new CPolygon(-1);
	for (unsigned int i = 0; i < points.size(); i++)
	{
		for (unsigned int j = 0; j < otherPoly.points.size(); j++)
			poly->points.push_back(otherPoly.TransformPoint(otherPoly.points[j]) - TransformPoint(points[i]));
	}
	poly->ConvexHull();
	return poly;
}

int CPolygon::Orientation(Vec2 pivot, Vec2 externalPoint, Vec2 anyOther)
{
	float val = (externalPoint.y - pivot.y) * (anyOther.x - externalPoint.x) - (externalPoint.x - pivot.x) * (anyOther.y - externalPoint.y);

	// Points are clockwise
	if (val > 0.f)
		return 1;

	// Points are counterclockwise
	else if (val < 0.f)
		return 2;

	// Points are colinear
	else
		return 0;
}

void CPolygon::ConvexHull()
{
	//	At least three points needed
	if (points.size() < 3)
		return;

	std::vector<Vec2> hull;

	int leftPoint = 0;

	// Find out the leftmost point
	for (unsigned int i = 1; i < points.size(); i++)
	{
		if (points[i].x < points[leftPoint].x || (points[i].x == points[leftPoint].x && points[i].y < points[leftPoint].y))
			leftPoint = i;
	}

	int pivot = leftPoint;
	int externalPoint = 0;

	do
	{
		hull.push_back(points[pivot]);

		externalPoint = (pivot + 1) % points.size();

		for (unsigned int i = 0; i < points.size(); i++)
		{
			// If counterclockwise
			if (Orientation(points[pivot], points[i], points[externalPoint]) == 2)
				externalPoint = i;
		}
		pivot = externalPoint;

	} while (pivot != leftPoint);

	points = hull;
	hull.clear();
}

bool	CPolygon::IsPointInside(const Vec2& point) const
{
	float maxDist = -FLT_MAX;

	for (const Line& line : m_lines)
	{
		Line globalLine = line.Transform(rotation, position);
		float pointDist = globalLine.GetPointDist(point);
		maxDist = Max(maxDist, pointDist);
	}

	return maxDist <= 0.0f;
}

int CPolygon::SupportPoint(Vec2& direction)
{
	int index = 0;
	float maxVal = points[index] | direction;
	float currentVal;

	for (int i = 1; i < points.size(); i++)
	{
		currentVal = points[i] | direction;
		if (maxVal < currentVal)
		{
			index = i;
			maxVal = currentVal;
		}
	}
	return index;
}

bool CPolygon::GJK(Vec2& impact, Vec2& normal, float distance)
{
	const Vec2 origin = Vec2::Zero();
	Simplex simplex = Simplex();
	simplex.AddPoint(points[0]);
	int prevSupportPointIndex = -1;

	Vec2 point;
	Vec2 direction;
	int index;

	do
	{
		point = simplex.GetClosestPoint(origin);

		if (simplex.ComparePoints(origin, point))
		{
			impact = points[index];
			return true;
		}

		simplex.Culling(origin);
		direction = origin - point;
		index = SupportPoint(direction);
		simplex.AddPoint(points[index]);

		if (index == prevSupportPointIndex)
			return false;

		prevSupportPointIndex = index;
	} while (true);
	return false;
}

bool	CPolygon::CheckCollision(const CPolygon& poly, Vec2& colPoint, Vec2& colNormal, float& colDist) const
{
	CPolygon* polyResult = MinkowskiDiff(poly);

	//	Can be drawn in debug mode
	for (int i = 0; i < polyResult->points.size() && aabb->GetIsDisplayingAABB(); i++)
	{
		if(i < polyResult->points.size() -1)
			gVars->pRenderer->DrawLine(polyResult->points[i], polyResult->points[i+1], 0.7f, 0.3f, 0.1f);

		else
			gVars->pRenderer->DrawLine(polyResult->points[i], polyResult->points[0], 0.7f, 0.3f, 0.1f);
	}
	
	//bool bGJKResult = polyResult->GJK(colPoint, colNormal, colDist);

	delete polyResult;

	//return bGJKResult;
	return false;
}



AABB*	CPolygon::GetOwnAABB()
{
	return aabb;
}

void	CPolygon::UpdateAABB()
{
	aabb->SetMinMaxPoints(*this);

	if(aabb->GetIsDisplayingAABB())
		aabb->RenderBoundingBox();
}

/*void CPolygon::ComputeArea()
{
	m_signedArea = 0.0f;
	for (size_t index = 0; index < points.size(); ++index)
	{
		const Vec2& pointA = points[index];
		const Vec2& pointB = points[(index + 1) % points.size()];
		m_signedArea += pointA.x * pointB.y - pointB.x * pointA.y;
	}
	m_signedArea *= 0.5f;
}*/

bool CPolygon::MinMaxTest(CPolygon& poly, CPolygon& otherPoly)
{
	return (poly.aabb->GetMinPnt().x < otherPoly.aabb->GetMinPnt().x);
}

void CPolygon::CreateBuffers()
{
	DestroyBuffers();

	float* vertices = new float[3 * points.size()];
	for (size_t i = 0; i < points.size(); ++i)
	{
		vertices[3 * i] = points[i].x;
		vertices[3 * i + 1] = points[i].y;
		vertices[3 * i + 2] = 0.0f;
	}

	glGenBuffers(1, &m_vertexBufferId);

	glBindBuffer(GL_ARRAY_BUFFER, m_vertexBufferId);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * points.size(), vertices, GL_STATIC_DRAW);

	glBindBuffer(GL_ARRAY_BUFFER, 0);

	delete[] vertices;
}

void CPolygon::BindBuffers()
{
	if (m_vertexBufferId != 0)
	{
		glBindBuffer(GL_ARRAY_BUFFER, m_vertexBufferId);

		glEnableClientState(GL_VERTEX_ARRAY);
		glVertexPointer(3, GL_FLOAT, 0, (void*)0);
	}
}

void CPolygon::DestroyBuffers()
{
	if (m_vertexBufferId != 0)
	{
		glDeleteBuffers(1, &m_vertexBufferId);
		m_vertexBufferId = 0;
	}
}

void CPolygon::BuildLines()
{
	for (size_t index = 0; index < points.size(); ++index)
	{
		const Vec2& pointA = points[index];
		const Vec2& pointB = points[(index + 1) % points.size()];

		Vec2 lineDir = (pointA - pointB).Normalized();

		m_lines.push_back(Line(pointB, lineDir));
	}
}

bool operator < (const CPolygonPtr& poly, const CPolygonPtr& otherPoly)
{
	return (poly->GetAABB()->GetMinPnt().x < otherPoly->GetAABB()->GetMinPnt().x);
}

