#ifndef _POLYGON_H_
#define _POLYGON_H_

#include <GL/glew.h>
#include <vector>
#include <memory>

#include "Maths.h"
#include "AABB.h"

struct SPolygonPair;


class CPolygon
{
private:
	friend class CWorld;

	CPolygon(size_t index);
public:
	~CPolygon();

	Vec2				position;
	Mat2				rotation;
	std::vector<Vec2>	points;

	void				Build();
	void				Draw();
	size_t				GetIndex() const;

	//float				GetArea() const;

	AABB*				GetAABB();

	Vec2				TransformPoint(const Vec2& point) const;
	Vec2				InverseTransformPoint(const Vec2& point) const;

	// Center of gravity's X and Y points through area computation
	Vec2				GetCenterOfGravity();
	void				DrawCenterOfGravity();

	// Minkowski's Algorithm
	CPolygon*			MinkowskiDiff(const CPolygon& otherPoly) const;
	int					Orientation(Vec2 pivot, Vec2 externalPoint, Vec2 anyOther);
	//	Jarvis' Algorithm
	void				ConvexHull();

	//	The Polygon finds its own Support point according to the given direction by returning its points' index
	int					SupportPoint(Vec2& direction);

	bool				GJK(Vec2& impact, Vec2& normal, float distance);

	// If point is outside then returned distance is negative (and doesn't make sense)
	bool				IsPointInside(const Vec2& point) const;

	bool				CheckCollision(const CPolygon& poly, Vec2& colPoint, Vec2& colNormal, float& colDist) const;

	AABB*				GetOwnAABB();
	void				UpdateAABB();

	//void				ComputeArea();

	bool				MinMaxTest(CPolygon& poly, CPolygon& otherPoly);

	// Physics
	float				density;
	Vec2				speed;

	float				invMass = 1.f;

private:
	void				CreateBuffers();
	void				BindBuffers();
	void				DestroyBuffers();

	void				BuildLines();

	GLuint				m_vertexBufferId;
	size_t				m_index;

	std::vector<Line>	m_lines;

	AABB*				aabb;

	float				m_signedArea;

};

typedef std::shared_ptr<CPolygon>	CPolygonPtr;

// Used in Custom BroadPhase
bool operator < (const CPolygonPtr& poly, const CPolygonPtr& otherPoly);

#endif