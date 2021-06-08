#ifndef _SPHERE_SIMULATION_H_
#define _SPHERE_SIMULATION_H_

#include "Behavior.h"

#define RADIUS 2.0f
#define DISTANCE 5.0f

struct SConstraint
{
	SConstraint(CPolygonPtr _A, CPolygonPtr _B)
		: A(_A), B(_B){}

	CPolygonPtr A, B;
	float distance = DISTANCE;
};

class CSphereSimulation : public CBehavior
{
private:

	void InitChain(size_t count, const Vec2& start);

	void SolveChainConstraints();

	virtual void Start() override;

	virtual void Update(float frameTime) override;

	CPolygonPtr AddCircle(const Vec2& pos, float radius = RADIUS);

	void OnCollision(CPolygonPtr A, CPolygonPtr B, Vec2& normal, float penetration, Vec2& point);

	std::vector<CPolygonPtr>	m_circles;
	std::vector<CPolygonPtr>	m_chain;
};

#endif