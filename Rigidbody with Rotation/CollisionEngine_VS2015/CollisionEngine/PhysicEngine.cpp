#include "PhysicEngine.h"

#include <iostream>
#include <string>
#include "GlobalVariables.h"
#include "World.h"
#include "Renderer.h" // for debugging only
#include "Timer.h"

#include "BroadPhase.h"
#include "SPBroadPhase.h"


void	CPhysicEngine::Reset()
{
	m_pairsToCheck.clear();
	m_collidingPairs.clear();

	m_active = true;

	m_broadPhase = new CSPBroadPhase;
}

void	CPhysicEngine::Activate(bool active)
{
	m_active = active;
}

void	CPhysicEngine::DetectCollisions()
{
	CTimer timer;
	timer.Start();
	CollisionBroadPhase();
	timer.Stop();
	if (gVars->bDebug)
	{
		gVars->pRenderer->DisplayText("Collision broadphase duration " + std::to_string(timer.GetDuration() * 1000.0f) + " ms");
	}

	timer.Start();
	CollisionNarrowPhase();
	timer.Stop();
	if (gVars->bDebug)
	{
		gVars->pRenderer->DisplayText("Collision narrowphase duration " + std::to_string(timer.GetDuration() * 1000.0f) + " ms, collisions : " + std::to_string(m_collidingPairs.size()));
	}
}


void	CPhysicEngine::Step(float deltaTime)
{
	deltaTime = Min(deltaTime, 1.0f / 15.0f);

	if (!m_active)
	{
		return;
	}

	Vec2 gravity(0, -9.8f);
	float elasticity = 0.6f;

	gVars->pWorld->ForEachPolygon([&](CPolygonPtr poly)
	{
		if (poly->density == 0.0f)
		{
			return;
		}

		poly->rotation.Rotate(RAD2DEG(poly->angularVelocity * deltaTime));
		poly->position += poly->speed * deltaTime;
		poly->speed += gravity * deltaTime;
	});

	DetectCollisions();
}

void	CPhysicEngine::CollisionBroadPhase()
{
	for (const SPolygonPair& polyPair : m_pairsToCheck)
	{
		polyPair.polyA->GetOwnAABB()->bIsColliding = false;
		polyPair.polyB->GetOwnAABB()->bIsColliding = false;
	}

	m_pairsToCheck.clear();
	m_broadPhase->GetCollidingPairsToCheck(m_pairsToCheck);

	for (const SPolygonPair& polyPair : m_pairsToCheck)
	{
		polyPair.polyA->GetOwnAABB()->bIsColliding = true;
		polyPair.polyB->GetOwnAABB()->bIsColliding = true;
	}
}

void	CPhysicEngine::CollisionNarrowPhase()
{
	m_collidingPairs.clear();

	for (const SPolygonPair& pair : m_pairsToCheck)
	{
		SCollision collision;
		collision.polyA = pair.polyA;
		collision.polyB = pair.polyB;

		if (pair.polyA->CheckCollision(*(pair.polyB), collision))
		{
			m_collidingPairs.push_back(collision);

			pair.polyA->speed *= -1.f;
			pair.polyB->speed *= -1.f;
		}
	}
}