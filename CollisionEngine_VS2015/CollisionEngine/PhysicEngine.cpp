#include "PhysicEngine.h"

#include <iostream>
#include <string>
#include "GlobalVariables.h"
#include "World.h"
#include "Renderer.h" // for debugging only
#include "Timer.h"

#include "BroadPhase.h"
#include "BroadPhaseBrut.h"
#include "SPBroadPhase.h"


void	CPhysicEngine::Reset()
{
	m_pairsToCheck.clear();
	m_collidingPairs.clear();

	m_active = true;

	m_broadPhase = new CSPBroadPhase();
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
	if (!m_active)
	{
		return;
	}

	DetectCollisions();
}

void	CPhysicEngine::CollisionBroadPhase()
{
	for (const SPolygonPair& polyPair : m_pairsToCheck)
	{
		polyPair.polyA->GetOwnAABB()->SetIsColliding(false);
		polyPair.polyB->GetOwnAABB()->SetIsColliding(false);
	}

	m_pairsToCheck.clear();
	//SPBroadPhase replaces the BroadPhaseBrut here
	m_broadPhase->GetCollidingPairsToCheck(m_pairsToCheck);

	for (const SPolygonPair& polyPair : m_pairsToCheck)
	{
		polyPair.polyA->GetOwnAABB()->SetIsColliding(true);
		polyPair.polyB->GetOwnAABB()->SetIsColliding(true);
	}
}

void	CPhysicEngine::CollisionNarrowPhase()
{
	m_collidingPairs.clear();
	SCollision collision;

	for (const SPolygonPair& pair : m_pairsToCheck)
	{
		collision.polyA = pair.polyA;
		collision.polyB = pair.polyB;
		if (pair.polyA->CheckCollision(*(pair.polyB), collision.point, collision.normal, collision.distance)) 
			m_collidingPairs.push_back(collision);
	}
}