#include "Behaviors/DisplayCollision.h"

#include "PhysicEngine.h"
#include "GlobalVariables.h"
#include "Renderer.h"
#include "RenderWindow.h"
#include "World.h"

void CDisplayCollision::Update(float frameTime)
{
	gVars->pPhysicEngine->Activate(true);

	Vec2 point, normal;
	float dist;
	if (polyA->CheckCollision(*polyB, point, normal, dist))
	{
		gVars->pRenderer->DisplayTextWorld("collision point", point);
		gVars->pRenderer->DisplayText("Collision distance : " + std::to_string(dist), 50, 50);

		gVars->pRenderer->DrawLine(point, point + normal * dist, 0.0f, 1.0f, 0.0f);
	}
}

void CDisplayCollision::SetPolyA(CPolygonPtr& poly)
{
	polyA = poly;
}

void CDisplayCollision::SetPolyB(CPolygonPtr& poly)
{
	polyB = poly;
}