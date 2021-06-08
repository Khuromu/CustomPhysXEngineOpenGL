#include "Behaviors/SimplePolygonBounce.h"

#include "PhysicEngine.h"
#include "GlobalVariables.h"
#include "Renderer.h"
#include "World.h"

void CSimplePolygonBounce::Update(float frameTime)
{
	gVars->pPhysicEngine->ForEachCollision([&](const SCollision& collision)
	{
		collision.polyA->position += collision.normal * collision.distance * -0.5f;
		collision.polyB->position += collision.normal * collision.distance * 0.5f;

		collision.polyA->speed.Reflect(collision.normal);
		collision.polyB->speed.Reflect(collision.normal);
	});

	float hWidth = gVars->pRenderer->GetWorldWidth() * 0.5f;
	float hHeight = gVars->pRenderer->GetWorldHeight() * 0.5f;

	gVars->pWorld->ForEachPolygon([&](CPolygonPtr poly)
	{
		poly->position += poly->speed * frameTime;

		if (poly->position.x < -hWidth)
		{
			poly->position.x = -hWidth;
			poly->speed.x *= -1.0f;
		}
		else if (poly->position.x > hWidth)
		{
			poly->position.x = hWidth;
			poly->speed.x *= -1.0f;
		}
		if (poly->position.y < -hHeight)
		{
			poly->position.y = -hHeight;
			poly->speed.y *= -1.0f;
		}
		else if (poly->position.y > hHeight)
		{
			poly->position.y = hHeight;
			poly->speed.y *= -1.0f;
		}
	});
}