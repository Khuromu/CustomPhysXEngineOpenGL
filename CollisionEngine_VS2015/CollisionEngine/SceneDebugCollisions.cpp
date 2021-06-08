#include "Scenes/SceneDebugCollisions.h"

void CSceneDebugCollisions::Create()
{
	CBaseScene::Create();

	CPolygonPtr firstPoly = gVars->pWorld->AddTriangle(30.0f, 20.0f);
	firstPoly->density = 0.0f;
	firstPoly->position = Vec2(-5.0f, -5.0f);
	firstPoly->Build();

	CPolygonPtr secondPoly = gVars->pWorld->AddTriangle(25.0f, 20.0f);
	secondPoly->position = Vec2(5.0f, 5.0f);
	secondPoly->density = 0.0f;

	CDisplayCollision* displayCollision = static_cast<CDisplayCollision*>(gVars->pWorld->AddBehavior<CDisplayCollision>(nullptr).get());
	displayCollision->SetPolyA(firstPoly);
	displayCollision->SetPolyB(secondPoly);
}