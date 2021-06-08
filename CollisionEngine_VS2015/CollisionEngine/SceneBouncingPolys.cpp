#include "Scenes/SceneBouncingPolys.h"

void CSceneBouncingPolys::Create()
{
	CBaseScene::Create();
	
	gVars->pWorld->AddBehavior<CSimplePolygonBounce>(nullptr);
	
	float width = gVars->pRenderer->GetWorldWidth();
	float height = gVars->pRenderer->GetWorldHeight();
	
	SRandomPolyParams params;
	params.minRadius = 1.0f;
	params.maxRadius = 1.0f;
	params.minBounds = Vec2(-width * 0.5f + params.maxRadius * 3.0f, -height * 0.5f + params.maxRadius * 3.0f);
	params.maxBounds = params.minBounds * -1.0f;
	params.minPoints = 3;
	params.maxPoints = 8;
	params.minSpeed = 1.0f;
	params.maxSpeed = 3.0f;
	
	for (size_t i = 0; i < m_polyCount; ++i)
	{
		gVars->pWorld->AddRandomPoly(params)->density = 0.0f;
	}
}