#include "Scenes/BaseScene.h"

void CBaseScene::Create()
{
	gVars->pRenderer->SetWorldHeight(m_worldHeight);
	CreateBorderRectangles();

	gVars->pWorld->AddBehavior<CPolygonMoverTool>(nullptr);
}

void CBaseScene::CreateBorderRectangles()
{
	CPolygonPtr poly;

	float halfWidth = gVars->pRenderer->GetWorldWidth() * 0.5f;
	float halfHeight = gVars->pRenderer->GetWorldHeight() * 0.5f;

	/*poly = gVars->pWorld->AddRectangle(halfWidth * 2.0f, m_borderSize);
	poly->position.y = -halfHeight + 0.5f * m_borderSize;
	poly->density = 0.0f;

	poly = gVars->pWorld->AddRectangle(halfWidth * 2.0f, m_borderSize);
	poly->position.y = halfHeight - 0.5f * m_borderSize;
	poly->density = 0.0f;

	poly = gVars->pWorld->AddRectangle(m_borderSize, halfHeight * 2.0f);
	poly->position.x = -halfWidth + 0.5f * m_borderSize;
	poly->density = 0.0f;

	poly = gVars->pWorld->AddRectangle(m_borderSize, halfHeight * 2.0f);
	poly->position.x = halfWidth - 0.5f * m_borderSize;
	poly->density = 0.0f;*/
}