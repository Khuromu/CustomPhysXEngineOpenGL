#include "Behaviors/PolygonMoverTool.h"

#include "PhysicEngine.h"
#include "GlobalVariables.h"
#include "Renderer.h"
#include "RenderWindow.h"
#include "World.h"

CPolygonPtr	CPolygonMoverTool::GetClickedPolygon()
{
	Vec2 pt, n;
	Vec2 mousePoint = gVars->pRenderer->ScreenToWorldPos(gVars->pRenderWindow->GetMousePos());
	CPolygonPtr clickedPoly;

	gVars->pWorld->ForEachPolygon([&](CPolygonPtr poly)
	{
		if (poly->IsPointInside(mousePoint))
		{
			clickedPoly = poly;
		}
	});

	return clickedPoly;
}

void CPolygonMoverTool::Update(float frameTime)
{
	if (gVars->pRenderWindow->GetMouseButton(0) || gVars->pRenderWindow->GetMouseButton(2))
	{
		if (!m_selectedPoly)
		{
			m_selectedPoly = GetClickedPolygon();
			m_prevMousePos = gVars->pRenderer->ScreenToWorldPos(gVars->pRenderWindow->GetMousePos());
			m_translate = gVars->pRenderWindow->GetMouseButton(0);
			m_clickMousePos = m_prevMousePos;

			if (m_selectedPoly)
				m_clickAngle = m_selectedPoly->rotation.GetAngle();
		}
		else
		{
			Vec2 mousePoint = gVars->pRenderer->ScreenToWorldPos(gVars->pRenderWindow->GetMousePos());

			if (m_translate)
			{
				m_selectedPoly->position += mousePoint - m_prevMousePos;
				m_selectedPoly->speed = Vec2();
			}
			else
			{
				Vec2 from = m_clickMousePos - m_selectedPoly->position;
				Vec2 to = mousePoint - m_selectedPoly->position;

				m_selectedPoly->rotation.SetAngle(m_clickAngle + from.Angle(to));
				m_selectedPoly->speed = Vec2();
			}

			m_prevMousePos = mousePoint;
		}
	}
	else
	{
		m_selectedPoly.reset();
	}
}