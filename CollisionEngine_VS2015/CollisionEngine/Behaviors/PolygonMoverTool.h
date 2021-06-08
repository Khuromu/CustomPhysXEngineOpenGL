#ifndef _POLYGON_MOVER_TOOL_H_
#define _POLYGON_MOVER_TOOL_H_

#include "Behavior.h"

class CPolygonMoverTool : public CBehavior
{
	CPolygonPtr	GetClickedPolygon();

	virtual void Update(float frameTime) override;
	

private:
	CPolygonPtr	m_selectedPoly;
	bool		m_translate;
	Vec2		m_prevMousePos;
	Vec2		m_clickMousePos;
	float		m_clickAngle;
};

#endif