#ifndef _BASE_SCENE_H_
#define _BASE_SCENE_H_

#include "SceneManager.h"

#include "Behavior.h"
#include "PhysicEngine.h"
#include "GlobalVariables.h"
#include "Renderer.h"
#include "World.h"

#include "Behaviors/PolygonMoverTool.h"

class CBaseScene : public IScene
{
protected:
	CBaseScene(float borderSize = 1.0f, float worldHeight = 50.0f)
		: m_borderSize(borderSize), m_worldHeight(worldHeight){}

	virtual void Create() override;
	void CreateBorderRectangles();

	float m_borderSize;
	float m_worldHeight;
};


#endif