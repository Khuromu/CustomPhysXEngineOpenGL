#ifndef _SCENE_BOUNCING_POLYS_H_
#define _SCENE_BOUNCING_POLYS_H_

#include "BaseScene.h"

#include "Behaviors/SimplePolygonBounce.h"

class CSceneBouncingPolys : public CBaseScene
{
public:
	CSceneBouncingPolys(size_t polyCount)
		: m_polyCount(polyCount){}

protected:
	virtual void Create() override;

private:
	size_t m_polyCount;
};

#endif