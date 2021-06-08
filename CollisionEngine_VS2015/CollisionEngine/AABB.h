#ifndef _AABB_H_
#define _AABB_H_

#include "Maths.h"

class CPolygon;

class AABB
{
public:
	AABB() = default;

	void SetMinMaxPoints(CPolygon& poly);
	void RenderBoundingBox();

	const Vec2	GetMinPnt();
	const Vec2	GetMaxPnt();
	const bool	GetIsDisplayingAABB();
	void SetIsColliding(bool colliding);

	void ToggleDisplaying();

private:
	Vec2 minPnt;
	Vec2 maxPnt;
	bool isColliding;
	bool isDisplayed;
};


#endif

