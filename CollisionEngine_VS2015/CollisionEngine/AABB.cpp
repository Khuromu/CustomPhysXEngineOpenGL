#include "AABB.h"

#include "GlobalVariables.h"
#include "Renderer.h"
#include "Polygon.h"

void AABB::SetMinMaxPoints(CPolygon& poly)
{
	Vec2 tempMin = Vec2(FLT_MAX, FLT_MAX);
	Vec2 tempMax = Vec2(-FLT_MAX, -FLT_MAX);

	//goes through all the polygon' points to get minX, maxX, minY, maxY
	for (Vec2& localPnt : poly.points)
	{
		Vec2 curPnt = poly.TransformPoint(localPnt);
		if (curPnt.x < tempMin.x)
			tempMin.x = curPnt.x;
		if (curPnt.y < tempMin.y)
			tempMin.y = curPnt.y;
		if (curPnt.x > tempMax.x)
			tempMax.x = curPnt.x;
		if (curPnt.y > tempMax.y)
			tempMax.y = curPnt.y;
	}
	minPnt = tempMin;
	maxPnt = tempMax;
}

void AABB::RenderBoundingBox()
{
	if (isColliding == true)
	{
		gVars->pRenderer->DrawLine(minPnt, Vec2(maxPnt.x, minPnt.y), 0.5f, 0.9f, 1);
		gVars->pRenderer->DrawLine(Vec2(maxPnt.x, minPnt.y), maxPnt, 0.5f, 0.9f, 1);
		gVars->pRenderer->DrawLine(maxPnt, Vec2(minPnt.x, maxPnt.y), 0.5f, 0.9f, 1);
		gVars->pRenderer->DrawLine(Vec2(minPnt.x, maxPnt.y), minPnt, 0.5f, 0.9f, 1);
	}

	else
	{
		gVars->pRenderer->DrawLine(minPnt, Vec2(maxPnt.x, minPnt.y), 0.3f, 0, 1);
		gVars->pRenderer->DrawLine(Vec2(maxPnt.x, minPnt.y), maxPnt, 0.3f, 0, 1);
		gVars->pRenderer->DrawLine(maxPnt, Vec2(minPnt.x, maxPnt.y), 0.3f, 0, 1);
		gVars->pRenderer->DrawLine(Vec2(minPnt.x, maxPnt.y), minPnt, 0.3f, 0, 1);
	}
}

const Vec2 AABB::GetMinPnt()
{
	return minPnt;
}

const Vec2 AABB::GetMaxPnt()
{
	return maxPnt;
}

const bool AABB::GetIsDisplayingAABB()
{
	return isDisplayed;
}

void AABB::SetIsColliding(bool colliding)
{
	isColliding = colliding;
}

void AABB::ToggleDisplaying()
{
	isDisplayed = !isDisplayed;
}
