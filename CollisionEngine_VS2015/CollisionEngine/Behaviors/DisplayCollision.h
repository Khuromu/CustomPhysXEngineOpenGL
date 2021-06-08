#ifndef _DISPLAY_COLLISION_H_
#define _DISPLAY_COLLISION_H_

#include "Behavior.h"

#include <string>
#include <iostream>

class CDisplayCollision : public CBehavior
{
public:
	void SetPolyA(CPolygonPtr& poly);
	void SetPolyB(CPolygonPtr& poly);


private:
	virtual void Update(float frameTime) override;

	CPolygonPtr polyA;
	CPolygonPtr polyB;
};


#endif