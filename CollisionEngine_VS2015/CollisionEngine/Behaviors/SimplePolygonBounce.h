#ifndef _SIMPLE_COLLISION_BOUNCE_H_
#define _SIMPLE_COLLISION_BOUNCE_H_

#include "Behavior.h"

class CSimplePolygonBounce : public CBehavior
{
private:
	virtual void Update(float frameTime) override;
};

#endif