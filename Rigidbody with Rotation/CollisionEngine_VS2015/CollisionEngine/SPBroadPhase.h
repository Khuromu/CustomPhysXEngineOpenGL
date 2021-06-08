#ifndef _CUSTOMBROADPHASE_H_
#define	_CUSTOMBROADPHASE_H_

#include "BroadPhase.h"
#include <algorithm>
#include "Polygon.h"
#include "GlobalVariables.h"
#include "World.h"

//Instead of BroadPhaseBrut, SPBroadPhase sorts the polygon according to their min.x first, then min.y
class CSPBroadPhase : public IBroadPhase
{
public:
	virtual void GetCollidingPairsToCheck(std::vector<SPolygonPair>& pairsToCheck) override
	{
		std::vector<CPolygonPtr> polyPtrVector;
		size_t polyCount = gVars->pWorld->GetPolygonCount();
		//used for optimization
		polyPtrVector.reserve(polyCount);

		for (size_t i = 0; i < polyCount; i++)
		{
			polyPtrVector.push_back(gVars->pWorld->GetPolygon(i));
		}

		//Please refer to the README to see where this method comes from
		std::sort(polyPtrVector.begin(), polyPtrVector.end());

		for (size_t i = 0; i < polyCount; ++i)
		{
			for (size_t j = i + 1; j < polyCount; ++j)
			{
				//if not, no need to go on on onther points
				if (polyPtrVector[i]->GetOwnAABB()->max.x > polyPtrVector[j]->GetOwnAABB()->min.x)
				{
					if(polyPtrVector[i]->GetOwnAABB()->max.y > polyPtrVector[j]->GetOwnAABB()->min.y
						&& polyPtrVector[i]->GetOwnAABB()->min.y < polyPtrVector[j]->GetOwnAABB()->max.y)
						pairsToCheck.push_back(SPolygonPair(polyPtrVector[i], polyPtrVector[j]));
				}
				else
				{
					break;
				}
			}
		}
	}
};

#endif
