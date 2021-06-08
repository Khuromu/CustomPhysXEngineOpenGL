#include "Scenes/SceneSpheres.h"

void CSceneSpheres::Create()
{
	gVars->pWorld->AddBehavior<CPolygonMoverTool>(nullptr);
	gVars->pWorld->AddBehavior<CSphereSimulation>(nullptr);
}