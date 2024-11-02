#include "SAT.h"

using namespace Manifest_Simulation;
  
MFbool Manifest_Simulation::SAT(const ConvexHull& hull0, const ConvexHull& hull1, SAT_Query& query)
{		
	query.faceQuery0 = QueryFaceDirection(hull0, hull1);
	if (query.faceQuery0.distance > EPSILON_SAT)
		return false;
	
	query.faceQuery1 = QueryFaceDirection(hull1, hull0);
	if (query.faceQuery1.distance > EPSILON_SAT)
		return false;
		
	query.edgeQuery = QueryEdgeDirection(hull0, hull1);
	if (query.edgeQuery.distance > EPSILON_SAT)
		return false;
	
	return true;
} 