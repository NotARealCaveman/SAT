#pragma once
#include "SAT_Contact.h"

namespace Manifest_Simulation
{
	//SAT between point v convex hull - returns true if overlap detected
	MFbool SAT(const MFpoint3& point, const ConvexHull& hull, FaceQuery& faceQuery);
	//SAT between line segment v convex hull - returns true if overlap detected
	MFbool SAT(const MFpoint3(& segment)[2], const ConvexHull& hull, SAT_Query& query);	
	//SAT between two convex hulls - returns true if overlap detected
	MFbool SAT(const ConvexHull& hull0, const ConvexHull& hull1, SAT_Query& query);
	//SAT between convex hull and convex hull snap shot
	MFbool SAT_Snapshot(const ConvexHull& completeHull, const ConvexHull& snapshotHull, SAT_Query& query);
}