#pragma once 
#include <ManifestMath/Clipping.h>

#include <ManifestPhysics/CollisionEngine/ContactManifold.h>
#include "SAT_Query.h"

namespace Manifest_Simulation
{
	//CONVEX HULL VS CONVEX HULL
	struct FaceContact
	{
		FaceQuery query;
		std::vector<MFpoint3> vertices;		
	};
	FaceContact CreateFaceContact(const FaceQuery& faceQuery, const ConvexHull& referenceHull, const ConvexHull& incidentHull);
	void RefineContactPoints(const MFvec3& incidentScale, FaceContact& faceContact);
	ContactManifold* ConvertFaceContact(const MFvec3& referenceHullScale, const FaceContact& contact, std::vector<ContactManifold>& contactManifolds);

	struct EdgeContact
	{
		EdgeQuery query;		
		MFpoint3 midPoint;
		MFvec3 separationAxis;
	};

	EdgeContact CreateEdgeContact(const EdgeQuery& edgeQuery, const ConvexHull& hull0, const ConvexHull& hull1);	
	ContactManifold* ConvertEdgeContact(const EdgeContact& contact, std::vector<ContactManifold>& contactManifolds);

	//CONVEX HULL VS CONVEX SNAP SHOT
	struct FaceContact_Snapshot
	{
		FaceQuery faceQuery;
	};
}