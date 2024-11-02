#pragma once

#include "Support.h"

namespace Manifest_Simulation
{
	constexpr MFfloat EPSILON_SAT{ 1e-4 };	

	//USED FOR CONVEX VS CONVEX

	struct FaceQuery
	{
		HullFace const* face;
		MFfloat distance;
	};	
	FaceQuery QueryFaceDirection(const ConvexHull& hull0, const ConvexHull& hul1l);

	struct EdgeQuery
	{
		 HullHalfEdge const* edgeA;
		 HullHalfEdge const* edgeB;
		 MFfloat distance;
	};	
		
	EdgeQuery QueryEdgeDirection(const ConvexHull& hull0, const ConvexHull& hull1);
	//derives vertices on unit sphere and the arcs between them
	void DeriveGaussMapping(const MFtransform& worldSpace, const HullHalfEdge const* edge, MFpoint3& edgeOrigin, MFvec3& arcEdge, MFvec3& vertexA, MFvec3& vertexB, MFvec3 s);
	//test if arcs AB and CD intersect on the unit sphere
	MFbool IsMinkowskiFace(const MFvec3& BxA, const MFvec3& DxC, const MFvec3& a, const MFvec3& b, const MFvec3& c, const MFvec3& d);
	MFfloat Distance(const MFpoint3& origin0, const MFvec3& BxA, const MFpoint3& origin1, const MFvec3& DxC, const MFpoint3& translation0);
	MFbool IntersectsWhenProjected(const ConvexHull& hull, const MFtriangle& triangle, const MFvec3& separatingAxis);	

	struct SAT_Query
	{
		FaceQuery faceQuery0;
		FaceQuery faceQuery1;
		EdgeQuery edgeQuery;
	};

	//USED FOR CONVEX SNAP SHOTS

}