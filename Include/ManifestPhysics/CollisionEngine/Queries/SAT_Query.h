#pragma once

#include "Support.h"

namespace Manifest_Simulation
{
	constexpr MFfloat EPSILON_SAT{ 1e-4 };	

	struct FaceQuery
	{
		HullFace const* face{ nullptr };
		MFfloat distance{ std::numeric_limits<MFfloat>::max() };
	};
	FaceQuery QueryFaceDirection(const ConvexHull& hull0, const ConvexHull& hul1l);

	struct EdgeQuery
	{
		HullHalfEdge const* edgeA{ nullptr };
		HullHalfEdge const* edgeB{ nullptr };
		MFfloat distance{ std::numeric_limits<MFfloat>::max() };
	};
		
	EdgeQuery QueryEdgeDirection(const ConvexHull& hull0, const ConvexHull& hull1);
	//derives vertices on unit sphere and the arcs between them
	void DeriveGaussMapping(const MFtransform& worldSpace, const HullHalfEdge const* edge, MFpoint3& edgeOrigin, MFvec3& arcEdge, MFvec3& vertexA, MFvec3& vertexB, MFvec3 s);
	//test if arcs AB and CD intersect on the unit sphere
	MFbool IsMinkowskiFace(const MFvec3& BxA, const MFvec3& DxC, const MFvec3& a, const MFvec3& b, const MFvec3& c, const MFvec3& d);
	MFfloat Distance(const MFpoint3& origin0, const MFvec3& BxA, const MFpoint3& origin1, const MFvec3& DxC, const MFpoint3& translation0); 

	struct SAT_Query
	{
		FaceQuery faceQuery0;
		FaceQuery faceQuery1;
		EdgeQuery edgeQuery;
	}; 
}