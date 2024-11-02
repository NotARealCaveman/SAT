#pragma once
#include <vector>

#include <ManifestMath/HalfEdgeMesh.h>

#include "AxisAlignedBoxes.h"

using namespace Manifest_Math;

namespace Manifest_Simulation
{
	struct ConvexHull
	{		
	public:
		HalfEdgeMesh mesh;
		MFtransform worldSpace;
		MFvec3 scale;	
		//please remove this eventually
		void BuildFinalHullFaces();
	};	

	AxisBoundingBox Encapsulate(const ConvexHull& hull);
}