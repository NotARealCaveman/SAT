#pragma once
#include <vector>
#include <ranges>

#include "Plane.h"
#include "HalfEdgeMesh.h"
#include "CachedVertex.h"

namespace Manifest_Math
{
	//clips segment 1 against the side planes of segment 0
	void ClipSegments(const MFpoint3(&segment0)[2], MFpoint3(&segment1)[2]);

	//returns clipped edges from incident polygonal face against side plane
	std::vector<MFpoint3> Sutherland_Hodgman(const MFplane& sidePlane, const std::vector<MFpoint3>& incidentVertices);
}