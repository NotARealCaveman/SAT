#include "ConvexHull.h"

using namespace Manifest_Simulation;

void ConvexHull::BuildFinalHullFaces()
{
	auto furthestDistance{ 0 };
	auto face{ mesh.faces };
	do
		face->facePlane = Normalize(CalculateScaledFacePlane(face, scale));
	while ((face = face->next) != mesh.faces);
}

AxisBoundingBox Manifest_Simulation::Encapsulate(const ConvexHull& hull)
{
	MFpoint3 min{ INFINITY }, max{ -INFINITY };

	HullVertex const* const start{ hull.mesh.vertices };
	HullVertex const* vertex{ start };
	do
	{
		const MFpoint3 worldPoint{ hull.worldSpace * ComponentMultiply(hull.scale,vertex->vertex) };
		min = Min(min, worldPoint);
		max = Max(max, worldPoint);
	} while ((vertex = vertex->next) != start);

	const MFpoint3 center{ (min + max) * 0.5f };
	const MFvec3 halfLength{ (max - min) * 0.5f };

	return { center, halfLength };
}
