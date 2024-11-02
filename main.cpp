#pragma once

#include <ManifestMath/Quickhull.h>
#include <ManifestPhysics/CollisionEngine/Queries/SAT.h>

using namespace Manifest_Math;
using namespace Manifest_Simulation;
using namespace Manifest_Utility;

const std::vector<MFpoint3> hullPointCloud
{
		MFpoint3{-1, -1, -1},
		MFpoint3{-1, 1, -1},
		MFpoint3{1, 1, -1},
		MFpoint3{1, -1, -1},
		MFpoint3{-1, -1, 1},
		MFpoint3{1, -1, 1},
		MFpoint3{-1, 1, 1},
		MFpoint3{1, 1, 1}
};

int main()
{
	ConvexHull hull0
	{
		.mesh {QuickHull(hullPointCloud) },
		.worldSpace{Identity()},
		.scale {1},
	};
	hull0.BuildFinalHullFaces();
	hull0.worldSpace.SetTranslation(MFpoint3{ -1.1,0,0 });
	ConvexHull hull1
	{
		.mesh {QuickHull(hullPointCloud) },
		.worldSpace{Identity()},
		.scale {1},
	};
	hull1.BuildFinalHullFaces();	
	hull1.worldSpace.SetTranslation(MFpoint3{ 1,0,0 });
	
	SAT_Query query;
	if (!SAT(hull0, hull1, query))
	{
		DLOG({ CONSOLE_BG_GREEN , CONSOLE_BLINK }, "Separation detected!");
		return 0;
	}
	else
	{
		DLOG({ CONSOLE_BG_RED , CONSOLE_BLINK }, "Collision detected!");
	}
}