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
	const MFmat3 rotation{ Rotation_Y(Radians(0)) };
	ConvexHull hull0
	{
		.mesh {QuickHull(hullPointCloud) },
		.worldSpace{Identity()},
		.scale {1},
	};
	hull0.BuildFinalHullFaces();
	hull0.worldSpace[0] = rotation[0];
	hull0.worldSpace[1] = rotation[1];
	hull0.worldSpace[2] = rotation[2];
	hull0.worldSpace.SetTranslation(MFpoint3{ -1.1,0,0 });
	ConvexHull hull1
	{
		.mesh {QuickHull(hullPointCloud) },
		.worldSpace{Identity()},
		.scale {1},
	};
	hull1.BuildFinalHullFaces();	
	hull1.worldSpace[0] = rotation[0];
	hull1.worldSpace[1] = rotation[1];
	hull1.worldSpace[2] = rotation[2];
	hull1.worldSpace.SetTranslation(MFpoint3{ 1,0,0 });		
	std::vector<ContactManifold> contactManifolds;	
	for (MFu32 step{ 0 }; step < 10; ++step)
	{
		//moves hull1 towards hull0
		const MFpoint3 hull1Position{ hull1.worldSpace.GetTranslation() };
		hull1.worldSpace.SetTranslation(hull1Position + MFvec3{ -0.01f,0.0f,0.0f });

		SAT_Query query;
		if (!SAT(hull0, hull1, query))
			DLOG({ CONSOLE_BG_GREEN }, "NO COLLISION DETECTED!","face distance 0:",query.faceQuery0.distance, "face distance 1:",query.faceQuery1.distance,"edge distance:", query.edgeQuery.distance);
		else
		{
			DLOG({ CONSOLE_BG_RED, CONSOLE_BLINK}, "COLLISION DETECTED!", "face distance 0:", query.faceQuery0.distance, "face distance 1:", query.faceQuery1.distance, "edge distance:", query.edgeQuery.distance);
			ContactManifold* contactManifold;
			const MFfloat faceDistance0{ query.faceQuery0.distance };
			const MFfloat faceDistance1{ query.faceQuery1.distance };

			//https://www.gamedev.net/forums/topic/667499-3d-sat-problem/?page=2#:~:text=MKS%20for%20the-,absolute%20tolerance,-)%3A - ty dirk
			const MFfloat kLinearSlop{ 0.005f };
			const MFfloat kRelEdgeTolerance{ 0.90f };
			const MFfloat kRelFaceTolerance{ 0.98f };
			const MFfloat kAbsTolerance{ 0.5f * kLinearSlop };

			//test for edge contact first
			if (query.edgeQuery.distance > Epsilon(kRelEdgeTolerance, std::fmaxf(faceDistance0, faceDistance1), kAbsTolerance))
			{				
				const EdgeContact contact{ CreateEdgeContact(query.edgeQuery,hull0,hull1) };
				contactManifold = ConvertEdgeContact(contact, contactManifolds); 
				DLOG({ CONSOLE_BG_CYAN, CONSOLE_BOLD }, "EDGE CONTACT DETECTED! Contact Normal:",contactManifold->normal);
			}
			else if (faceDistance1 > Epsilon(kRelFaceTolerance, faceDistance0, kAbsTolerance))
			{
				const FaceContact contact{ CreateFaceContact(query.faceQuery1, hull1,hull0) };
				contactManifold = ConvertFaceContact(hull1.scale, contact, contactManifolds);
				//invert normal and convert to world space	
				contactManifold->normal = -contactManifold->normal * Inverse(hull1.worldSpace); 
				DLOG({ CONSOLE_BG_CYAN, CONSOLE_BOLD }, "FACE 0 CONTACT DETECTED! Contact Normal:", contactManifold->normal);
			}
			else
			{
				const FaceContact contact{ CreateFaceContact(query.faceQuery0, hull0,hull1) };
				contactManifold = ConvertFaceContact(hull0.scale, contact, contactManifolds);
				//convert normal to world space	
				contactManifold->normal = contactManifold->normal * Inverse(hull0.worldSpace);
				DLOG({ CONSOLE_BG_CYAN, CONSOLE_BOLD }, "FACE 1 CONTACT DETECTED! Contact Normal:", contactManifold->normal);
			}
			contactManifold->constraintType = ConstraintType::CONTACT;
		}
	}
}