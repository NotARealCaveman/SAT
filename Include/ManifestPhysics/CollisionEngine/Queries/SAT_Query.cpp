#include "SAT_Query.h"

using namespace Manifest_Simulation;

FaceQuery Manifest_Simulation::QueryFaceDirection(const ConvexHull& hull0, const ConvexHull& hull1)
{
	HullFace const* face{ nullptr };
	FaceQuery result{ face,-std::numeric_limits<MFfloat>::infinity() };

	HullFace const* const start{ hull0.mesh.faces };
	face = start;
	do
	{
		//transform plane to world space, Fb = FaM-1			
		const MFplane planeA{ face->facePlane * Inverse(hull0.worldSpace) };
		MFu32 vertexIndex;//might be used later for caching?
		//express search direction in local space of hull1
		const MFvec3 localDirection{ -planeA.Normal() * hull1.worldSpace };
		const MFpoint3 vertexB{ FindFurthestPoint(hull1, localDirection,vertexIndex) };
		const MFfloat distance{ Dot(planeA,vertexB) };		
		if (distance <= result.distance)
			continue;
		result = { face ,distance };
		if (result.distance > EPSILON_SAT)
			return result;
	} while ((face = face->next) != start);

	return result;
}

EdgeQuery Manifest_Simulation::QueryEdgeDirection(const ConvexHull& hull0, const ConvexHull& hull1)
{
	// Find axis of minimum penetration
	EdgeQuery result{ nullptr,nullptr ,-std::numeric_limits<MFfloat>::max() };

	for (HullHalfEdge const* const edge0 : hull0.mesh.edges)
	{
		assert(edge0->twin->twin == edge0);		

		MFpoint3 origin0;
		MFvec3 BxA, a, b;
		DeriveGaussMapping(hull0.worldSpace, edge0, origin0, BxA, a, b, hull0.scale);
		//DLOG({ CONSOLE_GREEN }, "Gauss Mapping for hull 0. BxA:", BxA, "a:", a, "b:", b);
		for (HullHalfEdge const* const edge1 : hull1.mesh.edges)
		{
			assert(edge1->twin->twin == edge1);

			MFpoint3 origin1;
			MFvec3 DxC, c, d;
			DeriveGaussMapping(hull1.worldSpace, edge1, origin1, DxC, c, d, hull1.scale);
			//DLOG({ CONSOLE_CYAN }, "Gauss Mapping for hull 1. DxC:", DxC, "c:", c, "d:", d);
			if (!IsMinkowskiFace(-BxA, -DxC, a, b, -c, -d))
				continue;

			const MFfloat distance{ Distance(origin0, BxA, origin1, DxC, hull0.worldSpace.GetTranslation()) };
			//DLOG({ CONSOLE_RED }, "Minkowski face detected. detected Distance:", distance);
			if (distance <= result.distance)
				continue;

			result = { edge0,edge1 , distance };
			if (result.distance > EPSILON_SAT)
				return result;
		}
	}

	return result;
}

void Manifest_Simulation::DeriveGaussMapping(const MFtransform& worldSpace, const HullHalfEdge const* edge, MFpoint3& edgeOrigin, MFvec3& arcEdge, MFvec3& vertexA, MFvec3& vertexB, MFvec3 s)
{
	edgeOrigin = worldSpace * ComponentMultiply(s,edge->tail->vertex);
	MFpoint3 edgeDestination{ worldSpace * ComponentMultiply(s,edge->twin->tail->vertex) };
	arcEdge = edgeDestination - edgeOrigin;

	//vertexA = Normalize(CalculateFacePlane(edge->face)).Normal() * Inverse(worldSpace);
	vertexA = Normalize(CalculateScaledFacePlane(edge->face, s)).Normal() * Inverse(worldSpace);
	vertexB = Normalize(CalculateScaledFacePlane(edge->twin->face, s)).Normal() * Inverse(worldSpace);
}

MFbool Manifest_Simulation::IsMinkowskiFace(const MFvec3& BxA, const MFvec3& DxC, const MFvec3& a, const MFvec3& b, const MFvec3& c, const MFvec3& d)
{
	//plane test using Scalar Triple Products - only need sign
	const MFfloat halfSpaceCBA{ Dot(c,BxA) };
	const MFfloat halfSpaceDBA{ Dot(d,BxA) };
	const MFfloat halfSpaceADC{ Dot(a,DxC) };
	const MFfloat halfSpaceBDC{ Dot(b,DxC) };

	//test for intersections of great arcs - vertices a/b,c/d should be on +/- sides of plane
	const MFbool intersectsBA{ halfSpaceCBA * halfSpaceDBA < 0.0f };
	const MFbool intersectsDC{ halfSpaceADC * halfSpaceBDC < 0.0f };
	//test both arcs lie on same hemisphere	
	const MFbool sameHemisphere{ halfSpaceCBA * halfSpaceBDC > 0.0f };
	//DLOG({ CONSOLE_YELLOW }, "halfSpaceCBA:", halfSpaceCBA, "halfSpaceDBA:", halfSpaceDBA, "halfSpaceADC:", halfSpaceADC,"halfSpaceBDC:", halfSpaceBDC);
	//DLOG({ CONSOLE_MAGENTA }, "intersectsBA:", intersectsBA, "intersectsDC:", intersectsDC, "sameHemisphere:", sameHemisphere,"\n");

	//bitwise fine over logical here, save comparisons 	
	return  static_cast<MFbool>(intersectsBA & intersectsDC & sameHemisphere);
}

MFfloat Manifest_Simulation::Distance(const MFpoint3& origin0, const MFvec3& BxA, const MFpoint3& origin1, const MFvec3& DxC, const MFpoint3& translation0)
{
	// Build search direction
	const MFvec3 E1_x_E2 = Cross(BxA, DxC);
	//DLOG({ CONSOLE_GREEN }, "E1xE2:", E1_x_E2);
	// Skip near parallel edges: |e1 x e2| = sin(alpha) * |e1| * |e2|
	const MFfloat kTolerance = 0.005f;

	const MFfloat L = Magnitude(E1_x_E2);
	//DLOG({ CONSOLE_BLUE }, "L:", L,"MagnitudeSquared(BxA):", MagnitudeSquared(BxA),"MagnitudeSquared(DxC):", MagnitudeSquared(DxC),"kTolerance * std::sqrtf(MagnitudeSquared(BxA) * MagnitudeSquared(DxC)):", kTolerance * std::sqrtf(MagnitudeSquared(BxA) * MagnitudeSquared(DxC)));
	if (L < kTolerance * std::sqrtf(MagnitudeSquared(BxA) * MagnitudeSquared(DxC)))
	{
		//DLOG({ CONSOLE_RED}, "returning:", -std::numeric_limits<MFfloat>::max());
		return -std::numeric_limits<MFfloat>::max();
	}

	// Assure consistent normal orientation (here: Hull1 -> Hull2)
	MFvec3 N = E1_x_E2 / L;
	if (Dot(N, origin0 - translation0) < 0.0f)
		N = -N;
	//DLOG({ CONSOLE_MAGENTA}, "N:", N);
	// s = Dot(n, p2) - d = Dot(n, p2) - Dot(n, p1) = Dot(n, p2 - p1) 
	//DLOG({ CONSOLE_CYAN }, "returning",N," * ",origin1-origin0,":", Dot(N, origin1 - origin0));
	return Dot(N, origin1 - origin0);
}

MFbool Manifest_Simulation::IntersectsWhenProjected(const ConvexHull& hull, const MFtriangle& triangle, const MFvec3& separatingAxis)
{
	MFfloat aMin{ std::numeric_limits<MFfloat>::max() };
	MFfloat aMax{ std::numeric_limits<MFfloat>::min() };
	MFfloat bMin{ aMin };
	MFfloat bMax{ aMax };

	// Define two intervals, a and b. Calculate their min and max values
	HullVertex const* vertex{ hull.mesh.vertices };
	do
	{
		const MFpoint3 localPoint{ ComponentMultiply(hull.scale,vertex->vertex) };
		const MFpoint3 worldPoint{ hull.worldSpace * localPoint };
		MFfloat aDist{ Dot(worldPoint, separatingAxis) };
		aMin = std::fminf(aDist, aMin);
		aMax = std::fmaxf(aDist, aMax);
	} while ((vertex = vertex->next) != hull.mesh.vertices);
	std::ranges::for_each(triangle.vertices, [&](const MFpoint3& vertex)
		{
			MFfloat bDist{ Dot(vertex, separatingAxis) };
			bMin = std::fminf(bDist, bMin);
			bMax = std::fmaxf(bDist, bMax);
		});


	// One-dimensional intersection test between a and b
	MFfloat overlap{ std::fmax(0.0f, std::fminf(aMax, bMax) - std::fmaxf(aMin, bMin)) };

	return overlap == 0.0f;
}