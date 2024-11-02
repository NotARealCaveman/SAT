#include "Clipping.h"

using namespace Manifest_Math;

void Manifest_Math::ClipSegments(const MFpoint3(&segment0)[2],  MFpoint3(&segment1)[2])
{
	//calculate clipping planes of first segment
	const MFvec3 normal0{ segment0[0] - segment0[1] };	
	const MFplane plane0{ Normalize(MFplane{ normal0,-Dot(normal0,segment0[0])}) };
	const MFvec3 normal1{ segment0[1] - segment0[0] };
	const MFplane plane1{ Normalize(MFplane{ normal1,-Dot(normal1,segment0[1])}) };
	;// DLOG({ CONSOLE_DEFAULT }, "plane0", plane0, "plane1", plane1);
	//clip vertices of second segment against positive clipping planes
	for (MFu32 vertex{ 0 }; vertex < 2; ++vertex)
	{
		;// DLOG({ CONSOLE_BLUE }, "Clipping vertex:", segment1[vertex]);
		MFfloat projection0{ Dot(plane0, segment1[vertex]) };
		if (Dot(plane0, segment1[vertex]) > 0.0f)
			segment1[vertex] = ClosestPointOnPlane(plane0, segment1[vertex]);
		MFfloat projection1{ Dot(plane1, segment1[vertex]) };
		if (Dot(plane1, segment1[vertex]) > 0.0f)
			segment1[vertex] = ClosestPointOnPlane(plane1, segment1[vertex]);		
	}
}

//iterative implementation of Sutherland-Hodgman
//incident edges are collected and iteratively clipped against each reference side plane
std::vector<MFpoint3> Manifest_Math::Sutherland_Hodgman(const MFplane& sidePlane, const std::vector<MFpoint3>& incidentEdges)
{   
	std::vector<MFpoint3> result;
	
	//cache new vertices into result
	const auto& CacheVertex = [&](const MFpoint3& vertex, std::vector<CachedVertex>& cachedVertices)
	{
		const CachedVertex cachedVertex{ CreateCachedVertex(vertex) };
		if (std::ranges::find(cachedVertices, cachedVertex) != cachedVertices.end())
			return;

		cachedVertices.emplace_back(cachedVertex);
		result.emplace_back(vertex);
	};

	const MFsize totalVertices{ incidentEdges.size() };
	std::vector<CachedVertex> cachedVertices;
	for (MFu32 segmentVertex{ 0 }; segmentVertex < totalVertices; ++segmentVertex)
	{
		MFu32 nextVertex{ (segmentVertex + 1) % totalVertices };

		const MFpoint3& tail{ incidentEdges[segmentVertex] };
		const MFpoint3& head{ incidentEdges[nextVertex] };

		const MFfloat tailProjection{ Dot(sidePlane,tail) };
		const MFfloat headProjection{ Dot(sidePlane,head) };

		//check for negative side skip
		if (tailProjection <= 0.0f && headProjection <= 0.0f)
		{
			CacheVertex(tail, cachedVertices);
			CacheVertex(head, cachedVertices);

			continue;
		}
		//check front side projection
		if (tailProjection > 0.0f && headProjection > 0.0f)
		{
			CacheVertex(ClosestPointOnPlane(sidePlane, tail), cachedVertices);
			CacheVertex(ClosestPointOnPlane(sidePlane, head), cachedVertices);

			continue;
		}
		//determine intersection point
		const MFvec3 direction{ head - tail };
		MFpoint3 intersectionPoint;
		InteresectionRayPlane(tail, direction, sidePlane, intersectionPoint);
		//check for clip head
		if (tailProjection <= 0.0f)
		{
			CacheVertex(tail, cachedVertices);
			CacheVertex(intersectionPoint, cachedVertices);

			continue;
		}
		//clip tail		
		CacheVertex(intersectionPoint, cachedVertices);
		CacheVertex(head, cachedVertices);
		
	}

	return result;
}