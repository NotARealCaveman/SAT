#include "SAT_Contact.h"

using namespace Manifest_Simulation;

FaceContact Manifest_Simulation::CreateFaceContact(const FaceQuery& faceQuery, const ConvexHull& referenceHull, const ConvexHull& incidentHull)
{
	FaceContact result;	
	result.query = faceQuery;

	const MFtransform inverseReferenceWorld{ Inverse(referenceHull.worldSpace) };
	const MFtransform inverseIncidentWorld{ Inverse(incidentHull.worldSpace) };	

	const auto& FindIncidentFace = [&](const MFplane& referencePlane)-> HullFace const *const
	{
		HullFace const * result{ nullptr };		
		MFfloat currentPerpendicularProjection{ std::numeric_limits<MFfloat>::max() };
		HullFace const *const start{ incidentHull.mesh.faces };
		HullFace const * potentialFace{ start };
		do
		{
			const MFplane potentialPlane{ potentialFace->facePlane * inverseIncidentWorld };
			const MFfloat projection{ Dot(referencePlane.Normal(),potentialPlane.Normal()) };
			if (projection < currentPerpendicularProjection)
			{
				currentPerpendicularProjection = projection;
				result = potentialFace;
			}
		} while ((potentialFace = potentialFace->next) != start);

		return result;
	};

	//find most anti parallel face to reference hull on incident hull
	HullFace const* const referenceFace{ faceQuery.face };
	const MFplane referencePlane{ referenceFace->facePlane * inverseReferenceWorld };
	HullFace const *const incidentFace { FindIncidentFace(referencePlane) };
	
	//collect incident edge vertices
	const MFtransform IncidentWorldSpace{ incidentHull.worldSpace };
	std::vector<MFpoint3> incidentEdges;
	incidentEdges.reserve(incidentFace->vertexCount );
	{
		HullHalfEdge const* const start{ incidentFace->edge };
		HullHalfEdge const* edge{ start };
		do
		{
			const MFpoint3 localPoint{ ComponentMultiply(incidentHull.scale,edge->tail->vertex) };
			incidentEdges.emplace_back(IncidentWorldSpace * localPoint);
		} while ((edge = edge->next) != start);
	}; 
	//for (const auto& p : incidentEdges)
		//DLOG({ CONSOLE_CYAN }, "Incident Vertices", p);

	//clip incident edges against side planes of reference face
	HullHalfEdge const* const start{ referenceFace->edge };
	HullHalfEdge const* edge{ start };
	do
	{
		HullFace const* const referenceSideFace{ edge->face != referenceFace ? edge->face : edge->twin->face };
		const MFplane referenceSidePlane{ referenceSideFace->facePlane * inverseReferenceWorld };
		//DLOG({ CONSOLE_BOLD,CONSOLE_BG_RED }, "reference side plane:", (referenceSidePlane));
		incidentEdges = Sutherland_Hodgman(referenceSidePlane, incidentEdges);
	} while ((edge = edge->next) != start);
	//for (const auto& p : incidentEdges)
		//DLOG({ CONSOLE_MAGENTA }, "clipped Vertices", p);

	//move clipped incident vertices onto clip plane and generate initial face contact points
	//DLOG({ CONSOLE_DEFAULT }, "projecting vertices onto plane:", referencePlane);
	std::ranges::transform(incidentEdges, incidentEdges.begin(),
		[&](const MFpoint3& point)->MFpoint3
		{ return ClosestPointOnPlane(referencePlane,point ); });
	result.vertices = std::move(incidentEdges);
	//for (const auto& p : result.vertices)
		//DLOG({ CONSOLE_YELLOW}, "projected Vertices", p);
	//refine incident vertices for final face contact points
	RefineContactPoints(incidentHull.scale, result);
	//for (const auto& p : result.vertices)
		//DLOG({ CONSOLE_BLUE}, "refined Vertices", p);

	//kinda suspect but lets leave it for now. i cant see anything obviously missing	
	const MFplane incidentPlane{ incidentFace->facePlane * inverseIncidentWorld };
	result.vertices.erase(std::remove_if(result.vertices.begin(), result.vertices.end(), [&](MFpoint3& contactPoint)
		{
			return Dot(incidentPlane, contactPoint) > -0.001f;
		}), result.vertices.end());
		

	return result;
} 

void Manifest_Simulation::RefineContactPoints(const MFvec3& incidentScale, FaceContact& faceContact)
{
	//fine for now - could change later if needed(ex: CCD)
	const MFvec3 INITIAL_SEARCH_DIRECTION{ 0.5574 };//Nrm([1,1,1])
		
	std::vector<MFpoint3> refinedPoints(4); 
	std::vector<MFfloat> refinedDistances
	{
		-std::numeric_limits<MFfloat>::max(),
		-std::numeric_limits<MFfloat>::max(),
		0,0
	};
	//first point - fixed search direction
	for (const MFpoint3& potentialVertex : faceContact.vertices)
	{
		const MFfloat projection{ Dot(INITIAL_SEARCH_DIRECTION,potentialVertex) };
		if (projection > refinedDistances[0])
		{
			refinedPoints[0] = potentialVertex;
			refinedDistances[0] = projection;
		}
	}
	//second point - largest distance from point 1
	for (const MFpoint3& potentialVertex : faceContact.vertices)
	{
		const MFvec3 searchDirection{ potentialVertex - refinedPoints[0] };
		const MFfloat projection{ Dot(searchDirection,searchDirection) };
		if (projection > refinedDistances[1])
		{
			refinedPoints[1] = potentialVertex;
			refinedDistances[1] = projection;
		}
	}
	HullFace const* const hullFace{ faceContact.query.face };
	const MFvec3 faceNormal{ hullFace->facePlane.Normal() };
	//third point - triangle area maximizing point 
	for (const MFpoint3& potentialVertex : faceContact.vertices)
	{
		const MFvec3 triangleNormal{ Cross(refinedPoints[0] - potentialVertex,refinedPoints[1] - potentialVertex) };
		const MFfloat windingArea{ Dot(triangleNormal, faceNormal) };
		//positive winding is CCW 
		if (windingArea > refinedDistances[2])
		{
			refinedPoints[2] = potentialVertex;
			refinedDistances[2] = windingArea;
		}
	}
	//fourth point - quadrilateral area maximizing point
	for (const MFpoint3& potentialVertex : faceContact.vertices)
	{
		const MFvec3 normalABQ{ Cross(refinedPoints[0] - potentialVertex,refinedPoints[1] - potentialVertex) };
		const MFfloat windingAreaABQ{ Dot(normalABQ, faceNormal) };
		//negative is CW
		if (windingAreaABQ < refinedDistances[3])
		{
			refinedPoints[3] = potentialVertex;
			refinedDistances[3] = windingAreaABQ;
		}
		//test BCQ
		const MFvec3 normalBCQ{ Cross(refinedPoints[0] - potentialVertex,refinedPoints[1] - potentialVertex) };
		const MFfloat windingAreaBCQ{ Dot(normalBCQ, faceNormal) };
		if (windingAreaBCQ < refinedDistances[3])
		{
			refinedPoints[3] = potentialVertex;
			refinedDistances[3] = windingAreaBCQ;
		}
		//test CAQ
		const MFvec3 normalCAQ{ Cross(refinedPoints[0] - potentialVertex,refinedPoints[1] - potentialVertex) };
		const MFfloat windingAreaCAQ{ Dot(normalCAQ, faceNormal) };
		if (windingAreaCAQ < refinedDistances[3])
		{
			refinedPoints[3] = potentialVertex;
			refinedDistances[3] = windingAreaCAQ;
		}
	}	
	//update face contact with refined vertices
	faceContact.vertices = std::move(refinedPoints);	
}

ContactManifold* Manifest_Simulation::ConvertFaceContact(const MFvec3& referenceHullScale, const FaceContact& contact, std::vector<ContactManifold>& contactManifolds)
{
	ContactManifold& result{ contactManifolds.emplace_back() };
		
	HullFace const* const hullFace{contact.query.face };
	result.normal = hullFace->facePlane.Normal();
	ContactPoint contactPoint;
	contactPoint.interpenetration = contact.query.distance;
	for (const MFpoint3& vertex : contact.vertices)
	{
		contactPoint.collisionPointWorldSpace = vertex;
		result.contactPoints.emplace_back(contactPoint);
	}

	return &result;
}

EdgeContact Manifest_Simulation::CreateEdgeContact(const EdgeQuery& edgeQuery, const ConvexHull& hull0, const ConvexHull& hull1)
{
	EdgeContact result;
	result.query = edgeQuery;

	//compute closest points between two edges	
	const MFpoint3 edgeA0{ hull0.worldSpace * ComponentMultiply(hull0.scale,edgeQuery.edgeA->tail->vertex) };
	const MFpoint3 edgeA1{ hull0.worldSpace * ComponentMultiply(hull0.scale,edgeQuery.edgeA->twin->tail->vertex) };
	//variant is half edge
	HullHalfEdge const* const edgeB{edgeQuery.edgeB };	
	const MFpoint3 edgeB0{ hull1.worldSpace * ComponentMultiply(hull1.scale,edgeB->tail->vertex) };
	const MFpoint3 edgeB1{ hull1.worldSpace * ComponentMultiply(hull1.scale,edgeB->twin->tail->vertex) };


	MFpoint3 L0, L1;
	MFfloat s, t;
	ClosestPointsSegmentSegment(edgeA0, edgeA1, edgeB0, edgeB1, L0, L1, s, t);
	//compute center of closest points as contact point
	result.midPoint = (L0 + L1) * 0.5f;
	//compute separating axis between two edges
	result.separationAxis = Normalize(L0 - L1);
	if (Dot(result.separationAxis, edgeA0 - hull0.worldSpace.GetTranslation()) < 0.0f)
		result.separationAxis = -result.separationAxis;
	return result;
};

ContactManifold* Manifest_Simulation::ConvertEdgeContact(const EdgeContact& contact, std::vector<ContactManifold>& contactManifolds)
{
	ContactManifold& result{ contactManifolds.emplace_back() };

	ContactPoint contactPoint;
	contactPoint.collisionPointWorldSpace = contact.midPoint;
	contactPoint.interpenetration = contact.query.distance;

	result.normal = contact.separationAxis;
	result.contactPoints.emplace_back(contactPoint);

	return &result;
}
