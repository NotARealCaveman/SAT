#pragma once
#include "SAT_Contact.h"

namespace Manifest_Simulation
{ 
	MFbool SAT(const ConvexHull& hull0, const ConvexHull& hull1, SAT_Query& query); 
}