#include "CachedVertex.h"

using namespace Manifest_Math;

//if all values are equal xor, concatenation = 0
MFbool CachedVertex::operator==(const CachedVertex& cachedVertex) const
{	
	return (values[0] ^ cachedVertex.values[0] | values[1] ^ cachedVertex.values[1] | values[2] ^ cachedVertex.values[2]) == 0;
}


CachedVertex Manifest_Math::CreateCachedVertex(const MFpoint3& vertex)
{
	CachedVertex result;
	std::ranges::generate(result.values, [&, axis = 0]() mutable ->const MFu32
		{
			return reinterpret_cast<const MFu32&>(vertex[axis++]);
		});

	return result;
}