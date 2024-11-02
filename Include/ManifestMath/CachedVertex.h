#pragma once
#include <array>
#include <ranges>

#include <ManifestMath/Point3.h>

namespace Manifest_Math
{
	struct CachedVertex 
	{
		MFbool operator==(const CachedVertex& cachedVertex) const;		

		std::array<MFu32, 3> values;		
	};
	CachedVertex CreateCachedVertex(const MFpoint3& vertex);	

	template<typename T>
	inline MFsize hash_combine(const T& value, MFsize seed)
	{
		return seed ^= std::hash<T>{}(value)+0x9e3779b9 + (seed << 6) + (seed >> 2);
	}
}

namespace std
{
	template<>
	struct hash<Manifest_Math::CachedVertex>
	{
		size_t operator()(const Manifest_Math::CachedVertex& cachedVertex) const
		{
			size_t hashValue{ 0XDEADBEEF };
			std::ranges::for_each(cachedVertex.values, [&](const MFu32& value)
				{
					hashValue = Manifest_Math::hash_combine(value, hashValue);
				});
			return hashValue;
		}
	};
}