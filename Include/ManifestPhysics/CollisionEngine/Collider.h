#pragma once
#include <ManifestUtility/TypeAssist.h>
 
#include "Colliders/ConvexHull.h" 


namespace Manifest_Simulation
{
	enum class UNDERLYING_GEOMETRY : MFu8
	{ 
		HULL = Manifest_Utility::pow2(2), 
		NO_GEOMETRY = 0
	};

	template<typename T>
	constexpr UNDERLYING_GEOMETRY SelectType(T)
	{		
		if constexpr (std::is_same_v<T, ConvexHull>)
			return UNDERLYING_GEOMETRY::HULL;
		else if constexpr (!std::is_same_v<T, ConvexHull>)
			return UNDERLYING_GEOMETRY::NO_GEOMETRY;
	}

	class Collider
	{
	private:
		struct CollisionVolume
		{
			CollisionVolume(const CollisionVolume& other) = delete;
			CollisionVolume(const UNDERLYING_GEOMETRY _underlyingGeometry) : underlyingGeometry{ _underlyingGeometry } {};
			CollisionVolume(CollisionVolume&& other);
			virtual ~CollisionVolume() {};
			const UNDERLYING_GEOMETRY underlyingGeometry;
		};
		template<typename Geometry>
		struct ColliderGeometry : CollisionVolume
		{
			ColliderGeometry(const ColliderGeometry& other) = delete;
			ColliderGeometry(const Geometry& _geometry)
				: CollisionVolume{ SelectType(_geometry) }, geometry{ _geometry } {
			};
			ColliderGeometry(ColliderGeometry&& other)
				: collisionVolume{ std::move(other.collisionVolume) }
			{
				other.collisionVolume = nullptr;
			};
			~ColliderGeometry() override
			{ 
				if constexpr (std::is_same_v<Geometry, ConvexHull>)
					DeallocateBuffers(geometry.mesh.vertexBuffer,geometry.mesh.edgeBuffer,geometry.mesh.faceBuffer);
			}
			Geometry geometry;
		};		
		Collider(const Collider& other) = delete;
	public:
		Collider() = default;
		template<typename Geometry>
		Collider(const Geometry& _geometry)
			:collisionVolume{ new ColliderGeometry<Geometry>	{_geometry} } {}
		Collider(Collider&& other) noexcept;
		Collider& operator=(Collider&& other) noexcept;
		~Collider();
		template<typename Geometry>
		void SetCollider(const Geometry& _geometry)
		{
			if (!collisionVolume)
				collisionVolume = new ColliderGeometry<Geometry>{ _geometry };
		}
		template<typename Geometry>
		inline Geometry& GetColliderGeometry() const
		{
			return static_cast<ColliderGeometry<Geometry>&>(*collisionVolume).geometry;
		}

		const MFpoint3& Position() const
		{
			switch (collisionVolume->underlyingGeometry)
			{ 
				case UNDERLYING_GEOMETRY::HULL:
					return static_cast<ColliderGeometry<ConvexHull>&>(*collisionVolume).geometry.worldSpace.GetTranslation(); 

				default:
					return { 0 };
			}
		}

		CollisionVolume* collisionVolume{ nullptr };
		MFu64 physicsID;
	};
}