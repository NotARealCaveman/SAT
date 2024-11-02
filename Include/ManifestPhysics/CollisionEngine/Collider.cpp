#include "Collider.h"

using namespace Manifest_Simulation;

Collider::CollisionVolume::CollisionVolume(Collider::CollisionVolume&& other) : underlyingGeometry{ other.underlyingGeometry } {}

Collider::Collider(Collider&& other) noexcept
	:collisionVolume{ other.collisionVolume }, physicsID{ other.physicsID }
{
	other.physicsID = -1;
	other.collisionVolume = nullptr;
};

Collider& Collider::operator=(Collider&& other) noexcept
{	
	this->collisionVolume = other.collisionVolume;
	this->physicsID = other.physicsID;

	other.collisionVolume = nullptr;
	other.physicsID = -1;

	return *this;
}

Collider::~Collider()
{
	if (collisionVolume)
	{
		//only Hull collider geoemetries need deconstructors; shall deallocate mesh. all other colliders are stack memory
		delete collisionVolume;
		collisionVolume = nullptr;
	}
};