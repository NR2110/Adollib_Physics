#include "../../Inc/ray.h"

#include "../../Inc/ALP__physics_manager.h"

using namespace Adollib;
using namespace Physics_function;


bool Ray::ray_cast(Raycast_struct& str) {
	Vector3 normal;
	bool ret = Physics_manager::ray_cast(position, direction, str.collider_tag, str.ray_offset, str.raymin, str.raymax, normal, str.coll);
	str.normal = normal;
	return ret;
}

bool Ray::sphere_cast(const float& radius, DirectX::XMFLOAT3& contact_point, Raycast_struct& str) {
	Vector3 cp, normal;
	bool ret = Physics_manager::sphere_cast(position, direction, radius, cp, str.collider_tag, str.ray_offset, str.raymin, str.raymax, normal, str.coll);
	contact_point = cp;
	str.normal = normal;
	return ret;
}