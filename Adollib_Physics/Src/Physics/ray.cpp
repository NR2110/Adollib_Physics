#include "../../Inc/ray.h"

#include "../../Inc/ALP__physics_manager.h"

using namespace Adollib;
using namespace Physics_function;


bool Ray::ray_cast(Raycast_struct& str) {
	bool ret = Physics_manager::ray_cast(position, direction, collider_tag, str);
	return ret;
}

bool Ray::sphere_cast(const float& radius, Raycast_struct& str) {
	bool ret = Physics_manager::sphere_cast(position, direction, collider_tag, radius, str);
	return ret;
}

bool Ray::ray_castAll(std::vector<Raycast_struct>& str) {
	bool ret = Physics_manager::ray_castAll(position, direction, collider_tag, str);
	return ret;
}

bool Ray::sphere_castAll(const float& radius, std::vector<Raycast_struct>& str) {
	bool ret = Physics_manager::sphere_castAll(position, direction, collider_tag, radius, str);
	return ret;
}