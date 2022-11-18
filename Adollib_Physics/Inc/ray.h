#pragma once

#include <DirectXMath.h>
#include <limits.h>
#include <memory>
#include <vector>
#include "raycastStruct.h"

namespace Adollib {

	class Collider;

	class Ray {
	public:
		DirectX::XMFLOAT3 position = {};
		DirectX::XMFLOAT3 direction = {};
		unsigned int collider_tag = UINT_MAX;

	public:
		bool ray_cast(Raycast_struct& str);
		bool sphere_cast(const float& radius, Raycast_struct& str);

		bool ray_castAll(std::vector<Raycast_struct>& str);
		bool sphere_castAll(const float& radius, std::vector<Raycast_struct>& str);

	};

}