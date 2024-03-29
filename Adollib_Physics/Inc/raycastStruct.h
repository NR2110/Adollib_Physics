#pragma once

#include <memory>
#include <DirectXMath.h>

namespace Adollib {
	class Collider;

	struct Raycast_struct {
		float raymin = 0;              // Rayの衝突した最小値
		float raymax = 0;              // Rayの衝突した最大値
		DirectX::XMFLOAT3 normal;                // Rayの衝突した最近の場所の法線
		std::weak_ptr<Collider> coll;      // Rayの衝突した最近のCollider
		DirectX::XMFLOAT3 contactPoint;
	};


}