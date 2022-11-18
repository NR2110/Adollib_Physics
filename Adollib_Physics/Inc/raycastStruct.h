#pragma once

#include <memory>
#include <DirectXMath.h>

namespace Adollib {
	class Collider;

	struct Raycast_struct {
		float raymin = 0;              // Ray‚ÌÕ“Ë‚µ‚½Å¬’l
		float raymax = 0;              // Ray‚ÌÕ“Ë‚µ‚½Å‘å’l
		DirectX::XMFLOAT3 normal;                // Ray‚ÌÕ“Ë‚µ‚½Å‹ß‚ÌêŠ‚Ì–@ü
		std::weak_ptr<Collider> coll;      // Ray‚ÌÕ“Ë‚µ‚½Å‹ß‚ÌCollider
		DirectX::XMFLOAT3 contactPoint;
	};


}