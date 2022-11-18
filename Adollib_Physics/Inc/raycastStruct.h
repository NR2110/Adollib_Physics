#pragma once

#include <memory>
#include <DirectXMath.h>

namespace Adollib {
	class Collider;

	struct Raycast_struct {
		float raymin = 0;              // Ray�̏Փ˂����ŏ��l
		float raymax = 0;              // Ray�̏Փ˂����ő�l
		DirectX::XMFLOAT3 normal;                // Ray�̏Փ˂����ŋ߂̏ꏊ�̖@��
		std::weak_ptr<Collider> coll;      // Ray�̏Փ˂����ŋ߂�Collider
		DirectX::XMFLOAT3 contactPoint;
	};


}