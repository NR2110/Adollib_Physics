#pragma once

#include <DirectXMath.h>
#include <limits.h>

namespace Adollib {

	class Collider;

	class Ray {
	public:
		DirectX::XMFLOAT3 position;
		DirectX::XMFLOAT3 direction;

		struct Raycast_struct {
			unsigned int collider_tag = UINT_MAX; // �Փ˂���tag
			float ray_offset = 0;          // Raymin�̍ŏ��̒l
			float raymin = 0;              // Ray�̏Փ˂����ŏ��l
			float raymax = 0;              // Ray�̏Փ˂����ő�l
			DirectX::XMFLOAT3 normal;                // Ray�̏Փ˂����ŋ߂̏ꏊ�̖@��
			Collider* coll = nullptr;      // Ray�̏Փ˂����ŋ߂�Collider
		};
	public:
		bool ray_cast(Raycast_struct& str);

		bool sphere_cast(const float& radius, DirectX::XMFLOAT3& contact_point, Raycast_struct& str);
		bool sphere_cast(const float& radius, Raycast_struct& str) {
		DirectX::XMFLOAT3 cp;
		return sphere_cast(radius, cp, str);
		};

	};

}