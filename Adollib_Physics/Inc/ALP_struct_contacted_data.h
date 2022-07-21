#pragma once
#include <unordered_map>
#include <string>
#include <DirectXMath.h>

namespace Adollib {
	class Collider;

	struct Contacted_data {
		Collider* coll; //�����collider
		float penetrate = 0; //�ђʗ�
		DirectX::XMFLOAT3 normal; //�Փ˖@�� world���W�n
		DirectX::XMFLOAT3 contacted_pointA; //���g��GO���W�n�̏Փ˓_
		DirectX::XMFLOAT3 contacted_pointB; //�����GO���W�n�̏Փ˓_
	};
}