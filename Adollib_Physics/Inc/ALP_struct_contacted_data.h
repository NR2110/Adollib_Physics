#pragma once
#include <unordered_map>
#include <string>
#include <DirectXMath.h>

namespace Adollib {
	class Collider;

	struct Contacted_data {
		Collider* coll; //相手のcollider
		float penetrate = 0; //貫通量
		DirectX::XMFLOAT3 normal; //衝突法線 world座標系
		DirectX::XMFLOAT3 contacted_pointA; //自身のGO座標系の衝突点
		DirectX::XMFLOAT3 contacted_pointB; //相手のGO座標系の衝突点
	};
}