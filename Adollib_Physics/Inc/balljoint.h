#pragma once

#include "joint_base.h"

namespace Adollib {
	class Collider;

	namespace Physics_function {
		class ALP_Joint;
	}

	// ボールジョイント
	class BallJoint : public Joint_base {
	public:
		BallJoint(std::shared_ptr<Collider> l_colliderA_comp, std::shared_ptr<Collider> l_colliderB_comp, Physics_function::ALP_Joint* l_ALPjoint)
			: Joint_base(l_colliderA_comp, l_colliderB_comp, l_ALPjoint) {}

		Anchor anchor;

	public:
		void adapt_anchor() override {
			anchor_count = 1;

			anchors[0] = anchor;
		}

		DirectX::XMFLOAT3X3 tensor_effect() const override {
			return DirectX::XMFLOAT3X3(0,0,0,0,0,0,0,0,0);
		}

		void velocity_effect() const override {};

		bool limit_effect(DirectX::XMFLOAT3& contactP0, DirectX::XMFLOAT3& contactP1, float& penetrate) const override {
			return false;
		}


	};

};
