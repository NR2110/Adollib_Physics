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

		Physics_function::Matrix33 tensor_effect() const override {
			return Physics_function::matrix33_zero();
		}

		void velocity_effect() const override {};

		bool limit_effect(Physics_function::Vector3& contactP0, Physics_function::Vector3& contactP1, float& penetrate) const override {
			return false;
		}


	};

};
