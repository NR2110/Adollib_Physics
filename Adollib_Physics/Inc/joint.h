#pragma once

#include "balljoint.h"
#include "hingejoint.h"
#include "conejoint.h"
#include "coneTwistjoint.h"
#include "twistjoint.h"

namespace Adollib {
	class Collider;

	class Joint {
	public:
		//Joint�̍폜
		static void delete_joint(Joint_base*&);

		//BallJoint�̐���
		static BallJoint* add_balljoint(
			std::shared_ptr<Collider> colliderA, std::shared_ptr<Collider> colliderB,
			const DirectX::XMFLOAT3& anchorA, const DirectX::XMFLOAT3& anchorB,
			const float& bias = 0.1f
		);

		//HingeJoint�̐���
		static HingeJoint* add_Hingejoint(
			std::shared_ptr<Collider> colliderA, std::shared_ptr<Collider> colliderB,
			const DirectX::XMFLOAT3& anchorA_s, const DirectX::XMFLOAT3& anchorA_g,
			const DirectX::XMFLOAT3& anchorB_s, const DirectX::XMFLOAT3& anchorB_g,
			const float& hingepow = 0.01f,
			const float& bias = 0.1f
		);

		//ConeJoint�̐���
		static ConeJoint* add_Conejoint(
			std::shared_ptr<Collider> colliderA, std::shared_ptr<Collider> colliderB,
			const DirectX::XMFLOAT3& anchorA, const DirectX::XMFLOAT3& anchorB,
			const DirectX::XMFLOAT3& axisA, const DirectX::XMFLOAT3& axisB,
			const float& bias = 0.1f
		);

		//ConeTwistJoint�̐���
		static ConetwistJoint* add_Conetwistjoint(
			std::shared_ptr<Collider> colliderA, std::shared_ptr<Collider> colliderB,
			const DirectX::XMFLOAT3& anchorA, const DirectX::XMFLOAT3& anchorB,
			const DirectX::XMFLOAT3& axisA, const DirectX::XMFLOAT3& axisB,
			const float& bias = 0.1f
		);

		static TwistJoint* add_Twistjoint(
			std::shared_ptr<Collider> colliderA, std::shared_ptr<Collider> colliderB,
			const DirectX::XMFLOAT3& axisA,
			const DirectX::XMFLOAT3& axisB,
			const float& bias = 0.1f
		);

	};



}
