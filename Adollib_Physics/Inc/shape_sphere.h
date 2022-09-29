#pragma once
#include "collider_shape.h"

namespace Adollib {
	namespace Physics_function {
		class ALP_Collider;
	}

	//球体用クラス
	class Sphere : public Collider_shape {
	public:
		Physics_function::Vector3 center; //中心座標
		float r = 1; //半径

		Sphere(Physics_function::ALP_Collider* l_ALPcollider_ptr) : center(Physics_function::Vector3(0)), r(1) {
			shape_tag = Physics_function::ALPCollider_shape_type::Sphere;
			ALPcollider_ptr = l_ALPcollider_ptr;
		};

		void adapt_Colliderdata() override {
			local_position = center;
			local_orientation = Physics_function::quaternion_identity();
			local_scale = Physics_function::Vector3(r);
		};

		void initiazlie(Physics_function::Shape_InitData* base) override {
			center = base->center;
			r = base->radius;
		}

		void update_dop14() override {
			dop14.pos = world_position();
			for (int i = 0; i < Physics_function::DOP::DOP_size; i++) {
				dop14.max[i] = +world_scale().x * 1.0000001f;
				dop14.min[i] = -world_scale().x * 1.0000001f;
			}

		};

		const Physics_function::Matrix33 local_tensor() const override {
			const Physics_function::Vector3& Wsize = world_scale();
			Physics_function::Matrix33 ret;

			ret = Physics_function::matrix33_identity();
			ret._11 = 0.4f * Wsize.x * Wsize.x;
			ret._22 = 0.4f * Wsize.x * Wsize.x;
			ret._33 = 0.4f * Wsize.x * Wsize.x;
			return ret;
		};

		const float get_volume() const override {
			return 1.3333333f * DirectX::XM_PI * local_scale.x * local_scale.x * local_scale.x;
		}


	};

}