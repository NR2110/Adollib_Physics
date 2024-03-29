#pragma once
#include "collider_shape.h"

namespace Adollib {
	namespace Physics_function {
		class ALP_Collider;
	}

	//平面用クラス
	class Plane : public Collider_shape {
	public:
		Physics_function::Vector3 rotate; //平面の回転
		float distance; //平面の距離

		//不動オブジェクトとして生成
		Plane(Physics_function::ALP_Collider* l_ALPcollider_ptr) :rotate(Physics_function::Vector3(0, 0, 0)), distance(0)
		{
			shape_tag = Physics_function::ALPCollider_shape_type::Plane;
			ALPcollider_ptr = l_ALPcollider_ptr;
		};

		void adapt_Colliderdata() override {

			Physics_function::Quaternion orient = quaternion_from_euler(rotate);
			local_orientation = orient;
			local_position = vector3_quatrotate(Physics_function::Vector3(0,1,0), orient) * distance;
			local_scale = Physics_function::Vector3(FLT_MAX, 0, FLT_MAX);
		}

		void initiazlie(Physics_function::Shape_InitData* base) override {
			rotate = base->rotate;
			distance = base->distance;
		}

		void update_dop14() override {
				dop14.pos = world_position();
				for (int i = 0; i < Physics_function::DOP::DOP_size; i++) {
					dop14.max[i] = FLT_MAX;
				}

		}

		const Physics_function::Matrix33 local_tensor() const override {
			const Physics_function::Vector3& Wsize = world_scale();
			Physics_function::Matrix33 ret;

			ret = Physics_function::matrix33_identity();
			ret._11 = FLT_MAX;
			ret._22 = FLT_MAX;
			ret._33 = FLT_MAX;
			return ret;
		};

		const float get_volume() const override {
			return FLT_MAX;
		}

	};

}