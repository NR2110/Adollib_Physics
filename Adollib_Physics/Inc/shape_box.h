#pragma once
#include "collider_shape.h"

namespace Adollib {
	//Boxクラス
	class Box : public Collider_shape {
	public:
		Physics_function::Vector3	center;//中心座標
		Physics_function::Vector3	rotate;//回転
		Physics_function::Vector3	size;//大きさ

		Box(Physics_function::ALP_Collider* l_ALPcollider_ptr) : center(Physics_function::Vector3(0)), rotate(Physics_function::Vector3(0)), size(Physics_function::Vector3(1,1,1)) {
			shape_tag = Physics_function::ALPCollider_shape_type::BOX;
			ALPcollider_ptr = l_ALPcollider_ptr;
		};

		void adapt_Colliderdata() override {
			local_position = center;
			local_orientation = quaternion_from_euler(rotate);
			local_scale = size;

		};

		void initiazlie(Physics_function::Shape_InitData* base) override {
			center = base->center;
			rotate = base->rotate;
			size = base->size;
		}


		void update_dop14() override {
			dop14.pos = world_position();

			//各頂点のローカル座標
			Physics_function::Vector3 half[4]{
			{+world_scale().x, -world_scale().y, -world_scale().z},
			{+world_scale().x, -world_scale().y, +world_scale().z},
			{+world_scale().x, +world_scale().y, -world_scale().z},
			{+world_scale().x, +world_scale().y, +world_scale().z}
			};

			Physics_function::Quaternion WO = world_orientation();
			for (int i = 0; i < 4; i++) {
				half[i] = vector3_quatrotate(half[i], WO);
			}

			//DOPの更新
			float max_len = 0;
			for (int i = 0; i < Physics_function::DOP::DOP_size; i++) {
				max_len = 0;
				for (int o = 0; o < 4; o++) {
					float dis = fabsf(vector3_dot(Physics_function::DOP::DOP_14_axis[i], half[o]));
					if (max_len < dis) {
						dop14.max[i] = +dis * 1.00000001f;//確実にするためちょっと大きめにとる
						dop14.min[i] = -dis * 1.00000001f;//確実にするためちょっと大きめにとる
						max_len = dis;
					}
				}

			}
		};

		const Physics_function::Matrix33 local_tensor() const override {
			const Physics_function::Vector3& Wsize = world_scale();
			Physics_function::Matrix33 ret;

			ret = Physics_function::matrix33_identity();
			ret._11 = 0.3333333f * ((Wsize.y * Wsize.y) + (Wsize.z * Wsize.z));
			ret._22 = 0.3333333f * ((Wsize.z * Wsize.z) + (Wsize.x * Wsize.x));
			ret._33 = 0.3333333f * ((Wsize.x * Wsize.x) + (Wsize.y * Wsize.y));
			return ret;
		};

		const float get_volume() const override {
			return local_scale.x * local_scale.y * local_scale.z;
		}


	};





}