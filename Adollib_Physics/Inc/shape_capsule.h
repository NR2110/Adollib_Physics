#pragma once
#include "collider_shape.h"
namespace Adollib {
	namespace Physics_function {
		class ALP_Collider;
	}


	//CapsuleNX
	class Capsule : public Collider_shape {
	public:
		Physics_function::Vector3	center;	//SΐW
		Physics_function::Vector3	rotate;	//ρ]
		float r;		//Όa
		float length;	//·³

		Capsule(Physics_function::ALP_Collider* l_ALPcollider_ptr) : center(Physics_function::Vector3(0)), rotate(Physics_function::Vector3(0)), r(1), length(1) {
			shape_tag = Physics_function::ALPCollider_shape_type::Capsule;
			ALPcollider_ptr = l_ALPcollider_ptr;
		};

		void adapt_Colliderdata() override {
			local_position = center;
			local_orientation = Physics_function::quaternion_from_euler(rotate);
			local_scale = Physics_function::Vector3(r, length, r);

		};

		void initiazlie(Physics_function::Shape_InitData* base) override {
			center = base->center;
			rotate = base->rotate;
			r = base->radius;
			length = base->length;
		}

		void update_dop14() override {
			Physics_function::Vector3 p = Physics_function::vector3_quatrotate(Physics_function::Vector3(0, world_scale().y, 0), world_orientation());
			dop14.pos = world_position();
			for (int i = 0; i < Physics_function::DOP::DOP_size; i++) {
				float v0, v1, v2, v3;
				v0 = Physics_function::vector3_dot(+p, Physics_function::DOP::DOP_14_axis[i]) + +world_scale().x * 1.0000001f;
				v1 = Physics_function::vector3_dot(+p, Physics_function::DOP::DOP_14_axis[i]) + -world_scale().x * 1.0000001f;
				v2 = Physics_function::vector3_dot(-p, Physics_function::DOP::DOP_14_axis[i]) + +world_scale().x * 1.0000001f;
				v3 = Physics_function::vector3_dot(-p, Physics_function::DOP::DOP_14_axis[i]) + -world_scale().x * 1.0000001f;
				dop14.max[i] = ALmax(ALmax(v0, v1), ALmax(v2, v3));
				dop14.min[i] = ALmin(ALmin(v0, v1), ALmin(v2, v3));
			}
		};

		const Physics_function::Matrix33 local_tensor() const override {
			const DirectX::XMFLOAT3& Wsize = world_scale();
			Physics_function::Matrix33 ret;

			ret = Physics_function::matrix33_identity();

			//RΝΌa HΝ~Μ³
			//~
			//Mc = PI * r^2 * H
			//X : Mc(1/4 R^2 + 1/12 H^2)
			//Y : 1/2 Mc R^2
			//Z : Mc(1/4 R^2 + 1/12 H^2)
			//[Μ
			//Ms = PI * r^3 * 4/3
			//X : Ms(1/5 R^2 + 1/4 H^2) * 2
			//Y : 2/5 Ms R^2
			//Z : Ms(1/5 R^2 + 1/4 H^2) * 2

			//Wsize.yΝ~Μ³/2ΘΜΕ
			const float H = Wsize.y * 2;
			float Mc = Physics_function::PI * Wsize.x * Wsize.x * H;
			float Ms = Physics_function::PI * Wsize.x * Wsize.x * Wsize.x * 1.3333333f;
			Mc /= Mc + Ms; //ΜΟδΕ\·
			Ms /= Mc + Ms;

			constexpr float _inv2 = 0.5f;
			constexpr float _inv4 = 0.25f;
			constexpr float _2inv4 = 0.5f;
			constexpr float _2inv5 = 0.4f;
			constexpr float _inv12 = 0.08333333f;

			ret._11 = Wsize.x * Wsize.x * (_inv4 * Mc + _2inv5 * Ms) + H * H * (_inv12 * Mc + _2inv4 * Ms);
			ret._22 = Wsize.x * Wsize.x * (_inv2 * Mc + _2inv5 * Ms);
			ret._33 = ret._11;

			return ret;
		};

		const float get_volume() const override {
			const float h = local_scale.x * local_scale.x * DirectX::XM_PI * local_scale.y * 2;
			const float s = 1.3333333f * DirectX::XM_PI * local_scale.x * local_scale.x * local_scale.x;
			return h + s;
		}


	};

}