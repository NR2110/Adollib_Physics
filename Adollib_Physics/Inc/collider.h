#pragma once

#include <vector>
#include <assert.h>
#include <string>
#include <unordered_map>
#include <DirectXMath.h>

#include "id_struct.h"
#include "collider_shape.h"
#include "ALP_struct_contacted_data.h"
#include "rigitbody_params.h"

namespace Adollib {
	class Joint_base;

	namespace Physics_function {
		class ALP_Joint;
		class ALP_Collider;
		class ALP_Physics;
		namespace Contacts {
			struct Contact_pair;
		}
	}


	class Collider {

	public:
		//::: tag ::::::::
		unsigned int tag = 0; //自身のtag(bit)
		unsigned int ignore_tags = 0; //衝突しないtags(bit)

		//::: unityのphysics部分 分ける必要なんてないやろ ::::
		Rigitbody_params physics_data;

		//::: 自身の関わるcontact_pairの情報をメンバに保存するかどうか :::
		bool is_save_contacted_colls = false;

		//::: transform(落下方向はworld方向なので このColliderの親情報のinverseを使う)
		DirectX::XMFLOAT3 Wposition;
		DirectX::XMFLOAT4 Worientation;
		DirectX::XMFLOAT3 Wscale;
		DirectX::XMFLOAT4 pearent_Worientation_inverse;

	private:
		Physics_function::ALP_Physics*  ALPphysics_ptr = nullptr;
		Physics_function::ALP_Collider* ALPcollider_ptr = nullptr;

		int a = 0;

	public:
		//::: 後で変更する :::
		const DirectX::XMFLOAT3 linear_velocity() const;
		const DirectX::XMFLOAT3 angula_velocity() const;
		const void linear_velocity(DirectX::XMFLOAT3 v);
		const void angula_velocity(DirectX::XMFLOAT3 v);
		// 指定した一点での速度
		const DirectX::XMFLOAT3 get_point_velocity(const DirectX::XMFLOAT3& pos, bool is_local = false);

		// アタッチされたjointの数
		const int get_joint_count();
		// 指定した番号にアタッチされているjointの情報を得る
		Joint_base* get_joint(const int num);

		const Physics_ID get_UUID() const;

	public:
		// jointに自身の保持するALPColliderの情報を入れる
		void set_ptr_to_joint(Physics_function::ALP_Joint* joint_base);

	public:
		// 並進移動に力を加える
		void add_force(const DirectX::XMFLOAT3& force, const float& delta_time, const bool& is_force_local = false);

		// 並進移動に力を加える
		void add_force(const DirectX::XMFLOAT3& force, const DirectX::XMFLOAT3& position, const float& delta_time, const bool& is_position_local = false, const bool& is_force_local = false);

		// 角回転に力を加える
		void add_torque(const DirectX::XMFLOAT3& force, const float& delta_time, const bool& is_local = false);

		// 並進加速に値を加える
		void add_linear_acc(const DirectX::XMFLOAT3& force, const float& delta_time);

		// 角加速に値を加える
		void add_angula_acc(const DirectX::XMFLOAT3& force, const float& delta_time);

		// 現在かかっている速度、加速度、力を0にする
		void reset_force();

		// 速度制限を行う
		void set_max_linear_velocity(const float& max_scalar);
		void set_max_angula_velocity(const float& max_scalar);

		// shapeのアタッチ
		template<typename T>
		T* add_shape(Physics_function::Shape_InitData* basedata = nullptr) {
			static_assert(std::is_base_of<Collider_shape, T>::value == true, "template T must inherit Collider_shape");

			T* shape = newD T(ALPcollider_ptr);

			auto collider_shape = static_cast<Collider_shape*>(shape);
			if (basedata != nullptr) collider_shape->initiazlie(basedata);

			add_shape(collider_shape);

			return shape;
		};
	private:
		void add_shape(Collider_shape* shape);

	public:

		// meshcolliderのアタッチ
		//void add_mesh_shape(const char* filepass, bool is_right_rtiangle = true, bool is_permit_edge_have_many_facet = false);

		// 慣性モーメントをユーザー定義で設定する
		void set_tensor(const DirectX::XMFLOAT3X3& tensor);

		// 重心をユーザー定義で設定する
		void set_barycenter(const DirectX::XMFLOAT3& cent);

		// 現在の慣性モーメントの値
		const DirectX::XMFLOAT3X3 get_tensor();

		// 重心のlocal座標を返す
		const DirectX::XMFLOAT3 get_barycenter() const;

		// 衝突したcolliderの配列を返す is_save_contacted_collsがtrueになっていないと衝突したcolliderの情報は保存されない
		std::vector<Contacted_data> get_Contacted_data() const;

	public:

		//:::::::::
		// ID : ユニークな値ならなんでもOK 同じ値を持つものとは衝突しない
		// Wpos, Worient, Wscale : world transform の初期値
		// pearent_Worient_inv : 落下方向はWorld座標系での落下方向なので 変化量の算出時に親の回転のinvをかける
		//:::::::::
		void awake(std::weak_ptr<Collider> this_ptr, const Physics_ID ID, const DirectX::XMFLOAT3& Wpos, const DirectX::XMFLOAT4& Worient, const DirectX::XMFLOAT3& Wscale, const DirectX::XMFLOAT4& pearent_Worient_inv, bool is_use_defaultrigitbodyparam = true);

		void update();

		void update_Wtransform(const DirectX::XMFLOAT3& Wpos, const DirectX::XMFLOAT4& Worient, const DirectX::XMFLOAT3& Wscale, const DirectX::XMFLOAT4& pearent_Worient_inv);

		void Update_hierarchy();

		void finalize();

	};
}

#include "shape_box.h"
#include "shape_plane.h"
#include "shape_sphere.h"
#include "shape_capsule.h"
#include "shape_meshcoll.h"