#pragma once

namespace Adollib {

	//表示用のphysics_data ユーザーが簡単に変更できるように
	struct Rigitbody_params {
		float inertial_mass = 0; //質量
		float drag = 0; //空気抵抗
		float anglar_drag = 0; //空気抵抗
		float dynamic_friction = 0; //動摩擦
		float static_friction = 0; //静摩擦
		float restitution = 0;	 //反発係数
		float linear_sleep_threrhold = 0; //freezeの閾値
		float angula_sleep_threrhold = 0; //freezeの閾値

		bool is_fallable = false; // 落ちない
		bool is_kinematic = false;// 影響うけない(fallはする)
		bool is_kinmatic_anglar = false; // ほかの物体からの影響で回転速度が変化しない
		bool is_kinmatic_linear = false; // ほかの物体からの影響で並進速度が変化しない
		bool is_moveable = false; // 動かない
		bool is_hitable = false;  // 衝突しない
		bool is_static = false;  // static同士はoncoll_enterが使えない けど軽くなる
		bool is_active = true; //falseの時処理が行われない
	};
	//:::::::::::::::::::::::::
};