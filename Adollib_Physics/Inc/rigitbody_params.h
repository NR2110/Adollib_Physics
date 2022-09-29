#pragma once

namespace Adollib {

	//�\���p��physics_data ���[�U�[���ȒP�ɕύX�ł���悤��
	struct Rigitbody_params {
		float inertial_mass = 0; //����
		float drag = 0; //��C��R
		float anglar_drag = 0; //��C��R
		float dynamic_friction = 0; //�����C
		float static_friction = 0; //�Ö��C
		float restitution = 0;	 //�����W��
		float linear_sleep_threrhold = 0; //freeze��臒l
		float angula_sleep_threrhold = 0; //freeze��臒l

		bool is_fallable = false; // �����Ȃ�
		bool is_kinematic = false;// �e�������Ȃ�(fall�͂���)
		bool is_kinmatic_anglar = false; // �ق��̕��̂���̉e���ŉ�]���x���ω����Ȃ�
		bool is_kinmatic_linear = false; // �ق��̕��̂���̉e���ŕ��i���x���ω����Ȃ�
		bool is_moveable = false; // �����Ȃ�
		bool is_hitable = false;  // �Փ˂��Ȃ�
		bool is_static = false;  // static���m��oncoll_enter���g���Ȃ� ���ǌy���Ȃ�
		bool is_active = true; //false�̎��������s���Ȃ�
	};
	//:::::::::::::::::::::::::
};