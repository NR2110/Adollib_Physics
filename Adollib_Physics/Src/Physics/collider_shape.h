#pragma once

#include "../Math/math.h"
#include "collider_types.h"

namespace Adollib {
	namespace Physics_function {

		struct Meshcollider_data;
		class ALP_Collider;
	}

	class Collider_shape {
	public:
		Physics_function::Vector3 local_position;
		Physics_function::Quaternion local_orientation;
		Physics_function::Vector3 local_scale;

	protected:
		Physics_function::Vector3	world_position_;
		Physics_function::Quaternion	world_orientation_;
		Physics_function::Vector3	world_scale_;

		Physics_function::Vector3 old_world_position_;
		Physics_function::Quaternion old_world_orientation_;
		Physics_function::Vector3 old_world_scale_;

	public:
		// bool is_update_world_trans = false;

	public:
		const Physics_function::Vector3	world_position()const { return world_position_; };
		const Physics_function::Quaternion	world_orientation()const { return world_orientation_; };
		const Physics_function::Vector3	world_scale()const { return world_scale_; };

		const Physics_function::Vector3	old_world_position()const { return old_world_position_; };
		const Physics_function::Quaternion	old_world_orientation()const { return old_world_orientation_; };
		const Physics_function::Vector3	old_world_scale()const { return old_world_scale_; };

	public:

		// ���ʂ��l�����Ȃ��������[�����g  barycenter : ��]�̒��S
		Physics_function::Matrix33 get_tensor(const Physics_function::Vector3& barycenter = Physics_function::Vector3(0)) {
			Physics_function::Matrix33 tensor;
			//shape�̒��S����]���S�Ƃ����������[�����g
			tensor = local_tensor();

			tensor = local_orientation.get_rotate_matrix() * tensor * matrix_inverse(local_orientation.get_rotate_matrix());

			// barycenter����]���S�Ƃ��� shape��local���W�̕����l�� (���s���̒藝)
			Physics_function::Vector3 dis = local_position - barycenter;
			tensor._11 += (dis.y * dis.y + dis.z * dis.z);
			tensor._22 += (dis.z * dis.z + dis.x * dis.x);
			tensor._33 += (dis.x * dis.x + dis.y * dis.y);

			return tensor;
		};

		// shape�̑̐�
		virtual const float get_volume() const = 0;

	protected:
		// ����shape���ǂ̂悤�Ȍ`�Ȃ̂�
		Physics_function::ALPCollider_shape_type shape_tag = Physics_function::ALPCollider_shape_type::None;

		// DOP
		Physics_function::DOP::DOP_14	dop14;	//DOP �f�[�^

		// meshcollider�p vertexes,Edges,Facets�Ȃǂ̏�� resorce_manager�̒��ɂ�����ւ̃|�C���^
		Physics_function::Meshcollider_data* mesh_data = nullptr;

		// ����shape���A�^�b�`���ꂽcollider�ւ̃C�e���[�^
		Physics_function::ALP_Collider* ALPcollider_ptr = nullptr;

	public:
		const Physics_function::ALPCollider_shape_type get_shape_tag() const { return shape_tag; };

		const Physics_function::DOP::DOP_14 get_DOP() const { return dop14; };

		const Physics_function::Meshcollider_data* get_mesh_data() const { return mesh_data; };

		// TODO : ALPCollider::oncoll_tag��generate_contact�ŕύX����K�v�����邽��const���O���Ă��� �p�C��
		Physics_function::ALP_Collider* get_ALPcollider() { return ALPcollider_ptr; };

		const Physics_function::u_int get_tag() const; //����spahe���A�^�b�`���ꂽ collider : public component ��tag��Ԃ�

		const Physics_function::u_int get_ignore_tags() const;//����spahe���A�^�b�`���ꂽ collider : public component ��ignore_tags��Ԃ�

	public:

		// world���̍X�V
		void update_world_trans(const Physics_function::Vector3& GO_Wposiiton, const Physics_function::Quaternion& GO_Worientation, const Physics_function::Vector3& GO_Wscale);

		// gameobject��transform��ALPcollider�ɃR�s�[����ۂ�shape���^����e��(��{�I�ɂ͂Ȃ�)
		virtual void effect_for_copy_transform_to_collider(Physics_function::Vector3& GO_Wposiiton, Physics_function::Quaternion& GO_Worientation, Physics_function::Vector3& GO_Wscale) {};

		// ALPcollider��transform��gameobject�ɃR�s�[����ۂ�shape���^����e��(��{�I�ɂ͂Ȃ�)
		virtual void effect_for_copy_transform_to_gameobject(const Physics_function::Vector3& position_amount_of_change, const Physics_function::Quaternion& orientation_amount_of_change, const Physics_function::Vector3& position_amount_of_change_local, const Physics_function::Quaternion& orientation_amount_of_change_local) {};

	protected:
		// �eshape�̃��[�U�[�p�̏��(box:center,size)����v�Z�p�̏��(position,scale)�Ɏ���
		virtual void adapt_Colliderdata() = 0;

		// shape�̒��S����]���S�Ƃ����������[�����g
		virtual const Physics_function::Matrix33 local_tensor() const = 0;

	public:
		// �h���N���X�̏������N���X��data�ɒ���
		void update_Colliderdata() {
			adapt_Colliderdata();
		};

		// �q�G�����L�[�ւ̕\��
		virtual void Update_hierarchy(int hierarchy_num = 0) = 0;

		// DOP�̍X�V
		virtual void update_dop14() = 0;


	public:
		virtual ~Collider_shape() {}; //�h���N���X�̎���shaerd_ptr��lef�J�E���g�����炷����
	};



}