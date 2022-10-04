#include "ALP_physics.h"

#include "../../Inc/ALP__physics_manager.h"

#include "ALP_collider.h"
#include "ALP_joint.h"


using namespace Adollib;
using namespace Physics_function;

void ALP_Physics::add_force(const Vector3& force, const bool& is_force_local) {
	if (transform == nullptr)return;
	std::lock_guard <std::mutex> lock(mtx);

	if (is_force_local)accumulated_force += vector3_quatrotate(force, transform->orientation);
	else accumulated_force += force;

}

void ALP_Physics::add_force(const Vector3& force, const Vector3& position, const bool& is_position_local, const bool& is_force_local) {
	if (transform == nullptr)return;
	std::lock_guard <std::mutex> lock(mtx);

	//Vector3 local_position;
	//Vector3 local_force;
	//if (is_force_local)local_force = force;
	//else local_force = vector3_quatrotate(force, transform->orientation.inverse());

	//if (is_position_local)local_position = position;
	//else local_position = vector3_quatrotate(position - transform->position, transform->orientation.inverse());


	//accumulated_force += local_force;
	//accumulated_torque += vector3_cross(local_position, local_force.unit_vect()) * local_force.norm_sqr();

	Vector3 world_position;
	Vector3 world_force;
	if (is_force_local)world_force = vector3_quatrotate(force, transform->orientation);
	else world_force = force;

	if (is_position_local)world_position = vector3_quatrotate(position, transform->orientation);
	else world_position = position - transform->position;

	accumulated_force += world_force;
	accumulated_torque += vector3_cross(world_position, world_force.unit_vect()) * world_force.norm_sqr();

}

void ALP_Physics::add_torque(const Vector3& force, const bool& is_local) {
	if (transform == nullptr)return;
	std::lock_guard <std::mutex> lock(mtx);

	if(is_local)accumulated_torque += vector3_quatrotate(force, transform->orientation);
	else accumulated_torque += force;
}
void ALP_Physics::add_linear_acc(const Vector3& acc) {
	std::lock_guard <std::mutex> lock(mtx);

	linear_acceleration += acc;
}
void ALP_Physics::add_angula_acc(const Vector3& acc) {
	std::lock_guard <std::mutex> lock(mtx);

	angula_acceleration += acc;
}

bool ALP_Physics::is_movable() const {
	return (is_moveable && inertial_mass > 0 && inertial_mass < FLT_MAX);
}

float ALP_Physics::inverse_mass() const {
	if (is_movable()) return 1.0f / inertial_mass;
	//非可動オブジェクトなら0を返す
	else return 0;
}

Matrix33 ALP_Physics::inverse_inertial_tensor() const {
	if (transform == nullptr)return matrix33_zero();
	Matrix33 inverse_inertial_tensor;

	if (is_movable()) {
		inverse_inertial_tensor = matrix_inverse(inertial_tensor_);

		if (isnan(matrix_determinant(inverse_inertial_tensor))) assert(0 && "toooooo laaarrrrrrrrgge");

		Matrix33 rotation, transposed_rotation;
		rotation = transform->orientation.get_rotate_matrix();
		transposed_rotation = matrix_trans(rotation);
		inverse_inertial_tensor = rotation * inverse_inertial_tensor * transposed_rotation;
	}
	else {
		inverse_inertial_tensor = matrix33_identity();

		inverse_inertial_tensor._11 = 0;
		inverse_inertial_tensor._22 = 0;
		inverse_inertial_tensor._33 = 0;
	}
	return inverse_inertial_tensor;
}

void ALP_Physics::reset_force() {
	std::lock_guard <std::mutex> lock(mtx);

	linear_velocity_ = Vector3(0, 0, 0);
	angula_velocity_ = Vector3(0, 0, 0);

	linear_velocity_buffer_ = Vector3(0, 0, 0);
	angula_velocity_buffer_ = Vector3(0, 0, 0);

	accumulated_force = Vector3(0, 0, 0);
	accumulated_torque = Vector3(0, 0, 0);

	linear_acceleration = Vector3(0, 0, 0);
	angula_acceleration = Vector3(0, 0, 0);
}

void ALP_Physics::copy_transform_ptr() {
	transform = &ALPcollider->transform;
}

void ALP_Physics::apply_external_force(float duration, const float timeratio_60) {
	if (transform == nullptr)return;
	if (is_active == false)return;

	std::lock_guard <std::mutex> lock(mtx);

	if (is_movable()) {
		linear_velocity_ = linear_velocity_buffer_;
		angula_velocity_ = angula_velocity_buffer_;

		// 力を所定の秒数のの量に直す
		//accumulated_force *= timeratio_60;
		//accumulated_torque *= timeratio_60;

		linear_velocity_ = linear_velocity_ * pow(1 - linear_drag, duration);
		angula_velocity_ = angula_velocity_ * pow(1 - angula_drag, duration);

		const float inv_mass = 1 / inertial_mass;

		//空気抵抗の求め方
		// k は流体の密度やらなんやらを考慮した定数
		//F(t) = 1/2 * k * v(t)
		//F(t) = m * a(t)
		//a(t) = dv/dt = - * k * v(t) / m
		//∫ 1 / v(t) * dv/dt * dt = ∫ -k / m * dt
		//ln v(t) = -k / m * t + C
		//v(t) = exp(-k / m * t + C)
		//v(t) = C´ * exp(-k / m * t)
		//t=0の時 C´ = V(0)より
		//v(t) = V(0) * exp(-k / m * t)
		//const float kl = linear_drag * inv_mass; //空気抵抗やらなんやらを考慮した値 のはずだけど適当に簡略化
		//_linear_velocity = _linear_velocity * exp(-kl * duration); // 空気抵抗

		//const float ka = angula_drag * inv_mass; //空気抵抗やらなんやらを考慮した値 のはずだけど適当に簡略化
		//_angula_velocity = _angula_velocity * exp(-ka * duration); // 空気抵抗

		linear_acceleration /= duration;

		//並進移動に加える力(accumulated_force)から加速度を出して並進速度を更新する 向きを間違えないように!!
		linear_acceleration += accumulated_force / duration * inv_mass;

		if (is_fallable) linear_acceleration += Vector3(0, -Physics_manager::physicsParams.gravity, 0); //落下

		linear_velocity_ += linear_acceleration * duration;

		//各回転に加える力(accumulated_torque)から加速度を出して角速度を更新する
		Matrix33 inverse_inertia_tensor = matrix_inverse(inertial_tensor_);
		const Matrix33 rotation = transform->orientation.get_rotate_matrix();
		const Matrix33 transposed_rotation = matrix_trans(rotation);
		inverse_inertia_tensor = rotation * inverse_inertia_tensor * transposed_rotation;
		angula_acceleration += vector3_trans(accumulated_torque / duration, inverse_inertia_tensor);

		angula_velocity_ += angula_acceleration * duration;
		if (angula_velocity_.norm() < FLT_EPSILON)angula_velocity_ = Vector3(0, 0, 0);

		if (linear_velocity_.norm() > max_linear_velocity * max_linear_velocity) linear_velocity_ = linear_velocity_.unit_vect() * max_linear_velocity;
		if (angula_velocity_.norm() > max_angula_velocity * max_angula_velocity) angula_velocity_ = angula_velocity_.unit_vect() * max_angula_velocity;


		linear_velocity_buffer_ = linear_velocity_;
		angula_velocity_buffer_ = angula_velocity_;
	}
	else {
		linear_velocity_ = Vector3(0, 0, 0);
		angula_velocity_ = Vector3(0, 0, 0);
		linear_velocity_buffer_ = Vector3(0, 0, 0);
		angula_velocity_buffer_ = Vector3(0, 0, 0);
	}

	//加速を0にする
	accumulated_force = Vector3(0, 0, 0);
	accumulated_torque = Vector3(0, 0, 0);

	linear_acceleration = Vector3(0, 0, 0);
	angula_acceleration = Vector3(0, 0, 0);
}
void ALP_Physics::integrate(float duration) {
	std::lock_guard <std::mutex> lock(mtx); //linear_velocity_ = Vector3(0)があるため

	if ((is_movable() && is_active) == false)return;

	//float threrhold_pow = (duration * 60) * (duration * 60);
	float threrhold_pow = 1;

	if (linear_velocity_.norm() < linear_sleep_threrhold * linear_sleep_threrhold * threrhold_pow &&
		angula_velocity_.norm() < angula_sleep_threrhold * angula_sleep_threrhold * threrhold_pow ) {
		sleep_timer += duration;
	}
	else sleep_timer = 0;

	if (sleep_timer > 0.1f) {
		linear_velocity_ = Vector3(0);
		angula_velocity_ = Vector3(0);
		is_sleep_ = true;
		return;
	}
	else {
		is_sleep_ = false;
	}

	ALPcollider->integrate(duration, linear_velocity_, angula_velocity_, linear_velocity_buffer_, angula_velocity_buffer_);

}

Matrix33 ALP_Physics::get_tensor() {
	std::lock_guard <std::mutex> lock(mtx);
	ALPcollider->update_world_trans();
	return inertial_tensor_;
};
Matrix33 ALP_Physics::get_tensor_contain_added() {
	if (is_user_tensor)return inertial_tensor_;

	std::lock_guard <std::mutex> lock(mtx);

	// shapeなどのadaptを行う
	ALPcollider->update_world_trans_contain_added();

	// 質量などをcomponent::colliderからコピーする
	adapt_collider_component_data();

	{
		// colliderのshapes、addedshapesを触るため
		ALPcollider->lock_mtx();

		const auto& shapes = ALPcollider->get_shapes();
		const auto& added_shapes = ALPcollider->get_added_shapes();
		const auto& joints = ALPcollider->get_joints();
		float sum_valume = 0;


		{
			Vector3 bary = Vector3(0);
			for (const auto& shape : shapes) {
				const float shape_mass = shape->get_volume();
				bary += shape_mass * shape->local_position;
				sum_valume += shape_mass;
			}
			for (const auto& shape : added_shapes) {
				const float shape_mass = shape->get_volume();
				bary += shape_mass * shape->local_position;
				sum_valume += shape_mass;
			}

			bary /= sum_valume;

			if (is_user_barycnter == false)barycenter = bary;
		}


		// ユーザーに定義された慣性モーメントが無いとき
		if (is_user_tensor == false) {

			// 慣性モーメントの更新
			inertial_tensor_ = matrix33_zero();
			for (const auto& shape : shapes) {
				const float shape_mass = shape->get_volume();
				inertial_tensor_ += shape_mass / sum_valume * shape->get_tensor(barycenter);
			}
			for (const auto& shape : added_shapes) {
				const float shape_mass = shape->get_volume();
				inertial_tensor_ += shape_mass / sum_valume * shape->get_tensor(barycenter);
			}

			inertial_tensor_ *= inertial_mass;


			for (const auto& joint : joints) {
				inertial_tensor_ += joint->userjoint->tensor_effect();
			}
		}

		ALPcollider->unlock_mtx();
	}

	return inertial_tensor_;
};
void ALP_Physics::set_tensor(const Matrix33& tensor) {
	std::lock_guard <std::mutex> lock(mtx);

	is_user_tensor = true;
	inertial_tensor_ = tensor;
}



const Vector3 ALP_Physics::get_barycenter(bool is_calculate) const {

	if (is_user_barycnter) return barycenter;
	if (is_calculate == false) return barycenter;

	const auto shapes = ALPcollider->get_shapes();
	Vector3 val;
	float sum_valume = 0;
	{
		val = Vector3(0);
		for (const auto& shape : shapes) {
			const float shape_mass = shape->get_volume();
			val += shape_mass * shape->local_position;
			sum_valume += shape_mass;
		}
		val /= sum_valume;
	}

	return val;
}
const Vector3 ALP_Physics::get_barycenter_contain_added() {
	if (is_user_barycnter) return barycenter;

	std::lock_guard <std::mutex> lock(mtx);

	// shapeなどのadaptを行う
	ALPcollider->update_world_trans_contain_added();

	// colliderのshapes、addedshapesを触るため
	ALPcollider->lock_mtx();

	const auto& shapes = ALPcollider->get_shapes();
	const auto& added_shapes = ALPcollider->get_added_shapes();
	Vector3 val;
	float sum_valume = 0;
	{
		val = Vector3(0);
		for (const auto& shape : shapes) {
			const float shape_mass = shape->get_volume();
			val += shape_mass * shape->local_position;
			sum_valume += shape_mass;
		}
		for (const auto& shape : added_shapes) {
			const float shape_mass = shape->get_volume();
			val += shape_mass * shape->local_position;
			sum_valume += shape_mass;
		}

		val /= sum_valume;
	}

	ALPcollider->unlock_mtx();

	return val;
}
void ALP_Physics::set_barycenter(const Vector3& cent) {
	std::lock_guard <std::mutex> lock(mtx);

	barycenter = cent;
	is_user_barycnter = true;
}

//アタッチされたshapesから慣性モーメントと質量、ついでに重心の更新
void ALP_Physics::update_tensor_and_barycenter(const std::list<Collider_shape*>& shapes, const std::list<ALP_Joint*>& joints) {
	if (shapes.size() == 0) {
		barycenter = Vector3(0, 0, 0);
		inertial_tensor_ = matrix33_identity();
		return;
	}

	float sum_valume = 0;

	{
		//重心の更新
		Vector3 bary = Vector3(0);
		for (const auto& shape : shapes) {
			const float shape_mass = shape->get_volume();
			bary += shape_mass * shape->local_position;
			sum_valume += shape_mass;
		}

		if (is_user_barycnter == false) {
			bary /= sum_valume;
			barycenter = bary;
		}
	}


	//ユーザーに定義された慣性モーメントが無いとき
	if (is_user_tensor == false) {

		//慣性モーメントの更新
		inertial_tensor_ = matrix33_zero();
		for (const auto& shape : shapes) {
			const float shape_mass = shape->get_volume();
			inertial_tensor_ += shape_mass / sum_valume * shape->get_tensor(barycenter);
		}

		inertial_tensor_ *= inertial_mass;


		for (auto& joint : joints) {
			//auto a = joint->joint->get_colliderA();
			inertial_tensor_ += joint->userjoint->tensor_effect();
		}

	}


	//質量と重心の更新
	//inertial_mass = 0;
	//barycenter = Vector3(0);
	//for (const auto& shape : shapes) {
	//	inertial_mass += shape->mass;
	//	barycenter += shape->local_position * shape->mass;
	//}

	//barycenter /= inertial_mass;

	////重心の更新
	//barycenter = Vector3(0);
	//for (auto& shape : shapes) {
	//	barycenter += shape->local_position * shape->mass;
	//}

}

void ALP_Physics::adapt_collider_component_data() {
	Rigitbody_params Cdata = ALPcollider->get_collptr()->physics_data;

	inertial_mass     = Cdata.inertial_mass;
	linear_drag       = Cdata.drag;
	angula_drag       = Cdata.anglar_drag;
	dynamic_friction  = Cdata.dynamic_friction;
	static_friction   = Cdata.static_friction;
	restitution       = Cdata.restitution;
	linear_sleep_threrhold = Cdata.linear_sleep_threrhold; //freezeの閾値
	angula_sleep_threrhold = Cdata.angula_sleep_threrhold; //freezeの閾値

	is_fallable        = Cdata.is_fallable;
	is_kinmatic_anglar = Cdata.is_kinmatic_anglar; //ほかの物体からの影響で回転速度が変化しない
	is_kinmatic_linear = Cdata.is_kinmatic_linear; //ほかの物体からの影響で並進速度が変化しない
	is_moveable        = Cdata.is_moveable;
	is_hitable         = Cdata.is_hitable;
	is_static          = Cdata.is_static;
	is_active          = Cdata.is_active;
}

void ALP_Physics::destroy() {
	Physics_manager::remove_ALPphysics(this_itr);
}