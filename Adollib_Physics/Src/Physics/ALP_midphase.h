#pragma once
#include "ALP_contact.h"
#include "../../Inc/collider_shape.h"

namespace Adollib {
	namespace Physics_function {
		//��G�c�ȓ����蔻��
		void Midphase(
			std::vector<Contacts::Contact_pair*>& old_pairs,
			std::vector<Contacts::Contact_pair*>& new_pairs
		);

	}
}