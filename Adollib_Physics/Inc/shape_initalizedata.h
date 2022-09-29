#pragma once
#include "Math/math.h"

namespace Adollib {
	namespace Physics_function {
		struct Shape_InitData {

			Vector3 center = Vector3(0);
			Vector3 rotate = Vector3(0);
			Vector3 size = Vector3(1);
			float radius = 1;
			float length = 1;
			float distance = 0;
		};
	}
}