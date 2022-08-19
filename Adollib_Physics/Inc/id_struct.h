#pragma once

namespace Adollib {

	// IDにしたいものによって適当にサイズ合わせて
	struct Physics_ID {

////#ifdef Physics_ID_64
//
//		long long ID;
//
//		bool operator== (const Physics_ID& S) const {
//			return (ID == S.ID);
//		}
//
////#endif // DEBUG

#ifdef Physics_ID_128
		long long ID0;
		long long ID1;

		bool operator== (const Physics_ID& S) const {
			return (ID0 == S.ID0) && (ID1 == S.ID1);
		}
#else // Physics_ID_128

		// Physics_ID_64
		long long ID;

		bool operator== (const Physics_ID& S) const {
			return (ID == S.ID);
		}

#endif // Physics_ID_64

	};

}