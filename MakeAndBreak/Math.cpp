/*
 * Math.cpp
 *
 *  Created on: Jul 11, 2013
 *      Author: aida
 */

#include "Math.h"

namespace FM {
	float EPS = 1e-6;

	float abs(float a) {	return a > 0 ? a : -a;	}
	bool equ(float a, float b) {	return abs(a-b) < EPS;	}
	bool less(float a, float b) {	return a + EPS < b;	}
	bool greater(float a, float b) {	return less(b, a);	}
	bool lequ(float a, float b) {	return a < b + EPS;	}
	bool gequ(float a, float b) {	return lequ(b, a);	}
	bool min(float a, float b) {	return a < b ? a : b;	}
	bool max(float a, float b) {	return a > b ? a : b;	}
}
