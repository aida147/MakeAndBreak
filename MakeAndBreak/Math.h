/*
 * Block.h
 *
 *  Created on: Jun 25, 2013
 *      Author: aida
 */

#ifndef MATH_H_
#define MATH_H_

namespace FM {
	extern float EPS;

	float abs(float a);
	bool equ(float a, float b);
	bool less(float a, float b);
	bool greater(float a, float b);
	bool lequ(float a, float b);
	bool gequ(float a, float b);
	bool min(float a, float b);
	bool max(float a, float b);
}

#endif /* MATH_H_ */
