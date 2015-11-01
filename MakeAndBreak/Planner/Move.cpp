/*
 * Move.cpp
 *
 *  Created on: Jun 25, 2013
 *      Author: aida
 */

#include "Move.h"
#include "../Math.h"
#include <algorithm>

Move::Move() {
	left = right = NO_BLOCK;
	mode = ONE_HAND;
}

Move::Move(int left_, int right_, int mode_)
{
	left = left_;
	right = right_;
	mode = mode_;
}

void Move::changeHands()
{
	std::swap(left, right);
}

int Move::getBlock()	//use only for one-handed moves!
{
	if (left != NO_BLOCK)
		return left;
	return right;
}
