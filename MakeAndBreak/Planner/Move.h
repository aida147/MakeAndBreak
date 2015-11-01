/*
 * Move.h
 *
 *  Created on: Jun 25, 2013
 *      Author: aida
 */

#ifndef MOVE_H_
#define MOVE_H_

class Move {
public:
	Move();

	enum {ONE_HAND = 0, TWO_HANDS = 1};
	static const int NO_BLOCK = -1;
	int mode;
	int left, right;

	Move(int left_, int right_, int mode_);
	void changeHands();
	int getBlock();
};

#endif /* MOVE_H_ */
