/*
 * Block.h
 *
 *  Created on: Jun 25, 2013
 *      Author: aida
 */

#ifndef BLOCK_H_
#define BLOCK_H_

class Block {
public:
	Block();
	Block(float x_, float y_, float w_, float h_, int id = 0);

	float x, y;	// center
	float width, height;	// width : x, height : y
	int id;

	float xCenter();
	float yCenter();
	float left();
	float right();
	float top();
	float bottom();

	bool inY(Block b);
	bool inX(Block b);
	bool isOn(Block b);
	bool leftTo(Block b);
	bool rightTo(Block b);
};

#endif /* BLOCK_H_ */
