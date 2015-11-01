/*
 * Block.cpp
 *
 *  Created on: Jun 25, 2013
 *      Author: aida
 */

#include "Block.h"
#include "../Math.h"

Block::Block() {
	x = y = 0;
	width = height = 1;
}

Block::Block(float x_, float y_, float w_, float h_, int id_)
{
	x = x_;
	y = y_;
	width = w_;
	height = h_;
	id = id_;
}

float Block::xCenter()
{
	return x;
}

float Block::yCenter()
{
	return y;
}

float Block::left()
{
	return x - width / 2.0f;
}

float Block::right()
{
	return x + width / 2.0f;
}

float Block::top()
{
	return y + height / 2.0f;
}

float Block::bottom()
{
	return y - height / 2.0f;
}

bool Block::inY(Block b)
{
	if (FM::lequ(top(), b.bottom()) || FM::gequ(bottom(), b.top()))
		return false;
	return true;
}

bool Block::inX(Block b)
{
	if (FM::lequ(right(), b.left()) || FM::gequ(left(), b.right()))
		return false;
	return true;
}

bool Block::isOn(Block b)
{
	return inX(b) && FM::equ(bottom(), b.top());
}

bool Block::leftTo(Block b)
{
	return inY(b) && FM::equ(right(), b.left());
}

bool Block::rightTo(Block b)
{
	return inY(b) && FM::equ(left(), b.right());
}
