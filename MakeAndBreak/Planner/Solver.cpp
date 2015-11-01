/*
 * Solver.cpp
 *
 *  Created on: Jun 25, 2013
 *      Author: aida
 */

#include "Solver.h"
#include "../Math.h"

#include <stdlib.h>
#include <algorithm>
#include <sstream>
#include <iostream>

Solver::Solver(std::vector<Block> blocks_)
{
	blocks = blocks_;
	useBothHandsWithTwoHandActions();
}

void Solver::useOnlyRight()
{
	rightIsActive = true;
	leftIsActive = false;
	canUseBothHands = false;
}

void Solver::useOnlyLeft()
{
	rightIsActive = false;
	leftIsActive = true;
	canUseBothHands = false;
}

void Solver::useBothHandsWithoutTwoHandActions()
{
	rightIsActive = true;
	leftIsActive = true;
	canUseBothHands = false;
}

void Solver::useBothHandsWithTwoHandActions()
{
	rightIsActive = true;
	leftIsActive = true;
	canUseBothHands = true;
}

int Solver::getSpace(std::vector<Move> a, int v)
{
	int res = 0;
	if (a[v].right != Move::NO_BLOCK) {
		int b = a[v].right;
		int t = INF;

		for (int j = 0; j < v; j++) {
			for (int k = 0; k < 2; k++) {
				int o = (k == 0) ? a[j].right : a[j].left;
				if (o != Move::NO_BLOCK && blocks[b].x < blocks[o].x && blocks[o].inY(blocks[b]))
					t = FM::min(t, blocks[o].left() - blocks[b].right());
			}
		}

		if (t < 5)
			res+= 6-t;
	}
	if (a[v].left != Move::NO_BLOCK) {
		int b = a[v].left;
		int t = INF;

		for (int j = 0; j < v; j++) {
			for (int k = 0; k < 2; k++) {
				int o = (k == 0) ? a[j].right : a[j].left;
				if (o != Move::NO_BLOCK && blocks[b].x > blocks[o].x && blocks[o].inY(blocks[b]))
					t = FM::min(t, blocks[b].left() - blocks[o].right());
			}
		}

		if (t < 5)
			res+= 6-t;
		}
	return res;
}

int Solver::floatingScore(std::vector<Move> a)
{
	int floatingCounter = 0;
	bool mark[blocks.size()], floating[blocks.size()];
	for (int i = 0; i < (int) blocks.size(); i++)
		mark[i] = floating[i] = false;

	for (int i = 0; i < (int) a.size(); i++) {
		for (int k = 0; k < 2; k++) {
			int b = (k == 0) ? a[i].left : a[i].right;
			if (b == Move::NO_BLOCK)
				continue;

			mark[b] = true;
			if (!FM::equ(blocks[b].bottom(), 0))
				for (int j = 0; j < (int) blocks.size(); j++) {
					if (j != b && blocks[b].isOn(blocks[j]) && (!mark[j] || floating[j])) {
						floating[b] = true;
						floatingCounter++;
					}
				}
		}
	}

	return floatingCounter;
}

int Solver::score(std::vector<Move> a)
{
	int res = 0;
	for (int i = 1; i < (int) a.size(); i++)
		res+= getSpace(a, i) * 100;

	res+= floatingScore(a) * 1000;

	int unstableCounter = 0;
	std::vector<Block> c = std::vector<Block>();
	for (int i = 0; i < (int) a.size(); i++) {
		if (a[i].left != Move::NO_BLOCK)
			c.push_back(blocks[a[i].left]);
		if (a[i].right != Move::NO_BLOCK)
			c.push_back(blocks[a[i].right]);
		if (!checkStability(c)) {
			if (a[i].mode == Move::TWO_HANDS)
				unstableCounter++;
			unstableCounter++;
		}
	}
	res+= unstableCounter * 1000;

	for (int i = 0; i < (int) a.size(); i++) {
		if (a[i].mode == Move::TWO_HANDS) {
			res+= 5;
			if (blocks[a[i].left].isOn(blocks[a[i].right]) ||
					blocks[a[i].right].isOn(blocks[a[i].left]))
				res+= 1000;

			if (blocks[a[i].right].x < blocks[a[i].left].x) {
				res+= 1000;
			}
		}
	}

	int counter = 0;
	for (int i = 0; i < (int) a.size(); i++) {
		if (a[i].left != Move::NO_BLOCK)
			counter++;
		if (a[i].right != Move::NO_BLOCK)
			counter--;
	}
	counter = counter < 0 ? -counter : counter;
	res+= counter * 5;

	return -res;
}

std::vector<Move> Solver::createRandomNode()
{
	std::vector<Move> res;

	for (int i = 0; i < (int) blocks.size(); i++) {
		Move temp = Move();
		if (rightIsActive && leftIsActive) {
			if (rand() % 2 == 0)
				temp.left = i;
			else
				temp.right = i;
		} else {
			if (rightIsActive)
				temp.right = i;
			if (leftIsActive)
				temp.left = i;
		}
		temp.mode = Move::ONE_HAND;
		res.push_back(temp);
	}

	for (int i = 0; i < (int) res.size(); i++)
		std::swap(res[i], res[rand() % res.size()]);

	if (res.size() < 2 || !canUseBothHands)
		return res;

	int n = rand() % (res.size() / 2);
	for (int i = 0; i < n; i++) {
		int x = rand() % res.size(), y = rand() % res.size();
		if (res[x].mode == Move::ONE_HAND &&
				res[y].mode == Move::ONE_HAND && x != y) {
			int temp = res[y].right;
			if (res[y].left != Move::NO_BLOCK)
				temp = res[y].left;
			if (res[x].right != Move::NO_BLOCK)
				res[x].left = temp;
			else
				res[x].right = temp;
			res[x].mode = Move::TWO_HANDS;
			res[y] = Move(res[(int) res.size()-1]);
			res.resize((int) res.size()-1);
		}
	}

	return res;
}

std::vector< std::vector<Move> > Solver::getNeighbors(std::vector<Move> c)
{
	std::vector< std::vector<Move> > res;

	for (int i = 0; i < (int) c.size(); i++) {	// swap 2 moves
		for (int j = i+1; j < (int) c.size(); j++) {
			std::swap(c[i], c[j]);
			res.push_back(c);
			std::swap(c[i], c[j]);
		}
	}

	if (rightIsActive && leftIsActive) {
		for (int i = 0; i < (int) c.size(); i++) {	// change the operative hand
			c[i].changeHands();
			res.push_back(c);
			c[i].changeHands();
		}
	}

	if (canUseBothHands) {
		for (int i = 0; i < (int) c.size(); i++) {	// merge two one-handed moves
			if (c[i].mode == Move::TWO_HANDS)
				continue;
			for (int j = i+1; j < (int) c.size(); j++) {
				if (c[j].mode == Move::TWO_HANDS)
					continue;

				Move x = c[i], y = c[j];
				int a = c[j].getBlock(), b = c[i].getBlock();

				c[i].mode = Move::TWO_HANDS;
				c[i].left = b;
				c[i].right = a;
				if (j != (int) c.size()-1) {
					c[j] = c[(int) c.size()-1];
				}
				c.resize((int) c.size()-1);
				res.push_back(c);

				c[i].changeHands();
				res.push_back(c);

				c[i] = x;
				if (j == (int) c.size())
					c.push_back(y);
				else {
					c.push_back(c[j]);
					c[j] = y;
				}
			}
		}

		for (int i = 0; i < (int) c.size(); i++)	// unmerge a two-handed move
			if (c[i].mode == Move::TWO_HANDS) {
				Move t = c[i];

				c[i].mode = Move::ONE_HAND;
				c.insert(c.begin() + i+1, c[i]);

				c[i].left = Move::NO_BLOCK;
				c[i+1].right = Move::NO_BLOCK;
				res.push_back(c);

				std::swap(c[i], c[i+1]);
				res.push_back(c);

				c.erase(c.begin() + i + 1);
				c[i] = t;
			}
	}
	return res;
}

std::vector<Move> Solver::solve()
{
	if (!isStable())
		return std::vector<Move>();

	for (int r = 0; r < 10; r++) {
		std::vector<Move> currentNode = createRandomNode();
		while (true) {
			if (score(currentNode) == 0) {
				return currentNode;
			}

			std::vector< std::vector<Move> > l = getNeighbors(currentNode);
			int nextEval = -INF;
			std::vector<Move> nextNode = std::vector<Move>();
			for (int i = 0; i < (int) l.size(); i++) {
				if (score(l[i]) > nextEval) {
					nextNode = l[i];
					nextEval = score(l[i]);
				}
			}

			if (nextEval <= score(currentNode)) {
				if (score(currentNode) > -3 * (int) blocks.size()) {
					//print(currentNode);
					return currentNode;
				}
				break;
			}

			currentNode = nextNode;
		}
	}

	return std::vector<Move>();
}

bool Solver::isStable()
{
	return checkStability(blocks);
}

std::string Solver::print(std::vector<Move> a)
{
	std::stringstream ss;
	if (a.size() == 0) {
		ss << "No Solution";
		return ss.str();
	}

	for (int i = 0; i < (int) a.size(); i++)
		if (a[i].mode == Move::ONE_HAND)
			if (a[i].left != Move::NO_BLOCK)
				ss << "Left: (" << blocks[a[i].left].x << ", " << blocks[a[i].left].y << ") - ";
			else
				ss << "Right: (" << blocks[a[i].right].x << ", " << blocks[a[i].right].y << ") - ";
		else
			ss << "Left: (" << blocks[a[i].left].x << ", " << blocks[a[i].left].y <<
			") and Right: (" << blocks[a[i].right].x << ", " << blocks[a[i].right].y << ") - ";
	return ss.str();
}

bool Solver::checkStability(std::vector<Block> blocks)
{
	b2Vec2 gravity = b2Vec2(0, -10);
	b2World world = b2World(gravity);
	world.SetAllowSleeping(true);
	world.SetWarmStarting(true);

	{ // Floor
		b2FixtureDef fd;
		b2PolygonShape sd;
		sd.SetAsBox((float32) 50 - 0.01, (float32) 10 - 0.01);
		fd.friction = 0.01;
		fd.shape = &sd;

		b2BodyDef bd;
		bd.position = b2Vec2(0.0f, -10.0f);
		world.CreateBody(&bd)->CreateFixture(&fd);

	}

	{ // Blocks
		for (int i = 0; i < (int) blocks.size(); i++) {
			Block c = blocks[i];

			b2FixtureDef fd;
			b2PolygonShape sd;
			sd.SetAsBox(c.width/2.f - 0.01, c.height/2.f - 0.01);
			fd.shape = &sd;
			fd.density = 25.0f;
			fd.friction = .01f;

			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.angle =  0;
			bd.position = b2Vec2(c.x, c.y);
			b2Body* myBody = world.CreateBody(&bd);
			myBody->CreateFixture(&fd);
		}
	}

	b2Vec2 p[world.GetBodyCount()];
	b2Body *c = world.GetBodyList();
	for (int i = 0; i < world.GetBodyCount(); i++) {
		p[i] = b2Vec2();
		p[i].x = c->GetPosition().x;
		p[i].y = c->GetPosition().y;
		c = c->GetNext();
	}

	for (int i = 0; i < 180; i++)
		world.Step(1.f/60.0f, 8, 3);

	c = world.GetBodyList();
	for (int i = 0; i < world.GetBodyCount(); i++) {
		if ((p[i] - c->GetPosition()).Length() > 0.1 || c->IsAwake())
			return false;
		c = c->GetNext();
	}
	return true;
}
