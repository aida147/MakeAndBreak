/*
 * Solver.h
 *
 *  Created on: Jun 25, 2013
 *      Author: aida
 */

#ifndef SOLVER_H_
#define SOLVER_H_

#include "Block.h"
#include "Move.h"

#include <Box2D/Box2D.h>

#include <string>
#include <vector>
#include <cmath>

class Solver {
public:
	Solver(std::vector<Block> blocks);

	std::vector<Move> solve();
	bool isStable();
	std::string print(std::vector<Move> a);
	int score(std::vector<Move> a);

	/*Use these functions to choose which hand(s) to use*/
	void useOnlyRight();
	void useOnlyLeft();
	void useBothHandsWithoutTwoHandActions();
	void useBothHandsWithTwoHandActions();

	std::vector<Block> blocks;

private:
	int getSpace(std::vector<Move> a, int x);
	std::vector<Move> createRandomNode();
	std::vector< std::vector<Move> > getNeighbors(std::vector<Move> c);
	int floatingScore(std::vector<Move> a);
	bool checkStability(std::vector<Block> blocks);

	bool leftIsActive, rightIsActive, canUseBothHands;
	const static int INF = 100000;
};

#endif /* SOLVER_H_ */
