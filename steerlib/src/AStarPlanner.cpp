//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#include <vector>
#include <stack>
#include <set>
#include <map>
#include <iostream>
#include <algorithm> 
#include <functional>
#include <queue>
#include <math.h>
#include "planning/AStarPlanner.h"


#define COLLISION_COST  1000
#define GRID_STEP  1
#define OBSTACLE_CLEARANCE 1
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

namespace SteerLib
{
	AStarPlanner::AStarPlanner(){}

	AStarPlanner::~AStarPlanner(){}

	bool AStarPlanner::canBeTraversed ( int id ) 
	{
		double traversal_cost = 0;
		int current_id = id;
		unsigned int x,z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;

		x_range_min = MAX(x-OBSTACLE_CLEARANCE, 0);
		x_range_max = MIN(x+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsX());

		z_range_min = MAX(z-OBSTACLE_CLEARANCE, 0);
		z_range_max = MIN(z+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsZ());


		for (int i = x_range_min; i<=x_range_max; i+=GRID_STEP)
		{
			for (int j = z_range_min; j<=z_range_max; j+=GRID_STEP)
			{
				int index = gSpatialDatabase->getCellIndexFromGridCoords( i, j );
				traversal_cost += gSpatialDatabase->getTraversalCost ( index );
				
			}
		}

		if ( traversal_cost > COLLISION_COST ) 
			return false;
		return true;
	}



	Util::Point AStarPlanner::getPointFromGridIndex(int id)
	{
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(id, p);
		return p;
	}



	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path,  Util::Point start, Util::Point goal, SteerLib::GridDatabase2D * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;

		//TODO
		std::cout<<"\nIn A*";

		std::vector<SteerLib::AStarPlannerNode> ClosedSet;
		std::vector<SteerLib::AStarPlannerNode> OpenSet;
		std::vector<SteerLib::AStarPlannerNode> CameFrom;

		OpenSet.push_back() //TODO need to push Start Node, don't know how to get Node

		int lowestFScore = 0;
		int lowestFIndex = 0;
		AStarPlannerNode CurrentNode;

		while(!OpenSet.empty())
		{	

			//Get Node with the lowest Fscore

			lowestFScore = OpenSet[0].f;
			lowestFIndex = 0; 

			for( int i = 0: i< OpenSet.size(); i++)
			{
				if(OpenSet[i].f < lowestFScore)
				{
					lowestFScore = OpenSet[i].f;
					lowestFIndex = i;

				}


			}

			CurrentNode = OpenSet[lowestFIndex];

			if(CurrentNode.point == goal)
			{
				
				//TODO reconstruct path

			}

			ClosedSet.push_back(OpenSet[lowestFIndex]);
			OpenSet.erase(OpenSet.begin() + lowestFIndex);


			//TODO get neighbors of current Node
















		}




		return false;
	}
}
