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
		std::vector<SteerLib::AStarPlannerNode> NeighborVector;

		int lowestFScore = 0;
		int lowestFIndex = 0;
		double CostSoFar;
		SteerLib::AStarPlannerNode StartNode(start, 0,0, nullptr);
		OpenSet.push_back(StartNode);

		while(!OpenSet.empty())
		{
			//Get Node with the lowest Fscore

			lowestFScore = OpenSet[0].f;
			lowestFIndex = 0; 

			for( int i = 0; i< OpenSet.size(); i++)
			{
				if(OpenSet[i].f < lowestFScore)
				{
					lowestFScore = OpenSet[i].f;
					lowestFIndex = i;
				}
			}

			SteerLib::AStarPlannerNode CurrentNode = OpenSet[lowestFIndex];

			if(CurrentNode.point == goal)
			{
				//TODO reconstruct path
			}

			ClosedSet.push_back(OpenSet[lowestFIndex]);
			OpenSet.erase(OpenSet.begin() + lowestFIndex);

			NeighborVector = NeighborNodes(CurrentNode, goal);

			for( int NeighborIndex = 0; NeighborIndex< NeighborVector.size(); NeighborIndex++)
			{
				//Neighbor in ClosedSet
				if(std::find(ClosedSet.begin(), ClosedSet.end(), NeighborVector[NeighborIndex]) != ClosedSet.end()) 
				{
					continue;
				}

				//CostSoFar = CurrentNode.g + DistanceBetween(

				//Neighbor not in OpenSet
				if(std::find(OpenSet.begin(), OpenSet.end(), NeighborVector[NeighborIndex]) != OpenSet.end()) 
				{
					OpenSet.push_back(NeighborVector[NeighborIndex]);
				}
				//else if(NeighborVector[NeighborIndex].g >=

			}
		}
		return false;
	}


	std::vector<SteerLib::AStarPlannerNode> AStarPlanner::NeighborNodes(SteerLib::AStarPlannerNode OriginNode, Util::Point goal)
	{
		int x;
		int y;
		int z;
		double NonDiagonalCost;
		double DiagonalCost;

		DiagonalCost = sqrt(2);
		NonDiagonalCost = 1;

		std::vector<SteerLib::AStarPlannerNode> NeighborVector;

		x = OriginNode.point.x;
		y = OriginNode.point.y;
		z = OriginNode.point.z;

		Util::Point North = Util::Point(x,y,z+1);
		Util::Point South = Util::Point(x,y,z-1); 
		Util::Point East =  Util::Point(x+1,y,z); 
		Util::Point West =  Util::Point(x-1,y,z); 
		Util::Point NorthEast = Util::Point(x+1,y,z+1);
		Util::Point SouthEast = Util::Point(x+1,y,z-1); 
		Util::Point NorthWest = Util::Point(x-1,y,z+1); 
		Util::Point SouthWest = Util::Point(x-1,y,z-1);

		AddNode(North, NonDiagonalCost, OriginNode, NeighborVector, goal);
		AddNode(South, NonDiagonalCost, OriginNode, NeighborVector, goal);
		AddNode(East, NonDiagonalCost, OriginNode, NeighborVector, goal);
		AddNode(West, NonDiagonalCost, OriginNode, NeighborVector, goal);
		AddNode(NorthEast, DiagonalCost, OriginNode, NeighborVector, goal);
		AddNode(SouthEast, DiagonalCost, OriginNode, NeighborVector, goal);
		AddNode(NorthWest, DiagonalCost, OriginNode, NeighborVector, goal);
		AddNode(SouthEast, DiagonalCost, OriginNode, NeighborVector, goal);

		return NeighborVector;
	}


	void AStarPlanner::AddNode(Util::Point CurrentPoint, double cost, SteerLib::AStarPlannerNode FromNode, std::vector<SteerLib::AStarPlannerNode>& NeighborVector, Util::Point goal)
	{
		int NodeIndex;
		float DistanceInBetween;

		NodeIndex = gSpatialDatabase->getCellIndexFromLocation(CurrentPoint);

		if(!canBeTraversed (NodeIndex))
		{
			return;
		}
		else
		{
			DistanceInBetween = distanceBetween(CurrentPoint, goal);
			SteerLib::AStarPlannerNode InsertNode(CurrentPoint, FromNode.g + cost, (double)DistanceInBetween, &FromNode);
			NeighborVector.push_back(InsertNode);
		}

	}
}
