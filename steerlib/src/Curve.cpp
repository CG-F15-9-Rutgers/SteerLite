//
// Copyright (c) 2015 Mahyar Khayatkhoei
// Copyright (c) 2009-2014 Shawn Singh, Glen Berseth, Mubbasir Kapadia, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

#include <algorithm>
#include <vector>
#include <math.h>  
#include <util/Geometry.h>
#include <util/Curve.h>
#include <util/Color.h>
#include <util/DrawLib.h>
#include "Globals.h"

using namespace Util;

Curve::Curve(const CurvePoint& startPoint, int curveType) : type(curveType)
{
	controlPoints.push_back(startPoint);
}

Curve::Curve(const std::vector<CurvePoint>& inputPoints, int curveType) : type(curveType)
{
	controlPoints = inputPoints;
	sortControlPoints();
}

// Add one control point to the vector controlPoints
void Curve::addControlPoint(const CurvePoint& inputPoint)
{
	controlPoints.push_back(inputPoint);
	sortControlPoints();
}

// Add a vector of control points to the vector controlPoints
void Curve::addControlPoints(const std::vector<CurvePoint>& inputPoints)
{
	for (int i = 0; i < inputPoints.size(); i++)
		controlPoints.push_back(inputPoints[i]);
	sortControlPoints();
}

// Draw the curve shape on screen, usign window as step size (bigger window: less accurate shape)
void Curve::drawCurve(Color curveColor, float curveThickness, int window)
{
#ifdef ENABLE_GUI

	//================DELETE THIS PART AND THEN START CODING===================
	static bool flag = false;
	if (!flag)
	{
		std::cerr << "ERROR>>>>Member function drawCurve is not implemented!" << std::endl;
		flag = true;
	}
	//=========================================================================

	// Robustness: make sure there is at least two control point: start and end points

	// Move on the curve from t=0 to t=finalPoint, using window as step size, and linearly interpolate the curve points
	
	return;
#endif
}

//Function to compare two CurvePoints based on its time element
bool SortByTime(CurvePoint& p1, CurvePoint& p2) 
{
	return p1.time < p2.time;
}
// Sort controlPoints vector in ascending order: min-first
void Curve::sortControlPoints()
{

	std::sort(controlPoints.begin(),controlPoints.end(),SortByTime);
	return;
}

// Calculate the position on curve corresponding to the given time, outputPoint is the resulting position
bool Curve::calculatePoint(Point& outputPoint, float time)
{
	// Robustness: make sure there is at least two control point: start and end points
	if (!checkRobust())
		return false;

	// Define temporary parameters for calculation
	unsigned int nextPoint;
	float normalTime, intervalTime;

	// Find the current interval in time, supposing that controlPoints is sorted (sorting is done whenever control points are added)
	if (!findTimeInterval(nextPoint, time))
		return false;

	// Calculate position at t = time on curve
	if (type == hermiteCurve)
	{
		outputPoint = useHermiteCurve(nextPoint, time);
	}
	else if (type == catmullCurve)
	{
		outputPoint = useCatmullCurve(nextPoint, time);
	}

	// Return
	return true;
}

// Check Roboustness
bool Curve::checkRobust()
{
	
	if (controlPoints.size() > 1)
	{
		return true;
	}
	else
	{
		return false;
	}
	
}

// Find the current time interval (i.e. index of the next control point to follow according to current time)
bool Curve::findTimeInterval(unsigned int& nextPoint, float time)
{

	for(int i = 0; i<controlPoints.size(); i++)
	{
		if (controlPoints[i].time > time)
		{
			nextPoint = i;
			return true;
		}
	}


	return false;
}

// Implement Hermite curve
Point Curve::useHermiteCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
	float normalTime, intervalTime, PositionX, PositionY, PositionZ;
	float FirstX, SecondX, ThirdX, FourthX, FirstY, SecondY, ThirdY, FourthY;
	float FirstZ, SecondZ, ThirdZ, FourthZ;

	// Calculate time interval, and normal time required for later curve calculations
	intervalTime = controlPoints[nextPoint].time - controlPoints[nextPoint-1].time;
	normalTime = (time - controlPoints[nextPoint-1].time)/(intervalTime);
	
	// Calculate position at t = time on Hermite curve
	FirstX = (2*pow(normalTime,3) - 3*pow(normalTime,2) + 1)*controlPoints[nextPoint-1].position.x;
	SecondX = (pow(normalTime,3) - 2* pow(normalTime,2) + normalTime)*controlPoints[nextPoint-1].tangent.x;
	ThirdX = (-2*pow(normalTime,3) + 3*pow(normalTime,2))*controlPoints[nextPoint].position.x;
	FourthX = (pow(normalTime,3) - pow(normalTime,2)) * controlPoints[nextPoint].tangent.x;
	
	PositionX = FirstX + SecondX + ThirdX + FourthX;
	
	FirstY = (2*pow(normalTime,3) - 3*pow(normalTime,2) + 1)*controlPoints[nextPoint-1].position.y;
	SecondY = (pow(normalTime,3) - 2* pow(normalTime,2) + normalTime)*controlPoints[nextPoint-1].tangent.y;
	ThirdY = (-2*pow(normalTime,3) + 3*pow(normalTime,2))*controlPoints[nextPoint].position.y;
	FourthY = (pow(normalTime,3) - pow(normalTime,2)) * controlPoints[nextPoint].tangent.y;
	
	PositionY = FirstY + SecondY + ThirdY + FourthY;
	
	FirstZ = (2*pow(normalTime,3) - 3*pow(normalTime,2) + 1)*controlPoints[nextPoint-1].position.z;
	SecondZ = (pow(normalTime,3) - 2* pow(normalTime,2) + normalTime)*controlPoints[nextPoint-1].tangent.z;
	ThirdZ = (-2*pow(normalTime,3) + 3*pow(normalTime,2))*controlPoints[nextPoint].position.z;
	FourthZ = (pow(normalTime,3) - pow(normalTime,2)) * controlPoints[nextPoint].tangent.z;
	
	PositionZ = FirstZ + SecondZ + ThirdZ + FourthZ;
	// Return result
	newPosition = Point(PositionX,PositionY,PositionZ);
	return newPosition;
}

// Implement Catmull-Rom curve
Point Curve::useCatmullCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;

	//================DELETE THIS PART AND THEN START CODING===================
	static bool flag = false;
	if (!flag)
	{
		std::cerr << "ERROR>>>>Member function useCatmullCurve is not implemented!" << std::endl;
		flag = true;
	}
	//=========================================================================


	// Calculate time interval, and normal time required for later curve calculations

	// Calculate position at t = time on Catmull-Rom curve
	
	// Return result
	return newPosition;
}