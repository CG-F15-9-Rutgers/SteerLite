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
	
	// Robustness: make sure there is at least two control point: start and end points
	if(!checkRobust())
	{
		std::cerr << "Error: DrawCurve does not have enough points" << std::endl;
		return;
	}
	//std::cout<< window <<std::endl;
	// Move on the curve from t=0 to t=finalPoint, using window as step size, and linearly interpolate the curve points
	float TimeInterval, deltaTime, StartTime, EndTime;
	Point EndPoint;
	Point StartPoint;
	unsigned int CurrentIndex;
	for(unsigned int i = 1; i < controlPoints.size(); i++)
	{
		TimeInterval = controlPoints[i].time - controlPoints[i-1].time;
		deltaTime = window;
		StartTime = controlPoints[i-1].time;
		EndTime = controlPoints[i].time;
		CurrentIndex = i;
		StartPoint = controlPoints[i-1].position;
		for(float CurrTime = StartTime; CurrTime <= EndTime; CurrTime = CurrTime + deltaTime)
		{
			
			if (type == hermiteCurve)
			{
				EndPoint = useHermiteCurve(CurrentIndex, CurrTime);
			}
			else if (type == catmullCurve)
			{
				EndPoint = useCatmullCurve(CurrentIndex, CurrTime);
			}
			
						
			DrawLib::drawLine(StartPoint,EndPoint,curveColor,curveThickness);
			StartPoint = EndPoint;
			
		}
		
		
		
		
	}
	
	

	

	
	
	return;
#endif
}

//Function to compare two CurvePoints based on its time element
bool SortByTime(CurvePoint& p1, CurvePoint& p2) 
{
	return p1.time < p2.time;
}
//Function to check for time duplicates on two CurvePoints
bool CheckEquals(CurvePoint& p1, CurvePoint& p2) 
{
	return p1.time == p2.time;
}
// Sort controlPoints vector in ascending order: min-first
void Curve::sortControlPoints()
{

	std::sort(controlPoints.begin(),controlPoints.end(),SortByTime);
	controlPoints.erase(unique(controlPoints.begin(),controlPoints.end(),CheckEquals),controlPoints.end());
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
	
	if (type == hermiteCurve)
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
	else if (type == catmullCurve)
	{
		if (controlPoints.size() > 3)
		{
			return true;
		}
		else
		{
			return false;
		}
		
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
	float normalTime, intervalTime, ChangeInTime, PositionX, PositionY, PositionZ;
	float FirstX, SecondX, ThirdX, FourthX, FirstY, SecondY, ThirdY, FourthY;
	float FirstZ, SecondZ, ThirdZ, FourthZ;
       
	// Calculate time interval, and normal time required for later curve calculations
	intervalTime = controlPoints[nextPoint].time - controlPoints[nextPoint-1].time;
	normalTime = (time - controlPoints[nextPoint-1].time)/(intervalTime);
	ChangeInTime = time - controlPoints[nextPoint-1].time;
	
	// Calculate position at t = time on Hermite curve
	FirstX = (2*pow(normalTime,3) - 3*pow(normalTime,2) + 1)*controlPoints[nextPoint-1].position.x;
	SecondX = (pow(normalTime,3) - 2* pow(normalTime,2) + normalTime)*controlPoints[nextPoint-1].tangent.x*ChangeInTime;
	ThirdX = (-2*pow(normalTime,3) + 3*pow(normalTime,2))*controlPoints[nextPoint].position.x;
	FourthX = (pow(normalTime,3) - pow(normalTime,2)) * controlPoints[nextPoint].tangent.x*ChangeInTime;
	
	PositionX = FirstX + SecondX + ThirdX + FourthX;
        
	
	FirstY = (2*pow(normalTime,3) - 3*pow(normalTime,2) + 1)*controlPoints[nextPoint-1].position.y;
	SecondY = (pow(normalTime,3) - 2* pow(normalTime,2) + normalTime)*controlPoints[nextPoint-1].tangent.y*ChangeInTime;
	ThirdY = (-2*pow(normalTime,3) + 3*pow(normalTime,2))*controlPoints[nextPoint].position.y;
	FourthY = (pow(normalTime,3) - pow(normalTime,2)) * controlPoints[nextPoint].tangent.y*ChangeInTime;
	
	PositionY = FirstY + SecondY + ThirdY + FourthY;
	
	FirstZ = (2*pow(normalTime,3) - 3*pow(normalTime,2) + 1)*controlPoints[nextPoint-1].position.z;
	SecondZ = (pow(normalTime,3) - 2* pow(normalTime,2) + normalTime)*controlPoints[nextPoint-1].tangent.z*ChangeInTime;
	ThirdZ = (-2*pow(normalTime,3) + 3*pow(normalTime,2))*controlPoints[nextPoint].position.z;
	FourthZ = (pow(normalTime,3) - pow(normalTime,2)) * controlPoints[nextPoint].tangent.z*ChangeInTime;
	
	PositionZ = FirstZ + SecondZ + ThirdZ + FourthZ;
	// Return result
	newPosition = Point(PositionX,PositionY,PositionZ);
	return newPosition;
}

// Implement Catmull-Rom curve
Point Curve::useCatmullCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
	float normalTime, intervalTime,ChangeInTime, PositionX, PositionY, PositionZ;
	float FirstX, SecondX, ThirdX, FourthX, FirstY, SecondY, ThirdY, FourthY;
	float FirstZ, SecondZ, ThirdZ, FourthZ;
    float Xtangent0, Xtangent1, Ytangent0, Ytangent1, Ztangent0, Ztangent1;    
    
    intervalTime = controlPoints[nextPoint].time - controlPoints[nextPoint-1].time;
	normalTime = (time - controlPoints[nextPoint-1].time)/(intervalTime);
	ChangeInTime = time - controlPoints[nextPoint-1].time;
	
	//nextpoint is k+1
	
	//Beginning Segment of the Curve
	if(nextPoint == 1)
	{
		float firstPart = (controlPoints[nextPoint+1].time-controlPoints[nextPoint-1].time)/(controlPoints[nextPoint+1].time-controlPoints[nextPoint].time)*((controlPoints[nextPoint].position.x-controlPoints[nextPoint-1].position.x)/(controlPoints[nextPoint].time-controlPoints[nextPoint-1].time));
		float secondPart = (controlPoints[nextPoint].time-controlPoints[nextPoint-1].time)/(controlPoints[nextPoint+1].time-controlPoints[nextPoint].time)*((controlPoints[nextPoint+1].position.x-controlPoints[nextPoint-1].position.x)/(controlPoints[nextPoint+1].time-controlPoints[nextPoint-1].time));
		Xtangent0 = firstPart - secondPart;
		
		float firstPartY = (controlPoints[nextPoint+1].time-controlPoints[nextPoint-1].time)/(controlPoints[nextPoint+1].time-controlPoints[nextPoint].time)*((controlPoints[nextPoint].position.y-controlPoints[nextPoint-1].position.y)/(controlPoints[nextPoint].time-controlPoints[nextPoint-1].time));
		float secondPartY = (controlPoints[nextPoint].time-controlPoints[nextPoint-1].time)/(controlPoints[nextPoint+1].time-controlPoints[nextPoint].time)*((controlPoints[nextPoint+1].position.y-controlPoints[nextPoint-1].position.y)/(controlPoints[nextPoint+1].time-controlPoints[nextPoint-1].time));
		Ytangent0 = firstPartY - secondPartY;
		
		float firstPartZ = (controlPoints[nextPoint+1].time-controlPoints[nextPoint-1].time)/(controlPoints[nextPoint+1].time-controlPoints[nextPoint].time)*((controlPoints[nextPoint].position.z-controlPoints[nextPoint-1].position.z)/(controlPoints[nextPoint].time-controlPoints[nextPoint-1].time));
		float secondPartZ = (controlPoints[nextPoint].time-controlPoints[nextPoint-1].time)/(controlPoints[nextPoint+1].time-controlPoints[nextPoint].time)*((controlPoints[nextPoint+1].position.z-controlPoints[nextPoint-1].position.z)/(controlPoints[nextPoint+1].time-controlPoints[nextPoint-1].time));
		Ztangent0 = firstPartZ - secondPartZ;
		
		Xtangent1 = (controlPoints[nextPoint+1].position.x - controlPoints[nextPoint-1].position.x)/(controlPoints[nextPoint+1].time - controlPoints[nextPoint-1].time);
		Ytangent1 = (controlPoints[nextPoint+1].position.y - controlPoints[nextPoint-1].position.y)/(controlPoints[nextPoint+1].time - controlPoints[nextPoint-1].time);
		Ztangent1 = (controlPoints[nextPoint+1].position.z - controlPoints[nextPoint-1].position.z)/(controlPoints[nextPoint+1].time - controlPoints[nextPoint-1].time);
		
		FirstX = (2*pow(normalTime,3) - 3*pow(normalTime,2) + 1)*controlPoints[nextPoint-1].position.x;
		SecondX = (pow(normalTime,3) - 2* pow(normalTime,2) + normalTime)*Xtangent0*ChangeInTime;
		ThirdX = (-2*pow(normalTime,3) + 3*pow(normalTime,2))*controlPoints[nextPoint].position.x;
		FourthX = (pow(normalTime,3) - pow(normalTime,2)) *Xtangent1*ChangeInTime;
	
		PositionX = FirstX + SecondX + ThirdX + FourthX;
		
		FirstY = (2*pow(normalTime,3) - 3*pow(normalTime,2) + 1)*controlPoints[nextPoint-1].position.y;
		SecondY = (pow(normalTime,3) - 2* pow(normalTime,2) + normalTime)*Ytangent0*ChangeInTime;
		ThirdY = (-2*pow(normalTime,3) + 3*pow(normalTime,2))*controlPoints[nextPoint].position.y;
		FourthY = (pow(normalTime,3) - pow(normalTime,2)) *Ytangent1*ChangeInTime;
	
		PositionY = FirstY + SecondY + ThirdY + FourthY;
	
		FirstZ = (2*pow(normalTime,3) - 3*pow(normalTime,2) + 1)*controlPoints[nextPoint-1].position.z;
		SecondZ = (pow(normalTime,3) - 2* pow(normalTime,2) + normalTime)*Ztangent0*ChangeInTime;
		ThirdZ = (-2*pow(normalTime,3) + 3*pow(normalTime,2))*controlPoints[nextPoint].position.z;
		FourthZ = (pow(normalTime,3) - pow(normalTime,2)) *Ztangent1*ChangeInTime;
		
		PositionZ = FirstZ + SecondZ + ThirdZ + FourthZ;
		// Return result
		newPosition = Point(PositionX,PositionY,PositionZ);
		return newPosition;
	}
	//End Segment of the Curve
	else if (nextPoint + 1 == controlPoints.size())
	{
		Xtangent0 = (controlPoints[nextPoint].position.x - controlPoints[nextPoint-2].position.x)/(controlPoints[nextPoint].time - controlPoints[nextPoint-2].time);
		Ytangent0 = (controlPoints[nextPoint].position.y - controlPoints[nextPoint-2].position.y)/(controlPoints[nextPoint].time - controlPoints[nextPoint-2].time);
		Ztangent0 = (controlPoints[nextPoint].position.z - controlPoints[nextPoint-2].position.z)/(controlPoints[nextPoint].time - controlPoints[nextPoint-2].time);
		
		// 2 is nextpoint -2
		// 1 is nextpoint -1
		// 0 is nextpoint
		float firstPart = (controlPoints[nextPoint].time-controlPoints[nextPoint-2].time)/(controlPoints[nextPoint-1].time-controlPoints[nextPoint-2].time)*((controlPoints[nextPoint].position.x-controlPoints[nextPoint-1].position.x)/(controlPoints[nextPoint].time-controlPoints[nextPoint-1].time));
		float secondPart = (controlPoints[nextPoint].time-controlPoints[nextPoint-1].time)/(controlPoints[nextPoint-1].time-controlPoints[nextPoint-2].time)*((controlPoints[nextPoint].position.x-controlPoints[nextPoint-2].position.x)/(controlPoints[nextPoint].time-controlPoints[nextPoint-2].time));
		Xtangent1 = firstPart - secondPart;
		
		
		float firstPartY = (controlPoints[nextPoint].time-controlPoints[nextPoint-2].time)/(controlPoints[nextPoint-1].time-controlPoints[nextPoint-2].time)*((controlPoints[nextPoint].position.y-controlPoints[nextPoint-1].position.y)/(controlPoints[nextPoint].time-controlPoints[nextPoint-1].time));
		float secondPartY = (controlPoints[nextPoint].time-controlPoints[nextPoint-1].time)/(controlPoints[nextPoint-1].time-controlPoints[nextPoint-2].time)*((controlPoints[nextPoint].position.y-controlPoints[nextPoint-2].position.y)/(controlPoints[nextPoint].time-controlPoints[nextPoint-2].time));
		Ytangent1 = firstPartY - secondPartY;
		
		float firstPartZ = (controlPoints[nextPoint].time-controlPoints[nextPoint-2].time)/(controlPoints[nextPoint-1].time-controlPoints[nextPoint-2].time)*((controlPoints[nextPoint].position.x-controlPoints[nextPoint-1].position.x)/(controlPoints[nextPoint].time-controlPoints[nextPoint-1].time));
		float secondPartZ = (controlPoints[nextPoint].time-controlPoints[nextPoint-1].time)/(controlPoints[nextPoint-1].time-controlPoints[nextPoint-2].time)*((controlPoints[nextPoint].position.x-controlPoints[nextPoint-2].position.x)/(controlPoints[nextPoint].time-controlPoints[nextPoint-2].time));
		Ztangent1 = firstPartZ - secondPartZ;
		
		
		FirstX = (2*pow(normalTime,3) - 3*pow(normalTime,2) + 1)*controlPoints[nextPoint-1].position.x;
		SecondX = (pow(normalTime,3) - 2* pow(normalTime,2) + normalTime)*Xtangent0*ChangeInTime;
		ThirdX = (-2*pow(normalTime,3) + 3*pow(normalTime,2))*controlPoints[nextPoint].position.x;
		FourthX = (pow(normalTime,3) - pow(normalTime,2)) *Xtangent1*ChangeInTime;
	
		PositionX = FirstX + SecondX + ThirdX + FourthX;
		
		FirstY = (2*pow(normalTime,3) - 3*pow(normalTime,2) + 1)*controlPoints[nextPoint-1].position.y;
		SecondY = (pow(normalTime,3) - 2* pow(normalTime,2) + normalTime)*Ytangent0*ChangeInTime;
		ThirdY = (-2*pow(normalTime,3) + 3*pow(normalTime,2))*controlPoints[nextPoint].position.y;
		FourthY = (pow(normalTime,3) - pow(normalTime,2)) *Ytangent1*ChangeInTime;
	
		PositionY = FirstY + SecondY + ThirdY + FourthY;
	
		FirstZ = (2*pow(normalTime,3) - 3*pow(normalTime,2) + 1)*controlPoints[nextPoint-1].position.z;
		SecondZ = (pow(normalTime,3) - 2* pow(normalTime,2) + normalTime)*Ztangent0*ChangeInTime;
		ThirdZ = (-2*pow(normalTime,3) + 3*pow(normalTime,2))*controlPoints[nextPoint].position.z;
		FourthZ = (pow(normalTime,3) - pow(normalTime,2)) *Ztangent1*ChangeInTime;
		
		PositionZ = FirstZ + SecondZ + ThirdZ + FourthZ;
		// Return result
		newPosition = Point(PositionX,PositionY,PositionZ);
		return newPosition;
		
		
	}
	else
	{
		Xtangent0 = (controlPoints[nextPoint].position.x - controlPoints[nextPoint-2].position.x)/(controlPoints[nextPoint].time - controlPoints[nextPoint-2].time);
		Xtangent1 = (controlPoints[nextPoint+1].position.x - controlPoints[nextPoint-1].position.x)/(controlPoints[nextPoint+1].time - controlPoints[nextPoint-1].time);
		
		Ytangent0 = (controlPoints[nextPoint].position.y - controlPoints[nextPoint-2].position.y)/(controlPoints[nextPoint].time - controlPoints[nextPoint-2].time);
		Ytangent1 = (controlPoints[nextPoint+1].position.y - controlPoints[nextPoint-1].position.y)/(controlPoints[nextPoint+1].time - controlPoints[nextPoint-1].time);
		
		Ztangent0 = (controlPoints[nextPoint].position.z - controlPoints[nextPoint-2].position.z)/(controlPoints[nextPoint].time - controlPoints[nextPoint-2].time);
		Ztangent1 = (controlPoints[nextPoint+1].position.z - controlPoints[nextPoint-1].position.z)/(controlPoints[nextPoint+1].time - controlPoints[nextPoint-1].time);
		
		FirstX = (2*pow(normalTime,3) - 3*pow(normalTime,2) + 1)*controlPoints[nextPoint-1].position.x;
		SecondX = (pow(normalTime,3) - 2* pow(normalTime,2) + normalTime)*Xtangent0*ChangeInTime;
		ThirdX = (-2*pow(normalTime,3) + 3*pow(normalTime,2))*controlPoints[nextPoint].position.x;
		FourthX = (pow(normalTime,3) - pow(normalTime,2)) *Xtangent1*ChangeInTime;
	
		PositionX = FirstX + SecondX + ThirdX + FourthX;
		
		FirstY = (2*pow(normalTime,3) - 3*pow(normalTime,2) + 1)*controlPoints[nextPoint-1].position.y;
		SecondY = (pow(normalTime,3) - 2* pow(normalTime,2) + normalTime)*Ytangent0*ChangeInTime;
		ThirdY = (-2*pow(normalTime,3) + 3*pow(normalTime,2))*controlPoints[nextPoint].position.y;
		FourthY = (pow(normalTime,3) - pow(normalTime,2)) *Ytangent1*ChangeInTime;
	
		PositionY = FirstY + SecondY + ThirdY + FourthY;
	
		FirstZ = (2*pow(normalTime,3) - 3*pow(normalTime,2) + 1)*controlPoints[nextPoint-1].position.z;
		SecondZ = (pow(normalTime,3) - 2* pow(normalTime,2) + normalTime)*Ztangent0*ChangeInTime;
		ThirdZ = (-2*pow(normalTime,3) + 3*pow(normalTime,2))*controlPoints[nextPoint].position.z;
		FourthZ = (pow(normalTime,3) - pow(normalTime,2)) *Ztangent1*ChangeInTime;
		
		PositionZ = FirstZ + SecondZ + ThirdZ + FourthZ;
		// Return result
		newPosition = Point(PositionX,PositionY,PositionZ);
		return newPosition;
		
		
	}
    
    /*    int adjustedPoint;
        
        if(nextPoint + 2 >= controlPoints.size())
        {
          adjustedPoint = controlPoints.size() - 3;
        }
        else
        {
            adjustedPoint = nextPoint;
        }
       
        
        
        // Calculate time interval, and normal time required for later curve calculations
        intervalTime = controlPoints[adjustedPoint+1].time - controlPoints[adjustedPoint].time;
	normalTime = (time - controlPoints[adjustedPoint].time)/(intervalTime);
        
        std::cerr << normalTime << std::endl;
        
	// Calculate position at t = time on Catmull-Rom curve
	FirstX = 2* controlPoints[adjustedPoint].position.x;
	SecondX = (-controlPoints[adjustedPoint-1].position.x + controlPoints[adjustedPoint+1].position.x) * normalTime;
	ThirdX = (2*controlPoints[adjustedPoint-1].position.x + -5 * controlPoints[adjustedPoint].position.x + 4*controlPoints[adjustedPoint+1].position.x - controlPoints[adjustedPoint+2].position.x) * pow(normalTime,2);
	FourthX = (-controlPoints[adjustedPoint-1].position.x + 3 * controlPoints[adjustedPoint].position.x - 3 * controlPoints[adjustedPoint+1].position.x + controlPoints[adjustedPoint+2].position.x) * pow(normalTime,3);
	
	PositionX = 0.5 * (FirstX + SecondX + ThirdX + FourthX);
	
	FirstY = 2* controlPoints[adjustedPoint].position.y;
	SecondY = (-controlPoints[adjustedPoint-1].position.y + controlPoints[adjustedPoint+1].position.y) * normalTime;
	ThirdY = (2*controlPoints[adjustedPoint-1].position.y + -5 * controlPoints[adjustedPoint].position.y + 4*controlPoints[adjustedPoint+1].position.y - controlPoints[adjustedPoint+2].position.y) * pow(normalTime,2);
	FourthY = (-controlPoints[adjustedPoint-1].position.y + 3 * controlPoints[adjustedPoint].position.y - 3 * controlPoints[adjustedPoint+1].position.y + controlPoints[adjustedPoint+2].position.y) * pow(normalTime,3);
	
	PositionY = 0.5 * (FirstY + SecondY + ThirdY + FourthY);
	
	FirstZ = 2* controlPoints[adjustedPoint].position.z;
	SecondZ = (-controlPoints[adjustedPoint-1].position.z + controlPoints[adjustedPoint+1].position.z) * normalTime;
	ThirdZ = (2*controlPoints[adjustedPoint-1].position.z + -5 * controlPoints[adjustedPoint].position.z + 4*controlPoints[adjustedPoint+1].position.z - controlPoints[adjustedPoint+2].position.z) * pow(normalTime,2);
	FourthZ = (-controlPoints[adjustedPoint-1].position.z + 3 * controlPoints[adjustedPoint].position.z - 3 * controlPoints[adjustedPoint+1].position.z + controlPoints[adjustedPoint+2].position.z) * pow(normalTime,3);
	
	PositionZ = 0.5 * (FirstZ + SecondZ + ThirdZ + FourthZ);
	// Return result
	newPosition = Point(PositionX,PositionY,PositionZ);
	return newPosition;
	*/
}