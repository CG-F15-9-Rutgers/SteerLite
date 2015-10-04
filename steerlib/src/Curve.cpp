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

	// Move on the curve from t=0 to t=finalPoint, using window as step size, and linearly interpolate the curve points
	float deltaTime, StartTime, EndTime;
	Point EndPoint;
	Point StartPoint;
	unsigned int CurrentIndex;
	for(unsigned int i = 1; i < controlPoints.size(); i++)
	{
		deltaTime = window;
		StartTime = controlPoints[i-1].time;
		EndTime = controlPoints[i].time;
		CurrentIndex = i;
		StartPoint = controlPoints[i-1].position;

		for(float CurrTime = StartTime; CurrTime <= EndTime; CurrTime += deltaTime)
		{
			if (type == hermiteCurve)
			{
				EndPoint = useHermiteCurve(CurrentIndex, CurrTime);
			}
			else if (type == catmullCurve)
			{
				EndPoint = useCatmullCurve(CurrentIndex, CurrTime);
			}

			DrawLib::drawLine(StartPoint, EndPoint, curveColor, curveThickness);
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
	std::sort(controlPoints.begin(), controlPoints.end(), SortByTime);
	controlPoints.erase(unique(controlPoints.begin(), controlPoints.end(), CheckEquals), controlPoints.end());
	return;
}

// Calculate the position on curve corresponding to the given time, outputPoint is the resulting position
bool Curve::calculatePoint(Point& outputPoint, float time)
{
	// Robustness: make sure there is at least two control point: start and end points
	if (!checkRobust()) {
		return false;
	}

	// Define temporary parameters for calculation
	unsigned int nextPoint;

	// Find the current interval in time, supposing that controlPoints is sorted (sorting is done whenever control points are added)
	if (!findTimeInterval(nextPoint, time)) {
		return false;
	}

	// Calculate position at t = time on curve
	if (type == hermiteCurve)
	{
		outputPoint = useHermiteCurve(nextPoint, time);
	}
	else if (type == catmullCurve)
	{
		outputPoint = useCatmullCurve(nextPoint, time);
	}

	return true;
}

// Check Roboustness
bool Curve::checkRobust()
{
	if (type == hermiteCurve)
	{
		return (controlPoints.size() > 1);
	}
	else if (type == catmullCurve)
	{
		return (controlPoints.size() >= 3);
	}
	return false;
}

// Find the current time interval (i.e. index of the next control point to follow according to current time)
bool Curve::findTimeInterval(unsigned int& nextPoint, float time)
{
	for(unsigned int i = 0; i < controlPoints.size(); i++)
	{
		if (controlPoints[i].time > time)
		{
			nextPoint = i;
			return true;
		}
	}

	return false;
}

float hermite00(const float t) {
	// h_00 = 2t^3 - 3t^2 + 1
	return (2*pow(t, 3) - 3*pow(t, 2) + 1);
}

float hermite10(const float t) {
	// h_10 = t^3 - 2t^2 + t
	return (pow(t, 3) - 2*pow(t, 2) + t);
}

float hermite01(const float t) {
	// h_01 = -2t^3 + 3t^2
	return -2*pow(t, 3) + 3*pow(t, 2);
}

float hermite11(const float t) {
	// h_11 t^3 - t^2
	return pow(t,3) - pow(t,2);
}

// Implement Hermite curve
Point Curve::useHermiteCurve(const unsigned int nextPoint, const float time)
{
	// Calculate time interval, and normal time required for later curve calculations
	float intervalTime = controlPoints[nextPoint].time - controlPoints[nextPoint-1].time;
	float normalTime = (time - controlPoints[nextPoint-1].time) / (intervalTime);

	// Calculate position at t = time on Hermite curve
	// (2t^3 - 3t^2 + 1)*p0
	Point firstTerm = hermite00(normalTime)*controlPoints[nextPoint-1].position;
	// (t^3 - 2t^2 + t)*m0
	Vector secondTerm = hermite10(normalTime)*controlPoints[nextPoint-1].tangent*intervalTime;
	// (-2t^3 + 3t^2)*p1
	Point thirdTerm = (-2*pow(normalTime,3) + 3*pow(normalTime,2))*controlPoints[nextPoint].position;
	// (t^3 - t^2)*m1
	Vector fourthTerm = (pow(normalTime,3) - pow(normalTime,2)) * controlPoints[nextPoint].tangent*intervalTime;

	return firstTerm + secondTerm + thirdTerm + fourthTerm;
}

Vector Curve::catmullTangent(const unsigned int point) {
	// for some reason the implementers made Point subtraction return a Vector.
	// oh well. we can still use it like this.

	float firstCoeff, secondCoeff;
	Vector firstTerm, secondTerm;

	// First Segment of Curve
	if (point == 0)
	{
		firstCoeff = (controlPoints[2].time - controlPoints[0].time) /
			(controlPoints[2].time - controlPoints[1].time);

		firstTerm = (controlPoints[1].position - controlPoints[0].position) /
			(controlPoints[1].time - controlPoints[0].time);

		// second term is supposed to be subtracted, hence the -1.
		secondCoeff = -1 * (controlPoints[1].time - controlPoints[0].time) /
			(controlPoints[2].time - controlPoints[1].time);

		secondTerm = (controlPoints[2].position - controlPoints[0].position) /
			(controlPoints[2].time - controlPoints[0].time);
	}
	// Last Segment of Curve
	else if (point == controlPoints.size() - 1)
	{
		firstCoeff = (controlPoints[point].time-controlPoints[point-2].time) /
			(controlPoints[point-1].time-controlPoints[point-2].time);

		firstTerm = (controlPoints[point].position - controlPoints[point-1].position) /
			(controlPoints[point].time - controlPoints[point-1].time);

		// again, second term is subtracted.
		secondCoeff = -1 * (controlPoints[point].time-controlPoints[point-1].time) /
			(controlPoints[point-1].time-controlPoints[point-2].time);

		firstTerm = (controlPoints[point].position - controlPoints[point-2].position) /
			(controlPoints[point].time - controlPoints[point-2].time);
	}
	// Any segment in between
	else
	{
		// (t_i - t_{i-1}) / (t_{i+1} - t_{i-1})
		firstCoeff = (controlPoints[point].time - controlPoints[point-1].time) /
			(controlPoints[point+1].time - controlPoints[point-1].time);

		// (p_{i+1} - p_i) / (t_{i+1} - t_i)
		firstTerm = (controlPoints[point+1].position - controlPoints[point].position) /
			(controlPoints[point+1].time - controlPoints[point].time);

		// (t_{i+1} - t_i) / (t_{i+1} - t_{i-1})
		secondCoeff = (controlPoints[point+1].time - controlPoints[point].time) /
			(controlPoints[point+1].time - controlPoints[point-1].time);

		// (p_i - p_{i-1}) / (t_i - t_{i-1})
		secondTerm = (controlPoints[point].position - controlPoints[point-1].position) /
			(controlPoints[point].time - controlPoints[point-1].time);
	}

	return firstCoeff * firstTerm + secondCoeff * secondTerm;
}

// Implement Catmull-Rom curve
Point Curve::useCatmullCurve(const unsigned int nextPoint, const float time)
{
	float intervalTime = controlPoints[nextPoint].time - controlPoints[nextPoint-1].time;
	float normalTime = (time - controlPoints[nextPoint-1].time)/(intervalTime);

	Vector tangent0 = catmullTangent(nextPoint-1);
	Vector tangent1 = catmullTangent(nextPoint);

	Point firstTerm = hermite00(normalTime)*controlPoints[nextPoint-1].position;
	Vector secondTerm = hermite10(normalTime)*tangent0*intervalTime;
	Point thirdTerm = hermite01(normalTime)*controlPoints[nextPoint].position;
	Vector fourthTerm = hermite11(normalTime)*tangent1*intervalTime;

	return firstTerm + secondTerm + thirdTerm + fourthTerm;
}
