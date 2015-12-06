//
// Copyright (c) 2014-2015 VaHiD aZiZi
//
// Copyright (c) 2009-2014 Shawn Singh, Glen Berseth, Mubbasir Kapadia, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

#include "SteerLib.h"
#include "SocialForcesAgent.h"
#include "SocialForcesAIModule.h"
#include "SocialForces_Parameters.h"
// #include <math.h>


// #include "util/Geometry.h"


/// @file SocialForcesAgent.cpp
/// @brief Implements the SocialForcesAgent class.


#undef min
#undef max


#define AGENT_MASS 1.0f
float DIVIDE_GOAL = 5.0f;

using namespace Util;
using namespace SocialForcesGlobals;
using namespace SteerLib;


// #define _DEBUG_ENTROPY 1


SocialForcesAgent::SocialForcesAgent()
{
	_SocialForcesParams.sf_acceleration = sf_acceleration;
	_SocialForcesParams.sf_personal_space_threshold = sf_personal_space_threshold;
	_SocialForcesParams.sf_agent_repulsion_importance = sf_agent_repulsion_importance;
	_SocialForcesParams.sf_query_radius = sf_query_radius;
	_SocialForcesParams.sf_body_force = sf_body_force;
	_SocialForcesParams.sf_agent_body_force = sf_agent_body_force;
	_SocialForcesParams.sf_sliding_friction_force = sf_sliding_friction_force;
	_SocialForcesParams.sf_agent_b = sf_agent_b;
	_SocialForcesParams.sf_agent_a = sf_agent_a;
	_SocialForcesParams.sf_wall_b = sf_wall_b;
	_SocialForcesParams.sf_wall_a = sf_wall_a;
	_SocialForcesParams.sf_max_speed = sf_max_speed;

	_enabled = false;
}


SocialForcesAgent::~SocialForcesAgent()
{
}


void SocialForcesAgent::setParameters(Behaviour behave)
{
	this->_SocialForcesParams.setParameters(behave);
}


void SocialForcesAgent::disable()
{
	// DO nothing for now
	// if we tried to disable a second time, most likely we accidentally ignored that it was disabled, and should catch that error.
	// std::cout << "this agent is being disabled " << this << std::endl;
	assert(_enabled==true);

	//  1. remove from database
	AxisAlignedBox b = AxisAlignedBox(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);
	gSpatialDatabase->removeObject(dynamic_cast<SpatialDatabaseItemPtr>(this), b);

	//  2. set enabled = false
	_enabled = false;
}


void SocialForcesAgent::reset(const SteerLib::AgentInitialConditions & initialConditions, SteerLib::EngineInterface * engineInfo)
{
	// compute the "old" bounding box of the agent before it is reset.  its OK that it will be invalid if the agent was previously disabled
	// because the value is not used in that case.
	 //std::cout << "resetting agent " << "\n";
	_waypoints.clear();
	_midTermPath.clear();

	Util::AxisAlignedBox oldBounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.5f, _position.z-_radius, _position.z+_radius);

	// initialize the agent based on the initial conditions
	/*
	position_ = Vector2(initialConditions.position.x, initialConditions.position.z);
	radius_ = initialConditions.radius;
	velocity_ = normalize(Vector2(initialConditions.direction.x, initialConditions.direction.z));
	velocity_ = velocity_ * initialConditions.speed;
*/
	// initialize the agent based on the initial conditions
	_position = initialConditions.position;
	_forward = normalize(initialConditions.direction);
	_radius = initialConditions.radius;
	_velocity = initialConditions.speed * _forward;
	
	PathOfTestCase = initialConditions.TestCasePath;
	
	//std::cout << TestCasePath << "\n";
	// std::cout << "inital colour of agent " << initialConditions.color << std::endl;
	if ( initialConditions.colorSet == true )
	{
		this->_color = initialConditions.color;
	}
	else
	{
		this->_color = Util::gBlue;
	}

	// compute the "new" bounding box of the agent
	Util::AxisAlignedBox newBounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.5f, _position.z-_radius, _position.z+_radius);

	if (!_enabled) {
		// if the agent was not enabled, then it does not already exist in the database, so add it.
		// std::cout
		gSpatialDatabase->addObject( dynamic_cast<SpatialDatabaseItemPtr>(this), newBounds);
	}
	else {
		// if the agent was enabled, then the agent already existed in the database, so update it instead of adding it.
		// std::cout << "new position is " << _position << std::endl;
		// std::cout << "new bounds are " << newBounds << std::endl;
		// std::cout << "reset update " << this << std::endl;
		gSpatialDatabase->updateObject( dynamic_cast<SpatialDatabaseItemPtr>(this), oldBounds, newBounds);
		// engineInfo->getSpatialDatabase()->updateObject( this, oldBounds, newBounds);
	}

	_enabled = true;

	if (initialConditions.goals.size() == 0)
	{
		throw Util::GenericException("No goals were specified!\n");
	}

	while (!_goalQueue.empty())
	{
		_goalQueue.pop();
	}

	// iterate over the sequence of goals specified by the initial conditions.
	for (unsigned int i=0; i<initialConditions.goals.size(); i++) {
		if (initialConditions.goals[i].goalType == SteerLib::GOAL_TYPE_SEEK_STATIC_TARGET ||
				initialConditions.goals[i].goalType == GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL)
		{
			_goalQueue.push(initialConditions.goals[i]);
			if (initialConditions.goals[i].targetIsRandom)
			{
				// if the goal is random, we must randomly generate the goal.
				// std::cout << "assigning random goal" << std::endl;
				_goalQueue.back().targetLocation = gSpatialDatabase->randomPositionWithoutCollisions(1.0f, true);
			}
		}
		else {
			throw Util::GenericException("Unsupported goal type; SocialForcesAgent only supports GOAL_TYPE_SEEK_STATIC_TARGET and GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL.");
		}
	}

	//computePlan();
	//std::cout << initialConditions.goals.size() << "\n";
	if (PathOfTestCase.find("maze") != std::string::npos || PathOfTestCase.find("office-complex") != std::string::npos) 
	{
		
		runAStarPlanning();
	}
	else if(PathOfTestCase.find("plane_egress") != std::string::npos)
        {
            hardCodePathEgress();   
        }
        else if(PathOfTestCase.find("plane_ingress") != std::string::npos)
        {
            hardCodePathIngress();   
        }
        else if(PathOfTestCase.find("rounded-roundabout") != std::string::npos)
        {
            DIVIDE_GOAL = 2.0f;
            hardCodePathVortex();
        }
        else if(PathOfTestCase.find("crowd_crossing") != std::string::npos)
	{
	  hardCodeCrowdCrossing();
	  
	}
	else if(PathOfTestCase.find("wall-squeeze") != std::string::npos)
	{
	 hardCodeWallSqueeze(); 
	}
	/*else if(PathOfTestCase.find("doorway-two-way") != std::string::npos)
	{
	 hardCodeDoorway(); 
	}*/
	else
	{
	runLongTermPlanning();
            //hardCodePathVortex();
            //runAStarPlanning();
            
	}
	// std::cout << "first waypoint: " << _waypoints.front() << " agents position: " << position() << std::endl;
	/*
	 * Must make sure that _waypoints.front() != position(). If they are equal the agent will crash.
	 * And that _waypoints is not empty
	 */
	Util::Vector goalDirection;
	if ( !_midTermPath.empty() )
	{
		this->updateLocalTarget();
		goalDirection = normalize( this->_currentLocalTarget - position());
	}
	else
	{
		goalDirection = normalize( _goalQueue.front().targetLocation - position());
	}

	_prefVelocity =
			(
				(
					(
						Util::Vector(goalDirection.x, 0.0f, goalDirection.z) *
						PREFERED_SPEED
					)
				- velocity()
				)
				/
				_SocialForcesParams.sf_acceleration
			)
			*
			MASS;

	// _velocity = _prefVelocity;
#ifdef _DEBUG_ENTROPY
	std::cout << "goal direction is: " << goalDirection << " prefvelocity is: " << prefVelocity_ <<
			" and current velocity is: " << velocity_ << std::endl;
#endif

	// std::cout << "Parameter spec: " << _SocialForcesParams << std::endl;
	// gEngine->addAgent(this, rvoModule);
	assert(_forward.length()!=0.0f);
	assert(_goalQueue.size() != 0);
	assert(_radius != 0.0f);
}


void SocialForcesAgent::calcNextStep(float dt)
{
    //nothing to do here
}


std::pair<float, Util::Point> minimum_distance(Util::Point l1, Util::Point l2, Util::Point p)
{
  // Return minimum distance between line segment vw and point p
  float lSq = (l1 - l2).lengthSquared();  // i.e. |l2-l1|^2 -  avoid a sqrt
  if (lSq == 0.0)
	  return std::make_pair((p - l2).length(),l1 );   // l1 == l2 case
  // Consider the line extending the segment, parameterized as l1 + t (l2 - l1).
  // We find projection of point p onto the line.
  // It falls where t = [(p-l1) . (l2-l1)] / |l2-l1|^2
  const float t = dot(p - l1, l2 - l1) / lSq;
  if (t < 0.0)
  {
	  return std::make_pair((p - l1).length(), l1);       // Beyond the 'l1' end of the segment
  }
  else if (t > 1.0)
  {
	  return std::make_pair((p - l2).length(), l2);  // Beyond the 'l2' end of the segment
  }
  const Util::Point projection = l1 + t * (l2 - l1);  // Projection falls on the segment
  return std::make_pair((p - projection).length(), projection) ;
}


Util::Vector SocialForcesAgent::calcProximityForce(float dt)
{
  SteerLib::AgentInterface * tempAgent;
  SteerLib::ObstacleInterface * tempOb;
  Util::Vector away = Util::Vector(0,0,0);
  Util::Vector awayOb = Util::Vector(0,0,0);
  Util::Vector proximity_force = Util::Vector(0,0,0);
  
    std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;
		gSpatialDatabase->getItemsInRange(_neighbors,
				_position.x-(this->_radius + _SocialForcesParams.sf_query_radius),
				_position.x+(this->_radius + _SocialForcesParams.sf_query_radius),
				_position.z-(this->_radius + _SocialForcesParams.sf_query_radius),
				_position.z+(this->_radius + _SocialForcesParams.sf_query_radius),
				dynamic_cast<SteerLib::SpatialDatabaseItemPtr>(this));
		
    for(std::set<SteerLib::SpatialDatabaseItemPtr>:: iterator neighbor = _neighbors.begin(); neighbor!=_neighbors.end(); neighbor++)
    {
     if( (*neighbor)->isAgent() )
     {
      tempAgent = dynamic_cast<SteerLib::AgentInterface *>(*neighbor);
      
      Util::Vector awayTemp = normalize(position() - tempAgent->position());
      
      away = away + awayTemp*(_SocialForcesParams.sf_agent_a*exp((((this->radius()+tempAgent->radius())-(this->position()-tempAgent->position()).length())/_SocialForcesParams.sf_agent_b)));
      
       
     }
     
     else
     {
      tempOb = dynamic_cast<SteerLib::ObstacleInterface *>(*neighbor);
      Util::Vector wallNormal = calcWallNormal(tempOb);
      std::pair<Util::Point,Util::Point> line = calcWallPointsFromNormal(tempOb, wallNormal);
      std::pair<float, Util::Point> min_stuff = minimum_distance(line.first,line.second,position());
      
      Util::Vector awayObTemp = normalize(position()-min_stuff.second);
      
      awayOb = awayOb + awayObTemp * (_SocialForcesParams.sf_wall_a*exp((((this->radius())-(this->position()-min_stuff.second).length())/_SocialForcesParams.sf_wall_b)));

     }
    }
    proximity_force = away + awayOb;
    //proximity_force = ((proximity_force)/ AGENT_MASS)* dt; 
    return proximity_force;
}


Vector SocialForcesAgent::calcGoalForce(Vector _goalDirection, float _dt)
{
	//std::cout << PathOfTestCase << "\n";
    //std::cerr<<"<<<calcGoalForce>>> Please Implement my body\n";
    //Util::Vector prefForce = ((_goalDirection*PERFERED_SPEED - velocity()) / _dt);
   // Util::Vector prefForce = (((_goalDirection * PERFERED_SPEED) - velocity()) / (_SocialForcesParams.sf_acceleration/_dt)) + velocity();
  //Util::Vector goalForce = (_goalDirection * PERFERED_SPEED);
  Util::Vector goalForce = (((_goalDirection * PREFERED_SPEED) - velocity()) / _dt)/DIVIDE_GOAL;
  return goalForce;
    
}


Util::Vector SocialForcesAgent::calcRepulsionForce(float dt)
{
#ifdef _DEBUG_
	std::cout << "wall repulsion; " << calcWallRepulsionForce(dt) << " agent repulsion " <<
			(_SocialForcesParams.sf_agent_repulsion_importance * calcAgentRepulsionForce(dt)) << std::endl;
#endif
	return calcWallRepulsionForce(dt) + (_SocialForcesParams.sf_agent_repulsion_importance * calcAgentRepulsionForce(dt));
}


Util::Vector SocialForcesAgent::calcAgentRepulsionForce(float dt)
{
    //std::cerr<<"<<<calcAgentRepulsionForce>>> Please Implement my body\n";

    Util::Vector agent_repulsion_force =  Util::Vector(0,0,0);
    std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;
		gSpatialDatabase->getItemsInRange(_neighbors,
				_position.x-(this->_radius + _SocialForcesParams.sf_query_radius),
				_position.x+(this->_radius + _SocialForcesParams.sf_query_radius),
				_position.z-(this->_radius + _SocialForcesParams.sf_query_radius),
				_position.z+(this->_radius + _SocialForcesParams.sf_query_radius),
				dynamic_cast<SteerLib::SpatialDatabaseItemPtr>(this));
		
   SteerLib::AgentInterface * tempAgent;
   
   for(std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbor = _neighbors.begin(); neighbor != _neighbors.end(); neighbor++)
   {
    if( (*neighbor)->isAgent() )
      tempAgent = dynamic_cast<SteerLib::AgentInterface *> (*neighbor);
    
    else
      continue;
    
    if( (id() != tempAgent->id() ) && (tempAgent->computePenetration(this->position(),this->radius()) > 0.000001))
    {
     agent_repulsion_force = agent_repulsion_force + ( tempAgent->computePenetration(this->position(),this->radius()) * _SocialForcesParams.sf_agent_body_force * dt) * normalize(position() - tempAgent->position());  
     
    }
     
   }
   //agent_repulsion_force = (agent_repulsion_force / AGENT_MASS)* dt; 
   return agent_repulsion_force;
}


Util::Vector SocialForcesAgent::calcWallRepulsionForce(float dt)
{
  Util::Vector wall_repulsion_force = Util::Vector(0,0,0);

	std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;
		gSpatialDatabase->getItemsInRange(_neighbors,
				_position.x-(this->_radius + _SocialForcesParams.sf_query_radius),
				_position.x+(this->_radius + _SocialForcesParams.sf_query_radius),
				_position.z-(this->_radius + _SocialForcesParams.sf_query_radius),
				_position.z+(this->_radius + _SocialForcesParams.sf_query_radius),
				dynamic_cast<SteerLib::SpatialDatabaseItemPtr>(this));
		
	SteerLib::ObstacleInterface * tmp_ob;
	
	for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbor = _neighbors.begin(); neighbor!=_neighbors.end(); neighbor++)
	{
	 if(!(*neighbor)->isAgent())
	   tmp_ob = dynamic_cast<SteerLib::ObstacleInterface *>(*neighbor);
	 
	 else
	   continue;
	 
	 if(tmp_ob->computePenetration(this->position(), this->radius()) > 0.000001)
	 {
	  Util::Vector wallNormal = calcWallNormal(tmp_ob);
	  std::pair<Util::Point,Util::Point> line = calcWallPointsFromNormal(tmp_ob, wallNormal);
	  std::pair<float, Util::Point> min_stuff = minimum_distance(line.first, line.second, this->position());
	  wall_repulsion_force = wall_repulsion_force +  wallNormal * (min_stuff.first + this->radius()) * _SocialForcesParams.sf_body_force*dt;

	 }
	  
	}
	//wall_repulsion_force = (wall_repulsion_force / AGENT_MASS)* dt; 
	return wall_repulsion_force;

}


std::pair<Util::Point, Util::Point> SocialForcesAgent::calcWallPointsFromNormal(SteerLib::ObstacleInterface* obs, Util::Vector normal)
{
	Util::AxisAlignedBox box = obs->getBounds();
	if ( normal.z == 1)
	{
		return std::make_pair(Util::Point(box.xmin,0,box.zmax), Util::Point(box.xmax,0,box.zmax));
		// Ended here;
	}
	else if ( normal.z == -1 )
	{
		return std::make_pair(Util::Point(box.xmin,0,box.zmin), Util::Point(box.xmax,0,box.zmin));
	}
	else if ( normal.x == 1)
	{
		return std::make_pair(Util::Point(box.xmax,0,box.zmin), Util::Point(box.xmax,0,box.zmax));
	}
	else // normal.x == -1
	{
		return std::make_pair(Util::Point(box.xmin,0,box.zmin), Util::Point(box.xmin,0,box.zmax));
	}
}

/**
 * Basically What side of the obstacle is the agent on use that as the normal
 * DOES NOT SUPPORT non-axis-aligned boxes
 *
 *
 * 			   \		   /
 * 				\		  /
 * 				 \	 a	 /
 *				  \		/
 * 					 _
 * 			a		| |       a
 * 					 -
 * 				  /     \
 * 				 /   a   \
 * 				/	      \
 * 			   /	       \
 *
 *
 */


Util::Vector SocialForcesAgent::calcWallNormal(SteerLib::ObstacleInterface* obs)
{
	Util::AxisAlignedBox box = obs->getBounds();
	if ( position().x > box.xmax )
	{
		if ( position().z > box.zmax)
		{
			if ( abs(position().z - box.zmax ) >
				abs( position().x - box.xmax) )
			{
				return Util::Vector(0, 0, 1);
			}
			else
			{
				return Util::Vector(1, 0, 0);
			}

		}
		else if ( position().z < box.zmin )
		{
			if ( abs(position().z - box.zmin ) >
				abs( position().x - box.xmax) )
			{
				return Util::Vector(0, 0, -1);
			}
			else
			{
				return Util::Vector(1, 0, 0);
			}

		}
		else
		{ // in between zmin and zmax
			return Util::Vector(1, 0, 0);
		}

	}
	else if ( position().x < box.xmin )
	{
		if ( position().z > box.zmax )
		{
			if ( abs(position().z - box.zmax ) >
				abs( position().x - box.xmin) )
			{
				return Util::Vector(0, 0, 1);
			}
			else
			{
				return Util::Vector(-1, 0, 0);
			}

		}
		else if ( position().z < box.zmin )
		{
			if ( abs(position().z - box.zmin ) >
				abs( position().x - box.xmin) )
			{
				return Util::Vector(0, 0, -1);
			}
			else
			{
				return Util::Vector(-1, 0, 0);
			}

		}
		else
		{ // in between zmin and zmax
			return Util::Vector(-1, 0, 0);
		}
	}
	else // between xmin and xmax
	{
		if ( position().z > box.zmax )
		{
			return Util::Vector(0, 0, 1);
		}
		else if ( position().z < box.zmin)
		{
			return Util::Vector(0, 0, -1);
		}
		else
		{ // What do we do if the agent is inside the wall?? Lazy Normal
			return calcObsNormal( obs );
		}
	}

}

/**
 * Treats Obstacles as a circle and calculates normal
 */
Util::Vector SocialForcesAgent::calcObsNormal(SteerLib::ObstacleInterface* obs)
{
	Util::AxisAlignedBox box = obs->getBounds();
	Util::Point obs_centre = Util::Point((box.xmax+box.xmin)/2, (box.ymax+box.ymin)/2,
			(box.zmax+box.zmin)/2);
	return normalize(position() - obs_centre);
}


bool SocialForcesAgent::reachedCurrentWaypoint()
{

	if ( !_waypoints.empty())
	{
		return (position() - _waypoints.front()).lengthSquared() <= (radius()*radius());
	}
	else
	{
		false;
	}

	// return (position() - _currentLocalTarget).lengthSquared() < (radius()*radius());
}


bool SocialForcesAgent::hasLineOfSightTo(Util::Point target)
{
	float dummyt;
	Util::Vector _rightSide = rightSideInXZPlane(this->forward());
	SpatialDatabaseItemPtr dummyObject;
	Ray lineOfSightTestRight, lineOfSightTestLeft;
	// lineOfSightTestRight.initWithUnitInterval(_position + _radius*_rightSide, target - (_position + _radius*_rightSide));
	// lineOfSightTestLeft.initWithUnitInterval(_position + _radius*(_rightSide), target - (_position - _radius*_rightSide));
	lineOfSightTestRight.initWithUnitInterval(_position + _radius*_rightSide, target - _position);
	lineOfSightTestLeft.initWithUnitInterval(_position + _radius*(_rightSide), target - _position);

	return (!gSpatialDatabase->trace(lineOfSightTestRight,dummyt, dummyObject, dynamic_cast<SpatialDatabaseItemPtr>(this),true))
		&& (!gSpatialDatabase->trace(lineOfSightTestLeft,dummyt, dummyObject, dynamic_cast<SpatialDatabaseItemPtr>(this),true));

}


void SocialForcesAgent::updateAI(float timeStamp, float dt, unsigned int frameNumber)
{
	 //std::cout << "updating";
	// std::cout << "_SocialForcesParams.rvo_max_speed " << _SocialForcesParams._SocialForcesParams.rvo_max_speed << std::endl;
	Util::AutomaticFunctionProfiler profileThisFunction( &SocialForcesGlobals::gPhaseProfilers->aiProfiler );
	if (!enabled())
	{
		return;
	}

	Util::AxisAlignedBox oldBounds(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);

	SteerLib::AgentGoalInfo goalInfo = _goalQueue.front();
	Util::Vector goalDirection;
	if ( ! _midTermPath.empty() && (!this->hasLineOfSightTo(goalInfo.targetLocation)) )
	{
		if (reachedCurrentWaypoint())
		{
			this->updateMidTermPath();
		}

		this->updateLocalTarget();

		goalDirection = normalize(_currentLocalTarget - position());

	}
	else
	{
		goalDirection = normalize(goalInfo.targetLocation - position());
	}

    /*
     *  Goal Force
     */
    Util::Vector prefForce = calcGoalForce( goalDirection, dt );

    /*
     *  Repulsion Force
     */
	Util::Vector repulsionForce = calcRepulsionForce(dt);

	if ( repulsionForce.x != repulsionForce.x)
	{
		std::cout << "Found some nan" << std::endl;
		// repulsionForce = velocity() * 0;
	}

    /*
     *  Proximity Force
     */
	Util::Vector proximityForce = calcProximityForce(dt);

// #define _DEBUG_ 1
#ifdef _DEBUG_
	std::cout << "agent" << id() << " repulsion force " << repulsionForce << std::endl;
	std::cout << "agent" << id() << " proximity force " << proximityForce << std::endl;
	std::cout << "agent" << id() << " pref force " << prefForce << std::endl;
#endif
	// _velocity = _newVelocity;
	int alpha=1;
	if ( repulsionForce.length() > 0.0)
	{
		alpha=0;
	}

	Util::Vector acceleration = (prefForce + repulsionForce + proximityForce) / AGENT_MASS;
	_velocity = velocity() + acceleration * dt;
	
	//_velocity = velocity() + (prefForce) + repulsionForce + proximityForce;

	_velocity = clamp(velocity(), _SocialForcesParams.sf_max_speed);
	_velocity.y=0.0f;
#ifdef _DEBUG_
	std::cout << "agent" << id() << " speed is " << velocity().length() << std::endl;
#endif
	_position = position() + (velocity() * dt);
	// A grid database update should always be done right after the new position of the agent is calculated
	/*
	 * Or when the agent is removed for example its true location will not reflect its location in the grid database.
	 * Not only that but this error will appear random depending on how well the agent lines up with the grid database
	 * boundaries when removed.
	 */
	// std::cout << "Updating agent" << this->id() << " at " << this->position() << std::endl;
	Util::AxisAlignedBox newBounds(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);
	gSpatialDatabase->updateObject( this, oldBounds, newBounds);

/*
	if ( ( !_waypoints.empty() ) && (_waypoints.front() - position()).length() < radius()*WAYPOINT_THRESHOLD_MULTIPLIER)
	{
		_waypoints.erase(_waypoints.begin());
	}
	*/
	/*
	 * Now do the conversion from SocialForcesAgent into the SteerSuite coordinates
	 */
	// _velocity.y = 0.0f;

	if ((goalInfo.targetLocation - position()).length() < radius()*GOAL_THRESHOLD_MULTIPLIER ||
			(goalInfo.goalType == GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL &&
					Util::boxOverlapsCircle2D(goalInfo.targetRegion.xmin, goalInfo.targetRegion.xmax,
							goalInfo.targetRegion.zmin, goalInfo.targetRegion.zmax, this->position(), this->radius())))
	{
		_goalQueue.pop();
		// std::cout << "Made it to a goal" << std::endl;
		if (_goalQueue.size() != 0)
		{
			// in this case, there are still more goals, so start steering to the next goal.
			goalDirection = _goalQueue.front().targetLocation - _position;
			_prefVelocity = Util::Vector(goalDirection.x, 0.0f, goalDirection.z);
		}
		else
		{
			// in this case, there are no more goals, so disable the agent and remove it from the spatial database.
			disable();
			return;
		}
	}

	// Hear the 2D solution from RVO is converted into the 3D used by SteerSuite
	// _velocity = Vector(velocity().x, 0.0f, velocity().z);
	if ( velocity().lengthSquared() > 0.0 )
	{
		// Only assign forward direction if agent is moving
		// Otherwise keep last forward
		_forward = normalize(_velocity);
	}
	// _position = _position + (_velocity * dt);

}


/**
 * Removes a number of points from the begining of the path
 *
 * After size of path should be old size - FURTHEST_LOCAL_TARGET_DISTANCE
 */
void SocialForcesAgent::updateMidTermPath()
{
	if ( this->_midTermPath.size() < FURTHEST_LOCAL_TARGET_DISTANCE)
	{
		return;
	}
	if ( !_waypoints.empty())
	{
		_waypoints.erase(_waypoints.begin());
	}
	// save good path
	std::vector<Util::Point> tmpPath;
	// std::cout << "midterm path size " << _midTermPath.size() << std::endl;
	// std::cout << "distance between position and current waypoint " << (position() - _waypoints.front()).length() << std::endl;
	for (unsigned int i=(FURTHEST_LOCAL_TARGET_DISTANCE); i < _midTermPath.size();i++ )
	{
		tmpPath.push_back(_midTermPath.at(i));
	}
	_midTermPath.clear();

	for (unsigned int i=0; i < tmpPath.size(); i++)
	{
		_midTermPath.push_back(tmpPath.at(i));
	}

}


/**
 * Update the local target to the furthest point on the midterm path the agent can see.
 */
void SocialForcesAgent::updateLocalTarget()
{
	Util::Point tmpTarget = this->_goalQueue.front().targetLocation;
	unsigned int i=0;
	for (i=0; (i < FURTHEST_LOCAL_TARGET_DISTANCE) &&
			i < this->_midTermPath.size(); i++ )
	{
		tmpTarget = this->_midTermPath.at(i);
		if ( this->hasLineOfSightTo(tmpTarget) )
		{
			this->_currentLocalTarget = tmpTarget;
		}
	}
}


/**
 * finds a path to the current goal
 * puts that path in midTermPath
 */

bool SocialForcesAgent::hardCodeDoorway()
{
 if(position().x == -10)
 {
    SteerLib::AgentGoalInfo goalPoint =_goalQueue.front();
    _goalQueue.pop();
    SteerLib::AgentGoalInfo newGoal1;
    SteerLib::AgentGoalInfo newGoal2;
    SteerLib::AgentGoalInfo newGoal3;
    newGoal1.targetLocation = Point(-10,0,2);
    newGoal2.targetLocation = Point(-2,0,2);
    newGoal3.targetLocation = Point(0,0,1);
    _goalQueue.push(newGoal1);
    _goalQueue.push(newGoal2);
    _goalQueue.push(newGoal3);
    _goalQueue.push(goalPoint);
    runLongTermPlanning();
    return true;
   
 }
}

bool SocialForcesAgent::hardCodeWallSqueeze()
{
  
/*  if(position().x == 10 && position().z == 0)
  {
    
	    SteerLib::AgentGoalInfo goalPoint =_goalQueue.front();
             _goalQueue.pop();
	     SteerLib::AgentGoalInfo newGoal1;
             SteerLib::AgentGoalInfo newGoal2;
	     newGoal1.targetLocation = Point(1,0,1);
             newGoal2.targetLocation = Point(-0.5,0,0.5);
	     _goalQueue.push(newGoal1);
             _goalQueue.push(newGoal2);
             _goalQueue.push(goalPoint);
             runLongTermPlanning();
             return true;
  } */
	    SteerLib::AgentGoalInfo goalPoint =_goalQueue.front();
             _goalQueue.pop();
             SteerLib::AgentGoalInfo newGoal;
             newGoal.targetLocation = Point(-0.5,0,0.5);
             _goalQueue.push(newGoal);
             _goalQueue.push(goalPoint);
             runLongTermPlanning();
             return true;
  
}
bool SocialForcesAgent::hardCodePathVortex()
{
    std::vector<Util::Point> agentPath;
    if(position().z > 10) //top
    {
         //if(!astar.computePath(agentPath,position(),Point(9,0,4),gSpatialDatabase))
         //{
	//	return false;
         //}
         
         if(_goalQueue.front().targetLocation.z < -10)
         {
             SteerLib::AgentGoalInfo goalPoint =_goalQueue.front();
             _goalQueue.pop();
             SteerLib::AgentGoalInfo newGoal;
             SteerLib::AgentGoalInfo newGoal1;
             SteerLib::AgentGoalInfo newGoal2;
             newGoal.targetLocation = Point(9,0,4);
             newGoal1.targetLocation = Point(9,0,-6);
             _goalQueue.push(newGoal);
             _goalQueue.push(newGoal1);
             _goalQueue.push(goalPoint);
             runLongTermPlanning();
             return true;
             //newGoal2.targetLocation = Point(9,0,4);
            // _goalQueue.insert(_goalQueue.end(),agentPath.begin(),agentPath.end());
            // agentPath.push_back(Point(9,0,4));
            //  std::vector<Util::Point> agentPath1;
            // astar.computePath(agentPath1,Point(9,0,4),Point(9,0,-6),gSpatialDatabase);
            // agentPath.insert(agentPath.end(), agentPath1.begin(), agentPath1.end());
            //_goalQueue.insert(_goalQueue.end(),agentPath1.begin(),agentPath1.end());
             
             //agentPath.push_back(Point(9,0,-6));
             
         }
         /*else if(_goalQueue.front().targetLocation.x < -10)
         {
             SteerLib::AgentGoalInfo goalPoint =_goalQueue.front();
             _goalQueue.pop();
             SteerLib::AgentGoalInfo newGoal;
             SteerLib::AgentGoalInfo newGoal1;
             SteerLib::AgentGoalInfo newGoal2;
             newGoal.targetLocation = Point(9,0,4);
             newGoal1.targetLocation = Point(9,0,-6);
             newGoal2.targetLocation = Point(-8,0,-6);
             _goalQueue.push(newGoal);
             _goalQueue.push(newGoal1);
             _goalQueue.push(newGoal2);
             _goalQueue.push(goalPoint);
             runLongTermPlanning();
             return true;
             
             
         }
        */
    }
    else if(position().x > 10) //left 
    {
        //if(!astar.computePath(agentPath,position(),Point(6,0,-6),gSpatialDatabase))
         //{
	//	return false;
         //}
         
         if(_goalQueue.front().targetLocation.x < -10)
         {
             SteerLib::AgentGoalInfo goalPoint =_goalQueue.front();
             _goalQueue.pop();
             SteerLib::AgentGoalInfo newGoal;
             SteerLib::AgentGoalInfo newGoal1;
             SteerLib::AgentGoalInfo newGoal2;
             newGoal.targetLocation = Point(6,0,-6);
             newGoal1.targetLocation = Point(-8,0,-6);
             _goalQueue.push(newGoal);
             _goalQueue.push(newGoal1);
             _goalQueue.push(goalPoint);
             runLongTermPlanning();
             return true;
             
             
         }
         /*else if(_goalQueue.front().targetLocation.z > 10)
         {
             
             SteerLib::AgentGoalInfo goalPoint =_goalQueue.front();
             _goalQueue.pop();
             SteerLib::AgentGoalInfo newGoal;
             SteerLib::AgentGoalInfo newGoal1;
             SteerLib::AgentGoalInfo newGoal2;
             newGoal.targetLocation = Point(6,0,-6);
             newGoal1.targetLocation = Point(-8,0,-6);
             newGoal2.targetLocation = Point(-8,0,4);
             _goalQueue.push(newGoal);
             _goalQueue.push(newGoal1);
             _goalQueue.push(newGoal2);
             _goalQueue.push(goalPoint);
             runLongTermPlanning();
             return true;
          
             
         }
         */
        
    }
    else if(position().x < -10)
    {
        //if(!astar.computePath(agentPath,position(),Point(-8,0,4),gSpatialDatabase))
        // {
	//	return false;
        // }
         
         if(_goalQueue.front().targetLocation.x > 10)
         {
              SteerLib::AgentGoalInfo goalPoint =_goalQueue.front();
             _goalQueue.pop();
             SteerLib::AgentGoalInfo newGoal;
             SteerLib::AgentGoalInfo newGoal1;
             SteerLib::AgentGoalInfo newGoal2;
             newGoal.targetLocation = Point(-8,0,4);
             newGoal1.targetLocation = Point(6,0,4);
             _goalQueue.push(newGoal);
             _goalQueue.push(newGoal1);
             _goalQueue.push(goalPoint);
             runLongTermPlanning();
             return true;
             
             
         }
         /*else if(_goalQueue.front().targetLocation.z < -10)
         {
             SteerLib::AgentGoalInfo goalPoint =_goalQueue.front();
             _goalQueue.pop();
             SteerLib::AgentGoalInfo newGoal;
             SteerLib::AgentGoalInfo newGoal1;
             SteerLib::AgentGoalInfo newGoal2;
             newGoal.targetLocation = Point(-8,0,4);
             newGoal1.targetLocation = Point(6,0,4);
             newGoal2.targetLocation = Point(6,0,-6);
             _goalQueue.push(newGoal);
             _goalQueue.push(newGoal1);
             _goalQueue.push(newGoal2);
             _goalQueue.push(goalPoint);
             runLongTermPlanning();
             return true;
             
             
         }
         */
        
    }
    else //(position.z < -10)
    {
         //if(!astar.computePath(agentPath,position(),Point(-8,0,-6),gSpatialDatabase))
        // {
	//	return false;
         //}
         
         if(_goalQueue.front().targetLocation.z > 10)
         {
              SteerLib::AgentGoalInfo goalPoint =_goalQueue.front();
             _goalQueue.pop();
             SteerLib::AgentGoalInfo newGoal;
             SteerLib::AgentGoalInfo newGoal1;
             SteerLib::AgentGoalInfo newGoal2;
             newGoal.targetLocation = Point(-8,0,-6);
             newGoal1.targetLocation = Point(-8,0,4);
             _goalQueue.push(newGoal);
             _goalQueue.push(newGoal1);
             _goalQueue.push(goalPoint);
             runLongTermPlanning();
             return true;
             
             
         }
         /*else if(_goalQueue.front().targetLocation.x > 10)
         {
             SteerLib::AgentGoalInfo goalPoint =_goalQueue.front();
             _goalQueue.pop();
             SteerLib::AgentGoalInfo newGoal;
             SteerLib::AgentGoalInfo newGoal1;
             SteerLib::AgentGoalInfo newGoal2;
             newGoal.targetLocation = Point(-8,0,-6);
             newGoal1.targetLocation = Point(-8,0,4);
             newGoal2.targetLocation = Point(6,0,4);
             _goalQueue.push(newGoal);
             _goalQueue.push(newGoal1);
             _goalQueue.push(newGoal2);
             _goalQueue.push(goalPoint);
             runLongTermPlanning();
             return true;
             
             
             
         }
        */
        
    }
    
    //if(!astar.computePath(agentPath,position(),Point(6,0,4),gSpatialDatabase))
     //{
//		return false;
    // }       
    //agentPath.push_back(Point(6,0,4));
    //agentPath.push_back(Point(-8,0,4));
    //agentPath.push_back(Point(-8,0,-6));
    //agentPath.push_back(Point(6,0,-6));
    
    
    for  (int i=0; i <  agentPath.size(); i++)
	{
		_midTermPath.push_back(agentPath.at(i));
		if ((i % FURTHEST_LOCAL_TARGET_DISTANCE) == 0)
		{
			_waypoints.push_back(agentPath.at(i));
		}
	}
	if(agentPath.size()>0)
	{
		for(int i = 1; i<agentPath.size(); ++i)
		{
			Util::DrawLib::drawLine(agentPath[i-1], agentPath[i], gYellow);
		}
		//Util::DrawLib::drawCircle(__path[__path.size()-1], Util::Color(0.0f, 1.0f, 0.0f));
	}

	return true;
    
    
}

bool SocialForcesAgent::hardCodeCrowdCrossing()
{
 std::vector<Util::Point> agentPath;

 if(position().x <= -4)
 {
  _goalQueue.pop();
  SteerLib::AgentGoalInfo newGoal;
  
  if(position().z <= 4 && position().z >= 3)
  {
    newGoal.targetLocation = Point(-6,0,3);
    _goalQueue.push(newGoal);
    runLongTermPlanning();
    return true;
  }
  
  else if(position().z <= 2 && position().z >= 1)
  {
    newGoal.targetLocation = Point(-7,0,2);
    _goalQueue.push(newGoal);
    runLongTermPlanning();
    return true;
  }
  
  else if(position().z <= 1 && position().z >= -1)
  {
      newGoal.targetLocation = Point(-6,0,1);
    _goalQueue.push(newGoal);
    runLongTermPlanning();
    return true;
  }
  
  else if(position().z <= -1 && position().z >= -3)
  {
    newGoal.targetLocation = Point(-7,0,-1);
    _goalQueue.push(newGoal);
    runLongTermPlanning();
    return true;
  }
  
  else
  {
    newGoal.targetLocation = Point(-7,0,-3);
    _goalQueue.push(newGoal);
    runLongTermPlanning();
    return true;
    
  }
 }
 
  if(position().x >= -4 && position().z >= 3)
  {
    _goalQueue.pop();
    SteerLib::AgentGoalInfo newGoal;
    newGoal.targetLocation = Point(-4,0,4);
    _goalQueue.push(newGoal);
    runLongTermPlanning();
    return true;
  }
 /* else if(position().x >= -4 && position().z >= -7 && position().z <=-10)
  {
    _goalQueue.pop();
    SteerLib::AgentGoalInfo newGoal;
    newGoal.targetLocation = Point(-4,0,4);
    _goalQueue.push(newGoal);
    runLongTermPlanning();
    return true;
  }*/
 else
 {
  runLongTermPlanning();
  return true;
 }
  
  
  
}


bool SocialForcesAgent::hardCodePathEgress()
{
    std::vector<Util::Point> agentPath;

   if(position().z > 0)
   {
       
    agentPath.push_back(Point(10,0,37.7));
    agentPath.push_back(Point(-2,0,37.7));
    agentPath.push_back(Point(-2,0,33.7));
    agentPath.push_back(Point(-2,0,30.5));
    agentPath.push_back(Point(-2,0,27));
    agentPath.push_back(Point(-2,0,23.7));
    agentPath.push_back(Point(-2,0,20.5));
    agentPath.push_back(Point(-2,0,16.5));
    agentPath.push_back(Point(-2,0,13.6));
    agentPath.push_back(Point(-2,0,10.4));
    agentPath.push_back(Point(-2,0,6.7));
    agentPath.push_back(Point(-2,0,3.5));
    agentPath.push_back(Point(-2,0,-.2));
    agentPath.push_back(Point(-2,0,-3.5));
    std::reverse(agentPath.begin(),agentPath.end()); 
   }
   else
   {
    agentPath.push_back(Point(-2,0,-.2));
    agentPath.push_back(Point(-2,0,-3.5));
    agentPath.push_back(Point(-2,0,-6.7));
    agentPath.push_back(Point(-2,0,-10.4));
    agentPath.push_back(Point(-2,0,-14.1));
    agentPath.push_back(Point(-2,0,-16.8));
    agentPath.push_back(Point(-2,0,-20.5));
    agentPath.push_back(Point(-2,0,-24.2));
    agentPath.push_back(Point(-2,0,-27.5));
    agentPath.push_back(Point(-2,0,-31));
    agentPath.push_back(Point(-2,0,-34.2));
    agentPath.push_back(Point(-2,0,-38));
    agentPath.push_back(Point(10,0,-38));
   }
    
    for  (int i=0; i <  agentPath.size(); i++)
	{
		_midTermPath.push_back(agentPath.at(i));
		if ((i % FURTHEST_LOCAL_TARGET_DISTANCE) == 0)
		{
			_waypoints.push_back(agentPath.at(i));
		}
	}
	if(agentPath.size()>0)
	{
		for(int i = 1; i<agentPath.size(); ++i)
		{
			Util::DrawLib::drawLine(agentPath[i-1], agentPath[i], gYellow);
		}
		//Util::DrawLib::drawCircle(__path[__path.size()-1], Util::Color(0.0f, 1.0f, 0.0f));
	}

	return true;
    
}

bool SocialForcesAgent::hardCodePathIngress()
{
    std::vector<Util::Point> agentPath;
    
   if(_goalQueue.front().targetLocation.z > 0)
   {
       
    agentPath.push_back(Point(10,0,37.7));
    agentPath.push_back(Point(-2,0,37.7));
    agentPath.push_back(Point(-2,0,33.7));
    agentPath.push_back(Point(-2,0,30.5));
    agentPath.push_back(Point(-2,0,27));
    agentPath.push_back(Point(-2,0,23.7));
    agentPath.push_back(Point(-2,0,20.5));
    agentPath.push_back(Point(-2,0,16.5));
    agentPath.push_back(Point(-2,0,13.6));
    agentPath.push_back(Point(-2,0,10.4));
    agentPath.push_back(Point(-2,0,6.7));
    agentPath.push_back(Point(-2,0,3.5));
    agentPath.push_back(Point(-2,0,-.2));
    //agentPath.push_back(Point(-2,0,-3.5));
    //std::reverse(agentPath.begin(),agentPath.end()); 
   }
   else
   {
    agentPath.push_back(_goalQueue.front().targetLocation);
    agentPath.push_back(Point(-2,0,-.2));
    agentPath.push_back(Point(-2,0,-3.5));
    agentPath.push_back(Point(-2,0,-6.7));
    agentPath.push_back(Point(-2,0,-10.4));
    agentPath.push_back(Point(-2,0,-14.1));
    agentPath.push_back(Point(-2,0,-16.8));
    agentPath.push_back(Point(-2,0,-20.5));
    agentPath.push_back(Point(-2,0,-24.2));
    agentPath.push_back(Point(-2,0,-27.5));
    agentPath.push_back(Point(-3.4,0,-31));
    agentPath.push_back(Point(-2,0,-34.2));
    agentPath.push_back(Point(-2,0,-38));
    agentPath.push_back(Point(10,0,-38));
    std::reverse(agentPath.begin(),agentPath.end()); 
   }
    
    for  (int i=0; i <  agentPath.size(); i++)
	{
		_midTermPath.push_back(agentPath.at(i));
		if ((i % FURTHEST_LOCAL_TARGET_DISTANCE) == 0)
		{
			_waypoints.push_back(agentPath.at(i));
		}
	}
	if(agentPath.size()>0)
	{
		for(int i = 1; i<agentPath.size(); ++i)
		{
			Util::DrawLib::drawLine(agentPath[i-1], agentPath[i], gYellow);
		}
		//Util::DrawLib::drawCircle(__path[__path.size()-1], Util::Color(0.0f, 1.0f, 0.0f));
	}

	return true;
    
}



bool SocialForcesAgent::runAStarPlanning()
{
	_midTermPath.clear();
	//==========================================================================

	// run the main a-star search here
	std::vector<Util::Point> agentPath;
	Util::Point pos =  position();

	if(!astar.computePath(agentPath,pos,_goalQueue.front().targetLocation,gSpatialDatabase))
	{
		return false;
	}

	for  (int i=1; i <  agentPath.size(); i++)
	{
		_midTermPath.push_back(agentPath.at(i));
		if ((i % FURTHEST_LOCAL_TARGET_DISTANCE) == 0)
		{
			_waypoints.push_back(agentPath.at(i));
		}
	}
	if(agentPath.size()>0)
	{
		for(int i = 1; i<agentPath.size(); ++i)
		{
			Util::DrawLib::drawLine(agentPath[i-1], agentPath[i], Util::Color(1.0f, 0.0f, 0.0f), 2);
		}
		//Util::DrawLib::drawCircle(__path[__path.size()-1], Util::Color(0.0f, 1.0f, 0.0f));
	}

	return true;
	
	
}

bool SocialForcesAgent::runLongTermPlanning()
{
	_midTermPath.clear();
	//==========================================================================

	// run the main a-star search here
	std::vector<Util::Point> agentPath;
	Util::Point pos =  position();

	if ( !gSpatialDatabase->findPath(pos, _goalQueue.front().targetLocation,
			agentPath, (unsigned int) 50000))
	{
		return false;
	}

	for  (int i=1; i <  agentPath.size(); i++)
	{
		_midTermPath.push_back(agentPath.at(i));
		if ((i % FURTHEST_LOCAL_TARGET_DISTANCE) == 0)
		{
			_waypoints.push_back(agentPath.at(i));
		}
	}
	if(agentPath.size()>0)
	{
		for(int i = 1; i<agentPath.size(); ++i)
		{
			Util::DrawLib::drawLine(agentPath[i-1], agentPath[i], Util::Color(1.0f, 0.0f, 0.0f), 2);
		}
		//Util::DrawLib::drawCircle(__path[__path.size()-1], Util::Color(0.0f, 1.0f, 0.0f));
	}

	return true;
}


bool SocialForcesAgent::runLongTermPlanning2()
{

#ifndef USE_PLANNING
	return;
#endif
	_waypoints.clear();
	//==========================================================================

	// run the main a-star search here
	std::vector<Util::Point> agentPath;
	Util::Point pos =  position();
	if (gEngine->isAgentSelected(this))
	{
		// std::cout << "agent" << this->id() << " is running planning again" << std::endl;
	}

	if ( !gSpatialDatabase->findSmoothPath(pos, _goalQueue.front().targetLocation,
			agentPath, (unsigned int) 50000))
	{
		return false;
	}

	// Push path into _waypoints

	// Skip first node that is at location of agent
	for  (int i=1; i <  agentPath.size(); i++)
	{
		_waypoints.push_back(agentPath.at(i));

	}

	return true;

}


void SocialForcesAgent::draw()
{
#ifdef ENABLE_GUI
	// if the agent is selected, do some annotations just for demonstration
	if (gEngine->isAgentSelected(this))
	{
                
		Util::Ray ray;
		ray.initWithUnitInterval(_position, _forward);
		float t = 0.0f;
		SteerLib::SpatialDatabaseItem * objectFound;
		Util::DrawLib::drawLine(ray.pos, ray.eval(1.0f));
		if (gSpatialDatabase->trace(ray, t, objectFound, this, false))
		{
			Util::DrawLib::drawAgentDisc(_position, _forward, _radius, Util::gBlue);
		}
		else {
			Util::DrawLib::drawAgentDisc(_position, _forward, _radius);
		}
		Util::DrawLib::drawFlag( this->currentGoal().targetLocation, Color(0.5f,0.8f,0), 2);
		if ( this->currentGoal().goalType == GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL )
		{
			Color color(0.4,0.9,0.4);
			DrawLib::glColor(color);
			DrawLib::drawQuad(Point(this->currentGoal().targetRegion.xmin, 0.1, this->currentGoal().targetRegion.zmin),
					Point(this->currentGoal().targetRegion.xmin, 0.1, this->currentGoal().targetRegion.zmax),
					Point(this->currentGoal().targetRegion.xmax, 0.1, this->currentGoal().targetRegion.zmax),
					Point(this->currentGoal().targetRegion.xmax, 0.1, this->currentGoal().targetRegion.zmin));
		}
		int i;
		for (i=0; ( _midTermPath.size() > 1 ) && (i < (_midTermPath.size() - 1)); i++)
		{
			DrawLib::drawLine(_midTermPath.at(i), _midTermPath.at(i+1), gYellow);
			DrawLib::drawStar(_midTermPath.at(i), Util::Vector(1,0,0), 0.34f, gBlue);
		}
		// DrawLib::drawStar(_waypoints.at(i), Util::Vector(1,0,0), 0.34f, gBlue);
	}
	else {
		Util::DrawLib::drawAgentDisc(_position, _radius, this->_color);
	}
	if (_goalQueue.front().goalType == SteerLib::GOAL_TYPE_SEEK_STATIC_TARGET) {
		Util::DrawLib::drawFlag(_goalQueue.front().targetLocation);
	}
	//Util::DrawLib::drawFlag(Point(6,0,4));
       // Util::DrawLib::drawFlag(Point(-8,0,4));
       // Util::DrawLib::drawFlag(Point(-8,0,-6));
       // Util::DrawLib::drawFlag(Point(6,0,-6));
      // Util::DrawLib::drawFlag(Point(-7,0,2));
     //  Util::DrawLib::drawFlag(Point(-6,0,3));
      // Util::DrawLib::drawFlag(Point(-6,0,1));
      // Util::DrawLib::drawFlag(Point(-7,0,-1));
     //  Util::DrawLib::drawFlag(Point(-7,0,-3));
	Util::DrawLib::drawFlag(Point(-0.5,0,0.5));

#ifdef DRAW_COLLISIONS
	std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;
	gSpatialDatabase->getItemsInRange(_neighbors, _position.x-(this->_radius * 3), _position.x+(this->_radius * 3),
			_position.z-(this->_radius * 3), _position.z+(this->_radius * 3), dynamic_cast<SteerLib::SpatialDatabaseItemPtr>(this));

	for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbor = _neighbors.begin();  neighbor != _neighbors.end();  neighbor++)
	{
		if ( (*neighbor)->isAgent() && (*neighbor)->computePenetration(this->position(), this->_radius) > 0.00001f)
		{
			Util::DrawLib::drawStar(
					this->position()
					+
					(
						(
							dynamic_cast<AgentInterface*>(*neighbor)->position()
							-
							this->position()
						)
					/2), Util::Vector(1,0,0), 0.8f, gRed);
			// Util::DrawLib::drawStar(this->position(), Util::Vector(1,0,0), 1.14f, gRed);
		}
	}
#endif
#ifdef DRAW_HISTORIES
	__oldPositions.push_back(position());
	int points = 0;
	float mostPoints = 100.0f;
	while ( __oldPositions.size() > mostPoints )
	{
		__oldPositions.pop_front();
	}
	for (int q = __oldPositions.size()-1 ; q > 0 && __oldPositions.size() > 1; q--)
	{
		DrawLib::drawLineAlpha(__oldPositions.at(q), __oldPositions.at(q-1),gBlack, q/(float)__oldPositions.size());
	}

#endif

#ifdef DRAW_ANNOTATIONS

	for (int i=0; ( _waypoints.size() > 1 ) && (i < (_waypoints.size() - 1)); i++)
	{
		if ( gEngine->isAgentSelected(this) )
		{
			DrawLib::drawLine(_waypoints.at(i), _waypoints.at(i+1), gYellow);
		}
		else
		{
			//DrawLib::drawLine(_waypoints.at(i), _waypoints.at(i+1), gBlack);
		}
	}

	for (int i=0; ( _midTermPath.size() > 1 ) && (i < (_midTermPath.size() - 1)); i++)
	{
		if ( gEngine->isAgentSelected(this) )
		{
			DrawLib::drawLine(_midTermPath.at(i), _midTermPath.at(i+1), gMagenta);
		}
		else
		{
			// DrawLib::drawLine(_midTermPath.at(i), _midTermPath.at(i+1), gCyan);
		}
	}

	DrawLib::drawLine(position(), this->_currentLocalTarget, gGray10);
	DrawLib::drawStar(this->_currentLocalTarget+Util::Vector(0,0.001,0), Util::Vector(1,0,0), 0.24f, gGray10);

	/*
	// draw normals and closest points on walls
	std::set<SteerLib::ObstacleInterface * > tmp_obs = gEngine->getObstacles();

	for (std::set<SteerLib::ObstacleInterface * >::iterator tmp_o = tmp_obs.begin();  tmp_o != tmp_obs.end();  tmp_o++)
	{
		Util::Vector normal = calcWallNormal( *tmp_o );
		std::pair<Util::Point,Util::Point> line = calcWallPointsFromNormal(* tmp_o, normal);
		Util::Point midpoint = Util::Point((line.first.x+line.second.x)/2, ((line.first.y+line.second.y)/2)+1,
				(line.first.z+line.second.z)/2);
		DrawLib::drawLine(midpoint, midpoint+normal, gGreen);

		// Draw the closes point as well
		std::pair<float, Util::Point> min_stuff = minimum_distance(line.first, line.second, position());
		DrawLib::drawStar(min_stuff.second, Util::Vector(1,0,0), 0.34f, gGreen);
	}
	*/

#endif

#endif
}

