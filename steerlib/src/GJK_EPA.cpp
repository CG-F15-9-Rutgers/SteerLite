/*!
*
* \author VaHiD AzIzI
*
*/


#include "obstacles/GJK_EPA.h"
#include <math.h>
#define FLOAT_MAX 3.402823466e+38F /* max value */

SteerLib::GJK_EPA::GJK_EPA()
{
}


int GetFarthestIndexInDirection(Util::Vector direction,const std::vector<Util::Vector>& ShapeA)
{
    double MaxDot = direction*ShapeA[0];
    int FarthestIndex = 0;
    
    
    for(unsigned int i = 1; i < ShapeA.size(); i++)
    {
        double CurrentDot = direction*ShapeA[i];
        
        if(CurrentDot>MaxDot)
        {
            MaxDot = CurrentDot;
            FarthestIndex = i;
        }
    }
    
    return FarthestIndex;
    
}

Util::Vector Support(const std::vector<Util::Vector>& ShapeA,const std::vector<Util::Vector>& ShapeB,Util::Vector direction)
{
    Util::Vector FirstPoint = ShapeA[GetFarthestIndexInDirection(direction,ShapeA)];
    Util::Vector newDirection = -1 * direction;
    Util::Vector SecondPoint = ShapeB[GetFarthestIndexInDirection(newDirection,ShapeB)];
    Util::Vector MinkowskiDifference = FirstPoint - SecondPoint;
    
    return MinkowskiDifference;
    
}
bool CheckContainsOrigin(Util::Vector& Direction, std::vector<Util::Vector>& simplex)
{
    Util::Vector first = simplex.back();
    Util::Vector firstFromOrigin = -1 * first;
    Util::Vector second;
    Util::Vector third;
    Util::Vector firstSecond;
    Util::Vector firstThird;
    
    if(simplex.size() == 3)
    {
        second = simplex[1];
        third = simplex[0];
        firstSecond = second - first;
        firstThird = third - first;
        
        Direction = Util::Vector(firstSecond.z, firstSecond.y, -1 * firstSecond.x);
        
        if(Direction * third > 0)
        {
            Direction = Direction * -1;
        }
        
        if(Direction * firstFromOrigin > 0)
        {
            simplex.erase( simplex.begin() + 0);
            return false;
        }
        
        Direction = Util::Vector(firstThird.z, firstThird.y, -1* firstThird.x);
        
        if(Direction * firstFromOrigin > 0)
        {
            simplex.erase(simplex.begin() + 1);
            return false;
        }
        
        return true;
    }
    else //line segment
    {
        second = simplex[0];
        firstSecond = second - first;
        
        Direction = Util::Vector(firstSecond.z, firstSecond.y, -1 * firstSecond.x);
        
        if(Direction * firstFromOrigin < 0)
        {
            Direction = -1 * Direction;
        }
        
    }
    
    return false;
    
    
}

void getNearestEdge(std::vector<Util::Vector>& simplex, float& distance, Util::Vector& normal, int& index)
{
  distance = FLOAT_MAX;
  
  for(int i = 0; i < simplex.size(); i++)
  {
   int j;
   if(i+1 == simplex.size())
     j = 0;
   else
     j = i+1;
   
   Util::Vector v1 = simplex[i];
   Util::Vector v2 = simplex[j];
   
   Util::Vector edge = v2-v1;
   
   Util::Vector originTov1 = v1;
   
   Util::Vector n = originTov1*(edge*edge) - edge*(edge*originTov1); //triple product to get vector from edge towards the originTov1
   n = n/sqrt(pow(n.x,2)+pow(n.y,2)+pow(n.z,2)); //normalize
   float dist = n*v1; //distance from origin to edge
    
    if(dist < distance)
    {
     distance = dist;
     index = j;
     normal = n;
    }
   
  }
  
}

bool GJK(const std::vector<Util::Vector>& ShapeA,const std::vector<Util::Vector>& ShapeB, std::vector<Util::Vector>& simplex)
{
    Util::Vector DirectionVector(1,0,-1);
    simplex.push_back(Support(ShapeA, ShapeB, DirectionVector));
    Util::Vector newDirection = -1 * DirectionVector;
    
    while(true)
    {
        simplex.push_back(Support(ShapeA,ShapeB, newDirection));
        
        if(simplex.back() * newDirection <= 0)
        {
            
            return false;
        }
        else
        {
            if(CheckContainsOrigin(newDirection,simplex))
            {
                return true;
            }
        }
    }
}

bool EPA(const std::vector<Util::Vector>& shapeA, const std::vector<Util::Vector>& shapeB, std::vector<Util::Vector>& simplex, float& penetration_depth, Util::Vector& penetration_vector)
{
  
  
  while(true)
  {
    float distance;
    int index;
    Util::Vector normal;
    
    getNearestEdge(simplex, distance, normal, index);
    
    Util::Vector sup = Support(shapeA, shapeB, normal); //get support point in direction of edge's normal
    
    float d = sup*normal;
    
    if(d - distance <= 0)
    {
      penetration_vector = normal;
      penetration_depth = distance;
      return true;
      
    }
    
    else
    {
     simplex.insert(simplex.begin()+index, sup); 
    }
    
  }
  
  
}


//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
     std::vector<Util::Vector> simplex;
     bool InACollision = GJK(_shapeA,_shapeB,simplex); 
     
     if(InACollision)
     {
       EPA(_shapeA,_shapeB,simplex, return_penetration_depth, return_penetration_vector);
     }
     return InACollision;
     //return false; // There is no collision
}
