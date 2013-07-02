/***************************************************************************

    file                 : Driver.h
    created              : Sat Mar 10 17:20:30 CET 2013
    copyright            : (C) 2002 Daniel Frassinelli

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#include "opponent.h"

const double opponent::FRONT_CHECK_DIST  = 100.0; /* m */ 
const double opponent::BACK_CHECK_DIST   = -50.0; /* m */
const double opponent::LENGTH_MARGIN 	  = 2.0;   /* [m] safety margin */
const double opponent::SIDE_MARGIN 	  = 1.0;   /* [m] safety margin */

opponent::opponent() 
{}

void opponent::init(tCarElt *car , tTrack *track){
  this->car = car;
  this->track = track; 
}

void opponent::computeStatus(carData *myCar){
  
  //TODO there we compute if the cars are going to collide and we set the structure state

  status *tmp = &state[myCar->getCarIndex()]; 
  tmp->state = oppIgnore;
  
  if(car->_state & RM_CAR_STATE_NO_SIMU || car == myCar->getCarPnt())
    return; /* we don't need to compute if we are the car of the car is no longer in the simulation */

  tmp->distX = distToStart - myCar->getCarPnt()->_distFromStartLine;

  if(tmp->distX > FRONT_CHECK_DIST || tmp->distX < BACK_CHECK_DIST)
    return; /* if the car is too far away, we return */

  tmp->sideCollDist = MIN(car->_dimension_x, myCar->getCarPnt()->_dimension_x);  
    
   /*is opponent in front and slower */
 /* if (tmp->distX > tmp->sideCollDist && speed < myCar->getSpeedOpp()) {
      tmp->catchDist = myCar->getSpeedOpp()*tmp->distX/(myCar->getSpeedOpp()- speed);
      tmp->state |= OPP_FRONT;
      tmp->distX -= MAX(car->_dimension_x, myCar->getCarPnt()->_dimension_x);
      tmp->distX -= LENGTH_MARGIN;
      
      float cardist = car->_trkPos.toMiddle - mycar->_trkPos.toMiddle;
      sidedist = cardist;
      cardist = fabs(cardist) - fabs(width/2.0) - mycar->_dimension_y/2.0;
      
      if (cardist < SIDE_MARGIN) state |= OPP_COLL;
        }*/
  
}

void opponent::update(carData *myCar){
  
  //TODO there we compute the path followed by the car
  
  if(car->_state & RM_CAR_STATE_NO_SIMU)
    return; /* we don't need to compute if we are the car of the car is no longer in the simulation */
    
  pos.x = car->_pos_X;
  pos.y = car->_pos_Y;
  
  currentSeg = car->_trkPos.seg;
  
  trackAngle = RtTrackSideTgAngleL(&(car->_trkPos));
  
  if(car == myCar->getCarPnt())
    distToStart = car->_distFromStartLine;  
  else
    distToStart = currentSeg->lgfromstart + getDistToSegStart(); 
  
  /* update speed in track direction */
  speed = getSpeed();
  float cosa = speed/sqrt(car->_speed_X*car->_speed_X + car->_speed_Y*car->_speed_Y);
  float alpha = acos(cosa);
  width = car->_dimension_x*sin(alpha) + car->_dimension_y*cosa;

}

//TODO then we need a method that, given the graph of our driver, it computes the arcs with high probability of collision

double opponent::getDistToSegStart(){   
    if (car->_trkPos.seg->type == TR_STR) {
        return car->_trkPos.toStart;
    } else {
        return car->_trkPos.toStart*car->_trkPos.seg->radius;
    }  
}


double opponent::getSpeed(){   
  v2d speed(car->_speed_X, car->_speed_Y);
  v2d dir(cos(trackAngle) , sin(trackAngle));
  return speed*dir;  
}

/*------------------------------------------------ opponents */

opponent * opponents::cars;
int opponents::cars_num = 0;
double opponents::lastUpdate = - 125;

opponents::opponents(tSituation *s , tTrack *track){

 if(cars_num != 0)
    return;
 
  cars_num = s->_ncars;
  cars = new opponent[cars_num];
 
  for(int i=0; i < cars_num; i++)
    cars[i].init(s->cars[i] , track); 

  lastUpdate = s->currentTime;  
}

opponents::~opponents(){
  if(cars != NULL)
    delete [] cars;
  cars = NULL;
}

void opponents::updateCars(carData *myCar , tSituation *s){  
  if(lastUpdate != s->currentTime)
  {
    for(int i=0; i < cars_num; i++)
      cars[i].update(myCar);
    lastUpdate = s->currentTime;
  }
}

void opponents::computeStatus(carData *myCar){  
  for(int i=0; i < cars_num; i++)
    cars[i].computeStatus(myCar);
  
  enemies[myCar->getCarIndex()].clear(); 
  
  for(int i=0; i < cars_num; i++)
    if(cars[i].getState(myCar) != oppIgnore)
      enemies[myCar->getCarIndex()].push_back(&cars[i]);
}

