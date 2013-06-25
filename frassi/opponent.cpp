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

const double opponent::maxFrontCheckDist = 100.0; /* m */ 
const double opponent::maxBackCheckDist  = -50.0; /* m */
const double opponent::minSideDist       = 8.0;   /* m */ 

opponent::opponent() 
{}

void opponent::init(tCarElt *car , tTrack *track){
  this->car = car;
  this->track = track; 
}

void opponent::computeStatus(tCarElt *me){
  
  //TODO there we compute if the cars are going to collide and we set the structure state

  status *tmp = &state[me->_driverIndex]; 
  tmp->state = oppIgnore;
  
  if(car->_state & RM_CAR_STATE_NO_SIMU || car == me)
    return; /* we don't need to compute if we are the car of the car is no longer in the simulation */

  tmp->distX = distToStart - me->_distFromStartLine;

  if(tmp->distX > maxFrontCheckDist || tmp->distX < maxBackCheckDist)
    return; /* if the car is too far away, we return */
  
  if(tmp->distX > 0)
    tmp->state |= oppFront;
  else
    tmp->state |= oppBack;
  
  if(fabs(tmp->distX) < minSideDist)
    tmp->state |= oppSide;
  
  
  tmp->state = oppColl;
  
}

void opponent::update(tCarElt *me){
  
  //TODO there we compute the path followed by the car

  position.x = car->_pos_X;
  position.y = car->_pos_Y;
  
  currentSeg = car->_trkPos.seg;
  
  if(car == me)
    distToStart = car->_distFromStartLine;  
  else
    distToStart = currentSeg->lgfromstart + getDistToSegStart();  

}

//TODO then we need a method that, given the graph of our driver, it computes the arcs with high probability of collision

double opponent::getDistToSegStart(){   
    if (car->_trkPos.seg->type == TR_STR) {
        return car->_trkPos.toStart;
    } else {
        return car->_trkPos.toStart*car->_trkPos.seg->radius;
    }  
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

void opponents::updateCars(tCarElt *me , tSituation *s){  
  if(lastUpdate != s->currentTime)
  {
    for(int i=0; i < cars_num; i++)
      cars[i].update(me);
    lastUpdate = s->currentTime;
  }
}

void opponents::computeStatus(tCarElt *me){  
  for(int i=0; i < cars_num; i++)
    cars[i].computeStatus(me);
  
  enemies[me->_driverIndex].clear(); 
  
  for(int i=0; i < cars_num; i++)
    if(cars[i].getState(me) != oppIgnore)
      enemies[me->_driverIndex].push_back(&cars[i]);
}

