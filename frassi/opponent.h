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

#ifndef _OPPONENT_H_
#define _OPPONENT_H_

#include "carData.h"

using namespace std;

#define oppIgnore 0
#define oppFront  (1 << 0)
#define oppBack   (1 << 1)
#define oppSide   (1 << 2)
#define oppColl   (1 << 3)

class opponent{

public:
  
  typedef struct {
      int state;
      double distX , distY;
  } status;   
  
  opponent();

  void init(tCarElt *car , tTrack *track);
  void update(tCarElt *me);
  void computeStatus(tCarElt *me); /* compute the status between myCar and the opponent */
  
  inline v2d getPosition(){ return position; }
  inline double getSpeed() { return car->_speed_x; }
  inline int 	getState(tCarElt *me) { return state[me->_driverIndex].state; };
  inline tCarElt * getCarPnt() { return car; }
  inline status  * getStatus(tCarElt *me) { return &state[me->_driverIndex]; };

private:
  static const double maxBackCheckDist;
  static const double maxFrontCheckDist;
  static const double minSideDist;
  
  v2d position, direction;
  tCarElt *car;
  tTrack  *track;
  tTrackSeg *currentSeg;

  double trackAngle , distToStart;

  status state[10]; 	  /* array that contains data related to the driver with index i */
			  /* array size is 10 because there are at max 10 players */
			  
  double getDistToSegStart(); /* return the dist of a car from the start of the currentSeg */
			  			   
};

class opponents{
  
public:
  opponents(tSituation *s , tTrack *track);
  ~opponents();
  
  vector <opponent *> enemies[10];	/* for each driver, this vector contains a list of dangerouns oppoenents */
  
  static opponent * cars;
  static int cars_num;
  static double lastUpdate;
  
  inline int getEnemyCarsNum() { return cars_num - 1;}
  inline vector <opponent *> * getEnemyCarsPnt(tCarElt *me) { return &enemies[me->_driverIndex];}

  void updateCars(tCarElt *me , tSituation *s); 			/* update the param for each cars, this method is executed one time per robot call */
  void computeStatus(tCarElt *me); 					/* compute the structure status for each car */

};


    
#endif

