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
      double oppSpeed, sideCollDist, catchDist;
  } status;   
  
  opponent();

  void init(tCarElt *car , tTrack *track);
  void update(carData *myCar);
  void computeStatus(carData *myCar); /* compute the status between myCar and the opponent */
  
  inline int 	getState(carData *myCar) { return state[myCar->getCarIndex()].state; };
  inline tCarElt * getCarPnt() { return car; }
  inline status  * getStatus(carData *myCar) { return &state[myCar->getCarIndex()]; };

private:
  static const double BACK_CHECK_DIST;
  static const double FRONT_CHECK_DIST;
  static const double LENGTH_MARGIN;
  static const double SIDE_MARGIN;
  
  v2d pos, dir;
  tCarElt *car;
  tTrack  *track;
  tTrackSeg *currentSeg;

  double trackAngle , distToStart, width, speed;

  status state[10]; 	  /* array that contains data related to the driver with index i */
			  /* array size is 10 because our robot can have 10 drivers max */
			  
  double getDistToSegStart(); /* return the dist of a car from the start of the currentSeg */
  double getSpeed();	       /* return the speed in track direction */
			  			   
};

class opponents{
  
public:
  opponents(tSituation *s , tTrack *track);
  ~opponents();
  
  vector <opponent *> enemies[10];	/* for each driver, this vector contains a list of dangerous oppoenents */
  
  static opponent * cars;
  static int cars_num;
  static double lastUpdate;
  
  inline int getEnemyCarsNum() { return cars_num - 1;}
  inline vector <opponent *> * getEnemyCarsPnt(carData *myCar) { return &enemies[myCar->getCarIndex()];}

  void updateCars(carData *myCar , tSituation *s); 			/* update the param for each cars, this method is executed one time per robot call */
  void computeStatus(carData *myCar); 					/* compute the structure status for each car */

};


    
#endif

