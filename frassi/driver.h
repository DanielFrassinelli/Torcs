/***************************************************************************

    file                 : Driver.h
    created              : Sat Mar 2 17:02:55 CET 2013
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

#ifndef _DRIVER_H_
#define _DRIVER_H_

#include "carData.h"
#include "trajectoryPlanner.h"
#include "opponent.h"
#include "logger.h"

class Driver{
  public:
    Driver(int index);
    ~Driver();
    
    void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s); 
    void newrace(int index, tCarElt* car, tSituation *s); 
    void drive(int index, tSituation *s); 
    void endrace(int index, tSituation *s);
    int  pitCommand(tCarElt* car, tSituation *s);

  private:
    
    static const double STUCK_MAX_ANGLE;
    static const double STUCK_MAX_DIST;
    static const double STUCK_MAX_SPEED;
    static const int    STUCK_MAX_COUNT;
    
    static const double ACCEL_PI_KI;
    static const double ACCEL_PI_KE;
    static const double ACCEL_INTEG_LIMIT;	// we have to limit the error because sometimes can be infinite
    static const double ACCEL_CONST_LIMIT;	// we have to limit the intergral because grows too much
    static const double FILTER_ACCEL_START;
    static const double FILTER_ACCEL_START_MODE_SPEED;
    
    static const double STEER_K_SOFT;
    static const double STEER_K_SPEED;
    static const double STEER_K_YAW;
    
    static const double GEAR_UP_SPEED_GAIN;
    static const double GEAR_DOWN_SPEED_GAIN;
    static const double GEAR_MAX_TIME;
    
    static const double CLUTCH_FULL_MAX_TIME;
    static const double CLUTCH_SPEED;
    
    static const double ABS_MIN_SPEED;
    static const double ABS_SLIP;
    
    static const double TCL_SLIP;
    static const double TCL_MINSPEED; 

    int stuck, index;
    double mass , time_difference , stuckangle , allowedSpeed , trajangle , trajdist;
    double clutchTime , gearTime;

    tTrack    *track;
    tCarElt   *car;   		
    carData   *myCar;  
    opponents *allCars;
    logger    *log;
    vector <opponent *> * enemyCars; 
    trajectoryPlanner *trajectory;
    
    typedef struct {
      double speed_x;
      double trajangle;
      double carYaw;
      double allowedSpeed;
      double accelError;
      double accelErrorSum; 
    } past_data;
    
    past_data last;
    
    bool  isStuck();

    double getAccel();
    double getSteer();      
    int    getGear();  
    double getClutch();
    
    double filterAccelOpp(double accel);
    double filterSteerOpp(double steer);
       
    double filterABS(double brake);
    double filterTCL(double accel);   
    double filterAccel(double accel);

    void  initialUpdate(tSituation *s); 
    void  finalUpdate(tSituation *s);

};

#endif

