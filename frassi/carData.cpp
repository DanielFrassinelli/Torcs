/***************************************************************************

    file                 : driver.cpp
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

#include "carData.h"

const int    carData::LOG_STEP = 5;
const double carData::CAR_MAX_DEFAULT_SPEED = 150.0;

carData::carData(tCarElt *mycar){
  
  this->car = mycar;
  
  CW = initCW();
  CA = initCA();
  CARMASS =  GfParmGetNum(car->_carHandle, SECT_CAR, PRM_MASS, NULL, 1100);
  maxSpeed = GfParmGetNum(car->_carHandle, FRASSI_PRIV, SECT_MAXSPEED, (char*)NULL, CAR_MAX_DEFAULT_SPEED);
  friction  = GfParmGetNum(car->_carHandle, FRASSI_PRIV, SECT_FRICTION, (char*)NULL, 1.0);

  mass = CARMASS + car->_fuel;
  stuckTime = 0.0;
  mode = START;

  initTrainType();

  if(LOG_CAR_TEST)
    initCarLog();
}

void carData::updateCar(tSituation *s){
  
  mass = CARMASS + car->_fuel;
  speedSqr = car->_speed_x * car->_speed_x;  
  
  if(LOG_CAR_TEST && log != NULL)
    logCarData(s);
  
  if(mode == STUCK)
    stuckTime += RCM_MAX_DT_ROBOTS;
  else
    stuckTime = 0.0;
  
}

double  carData::initCA()
{
    char *WheelSect[4] = {(char *) SECT_FRNTRGTWHEEL, (char *) SECT_FRNTLFTWHEEL, (char *) SECT_REARRGTWHEEL, (char *) SECT_REARLFTWHEEL};    
    double  rearwingarea = GfParmGetNum(car->_carHandle, SECT_REARWING,PRM_WINGAREA, (char*) NULL, 0.0);
    double  rearwingangle = GfParmGetNum(car->_carHandle, SECT_REARWING,PRM_WINGANGLE, (char*) NULL, 0.0);
    double  wingca = 1.23*rearwingarea*sin(rearwingangle);
    double  cl = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS,PRM_FCL, (char*) NULL, 0.0) +
               GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS,PRM_RCL, (char*) NULL, 0.0);
    double  h = 0.0;
    int i;
    for (i = 0; i < 4; i++)
        h += GfParmGetNum(car->_carHandle, WheelSect[i], PRM_RIDEHEIGHT, (char*) NULL, 0.20);
    h*= 1.5; 
    h = h*h; 
    h = h*h; 
    h = 2.0 * exp(-3.0*h);
    return h*cl + 4.0*wingca;
}

double  carData::initCW()
{
    double  cx = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_CX, (char*) NULL, 0.0);
    double  frontarea = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_FRNTAREA, (char*) NULL, 0.0);
    return 0.645*cx*frontarea;
}

void carData::initTrainType()
{
    const char *traintype = GfParmGetStr(car->_carHandle, SECT_DRIVETRAIN, PRM_TYPE, VAL_TRANS_RWD);
    
    if (strcmp(traintype, VAL_TRANS_RWD) == 0) { 
      GET_DRIVEN_WHEEL_SPEED = &carData::filterTCL_RWD;
      GET_ACCEL_FUNCT = &carData::getAccel_RWD;
      drivetrain = DRWD;
    } 
    
    else if (strcmp(traintype, VAL_TRANS_FWD) == 0) {
      GET_DRIVEN_WHEEL_SPEED = &carData::filterTCL_FWD;
      GET_ACCEL_FUNCT = &carData::getAccel_FWD;
      drivetrain = DFWD;
    } 
    
    else if (strcmp(traintype, VAL_TRANS_4WD) == 0) {
      GET_DRIVEN_WHEEL_SPEED = &carData::filterTCL_4WD;
      GET_ACCEL_FUNCT = &carData::getAccel_4WD;
      drivetrain = D4WD; 
    }
}

double  carData::filterTCL_RWD(){
    return (car->_wheelSpinVel(REAR_RGT) + car->_wheelSpinVel(REAR_LFT)) * car->_wheelRadius(REAR_LFT) / 2.0;
}

double  carData::filterTCL_FWD(){
    return (car->_wheelSpinVel(FRNT_RGT) + car->_wheelSpinVel(FRNT_LFT)) * car->_wheelRadius(FRNT_LFT) / 2.0;
}

double  carData::filterTCL_4WD(){
    return (car->_wheelSpinVel(FRNT_RGT) + car->_wheelSpinVel(FRNT_LFT)) * car->_wheelRadius(FRNT_LFT) / 4.0 +
           (car->_wheelSpinVel(REAR_RGT) + car->_wheelSpinVel(REAR_LFT)) * car->_wheelRadius(REAR_LFT) / 4.0;
}

double carData::getAccel_RWD(double speed){
    return speed / car->_wheelRadius(REAR_RGT) * car->_gearRatio[car->_gear + car->_gearOffset] / car->_enginerpmRedLine;
}

double carData::getAccel_FWD(double speed){
    return speed / car->_wheelRadius(FRNT_RGT) * car->_gearRatio[car->_gear + car->_gearOffset] / car->_enginerpmRedLine; 
}

double carData::getAccel_4WD(double speed){
    return speed / (car->_wheelRadius(REAR_RGT) + car->_wheelRadius(FRNT_RGT)) * 2.0 * 
	   car->_gearRatio[car->_gear + car->_gearOffset] / car->_enginerpmRedLine;   
}

double carData::getBrake(double brake){
  if(maxSpeed == CAR_MAX_DEFAULT_SPEED)
    return brake;
  
  double weight = getMass()*G;
  double maxForce = weight + CA*maxSpeed*maxSpeed;
  double force = weight + CA*speedSqr;
  return brake*MIN(1.0, force/maxForce);  
}

void carData::initCarLog(){
  char filePath[255];
  
  strcpy(filePath , BASE_PATH);
  strcat(filePath , car->_carName);
  strcat(filePath , CAR_TEST_LOG_PATH);
  
  log.open(filePath);
  
  if(log == NULL) 
  { 
    cout << "--------------------------------------------------------------" << endl;
    cout << "Can't open path : " << filePath <<endl;
    cout << "--------------------------------------------------------------" << endl;
    return; 
  }
  
  cout << "--------------------------------------------------------------" << endl;
  cout << "Car's data will be logged in : " << filePath <<endl;
  cout << "--------------------------------------------------------------" << endl;    
  log << "# time , speed , gear , accel , radius , speedY , distanceToMiddle " << endl;
  
  /* car's data are logged in gnuplot format */
    
}

void carData::logCarData(tSituation *s){

  if(counter >= LOG_STEP)
  {  
  double  t = lastTime - s->currentTime;
  double  accel = (lastSpeed - car->_speed_x) / t;
  
  lastSpeed = car->_speed_x;
  lastTime  = s->currentTime;
  
  log << s->currentTime << " \t " << car->_speed_x << " \t " << car->_gear << " \t " << accel; 
  /* with this data you can recover max_speed , accel , decel */
  
  log << " \t " << car->_trkPos.seg->radius << " \t " << car->_speed_y << " \t " << car->_trkPos.toMiddle << endl;
  /* with this you can recover the min turning radius */
  
  counter = 0;
  }
  else
    counter ++;
  
}






