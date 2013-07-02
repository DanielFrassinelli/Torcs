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

#include "logger.h"

const bool logger::LOG_MATLAB_DATA 	= false;		
const bool logger::LOG_CAR_DATA    	= false; 		
const bool logger::LOG_TRACK_GNUPLOT	= false;
const bool logger::LOG_PATH_GNUPLOT	= false;
const bool logger::LOG_CUSTOM_DATA	= false;
const int  logger::LOG_STEP		= 1;

logger::logger(tTrack *track, tCarElt *car , carData *myCar, trajectoryPlanner *trajectory){
  
  this->track = track;
  this->car = car;
  this->myCar = myCar;
  this->trajectory = trajectory;

  step = 0;
  
  char	filePath[255];
  
  if(LOG_CAR_DATA)
  {
  strcpy(filePath , BASE_PATH);
  strcat(filePath , track->internalname);
  strcat(filePath , "/");
  strcat(filePath , car->_carName);
  strcat(filePath , CAR_DATA_LOGPATH);
  
  carLogger.open(filePath);
  
  if(carLogger == NULL) 
  { 
    cout << "--------------------------------------------------------------" << endl;
    cout << "Can't open : " << filePath <<endl;
    cout << "--------------------------------------------------------------" << endl;
    return; 
  }
  
  cout << "--------------------------------------------------------------" << endl;
  cout << "Car's data will be logged in : " << filePath <<endl;
  cout << "# time , carPosX, carPosY, speed , gear , accel , radius , speedY , distanceToMiddle " << endl; 
  cout << "--------------------------------------------------------------" << endl;    
 
  }
  
  if(LOG_CUSTOM_DATA)
  {
  strcpy(filePath , BASE_PATH);
  strcat(filePath , track->internalname);
  strcat(filePath , "/");
  strcat(filePath , car->_carName);
  strcat(filePath , CUSTOM_DATA_LOGPATH);
  
  customLogger.open(filePath);
  
  if(customLogger == NULL) 
  { 
    cout << "--------------------------------------------------------------" << endl;
    cout << "Can't open : " << filePath <<endl;
    cout << "--------------------------------------------------------------" << endl;
    return; 
  }
  
  cout << "--------------------------------------------------------------" << endl;
  cout << "Custom data will be logged in : " << filePath <<endl;
  cout << "--------------------------------------------------------------" << endl;    
 
  }
  
  if(LOG_MATLAB_DATA)
    logMatlabData();
    
  if(LOG_TRACK_GNUPLOT)
    logGnuplotTrack();
  
  if(LOG_PATH_GNUPLOT)
    logGnuplotPath();

}

logger::~logger(){  
  if(matlabLogger != NULL)
    matlabLogger.close();
  
  if(carLogger != NULL)
    carLogger.close();  
  
  if(pathLogger != NULL)
    pathLogger.close();  
  
  if(trackLogger != NULL)
    trackLogger.close();
  
  if(customLogger != NULL)
    customLogger.close();
}

void logger::log(tSituation *s){
  
 if(!LOG_CAR_DATA && !LOG_CUSTOM_DATA)
   return;
 
 step ++;
 
 if(step >= LOG_STEP)
 {
   if(LOG_CAR_DATA)  
    logCarData(s);
   
   if(LOG_CUSTOM_DATA)
     logCustomData(s);
   
   step = 0;
 }  
}

void logger::logCustomData(tSituation *s){
    
  customLogger << car->_pos_X << " \t " << car->_pos_Y << " \t " << car->_speed_x;
  
  double accel;
  
  if(car->_accelCmd == 0.0)
    accel = MAX(-1.0 , -car->_brakeCmd);
  else
    accel = MIN(1.0 , car->_accelCmd);
  
  customLogger << " \t " << accel << endl;
   
}

void logger::logCarData(tSituation *s){

  double  t = lastTime - s->currentTime;
  double  accel = (lastSpeed - car->_speed_x) / t;
  
  lastSpeed = car->_speed_x;
  lastTime  = s->currentTime;
  
  carLogger << s->currentTime << " \t " << car->_pos_X << " \t " << car->_pos_Y;
  /* with this you can plot the trajectory / velocity  */  
  
  carLogger << " \t " << car->_speed_x << " \t " << car->_gear << " \t " << accel; 
  /* with this data you can recover max_speed , accel , decel */
  
  carLogger << " \t " << car->_trkPos.seg->radius << " \t " << car->_speed_y << " \t " << car->_trkPos.toMiddle << endl;
  /* with this you can recover the min turning radius */

}

void logger::logGnuplotTrack(){
  
  char filePath[255];
  
  strcpy(filePath , BASE_PATH);
  strcat(filePath , track->internalname);
  strcat(filePath , "/");
  strcat(filePath , car->_carName);
  strcat(filePath , GNUPLOT_TARCK_LOGPATH);
  
  ofstream trackLogger(filePath);
  
  if(!trackLogger.is_open())
  {
    cout << "\n-------------------------------------------------------------" << endl;
    cout << "Error, can't open : " << filePath << endl; 
    cout << "-------------------------------------------------------------" << endl;
    return;
  }
  else
  {
    cout << "\n-------------------------------------------------------------" << endl;
    cout << "Logging gnuplot track in : " << filePath << endl; 
    cout << "-------------------------------------------------------------" << endl;
  }

  tTrackSeg *seg = track->seg->next; //segment 0
  
  int segId = seg->id; //0

  float angle = RtTrackSideTgAngleL(&(car->_trkPos)); // initial angle
  NORM0_2PI(angle);

  do{
    int points = seg->length / 4;
    float pnt = points + 1.0;

    if(seg->type == TR_STR)
    {    

    float UPPERXSECT = (seg->vertex[TR_EL].x - seg->vertex[TR_SL].x) / pnt;
    float UPPERYSECT = (seg->vertex[TR_EL].y - seg->vertex[TR_SL].y) / pnt;
    float LOWERXSECT = (seg->vertex[TR_ER].x - seg->vertex[TR_SR].x) / pnt;
    float LOWERYSECT = (seg->vertex[TR_ER].y - seg->vertex[TR_SR].y) / pnt;

    for(int i=0; i <= points; i++){
	float xl = seg->vertex[TR_SL].x + i*UPPERXSECT;
	float yl = seg->vertex[TR_SL].y + i*UPPERYSECT;
	float xr = seg->vertex[TR_SR].x + i*LOWERXSECT;
	float yr = seg->vertex[TR_SR].y + i*LOWERYSECT;
	trackLogger << xl << " \t " << yl << " \t " << xr << " \t " << yr <<  endl;	
      }     
    }
    else {
      float cx = seg->center.x;
      float cy = seg->center.y;
      float ca = seg->angle[TR_CS]; 
      int points = seg->length / 4;
      float pnt = points + 1.0;  
      float arcstep = seg->arc / pnt; 
      float radiusL = seg->radiusl;
      float radiusR = seg->radiusr;

      for(int i=0; i <= points; i++){
	
	float tmpAngle ;
	
	if(seg->type == TR_LFT)
	  tmpAngle = ca + arcstep*i;
	else
	  tmpAngle = ca - arcstep*i;
	
	float xl = cx + radiusL * cos(tmpAngle);
	float yl = cy + radiusL * sin(tmpAngle);
	float xr = cx + radiusR * cos(tmpAngle);
	float yr = cy + radiusR * sin(tmpAngle);
	trackLogger << xl << " \t " << yl << " \t " << xr << " \t " << yr <<  endl;
	
	if(seg->type == TR_LFT)
	  angle += arcstep;
	else
	  angle -= arcstep;
     }   
    }
    
    seg = seg->next;
    
  }while(seg->id != segId);
  
}

void logger::logGnuplotPath(){

  char filePath[255];
  
  strcpy(filePath , BASE_PATH);
  strcat(filePath , track->internalname);
  strcat(filePath , "/");
  strcat(filePath , car->_carName);
  strcat(filePath , GNUPLOT_PATH_LOGPATH);
  
  pathLogger.open(filePath);
  
  if(!pathLogger.is_open())
  {
    cout << "\n-------------------------------------------------------------" << endl;
    cout << "Error, can't open : " << filePath << endl; 
    cout << "-------------------------------------------------------------" << endl;
    return;
  }
  else
  {
    cout << "\n-------------------------------------------------------------" << endl;
    cout << "Logging path in : " << filePath << endl; 
    cout << "-------------------------------------------------------------" << endl;
  }
  
  trajectory->logPath(pathLogger);
  
}

void logger::logMatlabData(){
  
  char filePath[255];
  
  strcpy(filePath , BASE_PATH);
  strcat(filePath , track->internalname);
  strcat(filePath , "/");
  strcat(filePath , car->_carName);
  strcat(filePath , MATLAB_TRACK_LOGPATH);
  
  ofstream matlabLogger(filePath); 
  
  if(!matlabLogger.is_open())
  {
    cout << "\n-------------------------------------------------------------" << endl;
    cout << "Error, can't open : track log : " << filePath << endl; 
    cout << "-------------------------------------------------------------" << endl;
    return;
  }
  else
  {
    cout << "\n-------------------------------------------------------------" << endl;
    cout << "Logging track in : " << filePath << endl; 
    cout << "-------------------------------------------------------------" << endl;
  }
  
  tTrackSeg *seg = track->seg;
  
  int first_id = seg->id; 
  double length_sum = 0;
  bool exit = false;
  double arc_sum = 0;
  double prec_arc , prec_radius;
  
  double width = FLT_MAX;
  double mu = FLT_MAX;

  do{
    
    if(seg->width < width)
      width = seg->width;
    if(seg->surface->kFriction < mu)
      mu = seg->surface->kFriction;
    
    if(seg->type == TR_STR)
    {
      length_sum = 0;
      do{
	length_sum += seg->length;
	seg = seg->next;
	if(seg->id == first_id)
	  exit = true;
      }while(seg->type == TR_STR && exit == false);
	
      matlabLogger << "[0,0," << length_sum << "]"<< endl; 
       
    }
    else
    {
      if(seg->type == TR_LFT)
      {
	if(seg->prev->type != TR_STR)
	  matlabLogger << "[0,0,0.1]"<< endl; 	
	arc_sum = 0;
	do{
	arc_sum += seg->arc;
	prec_arc = seg->arc;
	prec_radius = seg->radius;
	seg = seg->next;
	if(seg->id == first_id)
	  exit = true;
  
	}while(seg->type == TR_LFT && exit == false && prec_radius == seg->radius && prec_arc == seg->arc);
	
	matlabLogger << "[1," << prec_radius << "," << arc_sum << "]" << endl;
	
      }
      else
      {
	if(seg->prev->type != TR_STR)
	  matlabLogger << "[0,0,0.1]"<< endl; 
	arc_sum = 0;
	do{
	arc_sum -= seg->arc;
	prec_arc = seg->arc;
	prec_radius = seg->radius;
	seg = seg->next;
	if(seg->id == first_id)
	  exit = true;
  
	}while(seg->type == TR_RGT && exit == false && prec_radius == seg->radius && prec_arc == seg->arc);
	
	matlabLogger << "[1," << prec_radius << "," << arc_sum << "]" << endl;

	}
      }

  }while(exit == false);
  
  matlabLogger.close();
 
  strcpy(filePath , BASE_PATH);
  strcat(filePath , track->internalname);
  strcat(filePath , "/");
  strcat(filePath , car->_carName);
  strcat(filePath , MATLAB_CAR_LOGPATH);
  
  matlabLogger.open(filePath); 
  
  if(!matlabLogger.is_open())
  {
    cout << "\n-------------------------------------------------------------" << endl;
    cout << "Error, can't open : car log : " << filePath << endl; 
    cout << "-------------------------------------------------------------" << endl;
    return;
  }
  else
  {
    cout << "\n-------------------------------------------------------------" << endl;
    cout << "Logging car in : " << filePath << endl; 
    cout << "-------------------------------------------------------------" << endl;
  }
  
  const double maxAccel = 	myCar->getMaxAccel();
  const double maxBrake = 	myCar->getMaxDecel();
  const double minTurning = 	myCar->getMinTurn();
  const double maxSpeed = 	myCar->getMaxSpeed();
  
  matlabLogger << "\n-------------------------------------------------------------" << endl;
  matlabLogger << "Track data : width -> " << width - car->_dimension_y/2.0 << " , mu -> " << mu << endl;
  matlabLogger << "0 -> straight , 1 -> curve "<< endl;  
  matlabLogger << "\n-------------------------------------------------------------" << endl;
  matlabLogger << "Car data " << endl;
  matlabLogger << " vehicle{1}.A  = "<< maxAccel <<  "; % Maximum longitudinal aceleration  - m/s^2" << endl;
  matlabLogger << " vehicle{1}.D  = "<< maxBrake <<  "; % Maximum longitudinal deceleration - m/s^2" << endl;
  matlabLogger << " vehicle{1}.Rm  = "<< minTurning << "; % Minimum turning radius - m" << endl;
  matlabLogger << " vehicle{1}.L  = "<< car->_dimension_x <<  "; % Car length - m" << endl;
  matlabLogger << " vehicle{1}.W  = "<< car->_dimension_y <<  "; % Car width - m" << endl;
  matlabLogger << " bw = 0.4*vehicle{1}.A/"<< maxAccel << "; % Normalized viscous friction" << endl;
  matlabLogger << " vehicle{1}.b = 0.45/0.4*bw;     % Viscous friction" << endl;
  matlabLogger << " vehicle{1}.VM    = " << maxSpeed << "; % Maximum velocity - m/s" << endl;
  matlabLogger << " % sqrt((mu*G*R) / (1.0 - MIN(1.0, R*CA*mu/mass)))  Maximum lateral acceleration - m/s^2 " << endl;
  matlabLogger << " % mu = " << mu << " , G = 9.81 , R = radius , CA = "<< myCar->getCA() <<" , mass = "<< myCar->getMass() << endl;
  matlabLogger << "\n-------------------------------------------------------------" << endl;
  
}






