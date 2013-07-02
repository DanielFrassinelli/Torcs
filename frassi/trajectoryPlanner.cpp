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

#include "trajectoryPlanner.h"

ListDigraph trajectoryPlanner::graph;

int trajectoryPlanner::sectors = 0;			  
int trajectoryPlanner::nodePerLine = 0;
int trajectoryPlanner::nodePerPos = 0;
int trajectoryPlanner::nodePerSector = 0;

const double trajectoryPlanner::PATH_MAX_DIST  = 0.35;
const double trajectoryPlanner::PATH_MAX_STEER = 0.30;
const double trajectoryPlanner::RECOMPUTE_TIME = 5.00; /* 5 seconds */

trajectoryPlanner::trajectoryPlanner(tCarElt *mycar , carData *myCarData ,  tTrack* mytrack) 
: arcDist(graph , FLT_MAX), arcDistBackUp(graph), arcData(graph), nodeData(graph), nodeDist(graph , FLT_MAX) , dijkstra(graph , arcDist) , arcLookup(graph) 

{
  this->track = mytrack;
  this->car = mycar;
  this->myCar = myCarData;

  initTrajectory();

}

trajectoryPlanner::~trajectoryPlanner(){}

void trajectoryPlanner::initTrajectory(){

  if(initGraph())
  {
   COMPUTE_TRAJECTORY = &trajectoryPlanner::computeOptimalTrajectory;
   trajType = optimalTraj;
   cout << "--------------------------------------------------" << endl;
   cout << "Using : Optimal trajectory " << endl;
   cout << "--------------------------------------------------" << endl;
  }
  else
  {
   COMPUTE_TRAJECTORY = &trajectoryPlanner::computeDefaultTrajectory;
   trajType = defaultTraj;
   cout << "--------------------------------------------------" << endl;
   cout << "Using : Default trajectory " << endl;
   cout << "--------------------------------------------------" << endl;
  }
}

void trajectoryPlanner::computeDefaultTrajectory(vector <opponent *> * enemyCars){
  
  /* enemyCars is useless in this method */
  
  tangentAngle = RtTrackSideTgAngleL(&(car->_trkPos));

  distance = car->_trkPos.toMiddle;
  
  angle  = tangentAngle;
  
  stuckangle = (angle - car->_yaw);
  NORM_PI_PI(stuckangle);

  allowedSpeed = getDefaultSpeed();

}

void trajectoryPlanner::computeOptimalTrajectory(vector <opponent *> * enemyCars){
  
  tangentAngle = RtTrackSideTgAngleL(&(car->_trkPos));
  
  /*TODO enemyCars -> for each obtain the conflict with the arc of our graph,
  * update the graph , and recompute dijktra
  * restore the default arcmap
  */

  findPosition();

  distance = traj[manIndex].getDistance(car);

  angle = traj[manIndex].getAngle(car);

  stuckangle = (tangentAngle - car->_yaw);
  NORM_PI_PI(stuckangle);
  
  allowedSpeed = getOptimalSpeed();

  checkPath();	/* this is used to decide is we need to recompute the path*/

  if(recomputePath)	// if I have to recompute the path 
  {  
    computeOptimalPath(findNearestNode());
    recomputePath = false;
    recomputeTime = RECOMPUTE_TIME;
  }
  else
    recomputeTime = MAX(0.0 , recomputeTime - RCM_MAX_DT_ROBOTS);

}

double trajectoryPlanner::getDefaultSpeed(){   
  tTrackSeg *seg = car->_trkPos.seg->next; 
  double mu = seg->surface->kFriction;
  double c = mu*G;
  double d = (myCar->getCA()*mu + myCar->getCW())/(myCar->getMass());
  double maxlookaheaddist = myCar->getSpeedSqr()/(2.0 * mu * G);
  double lookaheaddist = distToSegEnd();
  double brakeAllowedSpeed = getAllowedSpeed(seg);
  while(lookaheaddist < maxlookaheaddist)
    {
      brakeAllowedSpeed = getAllowedSpeed(seg);
      if(brakeAllowedSpeed < car->_speed_x)
	{
	  double v2sqr = brakeAllowedSpeed * brakeAllowedSpeed;
	  double brake_dist = -log((c + v2sqr*d)/(c + myCar->getSpeedSqr()*d))/(2.0*d);		
	if(brake_dist > lookaheaddist)
	  return brakeAllowedSpeed;
	}
      lookaheaddist += seg->length;
      seg = seg->next;
    }
  return getAllowedSpeed(car->_trkPos.seg);  
}

double trajectoryPlanner::getOptimalSpeed(){
  double mu = car->_trkPos.seg->surface->kFriction;
  double c = mu*G;
  double d = (myCar->getCA()*mu + myCar->getCW())/(myCar->getMass());
  double maxlookaheaddist = myCar->getSpeedSqr()/(2.0 * c);
  double lookaheaddist = traj[manIndex].distToEnd(car);  
  int i = pathIndex;
  int j = manIndex;
  maneuver *man = getManeuver(i,j);

  if(car->_speed_x > man->getSpeed())	/* I am too fast here and now */
    return man->getSpeed();
  
  while(lookaheaddist < maxlookaheaddist)	/* look if I have to brake in the near future */
    {
      incManeuver(i , j);
      man = getManeuver(i,j);
      if(man->getSpeed() < car->_speed_x)
	{
	double brake_dist = -log((c + man->getSpeedSqr()*d)/(c + myCar->getSpeedSqr()*d))/(2.0*d);		
	if(brake_dist > lookaheaddist)
	  return man->getSpeed();	/* use the lower speed */
	}
      lookaheaddist += man->getLength();
    }
    
  return getManeuver(pathIndex , manIndex)->getSpeed();	/* keep the current speed */
  
}

double trajectoryPlanner::getAllowedSpeed(tTrackSeg* seg){
  double mu = seg->surface->kFriction;
  if(seg->type == TR_STR)
    return myCar->getMaxSpeed();
  else
    return sqrt((mu*G*seg->radius)/(1.0 - MIN(1.0, seg->radius*myCar->getCA()*mu/(myCar->getMass()))));
}

double trajectoryPlanner::distToSegEnd(){
  if(car->_trkPos.seg->type == TR_STR)
    return car->_trkPos.seg->length - car->_trkPos.toStart;
  else
    return (car->_trkPos.seg->arc - car->_trkPos.toStart)*car->_trkPos.seg->radius;  
}

bool trajectoryPlanner::loadNodes(){

  if(countNodes(graph) > 0)	/* the graph is static, so the other drivers doesn't need to compute it again */
    return true;
  
  char nodesFilePath[255];	/* the path are written in variables.h */

  strcpy(nodesFilePath , BASE_PATH);
  strcat(nodesFilePath , track->internalname);
  strcat(nodesFilePath , "/");
  strcat(nodesFilePath , car->_carName);
  strcat(nodesFilePath , NODES_FILE);

  ifstream input(nodesFilePath);
  
  if(!input.is_open())
  {
    cout << "\n-------------------------------------------------------------" << endl;
    cout << "Error "<< car->_carName <<", can't open : node file : " << nodesFilePath << endl; 
    cout << "-------------------------------------------------------------" << endl;
    return false;  
  }
  else
  {
    cout << "\n-------------------------------------------------------------" << endl;
    cout << "Loading "<< car->_carName <<" : node file : " << nodesFilePath << endl; 
    cout << "-------------------------------------------------------------" << endl;
  }

  
  int sect = 0 , nodes = 0;
  double xpos = 0.0 , ypos = 0.0 , vel = 0.0, angle = 0.0;
  string line, temp;
  
  tTrackSeg *seg = track->seg;
  double baseY = (seg->vertex[TR_SL].y + seg->vertex[TR_SR].y)/2;
  double baseX = (seg->vertex[TR_SL].x + seg->vertex[TR_SR].x)/2; 
  
  while(input)
  {    
	getline(input , line);
	
	line = line.substr(0 , line.find('#'));	// # = comments
	
	if(line.length() == 0)
	    continue;
	
	for(int i=0; i < nodes_param; i++)
	{
	
	size_t pos = line.find(',');
	temp = 	line.substr(0, pos);
	line.erase(0, pos + 1);   

	switch(i)
	{
	  case 0 : xpos = atof(temp.c_str());  break;  //xpos
	  case 1 : ypos = atof(temp.c_str());  break;  //ypos 
	  case 2 : angle = atof(temp.c_str()); break;  //angle
	  case 3 : vel = atof(temp.c_str());   break;  //speed
	  case 4 : nodes = atoi(temp.c_str());  break; //position in line
	  case 5 : break; //node id
	  case 6 : sect = atoi(temp.c_str());  break; //sector id  
	  default : break;	  
	}
	
	}
	
	nodePerLine = MAX(nodePerLine , nodes);
	sectors	    = MAX(sectors , sect);
	
	node n = graph.addNode();
	node_data tmp(xpos + baseX , ypos + baseY , vel , angle, sect - 1);
	
	nodeData[n] = tmp;
		
  }
  
  nodePerSector = countNodes(graph) / sectors;
  nodePerPos    = nodePerSector / nodePerLine;

  input.close();
 
 //TODO beta version 
  
 for(nodeIt n(graph); n != INVALID; ++n)	/* for each node I find the segment that contains it */
 {
   tTrackSeg *seg = track->seg;
   tTrackSeg *base = seg;
   
   do{
     if(isInsideSeg(nodeData[n].pos, seg))
       break;
     else    
      seg = seg->next;
    }while(base != seg);

    nodeData[n].seg = seg;    
  }
  
  return true;

}

bool trajectoryPlanner::loadLinks(){
  
  char linksFilePath[255];
  string line , temp;
  int src = 0 , dst = 0;
  
  strcpy(linksFilePath , BASE_PATH);
  strcat(linksFilePath , track->internalname);
  strcat(linksFilePath , "/");
  strcat(linksFilePath , car->_carName);
  strcat(linksFilePath , LINKS_FILE);
  
  ifstream input(linksFilePath);
  
  if(!input.is_open())
  {
    cout << "\n-------------------------------------------------------------" << endl;
    cout << "Error car "<< car->_carName <<", can't open : links file : " << linksFilePath << endl; 
    cout << "-------------------------------------------------------------" << endl;
    return false;  
  }
  else
  {
    cout << "\n-------------------------------------------------------------" << endl;
    cout << "Loading "<< car->_carName <<" : links file : " << linksFilePath << endl; 
    cout << "-------------------------------------------------------------" << endl;
  }
  
  while (input) 
      {			
	getline(input , line);
	
	line = line.substr(0 , line.find('#'));	// # = comments
	
	if(line.length() == 0)
	    continue;
	
	for(int i=0; i < links_param; i++)
	{
	  size_t pos = line.find(',');
	  temp = line.substr(0, pos);
	  line.erase(0, pos + 1);   

	  switch(i)
	  {
	    case 0 : src = atoi(temp.c_str())-1;  break;  //src -1 because the first node id is 1 instead of 0
	    case 1 : dst = atoi(temp.c_str())-1;  break;  //dst 
	    default : break;	  
	  } 
	}
	node a = graph.nodeFromId(src);
	node b = graph.nodeFromId(dst);    
	graph.addArc(a,b);	
      }
 
  input.close();
  
  return true;
  
}

bool trajectoryPlanner::loadManeuvers(){
  
  char manFilePath[255];
  string line, temp;
  int src , dst , currentSector = 0;
  double speed = 0.0 , time = 0.0 , radius = 0.0 , dist = 0.0 , cx = 0.0 , cy = 0.0;
  arc currentArc;
  double baseY = (track->seg->vertex[TR_SL].y + track->seg->vertex[TR_SR].y)/2;
  double baseX = (track->seg->vertex[TR_SL].x + track->seg->vertex[TR_SR].x)/2; 
  
  strcpy(manFilePath , BASE_PATH);
  strcat(manFilePath , track->internalname);
  strcat(manFilePath , "/");
  strcat(manFilePath , car->_carName);
  strcat(manFilePath , MANEUVERS_FILE);
  
  ifstream input(manFilePath);
  
  if(!input.is_open())
  {
    cout << "\n-------------------------------------------------------------" << endl;
    cout << "Error "<< car->_carName <<", can't open : maneuvers file : " << manFilePath << endl; 
    cout << "-------------------------------------------------------------" << endl;
    return false;  
  }
  else
  {
    cout << "\n-------------------------------------------------------------" << endl;
    cout << "Loading "<< car->_carName <<" : maneuvers file : " << manFilePath << endl; 
    cout << "-------------------------------------------------------------" << endl;
  }
  
  arcLookup.refresh();		/* we update the arc lookup class */
  
  int start = firstIndexOfSector(0);
  int end = start + nodePerSector;	  
  double lastAngle = RtTrackSideTgAngleL(&car->_trkPos);
  NORM0_2PI(lastAngle);
  
  for(int i = start; i < end; i++)	/* init first sector */
    {
      node tmp = graph.nodeFromId(i);
      nodeData[tmp].angle = lastAngle;   
    }
  
  int cou = 0;
    
  while (input) 
      {	
       getline(input , line);
	
       line = line.substr(0 , line.find('#'));	// # = comments
	
	if(line.length() == 0)
	    continue;
	
	for(int i=0; i < maneuvers_param; i++)
	{
	  size_t pos = line.find(',');
	  temp = 	line.substr(0, pos);
	  line.erase(0, pos + 1);   

	  switch(i)
	  {
	    case 0 : src = atoi(temp.c_str())-1;  break;  //src -1 because the first node id is 1 instead of 0
	    case 1 : dst = atoi(temp.c_str())-1;  break;  //dst 
	    case 2 : time = atof(temp.c_str());   break;  //time
	    case 3 : speed = atof(temp.c_str());  break;  //speed
	    case 4 : dist = atof(temp.c_str());   break;  //length
	    case 5 : radius = atof(temp.c_str()); break;  //radius
	    case 6 : cx = atof(temp.c_str());     break;  //center x
	    case 7 : cy = atof(temp.c_str()); 	   break;  //center y    
	    default : break;	  
	  }	  
	}

	cou ++;

	if(time == 0.0)	 /* the first maneuver is useless */
	  continue;
	
	node s = graph.nodeFromId(src);
	node d = graph.nodeFromId(dst);
	
	currentArc = arcLookup(s , d);
	
	if(arcDist[currentArc] == FLT_MAX)
	  arcDist[currentArc] = 0;
	
	arcDist[currentArc] += time;


	if(currentSector != nodeData[s].sector)	// with this method we update all the angle of the new sector with the final angle of the previus 
	{
	  start = firstIndexOfSector(currentSector);	// first node of the first sector 
	  currentSector = (currentSector + 1) % sectors;	// new sector 
	  arc a = INVALID;
	  
	  for(int i = start; i < start + nodePerSector; i++)
	  {
	    outArcIt t(graph, graph.nodeFromId(i));
	    if( t != INVALID)
	     a = t; 
	  }

	  lastAngle = arcData[a].back().getEndAngle();		// last angle of the arc, should be equal for all the arcs 
	  
	  start = firstIndexOfSector(currentSector);	
	  end = start + nodePerSector;

	  for(int i = start; i < end; i++)	// update the new sector 
	  {
	      node tmp = graph.nodeFromId(i);
	      nodeData[tmp].angle = lastAngle;   
	  }  
	}

	v2d position = (arcData[currentArc].empty()) ? nodeData[graph.source(currentArc)].pos : arcData[currentArc].back().getEndPoint(); 
	angle        = (arcData[currentArc].empty()) ? nodeData[graph.source(currentArc)].angle : arcData[currentArc].back().getEndAngle();
	tTrackSeg *startSeg = (arcData[currentArc].empty()) ? nodeData[graph.source(currentArc)].seg : arcData[currentArc].back().getEndSeg();
	tTrackSeg *endSeg = nodeData[graph.target(currentArc)].seg;	
	
	maneuver man(position , myCar , dist , radius , angle , speed , cx + baseX , cy + baseY, startSeg, endSeg, this); 

	arcData[currentArc].push_back(man);

      }       
      return true;     
}

double trajectoryPlanner::distFromSegStart(const v2d point, const tTrackSeg *seg){
  
  if(seg->type == TR_STR)
  {
    v2d target;
    
    v2d sp(seg->vertex[TR_SL].x , seg->vertex[TR_SL].y);
    v2d ep(seg->vertex[TR_EL].x , seg->vertex[TR_EL].y);
    
    if(sp.y == ep.y)
    {
      target.x = point.x;
      target.y = sp.y;
    }
    else if(sp.x == ep.x)
    {
      target.y = point.y;
      target.x = sp.x;
    }
    else{
            
    double m1 = (sp.y-ep.y)/(sp.x-ep.x);
    double q1 = ep.y - ep.x*m1;
    
    double m2 = -1/m1;
    double q2 = point.x/m1 + point.y;

    target.x = (q1-q2)/(m2-m1);
    target.y = q1 + m1*target.x;
    }    
    return target.dist(point);
  }
  else
  {
    v2d center(seg->center.x , seg->center.y);
    v2d pnt = point - center; 
    v2d tmp = v2d(seg->vertex[TR_SL].x, seg->vertex[TR_SL].y) - center;
    double theta = acos((pnt * tmp)/ (pnt.len() * tmp.len())); 
    double radius = point.dist(center);
    return radius * theta;   
  }

}


bool trajectoryPlanner::isInsideSeg(const v2d point , const tTrackSeg *seg){  
  v2d P1(seg->vertex[TR_SL].x, seg->vertex[TR_SL].y);
  v2d P2(seg->vertex[TR_SR].x, seg->vertex[TR_SR].y);
  v2d P3(seg->vertex[TR_ER].x, seg->vertex[TR_ER].y);
  v2d P4(seg->vertex[TR_EL].x, seg->vertex[TR_EL].y);
  
  if(seg->type == TR_STR)
  {
  v2d P1_P4 = P1 - P4;
  v2d P3_P4 = P3 - P4;
  v2d TWO_P_C = 2.0*point - P1 - P3;    // TWO_P_C = 2P-C, C=Center of rectangle
 
  return (P3_P4*(TWO_P_C - P3_P4) <= 0 && P3_P4*(TWO_P_C + P3_P4) >= 0) &&
         (P1_P4*(TWO_P_C - P1_P4) <= 0 && P1_P4*(TWO_P_C + P1_P4) >= 0);
    	 
  }  
  else
  {
    v2d center(seg->center.x , seg->center.y);
    v2d pos = point - center; 
    v2d tmp = v2d(seg->vertex[TR_SL].x , seg->vertex[TR_SL].y) - center;
    double theta =  acos((pos * tmp)/ (pos.len() * tmp.len()));
    double dist = point.dist(seg->center.x , seg->center.y);
    
    if(seg->type == TR_LFT)
      return (dist >= seg->radiusl && dist <= seg->radiusr) && theta >= 0.0 && theta <= seg->arc; 
    else
      return (dist >= seg->radiusr && dist <= seg->radiusl) && theta >= 0.0 && theta <= seg->arc; 
  }
  
  return false;    
}
   

bool trajectoryPlanner::initGraph(){
  
  if(!loadNodes())
    return false;
  
  if(!loadLinks())
    return false;
  
  if(!loadManeuvers())
    return false;

  graphExpansion();
  
  dijkstra.distMap(nodeDist); //set the map that given a node return the distance computed by dijkstra
      
  computeOptimalPath(initNearestNode());

  recomputePath = false;
  recomputeTime = RECOMPUTE_TIME;

  for(pathIt p(path); p != INVALID; ++p)
  {
    cout << "------------------------------" << endl;
    cout << "src seg id : " << nodeData[graph.source(p)].seg->id << " dst seg id : " << nodeData[graph.target(p)].seg->id << endl;
    
    for(unsigned int i=0; i < arcData[p].size(); i++)
    {
      tTrackSeg *sr = arcData[p][i].getStartSeg();
      tTrackSeg *ds = arcData[p][i].getEndSeg();
      cout << "man : " << i << " -> startSeg : " << sr->id << " , endSeg : " << ds->id << endl;
    }
 
  }
  
  
  return true;
  
}

void trajectoryPlanner::graphExpansion(){
    
  //TODO graph copy?
  
  int start = firstIndexOfSector(graph.nodeFromId(countNodes(graph) - 1));	/*last sector */
  int end   = start + nodePerSector;
  int instart = firstIndexOfSector(0);				/* first sector */
  int inend   = instart + nodePerSector;
  
  for(int i = start ; i < end; i++)	/* make the graph cyclic by adding an arc for each final node 					  of weight zero connected with each node of the first sector*/
  {
    node tmp = graph.nodeFromId(i);   
    
    for(int k = instart; k < inend; k++)
    {
    node j = graph.nodeFromId(k);
    arc  a = graph.addArc(tmp , j);
    arcDist[a] = 0.0;        
    }    
  }
  
  for(arcIt a(graph); a != INVALID; ++a)	/* make the backUp map for arcs weight */
    arcDistBackUp[a] = arcDist[a];

}

ListDigraph::Node trajectoryPlanner::initNearestNode(){ 
  
  v2d carPosition (car->_pos_X , car->_pos_Y);

  double dist = FLT_MAX;
  node dst;
  
  int start = firstIndexOfSector(0);	/* first sector */ 
  int end = start + nodePerSector;
  
  for(int i = start ; i < end; i++) /* for each node of the first sector */
  {
    node tmp = graph.nodeFromId(i);
    double d = nodeData[tmp].pos.dist(carPosition); /* distance between node and car */
    if(d <= dist && nodeData[tmp].speed == 0)
    {
      dist = d;
      dst  = tmp;   
    }  
  }

  return dst;
  
}

ListDigraph::Node trajectoryPlanner::findNearestNode(){ 
  
  v2d carPosition (car->_pos_X , car->_pos_Y);

  double dist = FLT_MAX;
  double speed = FLT_MAX;
  
  node dst;
  
  /* I know that is one of the sibiling of the current source node of the arc of the current PathIndex */
  
  node src = graph.source(path.nth(pathIndex)); /* source node of current arc in the path */

  int start = firstIndexOfSector(src);
  int end   = start + nodePerSector;
  
  for(int i = start ; i < end ; i++) /*for each node in the sector*/
  {
    node tmp = graph.nodeFromId(i); /* candidate src node */
    double d = nodeData[tmp].pos.dist(carPosition); /* distance between node and car */
    
    if(d <= dist)
    {
      dist = d;
      double s = fabs(nodeData[tmp].speed - car->_speed_x); /* difference in speed */
      if(s <= speed)
      {
	  speed = s;
	  dst = tmp;
      }     
    }  
  }

  return dst;
  
}

void trajectoryPlanner::computeOptimalPath(node src){

  if(!path.empty())
    if(myCar->getMode() == STUCK || src == graph.source(path.nth(pathIndex))) //if I'm stuck or the src node is the same of the path 
      return;

  // I create a fake node to make possibile to have the src node as the dst node   
    
  node fake = graph.addNode();
  arc x     = graph.addArc(fake, src);
  arcDist[x] = 0.0;
    
  for(inArcIt a(graph, src); a != INVALID; ++a) 
  {
    x = graph.addArc(graph.source(a) , fake);
    arcDist[x] = arcDist[a];    
  }
     
  node dst = INVALID;  
  double dist = FLT_MAX;
  mapFill(graph, nodeDist , 0);
  
  dijkstra.run(src , fake);	
  
  int start = firstIndexOfSector(src);	/* sector of the currenct src node */
  int end   = start + nodePerSector;

  for(int i = start ; i < end; i++) 	/* for each node in the sector */
  {					/* this works only if the graph is cyclic */
    node tmp = graph.nodeFromId(i); 	
    if(nodeDist[tmp] > 0.0 && nodeDist[tmp] < FLT_MAX) 
      if(dist > nodeDist[tmp])
      {
	dst = tmp;
	dist = nodeDist[tmp];
      }
  }
  
  if(nodeDist[fake] > 0.0 && nodeDist[fake] < FLT_MAX && dist > nodeDist[fake])  
    dst = fake;

  if(dst != INVALID)
  {
  path.clear();
  path = dijkstra.path(dst); //path from dst node
  
  
  if(dst == fake)
  {
  arc last = path.back();		/* fake arc */  
  node parent = graph.source(last);	/* source node */  
  arc target  = INVALID;
  
  for(outArcIt out(graph, parent); out != INVALID; ++out) // find the arc connected with the src node
    if(graph.target(out) == src)
      target = out;
  
  path.eraseBack();		/* remove the fake arc */
  path.addBack(target);		/* add the corrent arc */
    
  }
  
  graph.erase(fake);		// erase the fake node
  
  pathIndex = 0;		 /* now we are on the first arc of the path */
  manIndex = 0;			 /* same for the manuevers */
  traj = arcData[path.front()]; /* maneuvers of the first arc */
  
  trajLength = 0;
  
  for(int i=0; i < path.length(); i++)
    trajLength += arcData[path.nth(i)].size(); 
 
  cout << "******************* " << endl;
  cout << "Dijsktra recomputed " << endl;
  cout << "src node : " << graph.id(graph.source(path.front())) << " dst node : " << graph.id(graph.target(path.back())) << endl;
  
  }
}

void trajectoryPlanner::checkPath(){

  if(recomputeTime > 0.0)
    return;

  tTrackSeg *seg = car->_trkPos.seg;
  
  double x = distance/seg->width;

  if(fabs(x) > PATH_MAX_DIST)
    recomputePath = true;
}

bool trajectoryPlanner::isCarInsideArc(arc t){
   
 //TODO base version, duplicate oparations
  
    int cid = car->_trkPos.seg->id;
    int sid = nodeData[graph.source(t)].seg->id;
    int did = nodeData[graph.target(t)].seg->id;
    
  // cout << "test : "  << cid << " did : " << did << " sid : " << sid << endl;
  
    if(sid <= did)
    {
      if(cid < did && cid > sid)
	return true;
    }
    else 
      if(cid < did || cid > sid)
	return true;
      

    if(cid == sid)
    {
      int i = 0; // first maneuver
      int length = arcData[t].size();
      maneuver *man;
      do{
	man = getManeuver(t,i);    
	if(man->isInside(car))
	  return true;
	  i++;   	    
	}while(man->getEndSeg()->id <= did && i < length);  
      return false;   
    }
    else
      if(cid == did)
      {
	int i = arcData[t].size() - 1;
	maneuver *man;
	do{
	  man = getManeuver(t,i);    
	  if(man->isInside(car))
	    return true;
	    i--;   	    
	  }while(man->getEndSeg()->id >= sid && i >= 0); 
	return false;
      }

    return false;      
}

void trajectoryPlanner::findPosition(){
  
  //TODO improve
  pathIt z(path);
  
  while(z != INVALID)
  {
   if(isCarInsideArc(z))
    {
     // cout << "path found : src node " << graph.id(graph.source(z)) << " dest node : " << graph.id(graph.target(z)) << endl; 
      break;
    }
    ++z;
  }

  int counter = 0;
  int p = pathIndex;
  int m = manIndex;
  
  while(!traj[manIndex].isInside(car) && counter <= trajLength)	/* while I'm not in the current Maneuver */
  {	
      manIndex ++;	/* inc maneuver index */     
      if((unsigned) manIndex == traj.size()) /* arc increment */
      {
	manIndex = 0;	
	pathIndex = incPathIndex(pathIndex);
	traj = arcData[path.nth(pathIndex)];
      }
  }  
  if(counter <= trajLength)
    return;
  pathIndex = p;
  manIndex  = m;  
  
 
}

inline void trajectoryPlanner::incManeuver(int &p , int &m){
  if(((unsigned) m + 1) == arcData[path.nth(p)].size())
  {
    m = 0;
    p = incPathIndex(p);
  }
  else
    m++;   
}

inline void trajectoryPlanner::decManeuver(int &p , int &i){  
  if(i > 0)
    i --;
  else
  {
    p = decPathIndex(p);
    i = arcData[path.nth(p)].size() - 1;
  }  
}

inline int trajectoryPlanner::incPathIndex(int p){ 
  p = (p + 1) % path.length(); 
  if(arcData[path.nth(p)].empty()) /* only if I'm in the "fake arc" with cost 0 */
    p = (p + 1) % path.length();
  return p;
}

inline int trajectoryPlanner::decPathIndex(int p){
  if(p == 0)
    p = path.length() - 1;
  else
    p --;
  
  if(arcData[path.nth(p)].empty())
  {
    if(p == 0)
      p = path.length() - 1;
    else
      p --;   
  }  
  
  return p;
}

void trajectoryPlanner::logPath(ofstream &log){

  for(pathIt p(path); p != INVALID; ++p)
  {
      for(unsigned int j=0; j < arcData[p].size(); j++)
      {
	arcData[p][j].logManeuver(log);
      }  
  } 
  
}

/* ------------------- maneuver ------------------------------------------------------- */

maneuver::maneuver(v2d start, carData *myCar , double dist , double radius, double angle, double speed ,
		   double cx , double cy, tTrackSeg *startSeg, tTrackSeg *endSeg, trajectoryPlanner *traj)
: startPoint(start) , center(cx,cy)
{

  if(radius >= FLT_MAX)
    type = line;
  else{
    if(radius < 0)
      type = curveL;
    else
      type = curveR;
  }
  
  if(type == line)
  {

    double distX = dist*cos(angle);
    double distY = dist*sin(angle);
    
    endPoint.x = start.x + distX;
    endPoint.y = start.y + distY;
    
    startAngle = angle; 
    endAngle = angle;
    
    length = dist;

    this->speed = myCar->getMaxSpeed();
    speedSqr = speed*speed;
    
    sx = (startPoint.rotate(v2d(0.0 , 0.0) , -startAngle)).x;
    fx = (endPoint.rotate(v2d(0.0 , 0.0) ,   -startAngle)).x;
    
    radius = -1.0;
    arc = -1.0;
    
    center.x = -1.0;
    center.y = -1.0;
    
  }
  else
  {
    this->radius = fabs(radius);
    startAngle = angle;
    length = dist;    
    arc = length / this->radius;
    this->speed = speed;
  
    if(type == curveL)
    {
      endPoint = startPoint.rotate(center , arc); 
      endAngle = startAngle + arc;
    }
    else
    {
      endPoint = startPoint.rotate(center , -arc); 
      endAngle = startAngle - arc;	
    }

  }
  
  //TODO beta version
  this->startSeg = startSeg; 
  tTrackSeg *seg = startSeg;
  
  do{
     if(traj->isInsideSeg(endPoint, seg))
       break;
     else    
      seg = seg->next;
   }while(seg != endSeg);
    
   if(seg == endSeg)
     this->endSeg = endSeg;
   else
     this->endSeg = seg;
}

double maneuver::getDistance(tCarElt *car){

  if(type == line)
  {
    v2d target;
    
    if(startPoint.y == endPoint.y)
    {
      target.x = car->_pos_X;
      target.y = startPoint.y;
    }
    else if(startPoint.x == endPoint.x)
    {
      target.y = car->_pos_Y;
      target.x = startPoint.x;
    }
    else{
            
    double m1 = (startPoint.y-endPoint.y)/(startPoint.x-endPoint.x);
    double q1 = endPoint.y - endPoint.x*m1;
    
    double m2 = -1/m1;
    double q2 = car->_pos_X/m1 + car->_pos_Y;

    target.x = (q1-q2)/(m2-m1);
    target.y = q1 + m1*target.x;
    }
    
    double result = target.dist(car->_pos_X , car->_pos_Y);
    
    double left = ((startPoint.x - endPoint.x)*(car->_pos_Y - endPoint.y) - (startPoint.y - endPoint.y)*(car->_pos_X - endPoint.x));

    if(left > 0)
      return -result;
    return  result;

  }
  else
  {
    double dist = center.dist(car->_pos_X , car->_pos_Y);
    
    if(type == curveL)    
      return radius - dist;
    return dist - radius;
  }

}

double maneuver::getAngle(tCarElt *car){
  if(type == line)
  {
    return startAngle;
  }
  else
  { 
    double theta = angleFromStart(car);
    if(type == curveL)
      return theta + startAngle;
    return startAngle - theta;
  }
   
}

inline double maneuver::angleFromStart(tCarElt *car){  
  v2d carPos = v2d(car->_pos_X , car->_pos_Y) - center; 
  v2d tmp = 	startPoint - center;
  return acos((carPos * tmp)/ (carPos.len() * tmp.len())); 
}

bool maneuver::isInside(tCarElt *car){
    
  int cid = car->_trkPos.seg->id;
  int sid = startSeg->id;
  int did = endSeg->id;
    
  //cout << "test : "  << cid << " did : " << did << " sid : " << sid << endl;
  
  if(sid <= did)
  {
    if(cid < did && cid > sid)
      return true;
  }
  else 
    if(cid < did || cid > sid)
      return true;
      
  if(type == line)
  {
    v2d carPos(car->_pos_X , car->_pos_Y);
    carPos = carPos.rotate(v2d(0.0,0.0) , -startAngle);
    return (sx <= carPos.x) && (carPos.x <= fx);
  } 
  else
  { 
    //TODO add radius check
    //double dist = center.dist(car->_pos_X , car->_pos_Y);    
    double theta =  angleFromStart(car); 
    return (theta > 0.0 && theta < arc);
  }
  
}

double maneuver::distToEnd(tCarElt *car){  
  if(type == line)
  {
    v2d carPos(car->_pos_X , car->_pos_Y);
    return carPos.dist(endPoint);   
  }
  else
  {
    double theta = angleFromStart(car);
    return fabs(arc - theta)*radius;
  }
}

void maneuver::logManeuver(ofstream &log){
  if(type == line)
  {
    log << startPoint.x << " \t " << startPoint.y << endl;
    log << endPoint.x << " \t " << endPoint.y << endl;
  }
  else
  {
    int step = (int) length;
    
    double arcStep = arc / step + 0.0;
    double a = 0;

    for(int i = 0; i < step; i++)
    {
      v2d tmp(startPoint);
      if(type == curveL)
	a += arcStep;
      else
	a -= arcStep;
      
      tmp = tmp.rotate(center , a);
      log << tmp.x << " \t " << tmp.y << endl; 
    }    
  } 
}







