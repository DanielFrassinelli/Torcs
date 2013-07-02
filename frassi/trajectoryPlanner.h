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

#ifndef _TRAJECTORYPLANNER_H_
#define _TRAJECTORYPLANNER_H_

#include "carData.h"
#include "opponent.h"

class trajectoryPlanner;

class maneuver {

  public:
    
    maneuver(v2d start, carData *myCar ,double dist , double radius, double angle , double speed , 
	     double cx , double cy, tTrackSeg *startSeg, tTrackSeg *endSeg, trajectoryPlanner *traj);
    
    inline int getType()       	{ return type; }
    inline double getLength()   	{ return length; }
    inline double getRadius()   	{ return radius; }
    inline double getArc()      	{ return arc; }
    inline double getStartAngle()   	{ return startAngle; }
    inline double getEndAngle()      	{ return endAngle; }
    inline double getSpeedSqr()	{ return speedSqr; }
    inline double getSpeed()		{ return speed;	   }
    inline tTrackSeg * getStartSeg()	{ return startSeg; }
    inline tTrackSeg * getEndSeg()	{ return endSeg;   }
    inline v2d getStartPoint() 	{ return startPoint; }
    inline v2d getEndPoint() 		{ return endPoint; }
    
    double getDistance(tCarElt *car);
    double getAngle(tCarElt *car);
    double distToEnd(tCarElt *car);
    bool isInside(tCarElt *car);

    /* log methods */
    
    void logManeuver(ofstream &log);

  private:
    
    v2d startPoint , endPoint , center;
    double length , radius , arc , startAngle , endAngle , speed , speedSqr;
    tTrackSeg *startSeg , *endSeg;
    double fx , sx;
    int type;
    
    inline double angleFromStart(tCarElt *car);

};

class trajectoryPlanner{
  
  public:

    trajectoryPlanner(tCarElt *mycar , carData *myCarData ,  tTrack *mytrack);
    ~trajectoryPlanner();
   
    double distToSegEnd();
    double getAllowedSpeed(tTrackSeg* seg);
        
    void (trajectoryPlanner::*COMPUTE_TRAJECTORY)(vector <opponent *> *);
    inline void computeTrajectory(vector <opponent *> * enemyCars){ (this->*COMPUTE_TRAJECTORY)(enemyCars);}
    
    inline double getTrajectoryAngle() 	{ return angle; }
    inline double getTrajectoryStuck() 	{ return stuckangle; }
    inline double getTrajectorySpeed() 	{ return allowedSpeed; }
    inline double getTrajectoryDistance() 	{ return distance; }
    inline int    getTrajectoryType()  	{ return trajType; }

    bool isInsideSeg(const v2d point , const tTrackSeg *seg);
    double distFromSegStart(const v2d point , const tTrackSeg *seg);
      
    /* log methods */
    
    void logPath(ofstream &log);
    
  private:
    
    static const double PATH_MAX_DIST;		/* max distance related to the track width from the trajectory */
    static const double PATH_MAX_STEER;	/* max steering related to the max steering of the car */
    static const double RECOMPUTE_TIME;	/* time that have to pass between two Dijktra recomputation */
    
    static int sectors;		/* number of sectors of the graph */			  
    static int nodePerLine;	/* number of nodes per line  */
    static int nodePerPos ;	/* number of nodes per speed */
    static int nodePerSector;  /* nodePerline * nodePerPos */
    
    bool recomputePath;		/* set true if you want to recompute the optimal path */
    
    tCarElt *car;
    carData *myCar;
    tTrack  *track; 
    double angle , distance , allowedSpeed , stuckangle; /* parameters needed by the controller, computed by computeTrajectory */
    double tangentAngle , recomputeTime;
    int trajType;

    typedef struct nodedata {
      v2d pos;
      double speed, angle , distance;
      int sector;
      tTrackSeg *seg;
      nodedata() : pos() {speed = 0.0; angle = 0.0; sector = 0; seg = NULL; distance = 0.0;}
      nodedata(double x , double y, double speed , double angle , int sector) 
      : pos(x , y) {this->seg = NULL; this->angle = angle; this->speed = speed; this->sector = sector; this->distance = 0.0;}
    } node_data;

    typedef ListDigraph::ArcMap  <double> arcDistMap;			/* map for the weight of the arcs , used by dijktra */
    typedef ListDigraph::ArcMap  <vector <maneuver> > arcDataMap;	/* map that contains for each arc a vector of maneuvers */
    typedef ListDigraph::NodeMap <node_data> nodeDataMap;		/* map that contains all the nodes data */
    typedef ListDigraph::NodeMap <double> nodeDistMap;			/* map filled by dijktra with the cost for each node */
    
    static ListDigraph graph;   		   	/* directed graph , shared between all the drivers */
    arcDistMap arcDist;    		          	/* Map that stores the weight of the arcs */
    arcDistMap arcDistBackUp;			   	/* Map that stores the backUp weight of the arc */	
    arcDataMap arcData;				  	/* Map that stores a vector of maneuvers for the arcs */
    nodeDataMap nodeData;     			   	/* Map that stores attributes for nodes */
    nodeDistMap nodeDist;			  	/* Map that stores the cost to go to that node, return after the execution of Dijkstra */
    ListPath <ListDigraph> path;		   	/* Path to follow , returned by dijkstra. Contains a list of arcs */
    Dijkstra <ListDigraph , arcDistMap> dijkstra;	/* Dijkstra class */
    ArcLookUp <ListDigraph> arcLookup;		  	/* this can be used to retrieve an arc, given src and dst node, in O(log m) time */
     
    int pathIndex;                              	/* index of the current arc , maneuver */
    int manIndex;
    int trajLength;
    vector <maneuver> traj;			   	/*Pointer to the current vector of maneuvers to follow */
    
    /* utils methods */
    
    inline int firstIndexOfSector(const node src) { return nodeData[src].sector*nodePerSector; }
    inline int firstIndexOfSector(const int src) {  return src*nodePerSector; } 
    inline int prevSector(const node src) { return (nodeData[src].sector == 0) ? sectors -1 : nodeData[src].sector - 1; }    
    inline int prevSector(const int  src) { return (src == 0) ? sectors -1 : src - 1; }   
    
    inline maneuver * getManeuver(int p, int m) { return &arcData[path.nth(p)][m]; }
    inline maneuver * getManeuver(arc p, int m) { return &arcData[p][m]; }
    inline void incManeuver(int &path , int &man);
    inline void decManeuver(int &p , int &i);
    inline int incPathIndex(int p);
    inline int decPathIndex(int p);
    
    inline bool isBetween(double p , double s , double f) { return (f > s) ? (p > s && p < f) : (p < s && p > f); }
           
    /* used during execution */
    
    void computeOptimalTrajectory(vector <opponent *> * enemyCars);	/* called by the driver */	
    double getOptimalSpeed();						/* compute the allowedSpeed */
    void checkPath();
    void findPosition();
    void computeOptimalPath(node src);
    ListDigraph::Node findNearestNode();
  
    /* initialisation mathods */
    
    void initTrajectory();	/* load the trajectory following method */
    
    bool loadNodes();		/* load nodes from path/torcs/track_name/car_name/nodes.csv */
    bool loadManeuvers();	/* load maneuvers from path/torcs/track_name/car_name/maneuvers.csv */
    bool loadLinks();		/* load links from path/torcs/track_name/car_name/links.csv */
    bool initGraph();		/* loadGraph , loadLinks , loadManeuvers , graphExpantion ,init */
    void graphExpansion();	/* extend the graph so we can plan more than one lap */
    ListDigraph::Node initNearestNode();	/* find the nearest node */
    
    bool isCarInsideArc(arc t);

    /*default methods for following the raceline , used if initGrap return false */
    
    void computeDefaultTrajectory(vector <opponent *> * enemyCars);
    double getDefaultSpeed(); 
    
};
 
    
#endif

