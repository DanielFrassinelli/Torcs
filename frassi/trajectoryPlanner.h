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

using namespace lemon; /* graph library */
using namespace std;

class maneuver {

  public:
    
    maneuver(v2d start, carData *myCar ,double dist , double radius, double angle , double speed , double cx , double cy);
    
    inline int getType()       	{ return type; }
    inline double getLength()   	{ return length; }
    inline double getRadius()   	{ return radius; }
    inline double getArc()      	{ return arc; }
    inline double getStartAngle()   	{ return startAngle; }
    inline double getEndAngle()      	{ return endAngle; }
    inline double getSpeedSqr()	{ return speedSqr; }
    inline double getSpeed()		{ return speed;	   }
    inline v2d getStartPoint() 	{ return startPoint; }
    inline v2d getEndPoint() 		{ return endPoint; }
    
    double getDistance(tCarElt *car);
    double getAngle(tCarElt *car);
    double distToEnd(tCarElt *car);
    bool isInside(tCarElt *car);

    /* log methods */
    
    void logManeuver(ofstream &log);
    void print();

  private:
    
    v2d startPoint , endPoint , center;
    double length , radius , arc , startAngle , endAngle , speed , speedSqr;
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
    
    bool isInside(const v2d point , const tTrackSeg *seg); /* true if the point is inside the segment */ 
    
    void (trajectoryPlanner::*COMPUTE_TRAJECTORY)(vector <opponent *> *);
    inline void computeTrajectory(vector <opponent *> * enemyCars){ (this->*COMPUTE_TRAJECTORY)(enemyCars);}
    
    inline double getTrajectoryAngle() 	{ return angle; }
    inline double getTrajectoryStuck() 	{ return stuckangle; }
    inline double getTrajectorySpeed() 	{ return allowedSpeed; }
    inline double getTrajectoryDistance() 	{ return distance; }
    inline int    getTrajectoryType()  	{ return trajType; }
    
  private:

    static const int LAP_PLANNING;	/* number of laps for which we compute the optimal trajectory */
           
    tCarElt *car;
    carData *myCar;
    tTrack  *track; 
    double angle , distance , allowedSpeed , stuckangle; /* parameters needed by the controller, computed by computeTrajectory */
    double tangentAngle , recomputeTime;
    int trajType;

    typedef struct nodedata {
      v2d pos;
      double speed;
      double angle;
      int sector;
      int lap;
      nodedata() : pos() {speed = 0.0; angle = 0.0; sector = 0; lap = 0;}
      nodedata(double x , double y, double speed , double angle , int sector) : pos(x , y) {this->lap = 0; this->angle = angle; this->speed = speed; this->sector = sector;}
    } node_data;

    typedef ListDigraph::Arc  arc; 	/*arc of the graph */
    typedef ListDigraph::Node node; 	/* node of the graph */
    typedef ListDigraph::ArcMap  <double> arcDistMap;	/* map for the weight of the arcs , used by dijktra */
    typedef ListDigraph::ArcMap  <vector <maneuver> > arcDataMap;	/* map that contains for each arc a vector of maneuvers */
    typedef ListDigraph::NodeMap <node_data> nodeDataMap;	/* map that contains all the nodes data */
    typedef ListDigraph::NodeMap <double> nodeDistMap;		/* map filled by dijktra with the cost for each node */
    typedef ListDigraph::ArcIt arcIt;  /* arc iterator */
    typedef ListDigraph::NodeIt nodeIt; /* node iterator */
    typedef ListDigraph::OutArcIt outArcIt; /* outgoing arc iterator of a node */
    typedef ListDigraph::InArcIt inArcIt;   /* ingoing  arc iterator of a node */ 	
    typedef ListPath<ListDigraph>::ArcIt pathIt; /* arc iterator for the path */
  
    static int sectors;		/* number of sectors of the graph , for one lap */			  
    static int nodePerLine;	/* number of nodes per line  */
    static int nodePerPos ;	/* number of nodes per speed */
    static int nodePerSector;  /* nodePerline * nodePerPos */
    
    bool recomputePath;		/* set true if you want to recompute the optimal path */
    
    static ListDigraph graph;   		   /* directed graph , shared between all the drivers */
    arcDistMap arcDist;    		           /* Map that stores the weight of the arcs */
    arcDistMap arcDistBackUp;			   /* Map that stores the backUp weight of the arc */	
    arcDataMap arcData;				   /* Map that stores a vector of maneuvers for the arcs */
    nodeDataMap nodeData;     			   /* Map that stores attributes for nodes */
    nodeDistMap nodeDist;			   /* Map that stores the cost to go to that node, return after the execution of Dijkstra */
    ListPath <ListDigraph> path;		   /* Path to follow , returned by dijkstra. Contains a list of arcs */
    Dijkstra <ListDigraph , arcDistMap> dijkstra;/* Dijkstra class */
    ArcLookUp <ListDigraph> arcLookup;		  /* this can be used to retrieve an arc, given src and dst node, in O(log m) time */
     
    int pathIndex;                               /* index of the current arc , maneuver */
    int manIndex;
    vector <maneuver> traj;			   /*Pointer to the current vector of maneuvers to follow */
    
    /* utils methods */
    
    inline int firstIndexOfSector(const node src) { return nodeData[src].sector*nodePerSector; }
    inline int firstIndexOfSector(const int src) {  return src*nodePerSector; } 
    inline int prevSector(const node src) { return (nodeData[src].sector == 0) ? sectors -1 : nodeData[src].sector - 1; }    
    inline int prevSector(const int  src) { return (src == 0) ? sectors -1 : src - 1; }   
    
    inline maneuver * getManeuver(int p, int m) { return &arcData[path.nth(p)][m]; }
    inline void incManeuver(int &path , int &man);
    inline void decManeuver(int &p , int &i);
    inline int incPathIndex(int p);
    inline int decPathIndex(int p);
    
    inline bool isBetween(double p , double s , double f) { return (f > s) ? (p > s && p < f) : (p < s && p > f); }
           
    /* used during execution */
    
    void computeOptimalTrajectory(vector <opponent *> * enemyCars);	/* called by the driver */	
    double getOptimalSpeed();						/* compute the allowedSpeed */
   
    static const double PATH_MAX_DIST;		/* max distance related to the track width from the trajectory */
    static const double PATH_MAX_STEER;	/* max steering related to the max steering of the car */
    static const double RECOMPUTE_TIME;	/* time that have to pass between two Dijktra recomputation */
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
    void graphExpansion(int n);/* extend the graph so we can plan more than one lap */
    ListDigraph::Node initNearestNode();	/* find the nearest node */
    
    /* logging methods */
    
    void logTrack();			/* log track data and car data */
    void logPath();			/* log track and path in gnuplot format */
    void printCurrentPath();
       
    /*default methods for following the raceline , used if initGrap return false */
    
    void computeDefaultTrajectory(vector <opponent *> * enemyCars);
    double getDefaultSpeed(); 
    
    
    void logPath2();

};


    
#endif

