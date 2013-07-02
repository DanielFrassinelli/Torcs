/***************************************************************************

    file                 : variables.h
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

#ifndef _VARIABLES_H_
#define _VARIABLES_H_

#ifdef _WIN32
  #include <windows.h>
#endif

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <stdlib.h> 
#include <string.h> 
#include <math.h>

#include <tgf.h> 
#include <track.h> 
#include <car.h> 
#include <raceman.h> 
#include <robottools.h>
#include <robot.h>

#include <lemon/list_graph.h>
#include <lemon/dijkstra.h>
#include <lemon/path.h>

#include "linalg.h"

using namespace lemon; /* graph library */
using namespace std;

enum {START = 0 , NORMAL = 1, STUCK = 2};	/* car mode */
enum {DRWD = 0, DFWD = 1, D4WD = 2 };  	/* car train type */
enum {line = 0 , curveL = 1 , curveR = 2 };	/* maneuvers type */
enum {optimalTraj = 0 , defaultTraj = 1};	/* algorithm type */
   
/* public definitions */   
   
typedef ListDigraph::Arc  arc; 		/*arc of the graph */
typedef ListDigraph::Node node; 		/* node of the graph */
typedef ListDigraph::ArcIt arcIt;  		/* arc iterator */
typedef ListDigraph::NodeIt nodeIt; 		/* node iterator */
typedef ListDigraph::OutArcIt outArcIt; 	/* outgoing arc iterator of a node */
typedef ListDigraph::InArcIt inArcIt;  	 /* ingoing  arc iterator of a node */ 	
typedef ListPath<ListDigraph>::ArcIt pathIt;	 /* arc iterator for the path */
    
/* root folder */

#define BASE_PATH      			"/usr/src/torcs/torcs-1.3.4/src/drivers/frassi/data/"

/* file needed for load the graph */

#define NODES_FILE		  "/nodes.csv"
#define LINKS_FILE		  "/links.csv"
#define MANEUVERS_FILE		  "/maneuvers.csv"

/* column per file */

const int nodes_param = 7;
const int links_param = 3;
const int maneuvers_param = 8;

/* XML pattern for private attributes */

#define FRASSI_PRIV "frassi private"

#define SECT_MAXSPEED "maxSpeed"
#define SECT_MAXDECEL "maxDecel"
#define SECT_MAXACCEL "maxAccel"
#define SECT_MINTURN  "minTurn"

#define SECT_FRICTION "friction"

#endif

