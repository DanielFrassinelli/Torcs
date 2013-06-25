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

const bool LOG_CAR_AND_TRACK_DATA	= false; /* log the track and car data , used by matlab -> trajectoryPlanner */
const bool LOG_CAR_TEST    		= false; /* log beavhiour of the car -> carData */
const bool LOG_PATH_GNUPLOT 		= true; /* log the path/track in gnuplot format -> trajectoryPlanner */
const bool DEBUG_MESSAGE		= true ; /* show debug messages */


enum {START = 0 , NORMAL = 1, STUCK = 2};	/* car mode */
enum {DRWD = 0, DFWD = 1, D4WD = 2 };  	/* car train type */
enum {line = 0 , curveL = 1 , curveR = 2 };	/* maneuvers type */
enum {optimalTraj = 0 , defaultTraj = 1};	/* algorithm type */


#define BASE_PATH      		"/home/daniel/Desktop/torcs/"

/* path used for logging tha path and track in gnuplot format */

#define TRACK_GNUPLOT_LOG_PATH  "/log/gnuplot_track.csv"
#define PATH_GNUPLOT_LOG_PATH	 "/log/gnuplot_path.csv"

/* path used for logging track and car Data */

#define TRACK_LOG_PATH  	"/log/track.csv"
#define CAR_DATA_LOG_PATH    	"/log/carData.csv"
#define CAR_TEST_LOG_PATH   	"/log/carTest.csv"

/* file needed for load a graph */

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
#define SECT_FRICTION "friction"

#endif

