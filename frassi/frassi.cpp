/***************************************************************************

    file                 : frassi.cpp
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

#ifdef _WIN32
#include <windows.h>
#endif

#include <stdio.h>
#include <stdlib.h> 
#include <string.h> 
#include <math.h>

#include <tgf.h> 
#include <track.h> 
#include <car.h> 
#include <raceman.h> 
#include <robottools.h>
#include <robot.h>

#include "driver.h"

#define BUFSIZE 20
#define NBOTS 3

static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s); 
static void newrace(int index, tCarElt* car, tSituation *s); 
static void drive(int index, tCarElt* car, tSituation *s); 
static void endrace(int index, tCarElt *car, tSituation *s);
static int  pitcmd(int index, tCarElt* car, tSituation *s);
static void shutdown(int index);
static int  InitFuncPt(int index, void *pt); 

static const char* botname[NBOTS] = {"frassi 1", "frassi 2", "frassi 3"};
static const char* botdesc[NBOTS] = {"frassi 1", "frassi 2", "frassi 3"};
static Driver * driver[NBOTS];


/* 
 * Module entry point  
 */ 
extern "C" int 
frassi(tModInfo *modInfo) 
{
    memset(modInfo, 0, 10*sizeof(tModInfo));
   
    int i;
    
    for (i = 0; i < NBOTS; i++) {
	modInfo[i].name = strdup(botname[i]);	/* name of the module (short) */
	modInfo[i].desc = strdup(botdesc[i]);	/* description of the module (can be long) */
        modInfo[i].fctInit = InitFuncPt;  /* init function */
        modInfo[i].gfId    = ROB_IDENT;	  /* supported framework version */
        modInfo[i].index   = i;           /* indices from 0 to 9 */
    }
    
    return 0; 
} 

/* Module interface initialization. */
static int InitFuncPt(int index, void *pt) 
{ 
    tRobotItf *itf  = (tRobotItf *)pt; 

    driver[index] = new Driver(index);
    
    itf->rbNewTrack = initTrack; /* Give the robot the track view called */ 
				 /* for every track change or new race */ 
    itf->rbNewRace  = newrace; 	 /* Start a new race */
    itf->rbDrive    = drive;	 /* Drive during race */
    itf->rbPitCmd   = pitcmd;
    itf->rbEndRace  = endrace;	 /* End of the current race */
    itf->rbShutdown = shutdown;	 /* Called before the module is unloaded */
    itf->index      = index; 	 /* Index used if multiple interfaces */
    return 0; 
} 

/* Called for every track change or new race. */
static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s) 
{ 
    driver[index]->initTrack(index , track , carHandle , carParmHandle , s);
} 

/* Start a new race. */
static void newrace(int index, tCarElt* car, tSituation *s) 
{ 
    driver[index]->newrace(index , car , s);
}

/* Drive during race. */
static void drive(int index, tCarElt* car, tSituation *s) 
{ 
    driver[index]->drive(index , s);
}

/* End of the current race */
static void endrace(int index, tCarElt *car, tSituation *s)
{
  driver[index]->endrace(index , s);
}

/* Pitstop callback */
static int pitcmd(int index, tCarElt* car, tSituation *s)
{
    return driver[index]->pitCommand(car, s);
}

/* Called before the module is unloaded */
static void shutdown(int index)
{
    delete driver[index];
}

