##############################################################################
#
#    file                 : Makefile
#    created              : Sat Mar 2 17:02:55 CET 2013
#    copyright            : (C) 2002 Daniel Frassinelli
#
##############################################################################
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
##############################################################################

ROBOT       = frassi
MODULE      = ${ROBOT}.so
MODULEDIR   = drivers/${ROBOT}
SOURCES     = ${ROBOT}.cpp driver.cpp carData.cpp trajectoryPlanner.cpp opponent.cpp logger.cpp

SHIPDIR     = drivers/${ROBOT}
SHIP        = ${ROBOT}.xml 155-DTM.rgb logo.rgb
SHIPSUBDIRS = 0 1  

PKGSUBDIRS  = ${SHIPSUBDIRS}
src-robots-frassi_PKGFILES = $(shell find * -maxdepth 0 -type f -print)
src-robots-frassi_PKGDIR   = ${PACKAGE}-${VERSION}/$(subst ${TORCS_BASE},,$(shell pwd))

include ${MAKE_DEFAULT}
