Torcs
=====

Driver for torcs


Requirements

- install torcs 1.3.4+ < http://torcs.sourceforge.net/ > 
  using this guide :   < http://www.berniw.org/tutorials/robot/tutorial.html > (mandatory)

- lemon graph library 1.2.3+ < http://lemon.cs.elte.hu/trac/lemon > (mandatory)


- gnuplot < http://www.gnuplot.info/ > (Optional, if you want to print plot of the track / car)


The folder /Frassi is the driver, you have to copy in torcs/src/drivers and then compile and install with make, make install.

The file variables.h contain the path where the driver log things or seek for the graph

If you want to print in a fast way the logs you can use the command for gnuplot which are ni /frassi/gnuplot.txt




