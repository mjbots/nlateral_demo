#!/bin/bash

nlateral_demo : nlateral_demo.cc
	g++ -O2 -g -Wall --std=c++14 \
	    -lpthread \
	    -Wno-psabi \
	    -I . -I /opt/vc/include -L /opt/vc/lib -l bcm_host \
	    -o nlateral_demo \
	    nlateral_demo.cc \
	    pi3hat.cc
