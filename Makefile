#!/bin/bash

nlateral_demo : nlateral_demo.cc
	g++ -O2 -g -Wall --std=c++14 \
	    -Wno-psabi \
	    -I . -I /opt/vc/include -L /opt/vc/lib \
	    -o nlateral_demo \
	    nlateral_demo.cc \
	    pi3hat.cc \
	    -lbcm_host \
	    -lpthread
