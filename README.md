/*-------|---------|---------|---------|---------|---------|---------|---------|
brewtroller.c	

a simple temperature/boil controller for brewing or sous vide

by chaz miller 2013

Controller has 2 knobs, temperature setpoint and duty cycle.
With no temp probe, it acts as an open-loop 0-100% boil controller.
With temp probe connected it acts as a simple heat-only thermostat.
The duty cycle adjustment is still applied even in temp control mode. 

HARDWARE:
for ATMEGAxx8 set at 1MHz. 
LM335 on PC0 (10mV/K absolute)
3-4V pot on PC1 (temperature setpoint knob)
0-5V pot on PC2 (duty cycle control knob)
SSR and LED connected to PBwhatever

This is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License version 3 or any later
version. This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
