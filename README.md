# ns3

This contains example code to run LTE simulation environment with an ability to define multiple configuration scenarios via configuration text files.  These are namely - 

1. Define one more eNodeBs with their x/y/z coordinates
2. Define antenna model with their config params eg: Azimuth & tilt
3. Define multiple UEs using x/y/z co-ordinates
4. Define UEs with constant velocity / random walk / constant position ability.
5. Ability to pick different propagation / fading models
6. Define any number of buildings using x/y/z (min and max) values to define their footprints.
7. Specify in seconds the time the simulation would be run.

Finally the simulation would run with any environment setup and obtain average throughput values over the time simulation has been run.

Usage:
In order to build an run this package from the "ns3/ns-allinone-3.35/ns-3.35" dir do the following:
> ./waf configure --enable-examples --enable-tests
> ./waf build
> ./waf --run lte-sim
