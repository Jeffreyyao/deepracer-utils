### For setting up the Localization Server:

The URL for the server is: http://CUBLabMediaServer:12345/OptiTrackRestServer

1. Run the motive only after starting the cameras.
2. Add the DeepRacer as a rigid body. This is done by first centering it in the arena, then adding it.
3. Add the targets and obstacles as rigid bodies as well.
4. Start the server using the [OptiTrackRESTServer](https://github.com/HyConSys/OptiTrackRESTServer)


### For setting up the Symbolic Control Server:

The URL for the server is: http://CUBLabMediaServer:12345/12345/pFaces/REST/dictionary/DeepRacer1

1. Run pFaces and the online synthesis with the DeepRacer example.


### For Connecting to the DeepRacer:

1. In a terminal, run the line: ssh deepracer@192.168.1.70 to connect
2. Run the line: source /opt/aws/deepracer/setup.sh
3. Run the line: python put_best_cal.py
4. Run the line: python sym_control/closedloop_online.py