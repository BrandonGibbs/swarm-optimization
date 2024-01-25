Swarm intelligent robot navigation for Senior Project Spring 2024 (UTRGV)

Once argos is installed and you can run the following command without errors
	$ argos3 -q all

you should be able to compile using:
	$ mkdir build
	$ cd build
	$ cmake ..
	$ make

Then, to run anything in this project using argos, you need to be in the 
root directory of this repo so that the linker can find your libraries:
	$ cd ..
	$ argos3 -c config/lubot_swarm.argos
