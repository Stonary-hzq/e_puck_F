**To use, simply place the folder Eclipse_e-puck2 where you want. It is just needed to have no space in the names of the parents folders.
Also avoid special characters.

Example : 	/home/student/epfl_stuff/Eclipse_e-puck2	OK
		/home/student/epfl stuff/Eclipse_e-puck2	NOT OK


Next you can create a shortcut of the Eclipse_e-puck2 executable if you want

**Some programs need to be installed in order to run Eclipse

--To install OpenJDK 8:
sudo add-apt-repository ppa:openjdk-r/ppa
sudo apt-get update
sudo apt-get install openjdk-8-jre


**If Eclipse_e-puck2 says « Could not open serial port /dev/tty* » when trying to debug, you have to type the following command in the terminal to put your user into the dialout group.

« sudo adduser $USER dialout »

Then restart your linux session and it should works.

