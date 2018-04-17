************************************
INSTRUCTIONS FOR BUILDING AND TESTING
************************************
Step 0:
Install ROS on your linux machine

Step 1:
On the home directory of your linux machine, create a folder called “rosws”
This can be done using the command in the terminal “mkdir rosws”

Step 2:
Inside of the “rosws” directory, create a folder named “src”
This can be done using the commands in the terminal “cd rosws” and then “mkdir src”

Step 3:
Inside the “src” folder, create a package called “hw2”
This can be done using the commands in the terminal “cd src” then “catkin_create_pkg hw3”
This will generate 2 files inside of the “hw3” folder called “CMakeLists.txt” and “package.xml”

Step 4:
Copy the cpp files from our turned in assignment into the “hw3” folder you just created. Now inside the hw3 folder, there should be 4 files, named “CMakeLists.txt”, “hw3.cpp”, “package.xml”, and “hw3test.cpp”

Step 5:
Edit the “CMakeLists.txt” file inside of the “hw3” folder. On line 10, there is a “find_package()” function. Starting on line 10, paste in all of the following commands:
find_package(catkin REQUIRED COMPONENTS 
	roscpp
	std_msgs 
	)
add_executable(hw3 hw3.cpp)
add_executable(hw3test hw3test.cpp)
target_link_libraries(hw3 ${catkin_LIBRARIES})
target_link_libraries(hw3test ${catkin_LIBRARIES})


Under the “build” section of the document, there is one uncommented line which reads “include_directories (“  uncomment the 2 lines directly underneath so that the block of code looks like this:
include_directories(
include
${catkin_INCLUDE_DIRS}
)
Save and close the file

Step 6:
Open a new terminal in the “rosws” folder
You can use the command in the terminal “cd rosws”
Use the command “catkin_make”

Step 7:
Go into “rosws” and then into “build” and then into “hw3”. Copy the 2 executables “hw3”, and “hw3test”. Then go into “rosws” and then “src” and then “hw3” and paste the executables. There should now be 6 files in your hw3 file.

Step 8:
Go back to the terminal which is already in “rosws” folder. Type the command “roscore”

Step 9: Open a new terminal window. Type in the following command “rosrun turtlesim turtlesim_node”. This will open a turtlesim window, leave it open for step 10.

Step 10:
Open a new terminal window. Change the directory to “rosws” using the command “cd rosws”. Type in the following command “source devel/setup.bash”. In this window, type the command “rosrun hw3 hw3test”. This will run the professor's turtle sim program. You can view it in the turtlesim window opened in step 9.

Step 11:
Open a new terminal window. Change the directory to “rosws” using the command “cd rosws”. Type in the following command “source devel/setup.bash”. In this window, type the command “rosrun hw3 hw3”. This will run the our turtle sim program. You can view it in the turtlesim window opened in step 9.
