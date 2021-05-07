#   ISR-ROS-Project
This repository provides my personal summary of the documentation for the ROS course taught by Dr.  Abiodun Yusuf as part of the Intelligent Systems and Robotics Module

#   Brief Introduction
ROS (Robot Operating System) is an open source operating system dedicated to building robot applications. For detailed descriptions, please visit: http://wiki.ros.org/
#   Folder Structure


#   ROS Noetic Installation
For this course, we have used the latest ROS distribution released in May 23rd, 2020 and refered to as ROS Noetic Ninjemys. This distribution is primarly aimed towards
Ubuntu 20.04. As such it was installed on this OS.

For installation, please  follow the steps provided in the following link: http://wiki.ros.org/noetic/Installation/Ubuntu

#   Pre Starting Actions

Before experimenting on each of the components needed for building ROS applications, we did the following:

  ###  Building a workspace
  We have created a folder called **catkin_ws** which represents our workspace. Within this folder we manually created the **src** folder then we used the command 
  `catkin_make`. This created the **build** and **devel** directories.
  
  ### Creating the my_robot_turial package
  This was done inside the folder **catkin_ws/src/** using the command `catkin_create_pkg my_robot_tutorial roscpp rospy std_msgs`

  ### Launching roscore
  This is the first thing to be done everytime we use ROS. This command launches the ROS Master which is the central node for all other ROS nodes that communicate to build a full ROS-based system. Only one instance of this can be called.
  **Note**
  Each time a new terminal is opened, we have to source our workspace using the command `source devel/setup.bash`
#   ROS Nodes
A ROS node can be considered the basic unit of ROS Packages. It is pretty much just an excecutable file. The node uses a ROS cient library to communicate with other nodes through a publish-subscribe relationship. All nodes of this course are created within the scripts located in the directory `catkin_ws/src/my_robot_tutorial/scripts/`

#   Publishers/Subcribers
These are common notions. Basically publishers are codes that send messages under a specified topic. Subscribers are codes that search to receive the messages sent under the topic they seek. So it's basically what we call a giver and receiver.
 ##  Functional Example
In the location `catkin_ws/src/my_robot_tutorial/scripts/` we find the python script `publisher.py` that contains the code to publish the string “Hello World” under the topic hello_world. Our Subscriber script is in the same folder under the name `subscriber.py`. This last subscribes to the topic hello_world in order to receive all messages directed to this topic.
Now for the execution, it goes as follows:
- Launching the ROS master with the command `roscore`
- In a separate terminal, we run the publisher script using `python3 publisher.py`.
- We confirm that our publisher is working well by using the command `rostopic list`. This displays the topics being published. If we want to see the contents of the messages sent, we use the command `rostopic echo /hello_world`.
- In a separate terminal again, we run the subscriber script using `python3 subscriber.py`. A print was added to the code to make sure the messages were received correctly.
 ##  Mini-Project Example
######  Summary
The task is creating 2 nodes, the first publishes a value (rpm), and the second subscribes to the value of rpm and publishes the calculated speed.
The detailed description of this mini project can be found in the pdf named `ROS - Pub-Sub-Mini-Project`

   ######  Codes
   The codes for this project are within the files **task1-pub.py** and **task1-sub-pub.py**.
   
#   ROS Parameter Server
This is basically a centralized space to store variables that are frequently used and usually related to physical attributes of robots.
Since these values change overtime, it is more convenient to have them stored in such a server and called by the scripts whenever needed.

- **To change** a variable within the parameter server, we use for example the command `rosparam set /wheel_radius 0.155`.
- **To display** a variable within a terminal, we use `rosparam get /wheel_radius`.
- **To call** a variable within our scripts, we use `rospy.get_param('/wheel_size')`.

#   ROS Launch Files
These are files used to launch multiple scripts within a ROS application and also to set variable values in the parameter server. This largely simplfies the execution of our codes as it's done in one go instead of the tedious work of manually launching each file in a different terminal.

  ##  Mini-Project Example
  In this mini-project, we aim to create a launch file that will execute all the steps required in the previous Pub-Sub mini-project. It will do the following: 
  - Starting the ROS Master.
  - Setting the wheel_radius parameter in the ROS parameter server.
  - Running the RPM publisher.
  - Running the file subscribing to RPM, calculating and publishing the Speed.

  The code is called `speed_cal_sim.launch` and can be found in the directory `catkin_ws/src/my_robot_tutorial/Launch/`
  To execute the code, we use the command `roslaunch my_robot_tutorial rpm_sim.launch`
  
  **Note:** Don't forget to source the workspace whenever you use a new terminal, using `source catkin_ws/devel/setup.bash`

#   ROS Bag Files
A ROS Bag is basically a tool used to record messages sent under a certain topic, and playing them again whenever requested.
- A Bag file can be launched using the command `rosbag record -a -O test.bag`
- The recorded file can be played in a loop (-l) using the command `rosbag play -l test.bag`

#   ROS Packages
Packages are organized sotfware in ROS. They allow us to break up our robot application into reusable blocks such that the update and debugging of said application are simplified. 
### Creating packages
Creating a package in ROS is simple and is done using the command `catkin_create_pkg package_name dependencies`
### Installing and using packages
This is pretty simple too.
- First, you can follow https://index.ros.org/packages/page/1/time/ and find the suitable package(s) to create your desired application.
- To install a package, use the command `sudo apt install ros-noetic-package_name`
- To use a package, use the command `rosrun package_name file_to_execute`. Note that in most cases the files to execute would be launch files. Also, to achieve your desired result you logically have to run your packages files in the right succession.

#   ROS Services
ROS Services are used to achieve the request-reply (client-server) interaction. So the service received a request from a client, then does the necessary computations and eventually gives a response to the client. Unlike ROS packages, the services are scarce, which means developpers have to create their own services.
The next two examples will be used to demonstrate the creation and usage of services. 

  ##  Functional Example: Odd Even Service
 The purpose is to create a simple service that tells the client if the number he sent is odd or even.
  
  - We create the service file named  **OddEvenCheck.srv** and specify the type of input msg to receive from clients and the type of output msg to answer said clients. This is done as follows:
```
int32 number

---

string answer
```

- In the file **package.xml**, we uncomment the following lines:
```
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

- We edit the **CMakeLists.txt** file as follows:
```
add_service_files(
    FILES
    OddEvenCheck.srv
 )
    
 generate_messages(
   DEPENDENCIES
   sensor_msgs
   std_msgs
 )
 ```
 - We run catkin_make from the catkin workspace root directory.
 - We create the service script called **odd_even_service.py**.
 - Now we can use our created service through a terminal by using the command `rosservice call \OddEvenCheck input` and we will receive an answer.
 - We can also create the client script called **odd_even_client.py** that calls our service and may use it for further things.
 
 **Note:**
 As mentioned before, all these files are available in the **scripts** folder.


  ##  Project Example: Image Retrieval
This is a bit more advanced example showing the usage of services. The purpose is a service that receives a desired camera angle from the client. It then turns to the camera to this angle, takes a picture and sends the image back to the client. 
For the sake of simplicity, a folder containing images in angles -30°, -15°, 0°, 15° and 30° is provided. So the service received the desired angle and returns the appropriate image from this folder.
For further details, please consult the pdf named **ROS-Services-Project-Brief**.
The steps are similar to the previous example.
- First, in the **srv** folder, we create the service file **TurnCamera.srv** containing the following:
```
float32 turn_degrees
---
sensor_msgs/Image image
```
- Second, we edit the **CMakeLists.txt** file as follows:
```
## Generate services in the 'srv' folder
add_service_files(
    FILES
    OddEvenCheck.srv
    TurnCamera.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   sensor_msgs
   std_msgs
   geometry_msgs
 )
```
- Third, we run catkin_make from the catkin workspace root directory.
- Next, we create our service script called **turn_camera_service.py**.
- Lastly, we create our client script called **turn_camera_client.py**

#   ROS Actions
Actions are basically an extensions of Services. While Services are handy in many cases in ROS applications, there are however cases when we have application that takes long for the execution. In which case, the client usually would want to receive period feedback from the service or have the ability to stop the execution. This is done by actionlib package where we have an Action Server and an Action Client. Three parameters are defined: Goal, Feedback and Result.

For further reading, please see http://wiki.ros.org/actionlib/.

The next project will showcase the usage of actions
  ##  Project Example
The description of the project is found in the pdf named **ROS ACTIONS PROJECT BRIEF**
To create this project, we follow the next steps:
- Create a folder named **action** within our **my_robot_tutorial** folder.
- Inside this new folder, we create the file **navigate2D.action** that contains the definition of our 3 parameters:
```
#Goal
geometry_msgs/Point point
---
#Result
float32 elapsed_time
---
#feedback
float32 distance_to_point

```
- We add the following dependencies in the **package.xml** file:
```
<build_depend>actionlib</build_depend>
<build_depend>actionlib_msgs</build_depend>
<exec_depend>actionlib</exec_depend>
<exec_depend>actionlib_msgs</exec_depend>
```
- We edit the **CMakeLists.txt** file as follows:
```
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  sensor_msgs
  geometry_msgs
  genmsg
  actionlib
  actionlib_msgs
)

## Generate actions in the 'action' folder
  add_action_files(
  FILES
  navigate2D.action
    )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   sensor_msgs
   std_msgs
   geometry_msgs
   actionlib_msgs
 )

```
- We run catkin_make from the catkin workspace root directory.
- We create our **action_server.py** script and run it with the command `rosrun my_robot_tutorial action_server.py`
- We create our **action_client.py** script and run it.

# Important Notes
- Make sure that all python files in the scripts folder are executable using the command `chmod +x *.py`
- Whenever using a new terminal, make sure to source using the command `source devel/setup.bash`
