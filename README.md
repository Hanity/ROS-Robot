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

  ####  Building a workspace
  We have created a folder called **catkin_ws** which represents our workspace. Within this folder we manually created the **src** folder then we used the command 
  `catkin_make`. This created the **build** and **devel** directories.

  #### Launching roscore
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
**Summary**: 
The task is creating 2 nodes, the first publishes a value (rpm), and the second subscribes to the value of rpm and publishes the calculated speed.
The detailed description of this mini project can be found in the pdf named `ROS - Pub-Sub-Mini-Project`
#   ROS Parameter Server

#   ROS Launch Files

  ##  Mini-Project Example

#   ROS Bag Files

#   ROS Packages

#   ROS Services

  ##  Functional Example: Odd Even Service
  ##  Project Example: Image Retrieval


#   ROS Actions

  ##  Project Example

