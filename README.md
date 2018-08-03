# Human-centered manipulation and navigation with Robot DE NIRO

#### Overview

Robot DE NIRO is the work of Dr. Petar Kormushev and his team at Imperial College London's Robot Intelligence Lab.

Project DE NIRO was built using Python and the Robot Operating System (ROS).

## Additional Resources

Comprehensive documentation has been written for the code in this repository with guidance on installing and running components. A link is provided below.
         
**Documentation** website: <https://robot-intelligence-lab.gitlab.io/fezzik-docs>

A video and further illustrations of DE NIRO can be found here:

**Video**: <https://www.youtube.com/watch?v=qnvPpNyWY2M> 

**Photos**: <http://www.imperial.ac.uk/robot-intelligence/robots/robot_de_niro/> 

Datasets of DE NIRO's sensors will soon be released soon.

**Sensor Datasets**: <http://www.imperial.ac.uk/robot-intelligence/software/> 

## Requirements

#### Hardware
Naturally, **Robot DE NIRO is a fundamental requirement**. Project Fezzik will not work with other robots.

In addition, the following hardware components are needed.
1. 2D Hokuyo LIDAR scanner
2. Microsoft Kinect
3. Mbed device on base
4. Microphone (Optional requirement for voice commands)

Further, two computers are recommended: a dedicated computer for navigation and another for the other components. The latter should be
running Ubuntu 14.04 and ROS Indigo - this is essential for the Microsoft Kinect to work.

#### Software
1. Python 2.7
2. ROS Indigo and Kinetic

#### Component Dependencies

Project Fezzik is comprised of a number of ROS packages. All these have their individual dependencies which are listed here. These will
need to be installed alongside their own dependencies. Extensive [project documentation](https://robot-intelligence-lab.gitlab.io/fezzik-docs)
has been written for each component. This should be consulted if help is required during installation and set-up.

##### Audio out
* [eSpeak speech synthesiser](http://espeak.sourceforge.net/)

##### Mapping and Navigation
* [urg_node](http://wiki.ros.org/urg_node)
* [hector_mapping](http://wiki.ros.org/hector_mapping)
* [move_base](http://wiki.ros.org/move_base)
* [teb_local_planner](http://wiki.ros.org/teb_local_planner)

##### Object Recognition
* [kinect2_bridge](https://github.com/code-iai/iai_kinect2/tree/master/kinect2_bridge)
* [ros_markers](https://github.com/chili-epfl/ros_markers)

#### Grasping
* [Baxter Interface](http://api.rethinkrobotics.com/baxter_interface/html/index.html)

##### Face Recognition
* cv2 (see documentation linked below for installation guidance)
* [cv_bridge](http://wiki.ros.org/cv_bridge)

##### Speech Recogntion
* [CMU Sphinx](https://cmusphinx.github.io/wiki/tutorialpocketsphinx/)

##### Brain
* [smach](http://wiki.ros.org/smach)

## Installation

Project Fezzik is a catkin workspace. Instructions are provided here on how to install and set-up.

1. Firstly, ensure the above dependencies are installed. ROS packages can be installed directly using `apt-get install ros-<distro>-<package_name>`
where <distro> refers to the version of ROS e.g. Indigo. For most packages, the Kinetic versions have been installed and used successfully. kinect2_bridge
is a special exception which requires ROS Indigo to work.

    Python packages can be installed with `pip install <package_name>`.

2. To build the catkin workspace, run 'catkin_make' inside the project root folder, i.e. fezzik-project/.
3. Source the *setup.bash* file within *fezzik-project/devel* with `source setup.bash`. This will have to be sourced in every terminal window that is opened.

## How to Run

The first step is connecting to Baxter as all nodes will need to run on the Baxter core. The baxter.sh script in the root folder provides this functionality.
Before running (`./baxter.sh`), the `your_ip` variable will need to be changed to your computer's IP address.

Once on Baxter, start the Navigation stack on the computer selected for navigation - `roslaunch navigation navigation.launch`.

On the second computer, launch the main program by running `roslaunch brain brain.launch`.
This will initialise the robot and put him in the Idle state. When prompted, simply press 'Enter' on the keyboard to begin the fetch process.

DE NIRO will then proceed through the following states: listening, remembering face, navigating, grasping, returning, seeking user, offering object.

Of these states, the following require user interaction. Here, DE NIRO will...
1. Listening: Wait for a voice command from the user. These are usually in the form, 'fetch me...' or 'give me...'.
2. Remembering Face: Wait till a face is recognised then store the face for later on.
3. Seeking User: Seek to matchgi the face remembered with the person at the home location.
4. Offering Object: Hold onto an object until gently pulled by the user.

Upon completion, DE NIRO will enter into an idle state, ready to be activated again with a simple keyboard press.

### Note

To successfully build the code, it was required that several component packages could import features from other packages. This was achieved
by adding the path of these files to the PYTHONPATH. In many of the source files, you will something like following snippet of code.

```
root_path = "/home/petar/fezzik-project"
sys.path.append(os.path.join(root_path, "src/ros_comms"))
```

To run the project, the root path will need to be changed for your computers.

## Practical Caution

On rare occasions, the robot will choose an unconventional approach to grasping an object. This is a result of the inverse kinematics solver returning
a sub-optimal path to the object. When this happens, it is possible that DE NIRO's arms may collide with other parts or the environment.cd


