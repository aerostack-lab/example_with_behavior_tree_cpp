
# Application: example_with_behavior_tree_cpp

This application uses the tool behavior_tree_cpp in order to formulate the behavior tree. The user can create the mission graphically and monitorize it using Groot.

In order to execute the mission, perform the following steps:

- Install behavior_tree_cpp and groot:

        $ ./install_from_source.sh
        
- In another console compile the project:

        $ cd $AEROSTACK_WORKSPACE
        $ catkin_make
        
- Execute the script that launches Gazebo for this project::

        $ ./launcher_gazebo.sh
        
- Wait until the following window is presented:
<img src="https://github.com/aerostack/airplane_inspection_gazebo/blob/master/doc/AirplaneInspectionFirstImage.png" width=600>

- Execute the script that launches the Aerostack components for this project:

        $ ./main_launcher.sh

As a result of this command, a set of windows are presented to monitor the execution of the mission. These windows include:
- Alphanumeric behavior viewer
- Belief viewers 
- Image view of the front cameras
- Groot

In order to monitorize the execution of the mission, press the button "monitor"and then "start"  (window groot) . The mission is previously stored as a behavior tree in configs/mission/mission.xml. The following window is presented:

<img src="https://i.ibb.co/CHFrDg5/groot.png" width=600>

Then look for the terminal of the below image and when it is presented press the button "connect" in Groot, if it fails press again the button.

<img src="https://i.ibb.co/zHP0QL8/Captura-de-pantalla-de-2021-06-23-12-20-52.png" width=600>

In groot you should visualize the following window:

<img src="https://i.ibb.co/sFGqhT8/Captura-de-pantalla-de-2021-06-23-12-20-59.png" width=600>


If you want to edit the mission you have to complete these steps:
- Open Groot editor (press editor and start):

        $ ./Groot/build/Groot
        
- Press "Load Tree" button and load the mission in "example_with_behavior_tree_cpp/configs/mission).
- Drag the nodes into the blackboard and connect them.
- In order to save the tree press the button "Save Tree" and save it with the same name and in the same location as the previous mission.
        
