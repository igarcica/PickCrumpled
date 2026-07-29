Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
Author Irene Garcia-Camacho (igarcia@iri.upc.edu).

# Demo Pick Crumpled

This package is used to perform a grasp of a cloth, either sending it to a given position (cfg) or given by a camera (highest point of the cloth (depth-based) or with Carlos' model).

## Getting started

Packages necessary for the demo:

- pick_crumpled: Contains the state machine to perform the grasp action.
- vision_pick_place: Contains all the necessary code related to perception (Segmentation, corner detection, grasp point selection, pile height, etc)
- iri_kinova_linear_movement: For execution cartesian movements with Kinova.


# Execution 

First launch the camera node and robot driver, in this example the rs camera and kinova robot:
Launch the camera and the kortex driver:

``roslaunch pick_n_place camera_n_kinova.launch``

Launch the nodes corresponding to the demo (iri_kinova_linear_movement, pick_n_place, vision_pick_place, prediction module and state estimation module):

``roslaunch pick_crumpled pickcrumpled_demo.launch``

This will also launch the RVIZ to visualize the perception system and rqt reconfigure to control the demo, which includes the following variables:

<!-- The rqt_reconfigure includes the following variables: -->

- ***Start SM:***
  - **get_grasp_point**: Confirm the grasp point selected (pink point in RVIZ). 
  - **start_demo**: Starts the state machine.
  - **start_experiments**: Starts the state machine from the placing state to obtain data.
  - **stop**: Stops the state machine.
  - **ok**: Continues with the placing execution after checking the deformation.
  - **diagonal_place**: This will place the object with a diagonal movement.
  - **vertical_place**: This will place the object with a vertical movement.
  - **rotating_place**: This will place the obejct first rotating the gripper 90º.
  - **dynamic_place**: -In progress-
- ***Configuration parameters***:
  - **handeye**: XYZ and RPY offsets for handeye transformation between camera and kinova base.
- ***Test pose parameters:***
  - **test**: Starts the state machine from initial state but for grasping the given position.
  - **frame_id**: Reference frame of the given position.
  - **grasp**: Grasping target pose for testing.

To grasp the heighest point of the cloth with a top-down grasp: 
1. Adjust the handeye parameters according to camera's position wrt base robot.
2. Check that the desired grasp point is being detectes (blue point in the RVIZ).
3. Start the execution pressing **start_demo**.

This approach follows a finite state machine (FSM) pipeline to execute top-down grasp of the detected highest point. 


