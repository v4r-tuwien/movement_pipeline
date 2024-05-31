Starting the Movement Pipeline
==============================

This section is going to explain how to start the HSR and how create a new map. Afterwards the implemented functions of the movement 
pipeline are going to be explained.

The movement pipeline currently consits of:

- RQT Reconfigure for the joint movement
- The statemachine for the general movement of the HSR


Starting the HSR
----------------

Before creating a map and running the movement pipeline, the robot needs to be started by turning on the power switch on the HSR and then 
pressing power button on. Afterwards the robot can be started by releasing the emergeny stop button.

You know that the HSR started properly when the robot says "Sasha start" and moves its head and arm to an initial position.

Afterwards you can connect to the robot via ssh using the following command:

.. code-block:: console

    $ ssh v4r@hsrb.local

Ask other team members for the password.

Creating and Saving a new Map
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The HSR uses waypoint inside the movement pipeline, which is why a map is needed. In case a map has already been created and the HSR is booting up 
correctly with this map, this step can be skipped.

.. important::

    When creating a new map, make sure the robot starts the map creation at the current map origin. Otherwise the map origin will shift and 
    implmeneted waypoints might not work correctly. 
    Additionally be aware that the robot is not able to detect obstacles when driving manually using RQT. 

The HSR uses SLAM to create a map. To start the SLAM, please refer to chapter 7.3 Maps of the `HSR Development Manual`_. If you do not have an HSR gitlab 
account, ask other team members for their help.

When saving a new map, the map should be saved in the following directory:

.. code-block:: text

    /etc/opt/tmc/robot/conf.d/maps

Please move the files `map.pgm` and `map.yaml` to a new directory inside of

.. code-block:: text

    /etc/opt/tmc/robot/conf.d/maps/old_maps/NEW_DIRECTORY

Use a fitting directory name in case someone needs to retrieve the old map. Afterwards add a changelog to

.. code-block:: text

    /etc/opt/tmc/robot/docker.hsrb.user

about the new map (this is also so that the next user knows which map is currently being used).

Restarting the HSR should now load the new map. This can be checked by starting RViz using

.. code-block:: console

    $ rv

inside the docker container, mentioned in the installation guide of the `grasping pipeline`_.

Using the new Map
~~~~~~~~~~~~~~~~~

When connected to the HSR via ssh, the robot can be aligned using:

.. code-block:: console

    v4r@hsrb $  startup

The robot should then start moving its head until it finds a marker on the wall and align the map accordingly.

.. note::

    In order for the alignment to work, the robot needs to have a marker somewhere around him. If new markers are needed, their positions needs to be added 
    to the python file

    .. code-block:: console
    
        v4r@hsrb $ ~/startup_ws/src/align_rviz/src/tf_broadcaster.py

    which is saved on the HSR. 
    
    If you need help retrieving the position of new markers, you can use the file

    .. code-block:: console

        v4r@hsrb $ ~/grasp_and_place/src/object_recognition/src/tf_calculator.py

    or ask other team members for help.


Starting the RQT Reconfigure
----------------------------

The RQT Reconfigure is used to control the joint movement of the HSR. To start the RQT Reconfigure, run the following command:

.. code-block:: console

    $ cd /path/to/your/catkin_ws/src/movement_pipeline/src
    $ python3 joint_movement.py

In the docker container you can now start the RQT Reconfigure by running:

.. code-block:: console

    $ rqt

Opening `Running` -> `Dynamic Reconfigure` you should now be able to change the joint positions of the HSR using the `joint_movement_node` tab.
Additional configurations can be added in 

.. code-block:: text

    /path/to/your/catkin_ws/src/movement_pipeline/cfg/JointMovement.cfg


Starting the Movement statemachine
----------------------------------

The statemachine can be started by running the following command:

.. code-block:: console

    $ roslaunch movement_pipeline movement_pipeline.launch

You should then be able to let the HSR drive around the office using inputs in the terminal.

.. _HSR Development Manual: https://www.hsr.io/
.. _grasping pipeline: https://github.com/v4r-tuwien/grasping_pipeline.git



