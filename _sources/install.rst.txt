Installation
============

The movement pipeline uses the `grasping pipeline`_ as a dependency. To install the grasping pipeline you can follow the instructions in the 
documentation of the repository.

After you have installed the grasping pipeline you can install the movement pipeline by using it as an additional ROS package. This is done by running the following commands:

.. code-block:: console

    $ cd /path/to/your/catkin_ws/src
    $ git clone git@github.com:v4r-tuwien/movement_pipeline.git

Don't forget to source and build your workspace afterwards.


.. _grasping pipeline: https://github.com/v4r-tuwien/grasping_pipeline.git
