Tutorial: Perception
====================

Head Camera Topics
------------------

The API for the head camera is documented under :ref:`camera_api`.

Some resources for accessing and processing camera data are:

 * `OpenCV <http://opencv.org/>`_ is a generic computer vision library
   which has good support within ROS.
 * `Point Cloud Library <http://pointclouds.org/>`_ allows manipulation
   of 3-dimensional images, or point clouds.
 * `cv_bridge <http://wiki.ros.org/cv_bridge>`_ is a ROS package that allows
   converting ROS image messages into OpenCV data structures in C++ or Python.
 * `pcl_conversions <http://wiki.ros.org/pcl_conversions>`_ is a ROS
   package for converting between ROS PointCloud2 messages and PCL data
   types in C++.
 * `pcl_ros <http://wiki.ros.org/pcl_ros>`_ is a ROS package that contains
   several nodelets for commonly used PCL components such as voxel grid
   filters for downsampling a point cloud or pass through filters for
   filtering out data beyond a certain distance.

Real Time Octomap
-----------------

Launch the moveit_config with the parameter ``allow_active_sensing`` changed to ture.
:: 

  roslaunch fetch_moveit_config move_group.launch allow_active_sensing:=true


The command will change the topic ``point_cloud_topic`` specified in ``fetch_ros/fetch_moveit_config/config/sensors.yaml``
to octomap in the planning scene. The planner will avoid planning in to the objects show in octomap.
If you would like to change the topic of the octomap. Change the ``point_cloud_topic`` in the ``sensors.yaml``
to the topic you want. 

Octomap Example
~~~~~~~~~~~~~~~

:: 

  roslaunch fetch_gazebo pickplace_playground.launch 

Lunch the Gazebo simulaion environment.

::

  roslaunch fetch_moveit_config move_group.launch allow_active_sensing:=true

Launch the moveit_config with allow_active_sensing on.

::

  rviz

Launch rviz and add the PlanningScene

::

  rosrun fetch_demo point_head

The the rviz should show a similar scene with the video.

.. raw:: html

    <div style="position: relative; padding-bottom: 5%; height: 0; overflow: hidden; max-width: 100%; height: auto;">
        <iframe width="560" height="315" src="https://www.youtube.com/embed/hxM9Ww_lpJU" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
    </div>
    
Running the Pick and Place Demo
-------------------------------

See :ref:`mm_demo`.
