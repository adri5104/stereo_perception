# Docker image to run the criticallity pipeline with Autoware

This Docker image has the same purpose as the normal criticallity one, but with Autoware instead if the 
perception pipeline.

It uses the stereo_perception_autoware_bridge to convert the autoware predicted objects in clustered objects messages from 
stereo_perception_msgs and then run the ttc_calculator over them.