clean: 
	rm -rf install log build
sp-perception-pipeline:
	colcon build --symlink-install --packages-up-to perception_pipeline_launch --parallel-workers 6 --cmake-args -DCMAKE_CUDA_ARCHITECTURES=86
sp-criticallity-pipeline:
	colcon build --symlink-install --packages-up-to criticallity_pipeline_launch --parallel-workers 6 --cmake-args -DCMAKE_CUDA_ARCHITECTURES=86
sp-criticallity-pipeline-autoware:
	colcon build --symlink-install --packages-up-to criticallity_pipeline_launch stereo_perception_autoware_bridge --cmake-args -DCMAKE_CUDA_ARCHITECTURES=86