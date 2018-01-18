# ARbench Part Publisher
This is a ROS node for publishing feature frames annotated using ARBench. It exposes a set of services:

1. `remove_frame` - Call by name of the frame
2. `get_frames` - get a list of frame names
3. `add_frame` - add frame using TFMessage
4. `add_frame_xyzq` - add frame using origin and quaternion

All published frames are prefixed with the name of the part. 
