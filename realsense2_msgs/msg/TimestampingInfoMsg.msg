std_msgs/Header   header

# Device information.
string            device_model                            # Device model, e.g. D435, D415 camera.
string            device_name                             # Device name, e.g. 'depth_camera_front'
string[]          active_streams                          # Active stream IDs, e.g. 'color', 'infra1', 'infra2', 'rgb', etc.

# Frame data.
FrameMetadataMsg  frame_metadata                          # Metadata from current frame.

# Time offsets.
TimeOffsetsMsg    time_offsets                            # Estimated time offsets for current frame.

# Timestamp estimation.
string            timestamping_method                     # Name of the timestamp estimation method employed, e.g. baseline, fixed_offset, varying_offsets, etc.
float64           fixed_offset_value                      # Fixed timestamp correction value used whenever Kernel Patch can't be installed..