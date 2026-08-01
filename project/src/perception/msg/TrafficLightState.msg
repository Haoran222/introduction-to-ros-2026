# Result of the camera-based traffic light recognition (see traffic_light_detector_node).

uint8 UNKNOWN=0
uint8 RED=1
uint8 YELLOW=2
uint8 GREEN=3

std_msgs/Header header

uint8 state            # one of UNKNOWN/RED/YELLOW/GREEN above
bool detected           # true if a traffic light blob was found in the current frame
float32 confidence      # 0..1, fraction of the candidate blob matching the classified color
int32 pixel_area        # size in pixels of the classified color blob, for debugging/tuning
