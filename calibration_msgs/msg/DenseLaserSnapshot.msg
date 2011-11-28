# Provides all the state & sensor information for
# a single sweep of laser attached to some mechanism.
# Most likely, this will be used with PR2's tilting laser mechanism
Header header

# Store the laser intrinsics. This is very similar to the
# intrinsics commonly stored in 
float32 angle_min        # start angle of the scan [rad]
float32 angle_max        # end angle of the scan [rad]
float32 angle_increment  # angular distance between measurements [rad]
float32 time_increment   # time between measurements [seconds]
float32 range_min        # minimum range value [m]
float32 range_max        # maximum range value [m]

# Define the size of the binary data
uint32 readings_per_scan    # (Width)
uint32 num_scans            # (Height)

# 2D Arrays storing laser data.
# We can think of each type data as being a single channel image.
# Each row of the image has data from a single scan, and scans are
# concatenated to form the entire 'image'.
float32[] ranges            # (Image data)
float32[] intensities       # (Image data)

# Store the start time of each scan
time[] scan_start
