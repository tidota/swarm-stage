# Desc: Device definitions for footbot. This comes from pioneer.inc
# Author: Tetsuya Idota
# Date: 15 Mar 2016

# http://www.swarmanoid.org/swarmanoid_hardware.php.html
# http://argos-sim.info/plow2015/

# manual: http://player-stage-manual.readthedocs.org/en/stable/WORLDFILES/

# The sonar array

#  prototype for proximity sensors
define proxsens sensor
(
  # define the size of each transducer [xsize ysize zsize] in meters
  size [0.01 0.01 0.01 ] 
  # define the range bounds [min max]
  range [0 0.1]
  # define the angular field of view in degrees
  fov 10
  # define the number of samples spread over the fov
  samples 1
  # define the color that ranges are drawn in the gui
  color_rgba [0 1 0 0.5]
)

define proxsens_list ranger
(
 color "blue"
 # 24 sensors
 proxsens( pose [0.085 0.000 0 0] )
 proxsens( pose [0.082 0.022 0 15] )
 proxsens( pose [0.074 0.043 0 30] )
 proxsens( pose [0.060 0.060 0 45] )
 proxsens( pose [0.043 0.074 0 60] )
 proxsens( pose [0.022 0.082 0 75] )
 proxsens( pose [0.000 0.085 0 90] )
 proxsens( pose [-0.022 0.082 0 105] )
 proxsens( pose [-0.043 0.074 0 120] )
 proxsens( pose [-0.060 0.060 0 135] )
 proxsens( pose [-0.074 0.043 0 150] )
 proxsens( pose [-0.082 0.022 0 165] )
 proxsens( pose [-0.085 0.000 0 180] )
 proxsens( pose [-0.082 -0.022 0 -165] )
 proxsens( pose [-0.074 -0.043 0 -150] )
 proxsens( pose [-0.060 -0.060 0 -135] )
 proxsens( pose [-0.043 -0.074 0 -120] )
 proxsens( pose [-0.022 -0.082 0 -105] )
 proxsens( pose [0.000 -0.085 0 -90] )
 proxsens( pose [0.022 -0.082 0 -75] )
 proxsens( pose [0.043 -0.074 0 -60] )
 proxsens( pose [0.060 -0.060 0 -45] )
 proxsens( pose [0.074 -0.043 0 -30] )
 proxsens( pose [0.08 -0.02 0 -15] )
)

define omnicam fiducial
(
  # fiducial properties
  range_min 0.0
  range_max 1.2
  range_max_id 1.2
  fov 360 #2*3.14159
  ignore_zloc 0

  # model properties
  size [ 1 1 1 ]

)

define marker position
(
  name "marker"
  color "black"
  size [0.02 0.02 0.14]
  block(
    points 4
    point[0] [0.005 0.005]
    point[1] [-0.005 0.005]
    point[2] [-0.005 -0.005]
    point[3] [0.005 -0.005]
    z [0 0.14]
  )
  fiducial_return 0             # Seen as "1" fiducial finders

  omnicam()
)

define footbot_no_sens position 
(
  color "blue"      # Default color.
  drive "diff"       # Differential steering model.
  gui_nose 1                    # Draw a nose on the robot so we can see which way it points
  obstacle_return 1             # Can hit things.
  ranger_return 1              # reflects sonar beams
  blob_return 1                 # Seen by blobfinders  
  fiducial_return 0             # Seen as "1" fiducial finders

  localization "gps"             
  localization_origin [0 0 0 0]   # Start odometry at (0, 0, 0).

  # alternative odometric localization with simple error model
  # localization "odom"             # Change to "gps" to have impossibly perfect, global odometry
  #  odom_error [ 0.05 0.05 0.1 ]     # Odometry error or slip in X, Y and Theta
                                     # (Uniform random distribution)   

  # four DOF kinematics limits
  # [ xmin xmax ymin ymax zmin zmax amin amax ]        
  velocity_bounds [-0.5 0.5 0 0 0 0 -90.0 90.0 ]          
  acceleration_bounds [-0.5 0.5 0 0 0 0 -90 90.0 ]

  # actual size
  # height: 0.15 = 0.29 - 0.14(the vertical bar's height)
  size [0.17 0.17 0.15] # sizes from http://www.swarmanoid.org/swarmanoid_hardware.php.html
  

  # the center of rotation
  origin [0 0 0 0]

  # draw a nose on the robot so we can see which way it points
  gui_nose 1

  # estimated mass in KG
  mass 1.8 
  # differential steering model
  drive "diff"

  # simplified Body shape:
  block(
    points 24
    point[0] [0.085 0.000]
    point[1] [0.082 0.022]
    point[2] [0.074 0.043]
    point[3] [0.060 0.060]
    point[4] [0.043 0.074]
    point[5] [0.022 0.082]
    point[6] [0.000 0.085]
    point[7] [-0.022 0.082]
    point[8] [-0.043 0.074]
    point[9] [-0.060 0.060]
    point[10] [-0.074 0.043]
    point[11] [-0.082 0.022]
    point[12] [-0.085 0.000]
    point[13] [-0.082 -0.022]
    point[14] [-0.074 -0.043]
    point[15] [-0.060 -0.060]
    point[16] [-0.043 -0.074]
    point[17] [-0.022 -0.082]
    point[18] [0.000 -0.085]
    point[19] [0.022 -0.082]
    point[20] [0.043 -0.074]
    point[21] [0.060 -0.060]
    point[22] [0.074 -0.043]
    point[23] [0.082 -0.022]
    z [0 0.15]
  )

  marker()
)

define footbot footbot_no_sens
(
  proxsens_list( pose [0 0 -0.1 0] ) 
)

