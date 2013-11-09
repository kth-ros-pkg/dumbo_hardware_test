/*This code shows the joint limits of the schunk dual arm setup and the end effector
position limits to avoid collisions and self-collisions. */

#define MIN_X	0
#define MAX_X	1
#define MIN_Y	2
#define MAX_Y	3
#define MIN_Z	4
#define MAX_Z	5

//joint limits for SCHUNK arms. [0] is for left arm, [1] is for right arm
const struct angle_struct joint_limits_min[2] = {{{0,0,-23,0,0,0,0}},{{0,0,-24,0,0,0,0}}}; 
const struct angle_struct joint_limits_max[2] = {{{0,0,0,0,0,0,0}},{{0,0,0,0,0,0,0}}};

//internal joint limits for Schunk arms. Same convention as above. These limits are sent to each of the modules of the arms
const struct angle_struct joint_internal_limits_min[2] = {{{0,0,0,0,0,0,0}},{{0,0,0,0,0,0,0}}};
const struct angle_struct joint_internal_limits_max[2] = {{{0,0,0,0,0,0,0}},{{0,0,0,0,0,0,0}}};

//end effector position limits
//[i][j]: i for selecting the arm (0 = LEFT ARM, 1 = RIGHT ARM) , j index indicates the limit ([min_x max_x min_y max_y min_z max_z])
const float ee_position_limits[2][6]={{0,0,0,0,0,0},{0,0,0,0,0,0}};
