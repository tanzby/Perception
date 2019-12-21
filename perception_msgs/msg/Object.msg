int8                         id                 # 每一帧object ID
int8                         type 		        # 分类
jsk_recognition_msgs/Rect    rect 		        # 图片BOX
geometry_msgs/Pose           pose
geometry_msgs/Vector3        dimensions
geometry_msgs/PolygonStamped convex_hull
float32                      confidence
int8                         track_id 	        # 无跟踪为0
float32                      velocity 	        # 合速度
geometry_msgs/Quaternion     velocity_direction # 速度方向
int8                         motion_state 	    # 运动状态
geometry_msgs/Vector3[]      trajectory 	    # 轨迹

#-type----------------------------
int8 TYPE_UNKNOWN = 0
int8 TYPE_PEDESTRIAN = 1
int8 TYPE_MOTOR = 2
int8 TYPE_CAR = 3
int8 TYPE_TRUCK = 4
int8 TYPE_EXCAVATOR = 5

#-motion--------------------------
int8 MOTION_UNKNOWN = 0
int8 MOTION_MOVING = 1
int8 MOTION_STATIONARY = 2

