uint32                          id
string                          label

geometry_msgs/Pose              pose
geometry_msgs/Twist             velocity
geometry_msgs/Twist             acceleration
geometry_msgs/Vector3           dimensions
geometry_msgs/PolygonStamped    convex_hull

bool                            pose_reliable
bool                            velocity_reliable

int8 behavior_state