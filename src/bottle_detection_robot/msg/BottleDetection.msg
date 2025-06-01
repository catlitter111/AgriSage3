# msg/BottleDetection.msg
std_msgs/Header header
float32 distance  # 瓶子距离，单位：米
int32 x           # 瓶子中心x坐标
int32 y           # 瓶子中心y坐标
float32 score     # 检测分数