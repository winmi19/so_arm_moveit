# 视觉检测结果消息
string object_class      # 物体类别: "bottle", "can", "box"
string color             # 颜色: "red", "blue", "green"
float32 x_pixel          # 检测框中心像素X坐标
float32 y_pixel          # 检测框中心像素Y坐标
float32 depth            # 深度值 (米), 无深度相机时为0
float32 confidence       # 检测置信度 [0.0, 1.0]
