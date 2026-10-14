# 决策层发给机械臂的命令消息
float32[] joint_positions  # 目标关节角度 [θ1, θ2, θ3, θ4, θ5, grip]
string action              # 动作类型: "pick", "place", "home"
float32 target_x           # 目标位置X (机械臂基座坐标系, 米)
float32 target_y           # 目标位置Y
float32 target_z           # 目标位置Z
