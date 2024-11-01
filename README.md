# BalanceCar
 基于freeRTOS的stm32平衡小车

 主控使用stm32f103c8t6
 实现了两个有刷电机、MPU6050、四个按钮、流水灯、oled显示器的驱动
 移植了MPU6050 DMP解算的移植，可以解算出yaw、pitch、row数据，写了PID算法
 项目进行到PID调参，小车可以根据倾斜角度来调节两个有刷电机的速度调控，平衡效果暂未实现
