# stm32-AHRS-EKF
stm32F4/AHRS/EKF
1.基于stm32F4实现的ekf算法，传感器mpu6050+HMC5883L，算上采样时间，解算速度约320~330Hz；
2.需要配置ARM针对单片机的数学优化库arm_math.h（用到的是矩阵运算部分MatrixFunctions）；
3.有待优化，一些中间矩阵和无效矩阵元素运算可以删去。