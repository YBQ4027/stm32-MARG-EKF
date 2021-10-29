#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "key.h"
#include "mpu6050.h"
#include "arm_math.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "hmc.h"
#include "math.h"
#include "arm_math.h"
#define Gyro_Gain 0.0609756f		//角速度转换为角度(LSBg = 2000*2 / 65535)
#define Gyro_Gr 0.0010641f			//角速度转换成弧度(3.1415 / 180 * LSBg)
#define G 9.80665f

#define Ki 0.005f
#define Kp 3.5f
#define halfT 0.00153f  //半个采样周期时长，单位s


//串口1发送1个字符 
//c:要发送的字符
void usart1_send_char(u8 c)
{

	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); 
    USART_SendData(USART1,c);   

} 

//传送数据给匿名四轴上位机软件(V4版本)
//fun:功能字
//len:data区有效数据个数
void usart1_niming_report(u8 fun,u8*data,u8 len)
{
	u8 send_buf[32]={0x00};
	u8 i;
	if(len>28)return;	//最多28字节数据 
	send_buf[len+4]=0;	//校验数置零
	send_buf[0]=0XAA;	//帧头
	send_buf[1]=0XAA;	//帧头
	send_buf[2]=fun;	//功能字
	send_buf[3]=len;	//数据长度
	for(i=0;i<len;i++)send_buf[4+i]=data[i];			//复制数据
	for(i=0;i<len+4;i++)send_buf[len+4]+=send_buf[i];	//计算校验和	
	for(i=0;i<len+5;i++)usart1_send_char(send_buf[i]);	//发送数据到串口1 
}


//以自定义帧(功能码F1)发送加速度传感器数据和陀螺仪数据
//aacx,aacy,aacz:x,y,z三个方向上面的加速度值
//gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
void mpu6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz)
{
	u8 tbuf[12]={0x00}; 
	tbuf[0]=(aacx>>8)&0XFF;
	tbuf[1]=aacx&0XFF;
	tbuf[2]=(aacy>>8)&0XFF;
	tbuf[3]=aacy&0XFF;
	tbuf[4]=(aacz>>8)&0XFF;
	tbuf[5]=aacz&0XFF; 
	tbuf[6]=(gyrox>>8)&0XFF;
	tbuf[7]=gyrox&0XFF;
	tbuf[8]=(gyroy>>8)&0XFF;
	tbuf[9]=gyroy&0XFF;
	tbuf[10]=(gyroz>>8)&0XFF;
	tbuf[11]=gyroz&0XFF;
	usart1_niming_report(0XF1,tbuf,12);
}	
//通过串口1上报结算后的姿态数据给电脑
//格式：AAAA 01 int16 ROL*100 int16 PIT*100 int16 YAW*100 int16 ALT_CSB(超声波高度,单位厘米) int32 ALT_PRS(气压计高度,单位毫米）
//只发送姿态角，其它置0
//roll:横滚角.单位0.01度。 -18000 -> 18000 对应 -180.00  ->  180.00度
//pitch:俯仰角.单位 0.01度。-9000 - 9000 对应 -90.00 -> 90.00 度
//yaw:航向角.单位为0.1度 0 -> 3600  对应 0 -> 360.0度
void usart1_report_Angle(short roll,short pitch,short yaw)
{
	u8 tbuf[12]={0x00}; 
	u8 i;
	for(i=0;i<12;i++)tbuf[i]=0;//清0
	tbuf[0]=(roll>>8)&0XFF;
	tbuf[1]=roll&0XFF;
	tbuf[2]=(pitch>>8)&0XFF;
	tbuf[3]=pitch&0XFF;
	tbuf[4]=(yaw>>8)&0XFF;
	tbuf[5]=yaw&0XFF;
	usart1_niming_report(0X01,tbuf,12);//姿态角显示帧,0X01
} 

//以传感器显示帧(功能码02)发送加速度传感器数据,陀螺仪数据和磁力计数据
void usart1_report_senserdata(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,
	short magx,short magy,short magz)
{
	u8 tbuf[18]={0x00}; 
	tbuf[0]=(aacx>>8)&0XFF;
	tbuf[1]=aacx&0XFF;
	tbuf[2]=(aacy>>8)&0XFF;
	tbuf[3]=aacy&0XFF;
	tbuf[4]=(aacz>>8)&0XFF;
	tbuf[5]=aacz&0XFF; 
	tbuf[6]=(gyrox>>8)&0XFF;
	tbuf[7]=gyrox&0XFF;
	tbuf[8]=(gyroy>>8)&0XFF;
	tbuf[9]=gyroy&0XFF;
	tbuf[10]=(gyroz>>8)&0XFF;
	tbuf[11]=gyroz&0XFF;
        tbuf[12]=(magx>>8)&0XFF;
	tbuf[13]=magx&0XFF;
	tbuf[14]=(magy>>8)&0XFF;
	tbuf[15]=magy&0XFF;
	tbuf[16]=(magz>>8)&0XFF;
	tbuf[17]=magz&0XFF;
	usart1_niming_report(0X02,tbuf,18);
}	

//陀螺仪、加速度计、磁力计数据融合出姿态四元数
float q0=1.0f, q1=0.0f, q2=0.0f, q3=0.0f;//四元数

static float32_t P[49]={0,0,0,0,0,0,0,       
	                0,0,0,0,0,0,0,
	                0,0,0,0,0,0,0,
                        0,0,0,0,0,0,0,
                        0,0,0,0,0,0,0,
                        0,0,0,0,0,0,0,
                        0,0,0,0,0,0,0};  //先验协方差矩阵
static float32_t PP[49]={0,0,0,0,0,0,0,
	                 0,0,0,0,0,0,0,
	                 0,0,0,0,0,0,0,
                         0,0,0,0,0,0,0,
                         0,0,0,0,0,0,0,
                         0,0,0,0,0,0,0,
                         0,0,0,0,0,0,0};  //中间矩阵 GkP
static float32_t R[36]={1,0,0,0,0,0,
	                0,1,0,0,0,0,
	                0,0,1,0,0,0,
                        0,0,0,1,0,0,
                        0,0,0,0,1,0,
                        0,0,0,0,0,1};    //测量噪声协方差矩阵
static float32_t Q[49]={0.0001,0,0,0,0,0,0,
	                0,0.0001,0,0,0,0,0,
	                0,0,0.0001,0,0,0,0,
                        0,0,0,0.0001,0,0,0,
                        0,0,0,0,0.0001,0,0,
                        0,0,0,0,0,0.0001,0,
                        0,0,0,0,0,0,0.0001};  //过程噪声协方差矩阵
static float32_t K_k[42]={0,0,0,0,0,0,
	                  0,0,0,0,0,0,
	                  0,0,0,0,0,0,
                          0,0,0,0,0,0,
                          0,0,0,0,0,0,
                          0,0,0,0,0,0,
                          0,0,0,0,0,0};    //卡尔曼增益
static float32_t PH[42]={0,0,0,0,0,0,
	                 0,0,0,0,0,0,
	                 0,0,0,0,0,0,
                         0,0,0,0,0,0,
                         0,0,0,0,0,0,
	                 0,0,0,0,0,0,
                         0,0,0,0,0,0};     //中间矩阵 PHkt
static float32_t HP[42]={0,0,0,0,0,0,0,
	                 0,0,0,0,0,0,0,
	                 0,0,0,0,0,0,0,
                         0,0,0,0,0,0,0,
                         0,0,0,0,0,0,0,
                         0,0,0,0,0,0,0};   //中间矩阵 HP
static float32_t HPH[36]={0,0,0,0,0,0,
	                  0,0,0,0,0,0,
	                  0,0,0,0,0,0,
                      	  0,0,0,0,0,0,
                       	  0,0,0,0,0,0,
	                  0,0,0,0,0,0};    //中间矩阵 HPH
static float32_t HPH_[36]={0,0,0,0,0,0,  
	                   0,0,0,0,0,0,
	                   0,0,0,0,0,0,
                           0,0,0,0,0,0,
                           0,0,0,0,0,0,
	                   0,0,0,0,0,0};    //中间矩阵 HPH的逆
static float32_t I[49]={1,0,0,0,0,0,0,
	                0,1,0,0,0,0,0,
	                0,0,1,0,0,0,0,
                        0,0,0,1,0,0,0,
                        0,0,0,0,1,0,0,
                        0,0,0,0,0,1,0,
                        0,0,0,0,0,0,1};  //单位矩阵
static float32_t KH[49]={0,0,0,0,0,0,0,
	                 0,0,0,0,0,0,0,
	                 0,0,0,0,0,0,0,
                         0,0,0,0,0,0,0,
                         0,0,0,0,0,0,0,
                         0,0,0,0,0,0,0,
                         0,0,0,0,0,0,0};  //中间矩阵 KH
static float32_t GK_t[49]={0,0,0,0,0,0,0,
	                   0,0,0,0,0,0,0,
	                   0,0,0,0,0,0,0,
                           0,0,0,0,0,0,0,
                           0,0,0,0,0,0,0,
                           0,0,0,0,0,0,0,
                           0,0,0,0,0,0,0};   //状态转移矩阵Gk的转置
static float32_t HK_t[42]={0,0,0,0,0,0,0,
	                   0,0,0,0,0,0,0,
	                   0,0,0,0,0,0,0,
                           0,0,0,0,0,0,0,
                           0,0,0,0,0,0,0,
                           0,0,0,0,0,0,0};   //测量方程雅可比矩阵Hk的转置
static float32_t KK_k[7]={0,0,0,0,0,0,0}; //先验误差 

//定义指向矩阵的指针（points to an instance of the floating-point matrix structure.）
static arm_matrix_instance_f32 p;//P
static arm_matrix_instance_f32 pp;//PP
static arm_matrix_instance_f32 r;//R
static arm_matrix_instance_f32 q;//Q
static arm_matrix_instance_f32 k_k;//K_k
static arm_matrix_instance_f32 ph;//PH
static arm_matrix_instance_f32 hp;//HP
static arm_matrix_instance_f32 hph;//HPH
static arm_matrix_instance_f32 hph_;//HPH_
static arm_matrix_instance_f32 Ii;//I
static arm_matrix_instance_f32 kh;//KH
static arm_matrix_instance_f32 g_kt;//GK_t
static arm_matrix_instance_f32 h_kt;//HK_t
static arm_matrix_instance_f32 kk_k;//KK_k

static arm_matrix_instance_f32 g_k;//G_k
static arm_matrix_instance_f32 h_k;//H_k
static arm_matrix_instance_f32 hh_k;//HH_k
static arm_matrix_instance_f32 z_k;//Z_k
static arm_matrix_instance_f32 x_k;//X_k

static float gx=0,gy=0,gz=0;
 void usart1_report_ekfAngle(short aax,	 short aay, short aaz, short ggx, short ggy, short ggz,
	 short mmx, short mmy, short mmz,short roll,short pitch,short yaw,u8 key,short t)
{
	u8 tbuf[18]={0x00}; 
	u8 i;
	static float yaw0=0;
	float norm;
	float ax,ay,az,mx,my,mz;
	double roll2,pitch2,yaw2;
        float hx, hy, hz, bx, bz;
	float Mx,My,Mz,wx,wy,wz; 
	//v*当前姿态计算得来的重力在机载坐标下三轴上的分量
        //先计算好相关乘积项为了后面做矩阵运算做准备
	float q0q0=q0*q0;
	float q0q1=q0*q1;
	float q0q2=q0*q2;
	float q0q3=q0*q3;
	float q1q1=q1*q1;
	float q1q2=q1*q2;
	float q1q3=q1*q3;
	float q2q2=q2*q2;
	float q2q3=q2*q3;
	float q3q3=q3*q3;


        ax=(float)aax;
	ay=(float)aay;
	az=(float)aaz;
			
	//将陀螺仪AD值转换为 弧度/s
	wx=(float)ggx*Gyro_Gr;
	wy=(float)ggy*Gyro_Gr;
	wz=(float)ggz*Gyro_Gr;

	mx=(float)mmx;
	my=(float)mmy;
	mz=(float)mmz;
	
	//预测
	//一阶龙格库塔法更新四元数
	q0=q0+(-q1*(wx-gx)-q2*(wy-gy)-q3*(wz-gz))*halfT;
        q1=q1+(q0*(wx-gx)+q2*(wz-gz)-q3*(wy-gy))*halfT;
    	q2=q2+(q0*(wy-gy)-q1*(wz-gz)+q3*(wx-gx))*halfT;
    	q3=q3+(q0*(wz-gz)+q1*(wy-gy)-q2*(wx-gx))*halfT; 
	norm=sqrt(q0*q0+q1*q1+q2*q2+q3*q3);
	q0=q0/norm;
	q1=q1/norm;
        q2=q2/norm;
	q3=q3/norm;	
    //加速度计和地磁计向量标准化
    norm=sqrt(ax*ax+ay*ay+az*az); 
    ax=ax/norm;
    ay=ay/norm;
    az=az/norm; 
    norm=sqrt(mx*mx+my*my+mz*mz);
    mx=mx/norm;
    my=my/norm;
    mz=mz/norm;	
    //计算得到地磁计在理论地磁坐标系下的机体上三个轴的分量  
    hx=2*mx*(0.5f-q2q2-q3q3)+2*my*(q1q2-q0q3)+2*mz*(q1q3+q0q2);
    hy=2*mx*(q1q2+q0q3)+2*my*(0.5f-q1q1-q3q3)+2*mz*(q2q3-q0q1);
    hz=2*mx*(q1q3-q0q2)+2*my*(q2q3+q0q1)+2*mz*(0.5f-q1q1-q2q2);
    //当罗盘水平旋转的时候，航向角在0-360之间变化
    bx=sqrt((hx*hx)+(hy*hy));
    bz=hz; 
    //地磁计在n系下磁向量转换到b系下，反向使用DCM得到
    Mx=2*bx*(0.5f-q2q2-q3q3)+2*bz*(q1q3-q0q2);
    My=2*bx*(q1q2-q0q3)+2*bz*(q0q1+q2q3);
    Mz=2*bx*(q0q2+q1q3)+2*bz*(0.5f-q1q1-q2q2);
    norm=sqrt(Mx*Mx+My*My+Mz*Mz);
    Mx=Mx/norm;
    My=My/norm;
    Mz=Mz/norm;	
	
    float32_t G_k[49]={1,-halfT*(wx-gx),-halfT*(wy-gy),-halfT*(wz-gz),halfT*q1,halfT*q2,halfT*q3,
	               halfT*(wx-gx),1,halfT*(wz-gz),-halfT*(wy-gy),-halfT*q0,halfT*q2,-halfT*q3,
	               halfT*(wy-gy),-halfT*(wz-gz),1,halfT*(wx-gx),-halfT*q3,-halfT*q0,halfT*q1,
                   halfT*(wz-gz),halfT*(wy-gy),-halfT*(wx-gx),1,halfT*q2,-halfT*q1,-halfT*q0,
                   0,0,0,0,1,0,0,
                   0,0,0,0,0,1,0,
                   0,0,0,0,0,0,1};	 //状态转移矩阵Gk
    float32_t H_k[42]={-2*q2,2*q3,-2*q0,2*q1,0,0,0,
	               2*q1,2*q0,2*q3,2*q2,0,0,0,
	               2*q0,-2*q1,-2*q2,2*q3,0,0,0,
                   2*(bx*q0-bz*q2),2*(bx*q1+bz*q3),-2*(bx*q2+bz*q0),2*(-bx*q3+bz*q1),0,0,0,
                   2*(-bx*q3+bz*q1),2*(bx*q2+bz*q0),2*(bx*q1+bz*q3),2*(-bx*q0+bz*q2),0,0,0,
                   2*(bx*q2+bz*q0),2*(bx*q3-bz*q1),2*(bx*q0-bz*q2),2*(bx*q1+bz*q3),0,0,0};  //测量方程雅可比矩阵Hk


    float32_t HH_k[6]={(2*q1*q3-2*q0*q2),(2*q2*q3+2*q0*q1),(q0q0+q3q3-q1*q1-q2*q2),Mx,My,Mz};//	测量值参考
    float32_t Z_k[6]={ax,ay,az,mx,my,mz};//测量值（经过归一化）
    float32_t X_k[7]={q0,q1,q2,q3,gx,gy,gz};//	后验估计

	//矩阵初始化
    arm_mat_init_f32(&g_k,7,7,(float32_t *)G_k);
    arm_mat_init_f32(&h_k,6,7,(float32_t *)H_k);
    arm_mat_init_f32(&hh_k,6,1,(float32_t *)HH_k);
    arm_mat_init_f32(&z_k,6,1,(float32_t *)Z_k);
    arm_mat_init_f32(&x_k,7,1,(float32_t *)X_k);

	//先验协方差矩阵计算
    arm_mat_trans_f32(&g_k,&g_kt);
    arm_mat_trans_f32(&h_k,&h_kt);	

	arm_mat_mult_f32(&g_k,&p,&pp);
	arm_mat_mult_f32(&pp,&g_kt,&p);				   
	arm_mat_add_f32(&p,&q,&p);	

	//更新
	//卡尔曼增益计算
	arm_mat_mult_f32(&p,&h_kt,&ph);	//7*6
	arm_mat_mult_f32(&h_k,&p,&hp);//6*7
	arm_mat_mult_f32(&hp,&h_kt,&hph);//6*6
	arm_mat_add_f32(&hph,&r,&hph);
	arm_mat_inverse_f32(&hph,&hph_);	
	arm_mat_mult_f32(&ph,&hph_,&k_k);//7*6				   
	//后验估计计算				   
	arm_mat_sub_f32(&z_k,&hh_k,&z_k);// 6*1				   
	arm_mat_mult_f32(&k_k,&z_k,&kk_k);//7*1	
	arm_mat_add_f32(&x_k,&kk_k,&x_k);
	//后验协方差矩阵计算	
	arm_mat_mult_f32(&k_k,&h_k,&kh);//7*7
	arm_mat_sub_f32(&Ii,&kh,&kh);
	arm_mat_mult_f32(&kh,&p,&pp);//7*7
	p=pp;
	float32_t *pIn1= x_k.pData;
	q0=pIn1[0];
	q1=pIn1[1];
	q2=pIn1[2];
	q3=pIn1[3];
	gx=pIn1[4];
	gy=pIn1[5];
	gz=pIn1[6];

	//归一化四元数
	norm=sqrt(q0*q0+q1*q1+q2*q2+q3*q3);
	q0=q0/norm;
	q1=q1/norm;
        q2=q2/norm;
	q3=q3/norm;	
	//计算得到俯仰角/横滚角/航向角
	pitch2=asin(-2*q1*q3+2*q0*q2)*57.3*100;// pitch
	roll2=-atan2(2*q2*q3+2*q0*q1,-2*q1*q1-2*q2*q2+1)*57.3*100;// roll
	yaw2=-((atan2(2*(q1*q2+q0*q3),q0*q0+q1*q1-q2*q2-q3*q3)*57.3)*100);//yaw
	
	//KEY1键按下时将偏航角置0
	if(key==KEY1_PRES)
	{
		yaw0=yaw2;
	}
	yaw2=yaw2-yaw0;
	
	//发送姿态角(ekf解算)
	usart1_report_Angle((int)(roll2),(int)(pitch2),(int)(yaw2)); 
	
	
	for(i=0;i<18;i++)tbuf[i]=0;//清0
	tbuf[0]=((short)roll2>>8)&0XFF;
	tbuf[1]=(short)roll2&0XFF;
	tbuf[2]=((short)pitch2>>8)&0XFF;
	tbuf[3]=(short)pitch2&0XFF;
	tbuf[4]=((short)yaw2>>8)&0XFF;
	tbuf[5]=(short)yaw2&0XFF;
	tbuf[6]=((short)(gx*10000)>>8)&0XFF;
	tbuf[7]=(short)(gx*10000)&0XFF;
	tbuf[8]=((short)(gy*10000)>>8)&0XFF;
	tbuf[9]=(short)(gy*10000)&0XFF;
	tbuf[10]=((short)(gz*10000)>>8)&0XFF;
	tbuf[11]=(short)(gz*10000)&0XFF;
	tbuf[12]=((short)((wx-gx)*10000)>>8)&0XFF;
	tbuf[13]=(short)((wx-gx)*10000)&0XFF;
	tbuf[14]=((short)((wy-gy)*10000)>>8)&0XFF;
	tbuf[15]=(short)((wy-gy)*10000)&0XFF;
	tbuf[16]=((short)((wz-gz)*10000)>>8)&0XFF;
	tbuf[17]=(short)((wz-gz*10000))&0XFF;
	usart1_niming_report(0XF2,tbuf,18);//姿态角显示帧,0X01  
 
}

  
int main(void)
{ 
	u8 report=1;			    //默认开启上报
	u8 key;
	float pitch,roll,yaw; 		//欧拉角
	short t=0;                  //记录循环用时
	short aacx,aacy,aacz;		//加速度传感器原始数据
	short gyrox,gyroy,gyroz;	//陀螺仪原始数据
	short mx,my,mz;		        //磁力计原始数据
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);            //初始化延时函数
	uart_init(500000);		    //初始化串口波特率为500000
	LED_Init();					//初始化LED 
	KEY_Init();					//初始化按键
	MPU_Init();					//初始化MPU6050
	Init_HMC5883L();
while(mpu_dmp_init())
	{
		LED0=!LED0;					 
        delay_ms(500);	//初始化失败，报错
	}
	//矩阵初始化
	 arm_mat_init_f32(&g_kt,7,7,(float32_t *)GK_t);
	 arm_mat_init_f32(&h_kt,7,6,(float32_t *)HK_t);
	 arm_mat_init_f32(&kk_k,7,1,(float32_t *)KK_k);
	 arm_mat_init_f32(&p,7,7,(float32_t *)P);
	 arm_mat_init_f32(&pp,7,7,(float32_t *)PP);
	 arm_mat_init_f32(&r,6,6,(float32_t *)R);
	 arm_mat_init_f32(&q,7,7,(float32_t *)Q);
	 arm_mat_init_f32(&k_k,7,6,(float32_t *)K_k);
	 arm_mat_init_f32(&ph,7,6,(float32_t *)PH);
	 arm_mat_init_f32(&hp,6,7,(float32_t *)HP);
	 arm_mat_init_f32(&hph,6,6,(float32_t *)HPH);
	 arm_mat_init_f32(&hph_,6,6,(float32_t *)HPH_);
	 arm_mat_init_f32(&Ii,7,7,(float32_t *)I);
	 arm_mat_init_f32(&kh,7,7,(float32_t *)KH);

 	while(1)
	{
		key=KEY_Scan(0);
		if(key==KEY0_PRES)
		{
			report=!report;
		}
		if(key==KEY2_PRES)
		{
			t=0;
		}
		if(report)
			 LED1=0;//数据上传时LED1亮
		else LED1=1;
		
		if(report)//
		{ 
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
			HMC_Get_magnetometer(&mx,&my,&mz);
			mx=mx-156;
			my=my-123;
			mz=mz-340;//磁力计数据校准

			usart1_report_ekfAngle(aacx,aacy,aacz,gyrox,gyroy,gyroz,mx,my,mz,(int)(-roll*100),(int)(pitch*100),(int)(-yaw*100),key,t);
			usart1_report_senserdata(aacx,aacy,aacz,gyrox,gyroy,gyroz,mx,my,mz);//发送传感器数据
			t++;
		}
	} 	
}


