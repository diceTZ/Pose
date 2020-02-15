//@作者			：tou_zi
//@编写时间	：2019年4月6日
//@修改时间	：2019年4月6日
//@文件名		：pose.h
//@描述			：姿态解算模块
#ifndef _POSE_H
#define _POSE_H

#ifndef u8 
#define u8 unsigned char
#endif

#ifndef s8 
#define s8 char
#endif

#ifndef XYZ_Data
#define XYZ_Data
typedef struct XYZ_Data_f
{
	float x;
	float y;
	float z;
}XYZ_Data_f;

typedef struct XYZ_Data_s32
{
	long x;
	long y;
	long z;
}XYZ_Data_s32;

typedef struct XYZ_Data_s16
{
	short x;
	short y;
	short z;
}XYZ_Data_s16;

typedef struct XYZ_Data_s8
{
	s8 x;
	s8 y;
	s8 z;
}XYZ_Data_s8;
#endif

////////////////////////////////////////////////////
typedef struct Pose_Flag
{
	u8 run;
	u8 use_mag;
}Pose_Flag;
////////////////////////////////////////////////////
typedef struct Pose_DInterface
{
	float *a_x;
	float *a_y;
	float *a_z;
	
	float *g_x;
	float *g_y;
	float *g_z;
	
	float *m_x;
	float *m_y;
	float *m_z;
}Pose_DInterface;

typedef struct Pose_Interface
{
	Pose_DInterface data;
}Pose_Interface;
////////////////////////////////////////////////////
typedef struct Pose_Data
{
	float yaw;
	float rol;
	float pit;
	
	float rotate_matrix[3][3];	//旋转矩阵
	
	XYZ_Data_f acc_world;				//世界坐标系下的加速度
	XYZ_Data_f mag_world;				//世界坐标系下的磁场强度	--	只与无人机位置有关的量
	
	XYZ_Data_f acc_correct;			//机体坐标系下的加速度	--	矫正了俯仰翻滚后的加速度
	XYZ_Data_f mag_correct;			//机体坐标系下的磁场强度	--	矫正了俯仰翻滚后的磁场强度
	XYZ_Data_f gyro_correct;		//融合加速度和磁力计数据，矫正后的陀螺仪值
}Pose_Data;
////////////////////////////////////////////////////
typedef struct Pose_Process
{
	float mag_yaw;							//磁力计的偏航角
	float mag_yaw_bias;					//磁力计矫正的偏航角偏差
	float quaternion[4];				//四元数
	XYZ_Data_f error;						//由加速度计与等效重力的偏差
	XYZ_Data_f error_integral;	//误差积分
}Pose_Process;
////////////////////////////////////////////////////
typedef struct Pose_Parameter
{
	float correct_kp;
	float error_kp;
	float error_ki;
}Pose_Parameter;
////////////////////////////////////////////////////
typedef struct Pose_Module
{
	Pose_Flag flag;
	Pose_Interface interface; 
	Pose_Process process;
	Pose_Data data;
	Pose_Parameter parameter;
}Pose_Module;

//初始化结构体
void initPose_Module(Pose_Module *pose);
//计算姿态
void calculatePose_Module(Pose_Module *pose, float cycle);
#endif

