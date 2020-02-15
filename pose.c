#include "pose.h"
#include "math.h"

//@作者			：tou_zi
//@编写时间	：2019年4月6日
//@修改时间	：2019年4月6日
//@函数名		：initPose_Module
//@函数功能	：初始化姿态解算模块
//@输入1		：*pose 姿态解算结构体指针
//@返回值		：无
void initPose_Module(Pose_Module *pose)
{
//标志位初始化
	pose->flag.run = 1;						//开启计算
	pose->flag.use_mag = 0;				//使用电子罗盘
//接口初始化
	pose->interface.data.a_x = 0;
	pose->interface.data.a_y = 0;
	pose->interface.data.a_z = 0;
	pose->interface.data.g_x = 0;
	pose->interface.data.g_y = 0;
	pose->interface.data.g_z = 0;
	pose->interface.data.m_x = 0;
	pose->interface.data.m_y = 0;
	pose->interface.data.m_z = 0;
//参数初始化	
	pose->parameter.error_ki = 0;
	pose->parameter.error_kp = 0.65f;
	pose->parameter.correct_kp = 0.45f;
//中间变量清空	
	pose->process.error.x = 0;
	pose->process.error.y = 0;
	pose->process.error.z = 0;
	pose->process.error_integral.x = 0;
	pose->process.error_integral.y = 0;
	pose->process.error_integral.z = 0;
	
	pose->process.quaternion[0] = 1;
	pose->process.quaternion[1] = 0;
	pose->process.quaternion[2] = 0;
	pose->process.quaternion[3] = 0;
//数据初始化
	pose->data.rotate_matrix[0][0] = 0;
	pose->data.rotate_matrix[0][1] = 0;
	pose->data.rotate_matrix[0][2] = 0;
	pose->data.rotate_matrix[1][0] = 0;
	pose->data.rotate_matrix[1][1] = 0;
	pose->data.rotate_matrix[1][2] = 0;
	pose->data.rotate_matrix[2][0] = 0;
	pose->data.rotate_matrix[2][1] = 0;
	pose->data.rotate_matrix[2][2] = 0;
	
	pose->data.mag_world.x = 0;
	pose->data.mag_world.y = 0;
	pose->data.mag_world.z = 0;
	
	pose->data.acc_world.x = 0;
	pose->data.acc_world.y = 0;
	pose->data.acc_world.z = 0;
	
	pose->data.mag_correct.x = 0;
	pose->data.mag_correct.y = 0;
	pose->data.mag_correct.z = 0;
	
	pose->data.acc_correct.x = 0;
	pose->data.acc_correct.y = 0;
	pose->data.acc_correct.z = 0;
	
	pose->data.gyro_correct.x = 0;
	pose->data.gyro_correct.y = 0;
	pose->data.gyro_correct.z = 0;
	
	pose->data.pit = 0;
	pose->data.rol = 0;
	pose->data.yaw = 0;
}

void simple_3d_trans(XYZ_Data_f *ref, XYZ_Data_f *in, XYZ_Data_f *out) //小范围内正确。
{
	static s8 pn;
	static float h_tmp_x,h_tmp_y;
	
	h_tmp_x = sqrt(ref->z * ref->z + ref->y * ref->y);
	h_tmp_y = sqrt(ref->z * ref->z + ref->x * ref->x);
	
	pn = ref->z < 0 ? -1 : 1;
	
	out->x = ( h_tmp_x *in->x - pn *ref->x *in->z ) ;
	out->y = ( pn *h_tmp_y *in->y - ref->y *in->z ) ;
	out->z = ref->x *in->x + ref->y *in->y + ref->z *in->z ;

}


//@作者			：tou_zi
//@编写时间	：2019年4月6日
//@修改时间	：2019年4月6日
//@函数名		：calculatePose_Module
//@函数功能	：姿态解算	--	四元数解算
//@输入1		：*pose 姿态解算结构体指针
//@输入2		：cycle 周期
//@返回值		：无
void calculatePose_Module(Pose_Module *pose, float cycle) 
{
	float length;
	XYZ_Data_f acc_tmp; 
	XYZ_Data_f error;
	
	if (pose->flag.run == 0)
		return;
	
/////////////////////////////////////////////////////////////////////////////////////////////////		
	//电子罗盘处理
	if (pose->flag.use_mag == 1)
	{	
		//利用电子罗盘计算yaw
		length = sqrt(pose->data.mag_correct.x * pose->data.mag_correct.x + pose->data.mag_correct.y * pose->data.mag_correct.y);
		if( pose->data.mag_correct.x != 0 && pose->data.mag_correct.y != 0 && pose->data.mag_correct.z != 0 && length != 0)
		{
			pose->process.mag_yaw = arctan2(pose->data.mag_correct.y / length, pose->data.mag_correct.x / length);
		}

		//计算yaw偏差
		if(pose->data.rotate_matrix[2][2] > 0.0f )	
		{
			pose->process.mag_yaw_bias = pose->parameter.correct_kp * translateAngle(pose->data.yaw - pose->process.mag_yaw);
			//矫正值过大 -- 矫正值错误
			if(pose->process.mag_yaw_bias > 360 || pose->process.mag_yaw_bias < -360)
			{
				pose->process.mag_yaw_bias = 0;
			}
		}
		
		else
		{
			pose->process.mag_yaw_bias = 0; //角度过大，停止修正，修正的目标值可能不正确
		}
	}

	else
	{
		pose->process.mag_yaw_bias = 0;
	}
/////////////////////////////////////////////////////////////////////////////////////////////////	
	//加速度计处理
	length = sqrt(	*(pose->interface.data.a_x) * *(pose->interface.data.a_x) + 
									*(pose->interface.data.a_y) * *(pose->interface.data.a_y) + 
									*(pose->interface.data.a_z) * *(pose->interface.data.a_z));   
	
	if(	ABS(*(pose->interface.data.a_x)) < 1050.0f && 
			ABS(*(pose->interface.data.a_y)) < 1050.0f && 
			ABS(*(pose->interface.data.a_z)) < 1050.0f )
	{	
		//加速度计归一化
		acc_tmp.x = *(pose->interface.data.a_x) / length;
		acc_tmp.y = *(pose->interface.data.a_y) / length;
		acc_tmp.z = *(pose->interface.data.a_z) / length;
		
		//叉乘计算偏差	--	在无人机稳定时进行补偿
		if(800.0f < length && length < 1200.0f)
		{
			/* 叉乘得到误差 */
			error.x = (acc_tmp.y * pose->data.rotate_matrix[2][2] - acc_tmp.z * pose->data.rotate_matrix[1][2]);
			error.y = (acc_tmp.z * pose->data.rotate_matrix[0][2] - acc_tmp.x * pose->data.rotate_matrix[2][2]);
	    error.z = (acc_tmp.x * pose->data.rotate_matrix[1][2] - acc_tmp.y * pose->data.rotate_matrix[0][2]);
			
			/* 误差低通 */
			pose->process.error.x += 1.0f * 3.14f * cycle *(error.x  - pose->process.error.x );
			pose->process.error.y += 1.0f * 3.14f * cycle *(error.y  - pose->process.error.y );
			pose->process.error.z += 1.0f * 3.14f * cycle *(error.z  - pose->process.error.z );
		}
	}
	else
	{
		pose->process.error.x = 0; 
		pose->process.error.y = 0  ;
		pose->process.error.z = 0 ;
	}

	// 误差积分	
	pose->process.error_integral.x += pose->process.error.x * pose->parameter.error_ki * cycle;
	pose->process.error_integral.y += pose->process.error.y * pose->parameter.error_ki * cycle;
	pose->process.error_integral.z += pose->process.error.z * pose->parameter.error_ki * cycle;
	
	//积分限幅 -- 2°以内
	pose->process.error_integral.x = LIMIT(pose->process.error_integral.x, - 0.035f ,0.035f );
	pose->process.error_integral.y = LIMIT(pose->process.error_integral.y, - 0.035f ,0.035f );
	pose->process.error_integral.z = LIMIT(pose->process.error_integral.z, - 0.035f ,0.035f );	
	
/////////////////////////////////////////////////////////////////////////////////////////////////
	//开始修正陀螺仪值
	pose->data.gyro_correct.x = (*(pose->interface.data.g_x) - pose->data.rotate_matrix[0][2] * pose->process.mag_yaw_bias) * 0.01745329f + 
						(pose->parameter.error_kp * pose->process.error.x + pose->process.error_integral.x) ;
	pose->data.gyro_correct.y = (*(pose->interface.data.g_y) - pose->data.rotate_matrix[1][2] * pose->process.mag_yaw_bias) * 0.01745329f + 
						(pose->parameter.error_kp * pose->process.error.y + pose->process.error_integral.y) ;		 
	pose->data.gyro_correct.z = (*(pose->interface.data.g_z) - pose->data.rotate_matrix[2][2] * pose->process.mag_yaw_bias) * 0.01745329f +
						(pose->parameter.error_kp * pose->process.error.z + pose->process.error_integral.z) ;
	
/////////////////////////////////////////////////////////////////////////////////////////////////
	// 一阶龙格库塔更新四元数值
	pose->process.quaternion[0] += (-pose->process.quaternion[1] * pose->data.gyro_correct.x - pose->process.quaternion[2] * pose->data.gyro_correct.y - pose->process.quaternion[3] * pose->data.gyro_correct.z) * cycle / 2.0f;
	pose->process.quaternion[1] +=  (pose->process.quaternion[0] * pose->data.gyro_correct.x + pose->process.quaternion[2] * pose->data.gyro_correct.z - pose->process.quaternion[3] * pose->data.gyro_correct.y) * cycle / 2.0f;
	pose->process.quaternion[2] +=  (pose->process.quaternion[0] * pose->data.gyro_correct.y - pose->process.quaternion[1] * pose->data.gyro_correct.z + pose->process.quaternion[3] * pose->data.gyro_correct.x) * cycle / 2.0f;
	pose->process.quaternion[3] +=  (pose->process.quaternion[0] * pose->data.gyro_correct.z + pose->process.quaternion[1] * pose->data.gyro_correct.y - pose->process.quaternion[2] * pose->data.gyro_correct.x) * cycle / 2.0f;  

	//四元数归一化
	length = sqrt(pose->process.quaternion[0] * pose->process.quaternion[0] + 
								pose->process.quaternion[1] * pose->process.quaternion[1] +
								pose->process.quaternion[2] * pose->process.quaternion[2] +
								pose->process.quaternion[3] * pose->process.quaternion[3]);
		
	if (length != 0)
	{
		pose->process.quaternion[0] /= length;
		pose->process.quaternion[1] /= length;
		pose->process.quaternion[2] /= length;
		pose->process.quaternion[3] /= length;
	}
	
///////////////////////////////////////////////////////////////////////////////////////////////////	
	//计算旋转矩阵
	pose->data.rotate_matrix[0][0] = 	pose->process.quaternion[0] * pose->process.quaternion[0] + pose->process.quaternion[1] * pose->process.quaternion[1] - 
																		pose->process.quaternion[2] * pose->process.quaternion[2] - pose->process.quaternion[3] * pose->process.quaternion[3];
	pose->data.rotate_matrix[0][1] = 	2 * (pose->process.quaternion[1] * pose->process.quaternion[2] + pose->process.quaternion[0] * pose->process.quaternion[3]);
	pose->data.rotate_matrix[0][2] =  2 * (pose->process.quaternion[1] * pose->process.quaternion[3] - pose->process.quaternion[0] * pose->process.quaternion[2]);
	
	pose->data.rotate_matrix[1][0] =  2 * (pose->process.quaternion[1] * pose->process.quaternion[2] - pose->process.quaternion[0] * pose->process.quaternion[3]);
	pose->data.rotate_matrix[1][1] = 	pose->process.quaternion[0] * pose->process.quaternion[0] - pose->process.quaternion[1] * pose->process.quaternion[1] + 
																		pose->process.quaternion[2] * pose->process.quaternion[2] - pose->process.quaternion[3] * pose->process.quaternion[3];
	pose->data.rotate_matrix[1][2] =  2 * (pose->process.quaternion[2] * pose->process.quaternion[3] + pose->process.quaternion[0] * pose->process.quaternion[1]);
	
	pose->data.rotate_matrix[2][0] = 	2 * (pose->process.quaternion[1] * pose->process.quaternion[3] + pose->process.quaternion[0] * pose->process.quaternion[2]);
	pose->data.rotate_matrix[2][1] = 	2 * (pose->process.quaternion[2] * pose->process.quaternion[3] - pose->process.quaternion[0] * pose->process.quaternion[1]);
	pose->data.rotate_matrix[2][2] = 	pose->process.quaternion[0] * pose->process.quaternion[0] - pose->process.quaternion[1] * pose->process.quaternion[1] - 
																		pose->process.quaternion[2] * pose->process.quaternion[2] + pose->process.quaternion[3] * pose->process.quaternion[3];
	
	//计算世界坐标系下的磁力计值
	if (pose->flag.use_mag == 1)
	{
		pose->data.mag_world.x = 	pose->data.rotate_matrix[0][0] * *(pose->interface.data.m_x) + 
															pose->data.rotate_matrix[1][0] * *(pose->interface.data.m_y) + 
															pose->data.rotate_matrix[2][0] * *(pose->interface.data.m_z) ;
															
		pose->data.mag_world.y = 	pose->data.rotate_matrix[0][1] * *(pose->interface.data.m_x) + 
															pose->data.rotate_matrix[1][1] * *(pose->interface.data.m_y) + 
															pose->data.rotate_matrix[2][1] * *(pose->interface.data.m_z) ;
															
		pose->data.mag_world.z = 	pose->data.rotate_matrix[0][2] * *(pose->interface.data.m_x) + 
															pose->data.rotate_matrix[1][2] * *(pose->interface.data.m_y) + 
															pose->data.rotate_matrix[2][2] * *(pose->interface.data.m_z) ;
	}
	
	//计算世界坐标系下的加速度值
	pose->data.acc_world.x = 	pose->data.rotate_matrix[0][0] * *(pose->interface.data.a_x) + 
														pose->data.rotate_matrix[1][0] * *(pose->interface.data.a_y) + 
														pose->data.rotate_matrix[2][0] * *(pose->interface.data.a_z) ;
														
	pose->data.acc_world.y = 	pose->data.rotate_matrix[0][1] * *(pose->interface.data.a_x) + 
														pose->data.rotate_matrix[1][1] * *(pose->interface.data.a_y) + 
														pose->data.rotate_matrix[2][1] * *(pose->interface.data.a_z) ;
														
	pose->data.acc_world.z = 	pose->data.rotate_matrix[0][2] * *(pose->interface.data.a_x) + 
														pose->data.rotate_matrix[1][2] * *(pose->interface.data.a_y) + 
														pose->data.rotate_matrix[2][2] * *(pose->interface.data.a_z) ;
	
	//求解欧拉角
	pose->data.rol = arctan2(pose->data.rotate_matrix[2][2], pose->data.rotate_matrix[1][2]);						
	pose->data.pit = -arcsin(pose->data.rotate_matrix[0][2]);
	pose->data.yaw = arctan2(pose->data.rotate_matrix[0][0], pose->data.rotate_matrix[0][1]);
	
/////////////////////////////////////////////////////////////////////////////////////////////////			
	//计算机体坐标系矫正后的加速度--不受俯仰和翻滚影响
	pose->data.acc_correct.x =   pose->data.acc_world.x * cos(pose->data.yaw) + pose->data.acc_world.y * sin(pose->data.yaw);
	pose->data.acc_correct.y =  -pose->data.acc_world.x * sin(pose->data.yaw) + pose->data.acc_world.y * cos(pose->data.yaw);
	pose->data.acc_correct.z =   pose->data.acc_world.z;
	
	//计算机体坐标系矫正后的磁场--不受俯仰和翻滚影响
	if (pose->flag.use_mag == 1)
	{
		XYZ_Data_f ref_v = (XYZ_Data_f){pose->data.rotate_matrix[0][2], pose->data.rotate_matrix[1][2], pose->data.rotate_matrix[2][2]};
		XYZ_Data_f mag_tmp = (XYZ_Data_f){*pose->interface.data.m_x, *pose->interface.data.m_y, *pose->interface.data.m_z};
		
		length = 	sqrt(	*(pose->interface.data.m_x) * *(pose->interface.data.m_x) + 
										*(pose->interface.data.m_y) * *(pose->interface.data.m_y) + 
										*(pose->interface.data.m_z) * *(pose->interface.data.m_z));   
		
//		length *= sqrt(	pose->data.rotate_matrix[0][2] * pose->data.rotate_matrix[0][2] +  
//										pose->data.rotate_matrix[1][2] * pose->data.rotate_matrix[1][2] +  
//										pose->data.rotate_matrix[2][2] * pose->data.rotate_matrix[2][2] );
//		
		if (length != 0)
		{
			simple_3d_trans(&ref_v, &mag_tmp, &pose->data.mag_correct);
//			pose->data.mag_correct.z = pose->data.mag_world.z;
//			pose->data.mag_correct.x = sqrt(1 - (pose->data.mag_correct.z / length) * (pose->data.mag_correct.z / length)) * *(pose->interface.data.m_x);
//			pose->data.mag_correct.y = sqrt(1 - (pose->data.mag_correct.z / length) * (pose->data.mag_correct.z / length)) * *(pose->interface.data.m_y);
		}
	}
	
}
