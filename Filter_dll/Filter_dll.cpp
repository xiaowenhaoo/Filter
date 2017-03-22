// Filter_dll.cpp : 定义 DLL 应用程序的导出函数。
//

#include "stdafx.h"

//一阶滞后滤波
__declspec(dllexport) float Filter0(float input_now, float cal_pre, float k)
{
	if (k >= 0 && k <= 1)
		return (1 - k)*cal_pre + k*input_now;
	else
		return 0;
}

//平均值滤波法
__declspec(dllexport) float Filter1(float buf[], int buf_size)
{
	float output = 0;
	float sum = 0;

	for (int i = 0; i < buf_size; i++)
	{
		sum += buf[i];
	}

	output = sum / buf_size;

	return output;
}


//限速、限加速度方法
//使用此限幅滤波方法，输出曲线时钟有延时
__declspec(dllexport) float limite_speed_acc(float input_now, float input_pre, float cal_pre, float cal_prepre, float Interval, float maxV, float maxA, float k_relative_vel)
{
	float calculate = 0;  //返回值
	float vel_input = 0;  //输入曲线速度
	float vel_relative = 0;  //相对速度
	float vel_cal = 0;
	float acc_cal = 0;
	float v1 = 0;

	v1 = (cal_pre - cal_prepre) / Interval;

	//根据输入值，反推速度、加速度和二阶加速度
	vel_input = (input_now - input_pre) / Interval; //目标曲线的速度

	vel_relative = (input_now - cal_pre) * k_relative_vel;

	vel_cal = vel_input + vel_relative;
	if (vel_cal > maxV)
		vel_cal = maxV;
	else if (vel_cal < -1 * maxV)
		vel_cal = -1 * maxV;

	acc_cal = (vel_cal - v1) / Interval;
	if (acc_cal > maxA)
		acc_cal = maxA;
	else if (acc_cal < -1 * maxA)
		acc_cal = -1 * maxA;


	//正推计算值
	vel_cal = v1 + acc_cal*Interval;
	calculate = cal_pre + vel_cal*Interval;

	return calculate;
}

//此方法限制速度和加速度
//此方法根据输入判断是否需要进行限幅计算
__declspec(dllexport) float limite_speed_acc_2(float input_now, float input_pre, float cal_pre, float cal_prepre, float Interval, float maxV, float maxA, float k_relative_vel)
{
	float calculate = 0;  //返回值
	float vel_input = 0;  //输入曲线速度
	float vel_relative = 0;  //相对速度
	float vel_cal = 0;
	float acc_cal = 0;
	float v1 = 0;
	float v0 = 0;  //当前输入需要速度
	float acc0 = 0; //当前输入需要加速度

	v0 = (input_now - cal_pre) / Interval;
	v1 = (cal_pre - cal_prepre) / Interval;
	acc0 = (v0 - v1) / Interval;

	if ((v0 > maxV) || (acc0 > maxA))
	{
		//根据输入值，反推速度、加速度和二阶加速度
		vel_input = (input_now - input_pre) / Interval; //目标曲线的速度

		vel_relative = (input_now - cal_pre) * k_relative_vel;

		vel_cal = vel_input + vel_relative;
		if (vel_cal > maxV)
			vel_cal = maxV;
		else if (vel_cal < -1 * maxV)
			vel_cal = -1 * maxV;

		acc_cal = (vel_cal - v1) / Interval;
		if (acc_cal > maxA)
			acc_cal = maxA;
		else if (acc_cal < -1 * maxA)
			acc_cal = -1 * maxA;


		//正推计算值
		vel_cal = v1 + acc_cal*Interval;
		calculate = cal_pre + vel_cal*Interval;
	}
	else
	{
		calculate = input_now;
	}
	return calculate;
}


//此方法同时限制位置、速度、加速度
__declspec(dllexport) float limite_pos_speed_acc(float input_now, float input_pre, float cal_pre, float cal_prepre, float Interval, float maxS, float maxV, float maxA, float k_relative_vel)
{
	float calculate = 0;  //返回值
	float vel_input = 0;  //输入曲线速度
	float vel_relative = 0;  //相对速度
	float vel_cal = 0;
	float acc_cal = 0;
	float v1 = 0;

	v1 = (cal_pre - cal_prepre) / Interval;

	////限制位置值
	//if (input_now > maxS)
	//	input_now = maxS;
	//else if (input_now < -1 * maxS)
	//	input_now = -1 * maxS;

	//根据输入值，反推速度、加速度和二阶加速度
	vel_input = (input_now - input_pre) / Interval; //目标曲线的速度

	vel_relative = (input_now - cal_pre) * k_relative_vel;

	vel_cal = vel_input + vel_relative;
	if (vel_cal > maxV)
		vel_cal = maxV;
	else if (vel_cal < -1 * maxV)
		vel_cal = -1 * maxV;

	acc_cal = (vel_cal - v1) / Interval;
	if (acc_cal > maxA)
		acc_cal = maxA;
	else if (acc_cal < -1 * maxA)
		acc_cal = -1 * maxA;


	//正推计算值
	vel_cal = v1 + acc_cal*Interval;
	calculate = cal_pre + vel_cal*Interval;

	//限制位置值
	if (calculate > maxS)
		calculate = maxS;
	else if (calculate < -1 * maxS)
		calculate = -1 * maxS;

	return calculate;
}


//功能函数，对输入值 x 做最大值和最小值限制；
float InRange(float x, float range)
{
	if (range < 0)  //确定范围值为正值
		range = -1 * range;

	if (x>range)
		return range;
	else if (x < -1 * range)
		return -1 * range;
	else
		return x;
}


//新稿滤波函数，类似于伺服控制器三环控制原理，并增加了位置环前馈控制；
__declspec(dllexport) float limiting_new(float input, float maxS, float maxV, float maxA, float maxJ, float kp)
{
	const float interval = 0.01f;
	const float kp_vel = 100;
	static float input_pre = 0;
	static float acc = 0;
	static float vel = 0;
	static float pos = 0;
	float input_temp = 0;
	float vel_temp = 0;
	float acc_temp = 0;
	float jerk_temp = 0;
	float vel_input=0;

	if (kp > 10)  //防止位置环增益过大造成超调和震荡
		kp = 10;
	else if (kp < 1)  //防止位置环增益过小，响应过于迟钝
		kp = 1;

	input_temp = InRange(input, maxS);

	vel_input = (input_temp - input_pre) / interval;//前馈

	vel_temp = InRange(kp*(input_temp - pos) + vel_input, maxV);//位置环闭环

	acc_temp = InRange((vel_temp - vel)*kp_vel, maxA);//速度环闭环

	jerk_temp = InRange((acc_temp - acc) / interval, maxJ);//加速度环闭环

	acc += jerk_temp*interval;

	vel += acc*interval;

	pos += vel*interval;

	input_pre = input_temp;

	return pos;
}