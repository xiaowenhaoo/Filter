// Filter_dll.cpp : ���� DLL Ӧ�ó���ĵ���������
//

#include "stdafx.h"

//һ���ͺ��˲�
__declspec(dllexport) float Filter0(float input_now, float cal_pre, float k)
{
	if (k >= 0 && k <= 1)
		return (1 - k)*cal_pre + k*input_now;
	else
		return 0;
}

//ƽ��ֵ�˲���
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


//���١��޼��ٶȷ���
//ʹ�ô��޷��˲��������������ʱ������ʱ
__declspec(dllexport) float limite_speed_acc(float input_now, float input_pre, float cal_pre, float cal_prepre, float Interval, float maxV, float maxA, float k_relative_vel)
{
	float calculate = 0;  //����ֵ
	float vel_input = 0;  //���������ٶ�
	float vel_relative = 0;  //����ٶ�
	float vel_cal = 0;
	float acc_cal = 0;
	float v1 = 0;

	v1 = (cal_pre - cal_prepre) / Interval;

	//��������ֵ�������ٶȡ����ٶȺͶ��׼��ٶ�
	vel_input = (input_now - input_pre) / Interval; //Ŀ�����ߵ��ٶ�

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


	//���Ƽ���ֵ
	vel_cal = v1 + acc_cal*Interval;
	calculate = cal_pre + vel_cal*Interval;

	return calculate;
}

//�˷��������ٶȺͼ��ٶ�
//�˷������������ж��Ƿ���Ҫ�����޷�����
__declspec(dllexport) float limite_speed_acc_2(float input_now, float input_pre, float cal_pre, float cal_prepre, float Interval, float maxV, float maxA, float k_relative_vel)
{
	float calculate = 0;  //����ֵ
	float vel_input = 0;  //���������ٶ�
	float vel_relative = 0;  //����ٶ�
	float vel_cal = 0;
	float acc_cal = 0;
	float v1 = 0;
	float v0 = 0;  //��ǰ������Ҫ�ٶ�
	float acc0 = 0; //��ǰ������Ҫ���ٶ�

	v0 = (input_now - cal_pre) / Interval;
	v1 = (cal_pre - cal_prepre) / Interval;
	acc0 = (v0 - v1) / Interval;

	if ((v0 > maxV) || (acc0 > maxA))
	{
		//��������ֵ�������ٶȡ����ٶȺͶ��׼��ٶ�
		vel_input = (input_now - input_pre) / Interval; //Ŀ�����ߵ��ٶ�

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


		//���Ƽ���ֵ
		vel_cal = v1 + acc_cal*Interval;
		calculate = cal_pre + vel_cal*Interval;
	}
	else
	{
		calculate = input_now;
	}
	return calculate;
}


//�˷���ͬʱ����λ�á��ٶȡ����ٶ�
__declspec(dllexport) float limite_pos_speed_acc(float input_now, float input_pre, float cal_pre, float cal_prepre, float Interval, float maxS, float maxV, float maxA, float k_relative_vel)
{
	float calculate = 0;  //����ֵ
	float vel_input = 0;  //���������ٶ�
	float vel_relative = 0;  //����ٶ�
	float vel_cal = 0;
	float acc_cal = 0;
	float v1 = 0;

	v1 = (cal_pre - cal_prepre) / Interval;

	////����λ��ֵ
	//if (input_now > maxS)
	//	input_now = maxS;
	//else if (input_now < -1 * maxS)
	//	input_now = -1 * maxS;

	//��������ֵ�������ٶȡ����ٶȺͶ��׼��ٶ�
	vel_input = (input_now - input_pre) / Interval; //Ŀ�����ߵ��ٶ�

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


	//���Ƽ���ֵ
	vel_cal = v1 + acc_cal*Interval;
	calculate = cal_pre + vel_cal*Interval;

	//����λ��ֵ
	if (calculate > maxS)
		calculate = maxS;
	else if (calculate < -1 * maxS)
		calculate = -1 * maxS;

	return calculate;
}


//���ܺ�����������ֵ x �����ֵ����Сֵ���ƣ�
float InRange(float x, float range)
{
	if (range < 0)  //ȷ����ΧֵΪ��ֵ
		range = -1 * range;

	if (x>range)
		return range;
	else if (x < -1 * range)
		return -1 * range;
	else
		return x;
}


//�¸��˲��������������ŷ���������������ԭ����������λ�û�ǰ�����ƣ�
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

	if (kp > 10)  //��ֹλ�û����������ɳ�������
		kp = 10;
	else if (kp < 1)  //��ֹλ�û������С����Ӧ���ڳٶ�
		kp = 1;

	input_temp = InRange(input, maxS);

	vel_input = (input_temp - input_pre) / interval;//ǰ��

	vel_temp = InRange(kp*(input_temp - pos) + vel_input, maxV);//λ�û��ջ�

	acc_temp = InRange((vel_temp - vel)*kp_vel, maxA);//�ٶȻ��ջ�

	jerk_temp = InRange((acc_temp - acc) / interval, maxJ);//���ٶȻ��ջ�

	acc += jerk_temp*interval;

	vel += acc*interval;

	pos += vel*interval;

	input_pre = input_temp;

	return pos;
}