#include "kalman.h"

kalman::kalman()
{

#pragma region λ�ø�ֵ
	F << 1, 0,
		0, 1;   //x y

	P << 1, 0,
		0, 1;

	H << 1, 0,
		0, 1;   //x y

	Q << 0.01, 0,
		0, 0.1;   //QԽС��Խ������������QΪ0��ֻ����Ԥ��ֵ

	R << 0.5, 0,
		0, 0.5;


	K << 0, 0,
		0, 0;

	now_temp << 0, 
				0;

	//last_temp << 0,
	//			 0;

	x_ << 0,
		  0;

	p_ << 0,0,
		  0,0;

	x << 0,
		 0;

	I << 1, 0,
		0, 1;
#pragma endregion


#pragma region  ���ٸ�ֵ
	delta_t = 0;

	v_F <<	1, 0, delta_t, 0,
			0, 1, 0, delta_t,
			0, 0, 1, 0,
			0, 0, 0, 1;   //x y vx vy

	v_P <<	1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;

	v_H << 1, 0, 0, 0,
		   0, 1, 0, 0;

	//v_H << 1, 0, 0, 0,
	//	   0, 1, 0, 0,
	//	   0, 0, 1, 0,
	//	   0, 0, 0, 1;


	//v_Q << 0.001936, 0, 0.03872, 0,
	//		0, 0.001936, 0, 0.03872,
	//	0.03872, 0, 0.7744, 0,
	//		0, 0.03872, 0, 0.7744;// |Q| << |R|   ģ��׼ȷ 

	v_Q <<      1, 0, 0, 0,
				0, 1, 0, 0,
				0, 0, 1, 0,
				0, 0, 0, 1;// |Q| << |R|   ģ��׼ȷ 

	//v_R <<  10, 0, 0, 0,
	//		0, 10, 0, 0,
	//		0, 0, 10, 0,
	//		0, 0, 0, 10;// |Q| >> |R|   ����׼ȷ ԭ��

	v_R << 10000, 0,
		   0, 10000; //����׼ȷ �޸ĺ�

	//v_K <<	0, 0, 0, 0,
	//		0, 0, 0, 0,
	//		0, 0, 0, 0,
	//		0, 0, 0, 0;

	//v_K << 0, 0, 0, 0,
	//	   0, 0, 0, 0;
	v_K << 0, 0,
		0, 0,
		0, 0,
		0, 0;

	//v_now_temp <<	0,
	//				0,
	//				0,
	//				0;

	//v_last_temp <<	0,
	//				0,
	//				0,
	//				0;

	v_now_temp << 0,
		0;

	v_last_temp << 0,
		0;




	v_x_ << 0,
			0,
			0,
			0;

	v_p_ << 0, 0, 0, 0,
			0, 0, 0, 0,
			0, 0, 0, 0,
			0, 0, 0, 0;

	v_x <<	0,
			0,
			0,
			0;

	v_I <<	1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;
#pragma endregion




}
kalman::~kalman()
{

}


//------------------------------------------------------------------------------------------------------------
//λ�ð汾
void kalman::predict(Point2f & mouse_center)
{
	//��һʱ�̵�λ��temp
	//Matrix<float, 2, 1> temp;
	now_temp << mouse_center.x, mouse_center.y;

	//x����
	//Matrix<float, 2, 1> x_;
	x_ = F * x;

	//p����
	//Matrix<float, 2, 1>p_;
	p_ = F * P * F.transpose() + Q;
}

Point2f kalman::update()
{
	K = (p_ * H.transpose()) * (H * p_ * H.transpose() + R).inverse();
	x = x_ + K * (H * now_temp - H * x_);
	P = (I - K) * p_;

	Point2f now_center;
	now_center = Point2f(x(0, 0), x(1, 0));


	return now_center;
	
}


//------------------------------------------------------------------------------------------------------------
//�����˶��汾 �ɹ�
void kalman::v_init(Point2f& v_now_mouse_center)
{
	//v_now_temp << v_now_mouse_center.x, v_now_mouse_center.y, 0, 0;
	v_now_temp << v_now_mouse_center.x, v_now_mouse_center.y;

}

void kalman::v_predict(Point2f& v_now_mouse_center, float& delta_time)
{
	delta_t = delta_time;

	//float vOfx = (v_now_mouse_center.x - v_last_temp(0)) / delta_t;
	//float vOfy = (v_now_mouse_center.y - v_last_temp(1)) / delta_t;

	//v_now_temp << v_now_mouse_center.x,
	//			  v_now_mouse_center.y,
	//			  vOfx,
	//	          vOfy;


	v_now_temp << v_now_mouse_center.x,
		v_now_mouse_center.y;

	v_last_temp = v_now_temp;



	v_F << 1, 0, delta_t, 0,
		   0, 1, 0, delta_t,
		   0, 0, 1, 0,
		   0, 0, 0, 1;   //x y
	//2 2�г�ǰЧ��

	v_x_ = v_F * v_x;
	std::cout << "v_x_ = " << std::endl << v_x_ << std::endl << std::endl;


	v_p_ = v_F * v_P * v_F.transpose() + v_Q;
}

Point2f kalman::v_update()
{
	//Matrix<float, 2, 1> z;//�۲��� [x , y]`
		
	//z << v_now_temp(0),
	//	 v_now_temp(1);



	v_K = (v_p_ * v_H.transpose()) * (v_H * v_p_ * v_H.transpose() + v_R).inverse();
	//v_x = v_x_ + v_K * (v_H * v_now_temp - v_H * v_x_);
	v_x = v_x_ + v_K * (v_now_temp - v_H * v_x_);
	//v_x = v_x_ + v_K * (H* - v_H * v_x_);
	v_P = (v_I - v_K * v_H) * v_p_;

	std::cout << "v_now_temp = " << std::endl << v_now_temp << std::endl << std::endl;
	std::cout << "v_x = " << std::endl << v_x << std::endl;
	std::cout << "------------------------------------------------" << std::endl;

	Point2f now_center;
	//std::cout << "vOfx = " << v_x(2, 0) << "     vOfy = " << v_x(3, 0) << std::endl;
	now_center = Point2f(v_x(0)+ 5 * v_x(2), v_x(1) + 4 * v_x(3));
	//now_center = Point2f(v_x(0, 0) + 80 * v_x(2, 0), v_x(1, 0) + 80 * v_x(3, 0));


	//now_center���������
	//����1�������չ��˳�����λ�ü����ӳ�ʱ��*�ٶȣ��ӳ�ʱ����Ҫ����
	//�پ�������ӵ�����ʱ��	�ڳ���������ӳ�	��ͨ���ӳ�	���



	//�ܽ�
	//�����Q R	�����в����ٶ�λ�ý��и�ֵ����û���˲�Ч���ģ����matlab����Ļ��ǻ����ë�̣�ͨ���Ա�ʵ�鷢�֣�
	//���˲����ǻ�����ͺ����������Ҫ�ﵽ��ǰ��Ч������Ҫ������λ������м���һ����ǰ������ǰ��=�ӳ�ʱ��*�ٶ�

	return now_center;
}


