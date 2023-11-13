#include<Eigen/Dense>
#include<opencv2/opencv.hpp>
#include<opencv2/core/eigen.hpp>
using namespace Eigen;
using namespace cv;


//һά��������λ��
//X^_(k) = A * X^(k-1)							K = P_(k) * C^(T) / (C * P_(k) * C^(T) + R)
//P_(k) = A * P(k-1) * A^(T) + Q				X^(k) = X^_(k) + K(y - C * X^_(k)
//												P = (1 - K * C) * P_(k)
//[ position.x , position.y ]


//����λ�ú��ٶȣ�������״̬
//[ position.x , position.y , v_x , v_y ]
//




class kalman
{
public:
	kalman();
	~kalman();

public:
#pragma region λ�ð汾
	//λ�ð汾

	Matrix<float, 2, 2> F;//״̬ת�� ���ø���

	Matrix<float, 2, 2> P;//Э������� ��Ҫ����

	Matrix<float, 2, 2> H;//�������� ���ø���

	Matrix<float, 2, 2> Q;//״̬��� ���ø���
	Matrix<float, 2, 2> R;//�۲���� ���ø���


	Matrix<float, 2, 2> K;//���������� ��Ҫ����

	Matrix<float, 2, 1> now_temp;//��λ��temp ��Ҫ����
	//Matrix<float, 2, 1> last_temp;//��һʱ��temp ��Ҫ����
	Matrix<float, 2, 1> x_;//x���� ��Ҫ����
	Matrix<float, 2, 2>p_;//p���� ��Ҫ����
	
	Matrix<float, 2, 1> x;//����ֵ ��Ҫ����

	Matrix<float, 2, 2> I;//��λ���� ����Ҫ����

	void predict(Point2f& mouse_center);
	Point2f update();
#pragma endregion


#pragma region ���ٰ汾
	//���ٰ汾
	float delta_t;//���ʱ��

	Matrix<float, 4, 4> v_F;//״̬ת�� ���ø���

	Matrix<float, 4, 4> v_P;//Э������� ��Ҫ����

	//Matrix<float, 4, 4> v_H;//�������� ���ø���     �۲���Ϊ[x,y,vx,vy]
	Matrix<float, 2, 4> v_H;//�������� ���ø���	�۲���Ϊ[x,y]

	Matrix<float, 4, 4> v_Q;//״̬��� ���ø���

	//Matrix<float, 4, 4> v_R;//�۲���� ���ø���
	Matrix<float, 2, 2> v_R;//�۲���� ���ø���


	//Matrix<float, 4, 4> v_K;//���������� ��Ҫ����
	Matrix<float, 4, 2> v_K;//���������� ��Ҫ����

	//Matrix<float, 4, 1> v_now_temp;//��λ��temp ��Ҫ����		�۲���Ϊ[x,y,vx,vy]
	//Matrix<float, 4, 1> v_last_temp;//��һʱ��temp ��Ҫ����	�۲���Ϊ[x,y,vx,vy]

	Matrix<float, 2, 1> v_now_temp;//��λ��temp ��Ҫ����	�۲���Ϊ[x,y]
	Matrix<float, 2, 1> v_last_temp;//��һʱ��temp ��Ҫ����	�۲���Ϊ[x,y]

	Matrix<float, 4, 1> v_x_;//x���� ��Ҫ����
	Matrix<float, 4, 4> v_p_;//p���� ��Ҫ����

	Matrix<float, 4, 1> v_x;//����ֵ ��Ҫ����

	Matrix<float, 4, 4> v_I;//��λ���� ����Ҫ����

	void v_init(Point2f& v_now_mouse_center);//��һ֡������ʼ��
	void v_predict(Point2f& v_now_mouse_center,float& delta_time);
	Point2f v_update();
#pragma endregion


};

//class kalman_vp {
//public:
//	kalman_vp();
//	~kalman_vp();
//
//public:
//	//λ�ð汾
//
//	Matrix<float, 2, 2> F;//״̬ת�� ���ø���
//
//	Matrix<float, 2, 2> P;//Э������� ��Ҫ����
//
//	Matrix<float, 2, 2> H;//�������� ���ø���
//
//	Matrix<float, 2, 2> Q;//״̬��� ���ø���
//	Matrix<float, 2, 2> R;//�۲���� ���ø���
//
//
//	Matrix<float, 2, 2> K;//���������� ��Ҫ����
//
//	Matrix<float, 2, 1> now_temp;//��λ��temp ��Ҫ����
//	Matrix<float, 2, 1> last_temp;//��һʱ��temp ��Ҫ����
//	Matrix<float, 2, 1> x_;//x���� ��Ҫ����
//	Matrix<float, 2, 2>p_;//p���� ��Ҫ����
//
//	Matrix<float, 2, 1> x;//����ֵ ��Ҫ����
//
//	Matrix<float, 2, 2> I;//��λ���� ����Ҫ����
//
//	void predict(Point2f& last_center);
//	Point2f update();
//
//};