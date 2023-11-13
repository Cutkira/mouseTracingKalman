#include<Eigen/Dense>
#include<opencv2/opencv.hpp>
#include<opencv2/core/eigen.hpp>
using namespace Eigen;
using namespace cv;


//一维，仅考虑位置
//X^_(k) = A * X^(k-1)							K = P_(k) * C^(T) / (C * P_(k) * C^(T) + R)
//P_(k) = A * P(k-1) * A^(T) + Q				X^(k) = X^_(k) + K(y - C * X^_(k)
//												P = (1 - K * C) * P_(k)
//[ position.x , position.y ]


//考虑位置和速度，当匀速状态
//[ position.x , position.y , v_x , v_y ]
//




class kalman
{
public:
	kalman();
	~kalman();

public:
#pragma region 位置版本
	//位置版本

	Matrix<float, 2, 2> F;//状态转移 不用更新

	Matrix<float, 2, 2> P;//协方差矩阵 需要更新

	Matrix<float, 2, 2> H;//测量矩阵 不用更新

	Matrix<float, 2, 2> Q;//状态误差 不用更新
	Matrix<float, 2, 2> R;//观测误差 不用更新


	Matrix<float, 2, 2> K;//卡尔曼增益 需要更新

	Matrix<float, 2, 1> now_temp;//本位置temp 需要更新
	//Matrix<float, 2, 1> last_temp;//上一时刻temp 需要更新
	Matrix<float, 2, 1> x_;//x先验 需要更新
	Matrix<float, 2, 2>p_;//p先验 需要更新
	
	Matrix<float, 2, 1> x;//修正值 需要更新

	Matrix<float, 2, 2> I;//单位矩阵 不需要更新

	void predict(Point2f& mouse_center);
	Point2f update();
#pragma endregion


#pragma region 匀速版本
	//匀速版本
	float delta_t;//间隔时间

	Matrix<float, 4, 4> v_F;//状态转移 不用更新

	Matrix<float, 4, 4> v_P;//协方差矩阵 需要更新

	//Matrix<float, 4, 4> v_H;//测量矩阵 不用更新     观测量为[x,y,vx,vy]
	Matrix<float, 2, 4> v_H;//测量矩阵 不用更新	观测量为[x,y]

	Matrix<float, 4, 4> v_Q;//状态误差 不用更新

	//Matrix<float, 4, 4> v_R;//观测误差 不用更新
	Matrix<float, 2, 2> v_R;//观测误差 不用更新


	//Matrix<float, 4, 4> v_K;//卡尔曼增益 需要更新
	Matrix<float, 4, 2> v_K;//卡尔曼增益 需要更新

	//Matrix<float, 4, 1> v_now_temp;//本位置temp 需要更新		观测量为[x,y,vx,vy]
	//Matrix<float, 4, 1> v_last_temp;//上一时刻temp 需要更新	观测量为[x,y,vx,vy]

	Matrix<float, 2, 1> v_now_temp;//本位置temp 需要更新	观测量为[x,y]
	Matrix<float, 2, 1> v_last_temp;//上一时刻temp 需要更新	观测量为[x,y]

	Matrix<float, 4, 1> v_x_;//x先验 需要更新
	Matrix<float, 4, 4> v_p_;//p先验 需要更新

	Matrix<float, 4, 1> v_x;//修正值 需要更新

	Matrix<float, 4, 4> v_I;//单位矩阵 不需要更新

	void v_init(Point2f& v_now_mouse_center);//第一帧进来初始化
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
//	//位置版本
//
//	Matrix<float, 2, 2> F;//状态转移 不用更新
//
//	Matrix<float, 2, 2> P;//协方差矩阵 需要更新
//
//	Matrix<float, 2, 2> H;//测量矩阵 不用更新
//
//	Matrix<float, 2, 2> Q;//状态误差 不用更新
//	Matrix<float, 2, 2> R;//观测误差 不用更新
//
//
//	Matrix<float, 2, 2> K;//卡尔曼增益 需要更新
//
//	Matrix<float, 2, 1> now_temp;//本位置temp 需要更新
//	Matrix<float, 2, 1> last_temp;//上一时刻temp 需要更新
//	Matrix<float, 2, 1> x_;//x先验 需要更新
//	Matrix<float, 2, 2>p_;//p先验 需要更新
//
//	Matrix<float, 2, 1> x;//修正值 需要更新
//
//	Matrix<float, 2, 2> I;//单位矩阵 不需要更新
//
//	void predict(Point2f& last_center);
//	Point2f update();
//
//};