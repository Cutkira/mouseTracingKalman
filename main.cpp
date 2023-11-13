#include<iostream>
#include"kalman.h"
#include<math.h>
#include <fstream>��// c++�ļ�����
#include <iomanip>��// ���������ʽ
using namespace std;

Point2f mouse_position = Point2f(0, 0);
Point2f v_now_mouse_position = Point2f(0, 0);
Point2f v_last_mouse_position = Point2f(0, 0);


//���ص�����
void on_mouse(int event, int x, int y, int flags, void* ustc)
{
	if(event== EVENT_MOUSEMOVE) {
		mouse_position = Point2f(x, y);
		v_now_mouse_position = Point2f(x, y);
	}
}


//���txt����
void fileEmpty(const string fileName)
{
	fstream file(fileName, ios::out);
}


//λ�ð汾
void p_run() {
	ofstream mouse_txt;
	ofstream predict_txt;
	mouse_txt.open("./position_mouse.txt", ios::out | ios::trunc);
	mouse_txt << fixed;

	predict_txt.open("./position_predict.txt", ios::out | ios::trunc);
	predict_txt << fixed;

	kalman kf;
	namedWindow("blank");
	setMouseCallback("blank", on_mouse, 0);
	int i = 0;


	while (1)
	{
		Point2f predict_position;
		Mat blank(720, 1280, CV_8UC3);
		circle(blank, mouse_position, 3, Scalar(0, 255, 255), 3, 2);


		kf.predict(mouse_position);
		predict_position = kf.update();

		string mouse_position_text = "mouse_position_text: [" + to_string(mouse_position.x) + "," + to_string(mouse_position.y) + "]";
		string predict_position_text = "predict_position_text: [" + to_string(predict_position.x) + "," + to_string(predict_position.y) + "]";
		//string mouse_position_text = "[" + to_string(mouse_position.x) + "," + to_string(mouse_position.y) + "]";
		//string predict_position_text ="[" + to_string(predict_position.x) + "," + to_string(predict_position.y) + "]";


		circle(blank, kf.update(), 3, Scalar(0, 0, 255), 3, 2);
		putText(blank, mouse_position_text, Point(0, 30), FONT_HERSHEY_PLAIN, 2, Scalar(0, 0, 255), 1);
		putText(blank, predict_position_text, Point(0, 60), FONT_HERSHEY_PLAIN, 2, Scalar(0, 0, 255), 1);
		//putText(blank, mouse_position_text, mouse_position, FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 255), 1);
		//putText(blank, predict_position_text, predict_position, FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);

		mouse_txt << setprecision(4) << mouse_position.x << " " << mouse_position.y << endl;
		predict_txt << setprecision(4) << predict_position.x << " " << predict_position.y << endl;


		line(blank, mouse_position, predict_position, Scalar(255, 0, 0), 2, 1);



		imshow("blank", blank);
		waitKey(1);
	}

	mouse_txt.close();
	predict_txt.close();

}


//��t��ֵ�汾����kalman
void pVDeltaT_run() {
	//-----------------------------------------------------------
	//��t��ֵ�汾����kalman
	kalman kf;
	namedWindow("blank");
	setMouseCallback("blank", on_mouse, 0);
	int i = 0;

	TickMeter time;
	Matrix<float, 1, 2>Time;
	while (1)
	{
		//time.start();//��ʼ��ʱ

		Point2f predict_position;
		Mat blank(720, 1080, CV_8UC3);
		//circle(blank, mouse_position, 3, Scalar(0, 255, 255), 3, 2);

		//time.stop();
		//float process_time = time.getAvgTimeSec();
		//cout << "process time = " << process_time << "s" << endl;
		//time.reset();

		//kf.v_predict(v_now_mouse_position, process_time);
		//predict_position = kf.v_update();

		float pixelX_difference = v_now_mouse_position.x - v_last_mouse_position.x;
		float pixelY_difference = v_now_mouse_position.y - v_last_mouse_position.y;
		float vOfx = pixelX_difference / 2;//delta_tine = 2ms
		float vOfy = pixelY_difference / 2;
		cout << "vOfx = " << vOfx << endl;
		cout << "vOfy = " << vOfy << endl;

		predict_position.x = v_now_mouse_position.x + vOfx * 8;
		predict_position.y = v_now_mouse_position.y + vOfy * 8;


		string v_now_mouse_position_text = "mouse_position_text: [" + to_string(v_now_mouse_position.x) + "," + to_string(v_now_mouse_position.y) + "]";
		string predict_position_text = "predict_position_text: [" + to_string(predict_position.x) + "," + to_string(predict_position.y) + "]";
		//string mouse_position_text = "[" + to_string(mouse_position.x) + "," + to_string(mouse_position.y) + "]";
		//string predict_position_text ="[" + to_string(predict_position.x) + "," + to_string(predict_position.y) + "]";

		circle(blank, v_now_mouse_position, 3, Scalar(0, 255, 255), 3, 2);
		circle(blank, predict_position, 3, Scalar(0, 0, 255), 3, 2);

		putText(blank, v_now_mouse_position_text, Point(0, 30), FONT_HERSHEY_PLAIN, 2, Scalar(0, 0, 255), 1);
		putText(blank, predict_position_text, Point(0, 60), FONT_HERSHEY_PLAIN, 2, Scalar(0, 0, 255), 1);
		//putText(blank, mouse_position_text, mouse_position, FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 255), 1);
		//putText(blank, predict_position_text, predict_position, FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);


		//line(blank, mouse_position, predict_position, Scalar(255, 0, 0), 2, 1);
		v_last_mouse_position = mouse_position;


		imshow("blank", blank);
		waitKey(1);
	}
}


//λ���ٶȰ汾���� �ɹ�
void pVCorrect_run() {
	// ����/�������txt�ļ�
	ofstream mouse_txt;
	ofstream predict_txt;
	ofstream mouseVelocity_txt;
	ofstream predictVelocity_txt;

	mouse_txt.open("./positionVelocity_mouseOfPosition.txt", ios::out | ios::trunc);
	mouse_txt << fixed;
	predict_txt.open("./positionVelocity_predictOfPosition.txt", ios::out | ios::trunc);
	predict_txt << fixed;

	mouseVelocity_txt.open("./positionVelocity_mouseOfVelocity.txt", ios::out | ios::trunc);
	mouseVelocity_txt << fixed;
	predictVelocity_txt.open("./positionVelocity_predictOfVelocity.txt", ios::out | ios::trunc);
	predictVelocity_txt << fixed;

	fileEmpty("positionVelocity_mouseOfPosition.txt");//���txt
	fileEmpty("positionVelocity_predictOfPosition.txt");//���txt
	fileEmpty("positionVelocity_mouseOfVelocity.txt");//���txt
	fileEmpty("positionVelocity_predictOfVelocity.txt");//���txt

	kalman kf;
	namedWindow("blank");
	setMouseCallback("blank", on_mouse, 0);
	int i = 0;

	TickMeter time;
	while (1)
	{
		//if (i == 0) {
		//	kf.v_init(v_now_mouse_position);
		//	i++;
		//}
	    //time.reset();
		//time.start();//��ʼ��ʱ

		Point2f predict_position;//Ԥ��λ��
		Mat blank(720, 1920, CV_8UC3);
		//circle(blank, mouse_position, 3, Scalar(0, 255, 255), 3, 2);

		float process_time = 1;//��50hzΪ����Ƶ�� 20ms 5ms
		//cout << "process time = " << process_time << "s" << endl;

		kf.v_predict(v_now_mouse_position, process_time);
		predict_position = kf.v_update();

		//float pixelX_difference = v_now_mouse_position.x - v_last_mouse_position.x;
		//float pixelY_difference = v_now_mouse_position.y - v_last_mouse_position.y;
		//float vOfx = pixelX_difference / 2;//delta_tine = 2ms
		//float vOfy = pixelY_difference / 2;
		//cout << "vOfx = " << vOfx << endl;
		//cout << "vOfy = " << vOfy << endl;

		//predict_position.x = v_now_mouse_position.x + vOfx * 8;
		//predict_position.y = v_now_mouse_position.y + vOfy * 8;


		string v_now_mouse_position_text = "mouse_position_text: [" + to_string(v_now_mouse_position.x) + "," + to_string(v_now_mouse_position.y) + "]";
		string predict_position_text = "predict_position_text: [" + to_string(predict_position.x) + "," + to_string(predict_position.y) + "]";
		//string mouse_position_text = "[" + to_string(mouse_position.x) + "," + to_string(mouse_position.y) + "]";
		//string predict_position_text ="[" + to_string(predict_position.x) + "," + to_string(predict_position.y) + "]";

		circle(blank, v_now_mouse_position, 3, Scalar(0, 255, 255), 3, 2);
		circle(blank, predict_position, 3, Scalar(0, 0, 255), 3, 2);

		putText(blank, v_now_mouse_position_text, Point(0, 30), FONT_HERSHEY_PLAIN, 2, Scalar(0, 0, 255), 1);
		putText(blank, predict_position_text, Point(0, 60), FONT_HERSHEY_PLAIN, 2, Scalar(0, 0, 255), 1);
		//putText(blank, mouse_position_text, mouse_position, FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 255), 1);
		//putText(blank, predict_position_text, predict_position, FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);


		//line(blank, mouse_position, predict_position, Scalar(255, 0, 0), 2, 1);
		v_last_mouse_position = v_now_mouse_position;
		//λ��д��
		mouse_txt << setprecision(4) << v_now_mouse_position.x << " " << v_now_mouse_position.y << endl;
		predict_txt << setprecision(4) << predict_position.x << " " << predict_position.y << endl;
		//�ٶ�д��
		//mouseVelocity_txt << setprecision(4) << kf.v_now_temp(2, 0) << " " << kf.v_now_temp(3, 0) << endl;
		mouseVelocity_txt << setprecision(4) << 0 << " " << 0 << endl;
		predictVelocity_txt << setprecision(4) << kf.v_x(2, 0) << " " << kf.v_x(3, 0) << endl;

		imshow("blank", blank);
		waitKey(1);


		//time.stop();
		//float process_time_1 = time.getTimeMilli();
		//cout << "process_time = " << process_time_1 << endl;
	}
	mouse_txt.close();
	predict_txt.close();
	mouseVelocity_txt.close();
	predictVelocity_txt.close();
}



template<int V_Z = 1, int V_X = 3>
class Kalman {
public:
	using Matrix_zzd = Eigen::Matrix<double, V_Z, V_Z>;
	using Matrix_xxd = Eigen::Matrix<double, V_X, V_X>;
	using Matrix_zxd = Eigen::Matrix<double, V_Z, V_X>;
	using Matrix_xzd = Eigen::Matrix<double, V_X, V_Z>;
	using Matrix_x1d = Eigen::Matrix<double, V_X, 1>;
	using Matrix_z1d = Eigen::Matrix<double, V_Z, 1>;

private:
	Matrix_x1d x_k1; // k-1ʱ�̵��˲�ֵ������k-1ʱ�̵�ֵ	3*1
	Matrix_xzd K;    // Kalman����	3*1
	Matrix_xxd A;    // ת�ƾ���	3*3
	Matrix_zxd H;    // �۲����	1*3
	Matrix_xxd R;    // Ԥ���������ƫ��ķ���	3*3
	Matrix_zzd Q;    // ��������ƫ�(ϵͳ����Ժ�ͨ������ͳ��ʵ����)	1*1
	Matrix_xxd P;    // �������Э����	3*3
	double last_t{ 0 };

public:
	Kalman() = default;

	Kalman(Matrix_xxd A, Matrix_zxd H, Matrix_xxd R, Matrix_zzd Q, Matrix_x1d init, double t) {
		reset(A, H, R, Q, init, t);
	}

	void reset(Matrix_xxd A, Matrix_zxd H, Matrix_xxd R, Matrix_zzd Q, Matrix_x1d init, double t) {
		this->A = A;
		this->H = H;
		this->P = Matrix_xxd::Zero();
		this->R = R;
		this->Q = Q;
		x_k1 = init;
		last_t = t;
	}

	void reset(Matrix_x1d init, double t) {
		x_k1 = init;
		last_t = t;
	}

	void reset(double x, double t) {
		x_k1(0, 0) = x;
		last_t = t;
	}

	Matrix_x1d update(Matrix_z1d z_k, double t) {
		// ����ת�ƾ����е�ʱ����
		for (int i = 1; i < V_X; i++) {
			A(i - 1, i) = t - last_t;
		}
		last_t = t;

		// Ԥ����һʱ�̵�ֵ
		Matrix_x1d p_x_k = A * x_k1;   //x�������������һ��ʱ���ĺ������ֵ��������Ϣ�������˴���Ҫ���ݻ�վ�߶���һ���޸�

		//��Э����
		P = A * P * A.transpose() + R;  //������������� p(n|n-1)=A^2*p(n-1|n-1)+q

		//����kalman����
		K = P * H.transpose() * (H * P * H.transpose() + Q).inverse();  //Kg(k)= P(k|k-1) H�� / (H P(k|k-1) H�� + R)
		//����������������˲�ֵ
		x_k1 = p_x_k +
			K * (z_k - H * p_x_k);  //���ò������Ϣ���ƶ�x(t)�Ĺ��ƣ�����������ƣ����ֵҲ�������  X(k|k)= X(k|k-1)+Kg(k) (Z(k)-H X(k|k-1))
	 //���º������
		P = (Matrix_xxd::Identity() - K * H) * P;//������������  P[n|n]=(1-K[n]*H)*P[n|n-1]

		return x_k1;
	}
};



//-----------------------------------------------------------
//������
int main() {
	pVCorrect_run();
	//double q_[4];
	//for (int i = 0; i <= 3; i++) {
	//	q_[i] = i;
	//}
	//Eigen::Quaternionf q_raw(q_[0], q_[1], q_[2], q_[3]);
	//Eigen::Quaternionf q(q_raw.matrix().transpose());
	//Eigen::Matrix3d R_IW = q.matrix().cast<double>();
	//
	//Mat tVec = (cv::Mat_<double>(3, 1) << 10, 20, 30);
	//float cam_x = tVec.at<double>(0);
	//float cam_y = tVec.at<double>(1);
	//float cam_z = tVec.at<double>(2);
	//cout << "tVec:" << tVec << endl;

	//Eigen::Vector3d pc;
	//cv::cv2eigen(tVec, pc);
	//cout << "pc = "<< pc << endl;

	//Eigen::Vector3d m_pc = pc;         // point camera: Ŀ�����������ϵ�µ�����
	//Eigen::Matrix3d R_CI;           // ����������ϵ���������ϵ��ת����EIGEN-Matrix
	//R_CI << 1.000000e+00, 1.197624e-11, 1.704639e-10,
	//	1.197625e-11, 1.000000e+00, 3.562503e-10,
	//	1.704639e-10, 3.562503e-10, 1.000000e+00;
	//auto R_WC = (R_CI * R_IW).transpose();
	//Eigen::Vector3d m_pw = R_WC * pc;
	//cout << "m_pw = " << m_pw<<endl;
	//double m_yaw = std::atan2(m_pw(1, 0), m_pw(0, 0));   // yaw�Ĳ���ֵ����λ����   atan2(y/x)
	//std::cout << "m_yaw=" << m_yaw * 180. / CV_PI << std::endl;

	//int a = 100;
	//if (a > 10) {
	//	cout << 1 << endl;
	//}
	//else if (a > 20) {
	//	cout << 2 << endl;
	//}


	return 0;

}




//λ���ٶȰ汾����
void pVbackup_run()
{
//-----------------------------------------------------------
//λ���ٶȰ汾���� �ɹ�
		// ����/�������txt�ļ�
	ofstream mouse_txt;
	ofstream predict_txt;
	mouse_txt.open("./positionVelocity_mouseOfPosition.txt", ios::out | ios::trunc);
	mouse_txt << fixed;

	predict_txt.open("./positionVelocity_predictOfPosition.txt", ios::out | ios::trunc);
	predict_txt << fixed;

	fileEmpty("positionVelocity_mouseOfPosition.txt");//���txt
	fileEmpty("positionVelocity_predictOfPosition.txt");//���txt



	kalman kf;
	namedWindow("blank");
	setMouseCallback("blank", on_mouse, 0);
	int i = 0;

	TickMeter time;
	while (1)
	{
		if (i == 0) {
			kf.v_init(v_now_mouse_position);
			i++;
		}
		//time.start();//��ʼ��ʱ

		Point2f predict_position;//Ԥ��λ��
		Mat blank(720, 1080, CV_8UC3);
		//circle(blank, mouse_position, 3, Scalar(0, 255, 255), 3, 2);

		//time.stop();
		//float process_time = time.getAvgTimeSec();
		float process_time = 20;//��50hzΪ����Ƶ�� 20ms
		//cout << "process time = " << process_time << "s" << endl;
		//time.reset();

		kf.v_predict(v_now_mouse_position, process_time);
		predict_position = kf.v_update();


		string v_now_mouse_position_text = "mouse_position_text: [" + to_string(v_now_mouse_position.x) + "," + to_string(v_now_mouse_position.y) + "]";
		string predict_position_text = "predict_position_text: [" + to_string(predict_position.x) + "," + to_string(predict_position.y) + "]";

		circle(blank, v_now_mouse_position, 3, Scalar(0, 255, 255), 3, 2);
		circle(blank, predict_position, 3, Scalar(0, 0, 255), 3, 2);

		putText(blank, v_now_mouse_position_text, Point(0, 30), FONT_HERSHEY_PLAIN, 2, Scalar(0, 0, 255), 1);
		putText(blank, predict_position_text, Point(0, 60), FONT_HERSHEY_PLAIN, 2, Scalar(0, 0, 255), 1);


		//line(blank, mouse_position, predict_position, Scalar(255, 0, 0), 2, 1);
		v_last_mouse_position = v_now_mouse_position;
		mouse_txt << setprecision(4) << v_now_mouse_position.x << " " << v_now_mouse_position.y << endl;
		predict_txt << setprecision(4) << predict_position.x << " " << predict_position.y << endl;

		imshow("blank", blank);
		waitKey(1);
	}
	mouse_txt.close();
	predict_txt.close();


}