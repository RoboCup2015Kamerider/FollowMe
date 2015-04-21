#include "ControllerEvent.h"  
#include "Controller.h"  
#include "Logger.h"  

#include <iostream>
#include <fstream>

#include <ViewImage.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
//#include <opencv/cv.h>
//#include <opencv/cxcore.h>
//#include <opencv/cvaux.h>

#define PI 3.1415926535797
#define TIMEOUT 0.01
#define VEL 3.0

#define __DEBUG_

using namespace std;

void __SHOW(const char* msg)
{
#ifdef __DEBUG
	cout << "\r" << msg << endl;
#endif
}

class Coordinate
{
public:
	double x;
	double z;
public:
	Coordinate(){}
	Coordinate(double x, double z)
	{
		this->x = x;
		this->z = z;
	}
};

class MyController : public Controller 
{  
public:  
	// basic function
  	void onInit(InitEvent &evt);  
  	double onAction(ActionEvent&);  
  	void onRecvMsg(RecvMsgEvent &evt); 
  	void onCollision(CollisionEvent &evt);
  	
public:	
	// basic param
  	RobotObj *m_pRO_robot;
  	ViewService *m_pVS_view;
	
	double m_d_wheelRadius;				// 机器人轮子半径
	double m_d_wheelDistance;			// 机器人两轮间距离
	
	int m_i_coat_rmin;
	int m_i_coat_rmax;
	int m_i_coat_gmin;
	int m_i_coat_gmax;
	int m_i_coat_bmin;
	int m_i_coat_bmax; 
	
public:
	// global signal
	bool m_b_start;					// 开始标志，接收 Task start 开始
	bool m_b_getColor;
	bool m_b_move;
	bool m_b_goon_move;
	
	bool m_b_elevator;
	
public:
	// sensor signal
	bool m_b_getCoat1;
	bool m_b_getCoat2;
	bool m_b_getCoat3;
	bool m_b_getCoat4;
	
	bool m_b_getDis1;
	bool m_b_getDis2;
	bool m_b_getDis3;
	bool m_b_getDis4;
	
	bool m_b_moveLeft;
	bool m_b_moveRight;
	bool m_b_turnLeft;
	bool m_b_turnRight;
	
	bool m_b_control1;
	bool m_b_control2;
	
	bool m_b_center3;
	bool m_b_center2;
	bool m_b_center1;
	bool m_b_center4;
	
	bool m_b_getLine2;
	bool m_b_getLine3;
public:

	// state control
	int m_i_state;
	int m_i_call_state;
	vector<int> m_vi_state;
	int m_i_time;
	
public:
	// temp data
	Coordinate m_Co_man;
	
	
	double m_d_vel;
	double m_d_center;
	
	double m_d_left;
	double m_d_right;
	
	double m_d_min2;
	double m_d_min3;
	
	double m_d_mean2;
	double m_d_mean3;
	
	double m_d_fx2;
	double m_d_fy2;
	
	int m_i_min2;
	int m_i_min3;
	
	int m_i_dis;
	int m_i_dangerous;
	
	double m_d_last;
	
	vector<CvPoint> m_vCP_coat1;
	vector<CvPoint> m_vCP_coat2;
	vector<CvPoint> m_vCP_coat3;
	vector<CvPoint> m_vCP_coat4;
	
	vector<Coordinate> m_vCo_R_op;
	
	ofstream data;
	
public:
	// call function
	void GETCOAT(int camID)
	{
		m_i_state = camID * 10 + 1;
	}
	void GETCOOR(int camID)
	{
		m_i_state = camID * 10 + 2;
	}
public:
	// aux function
	void init()
	{
		
	
	m_b_start = false;
	
	m_i_state = 0;
	m_i_call_state = 0;
	m_vi_state.push_back(m_i_state);
	m_vi_state.push_back(m_i_state);
	
	
	m_b_getColor = false;
	m_b_move = false;
	m_b_elevator = false;
	
	m_i_coat_rmin = 255;
	m_i_coat_rmax = 0;
	m_i_coat_gmin = 255;
	m_i_coat_gmax = 0;
	m_i_coat_bmin = 255;
	m_i_coat_bmax = 0;
	
	m_b_getCoat1 = false;
	m_b_getCoat2 = false;
	m_b_getCoat3 = false;
	m_b_getCoat4 = false;
	
	m_b_getDis1 = false;
	m_b_getDis2 = false;
	m_b_getDis3 = false;
	m_b_getDis4 = false;
	
	m_b_moveLeft = false;
	m_b_moveRight = false;
	m_b_turnLeft = false;
	m_b_turnRight = false;
	
	m_b_control1 = false;
	m_b_control2 = false;
	
	m_b_center3 = false;
	m_b_center2 = false;
	m_b_center1 = false;
	m_b_center4 = false;
	
	m_b_getLine2 = false;
	m_b_getLine3 = false;
	
	m_b_goon_move = false;
	
	m_d_left = 319;
	m_d_right = 0;
	
	m_d_last = m_d_left + m_d_right;
	
	m_d_fx2 = 0;
	m_d_fy2 = 0;

	m_i_dis = -1;
	
	m_i_time = 0;
	
	m_i_dangerous = 0;
	
	data.open("data.txt");
	}
	void getDisImage(IplImage* src, int camID, double d_max = 255);
	void getDisLine(char* src, int camID, double d_max = 255);
	
	void getDisMin(char* src, double &minDis, int &minIdx, double& meanDis);
	void getForce(char* src, double& d_fx, double& d_fy)
	{
		double k = 1;
		double T = PI / 3;
		double Fx = 0;
		double Fy = 0;
		double r2 = 1;
		for(int i = 0; i < 320; i++)
		{
			double d_r = int((unsigned char)src[i]);
			if(d_r == 255)
			{
				r2 = -1e5;
			}
			else
			{
				r2 = d_r * d_r;
			}
			double theta = T * (double(i) - 160.0) / 320.0;
			//cout << theta << endl;
			double fx = -sin(theta);
			//cout << fx << endl;
			double fy = -cos(theta);
			Fx += k / r2 * fx;
			Fy += k / r2 * fy;
		}
		cout << "Fx = " << Fx << "\t" << Fy << endl;
		d_fx = Fx;
		d_fy = Fy;
	}
	
	void getColorImage(IplImage* src, int camID);
	void getCoatVector(IplImage* src, vector<CvPoint> &vCP_coat);
	void getCoatMeanDis(IplImage* src, vector<CvPoint> &vCP_coat, double &robot_x, double &robot_z);
};  
  
void MyController::onInit(InitEvent &evt) 
{  
	
	m_pRO_robot = getRobotObj(myname());		// 得到机器人指针
	m_d_wheelRadius = 10.0;
	m_d_wheelDistance = 10.0;
	m_pRO_robot->setWheel(m_d_wheelRadius, m_d_wheelDistance);
	
	m_d_vel = 0;
	m_d_center = 0;
	init();
	m_pVS_view = (ViewService*)connectToService("SIGViewer");
}  
  
double MyController::onAction(ActionEvent &evt) 
{  
	/*////////////////////////////
	m_i_state
	0 --> controller
	
	1 --> init coat color
	11 --> get vector cvpoint of coat
	12 --> get distance of the coat
	
	-10 --> vel control
	
	100 --> aux control
	////////////////////////////*/
	m_i_state = m_vi_state[m_vi_state.size() - 1];
	if(m_i_state == 0)
	{
		__SHOW("STATE\t0");
		if(m_b_getColor == false)
		{
			m_vi_state.push_back(1);
		}
		else if(m_b_getCoat2 == false)
		{
			m_vi_state.push_back(21);
		}
		else if(m_b_getCoat3 == false)
		{
			m_vi_state.push_back(31);
		}
		else 
		{
			if(m_vCP_coat2.size() != 0 && m_b_center2 == false)
			{
				m_vi_state.push_back(25);
			}
			else if(m_vCP_coat3.size() != 0 && m_b_center3 == false)
			{
				m_vi_state.push_back(35);
			}
			else if(m_vCP_coat2.size() == 0 && m_vCP_coat3.size() == 0 && m_i_time * TIMEOUT > 5)
			{
				if(m_b_getCoat1 == false && m_b_elevator == true)
				{
					m_vi_state.push_back(11);
				}
				else
				{ 
					if(m_vCP_coat1.size() != 0 && m_b_elevator == true)
					{
						m_pRO_robot->setWheelVelocity(-PI / 4, PI / 4);
						sleep(1.0);
						m_pRO_robot->setWheelVelocity(0, 0);
						m_b_getCoat1 = false;
						m_vCP_coat1.clear();
						m_b_center2 = false;
						m_b_center3 = false;
						m_b_getCoat2 = false;
						m_b_getCoat3 = false;
						m_vCP_coat2.clear();
						m_vCP_coat3.clear();
						m_d_left = 319;
						m_d_right = 0;
					}
					else if(m_b_getCoat4 == false && m_b_elevator == true)
					{
						m_vi_state.push_back(41);
					}
					else 
					{
						if(m_vCP_coat4.size() != 0 && m_b_elevator == true)
						{
							m_pRO_robot->setWheelVelocity(PI / 4, -PI / 4);
							sleep(1.0);
							m_pRO_robot->setWheelVelocity(0, 0);
							m_b_getCoat4 = false;
							m_vCP_coat4.clear();
						}
						m_b_getCoat1 = false;
						m_b_getCoat4 = false;
						m_b_center2 = false;
						m_b_center3 = false;
						m_b_getCoat2 = false;
						m_b_getCoat3 = false;
						m_vCP_coat1.clear();
						m_vCP_coat4.clear();
						m_vCP_coat2.clear();
						m_vCP_coat3.clear();
						m_d_left = 319;
						m_d_right = 0;
						
					}
				}
			}
			/*else
			{
				//m_b_getLine2 = false;
				//m_b_getLine3 = false;
				m_b_getCoat1 = false;
				m_b_getCoat4 = false;
				m_b_center2 = false;
				m_b_center3 = false;
				m_b_getCoat2 = false;
				m_b_getCoat3 = false;
				m_vCP_coat1.clear();
				m_vCP_coat4.clear();
				m_vCP_coat2.clear();
				m_vCP_coat3.clear();
				m_d_left = 319;
				m_d_right = 0;
			}*/
			else
			{
				if(m_b_getLine2 == false)
				{
					m_vi_state.push_back(23);
				}
				else if(m_b_getLine3 == false)
				{
					m_vi_state.push_back(33);
				}
				else
				{
					//data << m_i_min2 << "\t" << m_d_min2 << "\t" << m_i_min3 << "\t" << m_d_min3 << "\t" << endl;
					double d_sum = m_d_left + m_d_right;
					double Kp = 0.00054;
					double Kd = 0.00064;
					
					double d_xfx = (d_sum - 320.0) / 640.0;
					double d_xfy = fabs(m_d_left - m_d_right) / 320.0;
					double d_r = fabs(m_d_left - m_d_right) + 0.00001;
					
					double d_xFx = 0.6 * d_xfx / d_r / d_r;
					double d_xFy = 0.6 * d_xfy / d_r / d_r;
					
					double d_sumFx = d_xFx + m_d_fx2;
					double d_sumFy = d_xFy + m_d_fy2;
				
					cout << "\t\tSum F = " << d_sumFx << "\t" << d_sumFy << endl;
					
					double Ks = 0.0005;
					double d_s = m_d_min2 * (exp((319 - m_i_min2) / 160) / 2 + 0.5);
					
					double d_d = fabs(m_d_left + m_d_right) - m_d_last;
			
					double d_org_vel = 3.5 - fabs(m_d_left - m_d_right) * 5.5 / 320;
					double d_left_vel = Kp * (d_sum - 319) + d_org_vel + Kd * d_d;// - Ks * (d_sum - 319 - 50);
					double d_right_vel = -Kp * (d_sum - 319) + d_org_vel - Kd * d_d;// - Ks * (d_sum - 319 - 50);
					if(m_b_elevator == true && m_i_time * TIMEOUT > 5)
					{
						double Kf = 40.0;
						double vy = (d_left_vel + d_right_vel) / 2;
						double vx = (d_left_vel - d_right_vel) / 2;
						vy += Kf * d_sumFy;
						vx += Kf * d_sumFx;
						d_left_vel = vx + vy;
						d_right_vel = vy - vx;
					}
			
					m_d_last = m_d_left + m_d_right;
					
			
					m_pRO_robot->setWheelVelocity(d_left_vel, d_right_vel);

					m_b_getCoat1 = false;
					m_b_getCoat4 = false;
					m_b_getLine2 = false;
					m_b_getLine3 = false;
					m_b_center2 = false;
					m_b_center3 = false;
					m_b_getCoat2 = false;
					m_b_getCoat3 = false;
					m_vCP_coat2.clear();
					m_vCP_coat3.clear();
					m_vCP_coat1.clear();
					m_vCP_coat4.clear();
					m_d_left = 319;
					m_d_right = 0;
				}
			}
		}
		
	}
	else if(m_i_state == 1000)
	{
		cout << "elevator deal" << endl;
		broadcastMsg("Door_close");
		m_vi_state.pop_back();
		m_vi_state.push_back(1100);
		return 5.0;
	}
	else if(m_i_state == 1100)
	{
		cout << "in 1100" << endl;
		m_pRO_robot->setWheelVelocity(-10, -10);
		m_vi_state.pop_back();
		m_vi_state.push_back(1200);
		return 4.0;	
	}
	else if(m_i_state == 1200)
	{
		cout << "in 1200" << endl;
		m_pRO_robot->setWheelVelocity(0, 0);
		broadcastMsg("Get_off");
		m_vi_state.pop_back();
		m_vi_state.push_back(1300);
		return 11.0;
	}
	else if(m_i_state == 1300)
	{
		cout << "in 1300" << endl;
		m_vi_state.pop_back();
		m_vi_state.push_back(0);
		m_b_getCoat1 = false;
					m_b_getCoat4 = false;
					m_b_getLine2 = false;
					m_b_getLine3 = false;
					m_b_center2 = false;
					m_b_center3 = false;
					m_b_getCoat2 = false;
					m_b_getCoat3 = false;
					m_vCP_coat2.clear();
					m_vCP_coat3.clear();
					m_vCP_coat1.clear();
					m_vCP_coat4.clear();
					m_d_left = 319;
					m_d_right = 0;
	}
	
	
	//return TIMEOUT;
	
	/*m_i_state = m_vi_state[m_vi_state.size() - 1];
	//m_vi_state.pop_back();
	// core controller
	if(m_i_state == 0)
	{
		m_i_call_state = 0;
		__SHOW("STATE\t0");
		if(m_b_getColor == false)
		{
			m_vi_state.push_back(1);
		}
		else if(m_b_getCoat2 == false)
		{
			if(m_b_goon_move == false)
			{
				m_pRO_robot->setWheelVelocity(0, 0);
			}
			else
			{
				m_pRO_robot->setWheelVelocity(2, 2);
			}
			m_vi_state.push_back(21);
		}
		else 
		{
			if(m_vCP_coat2.size() != 0)
			{
				if(m_b_center2 == false)
				{
					m_vi_state.push_back(25);
				}
				
				else if(m_b_getDis2 == false)
				{
					if(fabs(m_d_center) > 10.0)
					{
						cout << "============gooon=================" << endl;
						m_b_goon_move = true;
					}
					m_vi_state.push_back(22);
				}
				else if(m_b_control1 == false)
				{
					m_vi_state.push_back(100);
				}
				else if(m_b_move == false)
				{
					m_vi_state.push_back(-10);
				}
				else
				{
					m_b_getCoat2 = false;
					m_vCP_coat2.clear();
					m_b_getDis2 = false;
					m_b_control1 = false;
					m_b_move = false;
					m_b_center2 = false;
					m_vi_state.push_back(0);
				}
			}
			else
			{
				m_b_getCoat2 = false;
				m_vCP_coat2.clear();
				m_b_getDis2 = false;
				m_b_control1 = false;
				m_b_move = false;
				m_b_center2 = false;
				m_vi_state.push_back(140);
			}
		}
	}
	
	// aux control
	
	else if(m_i_state == 110)	// turn left
	{
		m_i_call_state = 110;
		__SHOW("STATE\t110");
		if(m_b_control2 == false)
		{
			m_vi_state.push_back(400);
		}
		else if(m_b_move == false)
		{
			m_vi_state.push_back(-20);
		}
		else
		{
			//if(m_b_goon_move == false)
			{
				m_pRO_robot->setWheelVelocity(0, 0);
			}
			//else
			{
			//	m_pRO_robot->setWheelVelocity(2, 2);
			}
			if(m_b_getCoat2 == false)
			{
				m_vi_state.push_back(21);
			}
			else
			{
				if(m_vCP_coat2.size() != 0)
				{
					if(m_b_center2 == false)
					{
						m_vi_state.push_back(25);
					}
					else if(fabs(m_d_center) > 5)
					{
						data << m_d_center << endl;
						m_b_control2 = false;
						m_b_move = false;
						m_b_getCoat2 = false;
						m_vCP_coat2.clear();
						m_b_center2 = false;
						m_vi_state.push_back(110);
					}
					else
					{
						m_b_control2 = false;
						m_b_move = false;
						m_b_getCoat2 = false;
						m_vCP_coat2.clear();
						m_b_center2 = false;
						m_vi_state.push_back(0);
					}
				}
				else
				{
					m_b_control2 = false;
					m_b_move = false;
					m_b_getCoat2 = false;
					m_vCP_coat2.clear();
					m_b_center2 = false;
					m_vi_state.push_back(110);
				}
			}
		}
	}
	else if(m_i_state == 120)	// turn right
	{
		m_i_call_state = 120;
		__SHOW("STATE\t120");
		if(m_b_control2 == false)
		{
			m_pRO_robot->setWheelVelocity(PI / 4, -PI / 4);
			m_b_control2 = true;
			return 1.0;
		}
		else
		{
			m_pRO_robot->setWheelVelocity(0, 0);
			m_b_control2 = false;
			m_vi_state.push_back(0);
		}
		/*if(m_b_control2 == false)
		{
			m_vi_state.push_back(400);
		}
		else if(m_b_move == false)
		{
			m_vi_state.push_back(-20);
		}
		else
		{
			//if(m_b_goon_move == false)
			{
				m_pRO_robot->setWheelVelocity(0, 0);
			}
			//else
			{
			//	m_pRO_robot->setWheelVelocity(2, 2);
			}
			if(m_b_getCoat2 == false)
			{
				m_vi_state.push_back(21);
			}
			else
			{
				if(m_vCP_coat2.size() != 0)
				{
					if(m_b_center2 == false)
					{
						m_vi_state.push_back(25);
					}
					else if(fabs(m_d_center) > 5)
					{
						data << m_d_center << endl;
						m_b_control2 = false;
						m_b_move = false;
						m_b_getCoat2 = false;
						m_vCP_coat2.clear();
						m_b_center2 = false;
						m_vi_state.push_back(120);
					}
					else
					{
						m_b_control2 = false;
						m_b_move = false;
						m_b_getCoat2 = false;
						m_vCP_coat2.clear();
						m_b_center2 = false;
						m_vi_state.push_back(0);
					}
				}
				else
				{
					m_b_control2 = false;
					m_b_move = false;
					m_b_getCoat2 = false;
					m_vCP_coat2.clear();
					m_b_center2 = false;
					m_vi_state.push_back(120);
				}
			}
		}
	}
	else if(m_i_state == 130) // left
	{
		m_i_call_state = 130;
		__SHOW("STATE\t130");
		if(m_b_getCoat1 == false)
		{
			if(m_b_goon_move == false)
			{
				m_pRO_robot->setWheelVelocity(0, 0);
			}
			else
			{
				m_pRO_robot->setWheelVelocity(2, 2);
			}
			m_vi_state.push_back(11);
		}
		else if(m_vCP_coat1.size() != 0)
		{
			if(m_b_center1 == false)
			{
				m_vi_state.push_back(15);
			}
			else
			{
				if(fabs(m_d_center) > 5)
				{
					if(fabs(m_d_center) < 40)
					{
						m_b_goon_move = false;
					}
					cout << "\t center = " << m_d_center << endl;
					if(m_b_control1 == false)
					{
						m_vi_state.push_back(100);
					}
					else if(m_b_move == false)
					{
						m_vi_state.push_back(-10);
					}
					else
					{
						m_b_getCoat1 = false;
						m_vCP_coat1.clear();
						m_b_center1 = false;
						m_b_control1 = false;
						m_b_move = false;
						m_vi_state.push_back(130);
					}
				}
				else
				{
					m_b_getCoat1 = false;
					m_vCP_coat1.clear();
					m_b_center1 = false;
					m_b_control1 = false;
					m_b_move = false;
					m_d_center = -160;
					m_vi_state.push_back(110);
				}
			}
		}
		else
		{
			m_b_getCoat1 = false;
			m_vCP_coat1.clear();
			m_b_center1 = false;
			m_b_control1 = false;
			m_b_move = false;
			m_vi_state.push_back(0);
		}
	}
	else if(m_i_state == 140) // right
	{
		m_i_call_state = 140;
		__SHOW("STATE\t140");
		if(m_b_getCoat4 == false)
		{
			if(m_b_goon_move == false)
			{
				m_pRO_robot->setWheelVelocity(0, 0);
			}
			else
			{
				m_pRO_robot->setWheelVelocity(2, 2);
			}
			m_vi_state.push_back(41);
		}
		else if(m_vCP_coat4.size() != 0)
		{
			if(m_b_center4 == false)
			{
				m_vi_state.push_back(45);
			}
			else 
			{
				if(fabs(m_d_center) > 5)
				{
					if(fabs(m_d_center) < 40)
					{
						m_b_goon_move = false;
					}
					if(m_b_control1 == false)
					{
						m_vi_state.push_back(100);
					}
					else if(m_b_move == false)
					{
						m_vi_state.push_back(-10);
					}
					else
					{
						m_b_getCoat4 = false;
						m_vCP_coat4.clear();
						m_b_center4 = false;
						m_b_control1 = false;
						m_b_move = false;
						m_vi_state.push_back(140);
					}
				}
				else
				{
					m_b_getCoat4 = false;
					m_vCP_coat4.clear();
					m_b_center4 = false;
					m_b_control1 = false;
					m_b_move = false;
					m_d_center = 160;
					m_vi_state.push_back(120);
				}
			}
		}
		else
		{
			m_b_getCoat4 = false;
			m_vCP_coat4.clear();
			m_b_center4 = false;
			m_b_control1 = false;
			m_b_move = false;
			m_vi_state.push_back(130);
		}
	}*/
	
	else if(m_i_state == 100)		// move forward
	{
		__SHOW("STATE\t100");
		double d_dis = m_Co_man.z;
		double K = 0.7;
		double d_vel = d_dis / m_d_wheelRadius;
		//d_vel = d_vel > 3.5 ? 3.5 : d_vel;
		m_d_vel = d_vel;
		m_b_control1 = true;
		m_vi_state.pop_back();
	}
	else if(m_i_state == 400)		// move forward
	{
		__SHOW("STATE\t400");
		double K = 0.1;
		double d_vel = K * m_d_center / m_d_wheelRadius;
		m_d_vel = d_vel > 1.5 ? 1.5 : d_vel;
		m_d_vel = d_vel < -1.5 ? -1.5 : d_vel;
		cout << "400 d_vel = "  << m_d_center << "\t" << m_d_vel << endl;
		m_b_control2 = true;
		m_vi_state.pop_back();
	}
	// move
	else if(m_i_state == -20)
	{
		__SHOW("STATE\t-20");
		m_pRO_robot->setWheelVelocity(m_d_vel, -m_d_vel);
		m_b_move = true;
		m_vi_state.pop_back();
	}
	else if(m_i_state == -10)
	{
		__SHOW("STATE\t-10");
		m_pRO_robot->setWheelVelocity(m_d_vel, m_d_vel);
		m_b_move = true;
		m_vi_state.pop_back();
	}
	
	// get coordinate dis

	else if(m_i_state == 23)
	{
		__SHOW("STATE\t23");
		char* pd_dis = new char[320];
		getDisLine(pd_dis, 2);
		double d_min;
		int i_min;
		getDisMin(pd_dis, d_min, i_min, m_d_mean2);
		double d_thres = 140;
		if(m_b_elevator == true)
		{
			getForce(pd_dis, m_d_fx2, m_d_fy2);
			d_thres = 60;
		}
		if(m_d_mean2 < d_thres)
		{
			m_i_dangerous++;
			if(m_i_dangerous > 3 && m_b_elevator == false)
			{
				cout << "In the elevator" << endl;
				m_pRO_robot->setWheelVelocity(0, 0);
				m_d_min2 = d_min;
				m_i_min2 = i_min;
		
				m_b_getLine2 = true;
				m_vi_state.pop_back();
				m_vi_state.push_back(1000);
				m_b_elevator = true;
				return TIMEOUT;
			}
			else if(m_b_elevator == false)
			{
				cout << "Dangerous" << endl;
				m_pRO_robot->setWheelVelocity(-2, -2);
				sleep(1.0);
				m_pRO_robot->setWheelVelocity(0, 0);
				sleep(5);
			}
			else
			{
				cout << "Dangerous" << endl;
				m_pRO_robot->setWheelVelocity(-2, -2);
				sleep(1.0);
				m_pRO_robot->setWheelVelocity(0, 0);
			}
		}
		m_d_min2 = d_min;
		m_i_min2 = i_min;
		
		m_b_getLine2 = true;
		m_vi_state.pop_back();
	}
	else if(m_i_state == 33)
	{
		__SHOW("STATE\t33");
		char* pd_dis = new char[320];
		getDisLine(pd_dis, 3);
		double d_min;
		int i_min;
		getDisMin(pd_dis, d_min, i_min, m_d_mean3);
		double d_thres = 140;
		if(m_b_elevator == true)
		{
			d_thres = 60;
		}
		if(m_d_mean3 < d_thres)
		{
			m_i_dangerous++;
			if(m_i_dangerous > 3 && m_b_elevator == false)
			{
				cout << "In the elevator" << endl;
				m_d_min3 = d_min;
				m_i_min3 = i_min;
				
				m_b_getLine3 = true;
				m_vi_state.pop_back();
				m_vi_state.push_back(1000);
				m_b_elevator = true;
				return TIMEOUT;
			}
			else if(m_b_elevator == false)
			{
				cout << "Dangerous" << endl;
				m_pRO_robot->setWheelVelocity(-2, -2);
				sleep(1.0);
				m_pRO_robot->setWheelVelocity(0, 0);
				sleep(5);
			}
			else
			{
				cout << "Dangerous" << endl;
				m_pRO_robot->setWheelVelocity(-2, -2);
				sleep(1.0);
				m_pRO_robot->setWheelVelocity(0, 0);
			}
			
		}
		m_d_min3 = d_min;
		m_i_min3 = i_min;
				
		m_b_getLine3 = true;
		m_vi_state.pop_back();
	}
	else if(m_i_state == 42)
	{
		__SHOW("STATE\t42");
		IplImage* ip_dis = cvCreateImage(cvSize(320, 240), 8, 1);
		getDisImage(ip_dis, 4);
		double mean_x, mean_z;
		getCoatMeanDis(ip_dis, m_vCP_coat4, mean_x, mean_z);
		
		mean_x = -(mean_z + 15);
		mean_z = -mean_x;
		
		m_i_dis = mean_z;
		
		m_Co_man.x = mean_x;
		m_Co_man.z = mean_z;
		
		cout << "mean_dis = " << mean_z << "\t" << mean_x << endl;
		cvReleaseImage(&ip_dis);
		
		m_vCP_coat4.clear();
		m_b_getCoat4 = false;
		m_b_getDis4 = true;
		m_i_state = m_i_call_state;
	}
	else if(m_i_state == 32)
	{
		__SHOW("STATE\t32");
		IplImage* ip_dis = cvCreateImage(cvSize(320, 240), 8, 1);
		getDisImage(ip_dis, 3);
		double mean_x, mean_z;
		getCoatMeanDis(ip_dis, m_vCP_coat3, mean_x, mean_z);
		
		mean_x = -(mean_x + 15);
		mean_z = mean_z + 15;
		
		m_Co_man.x = mean_x;
		m_Co_man.z = mean_z;
		
		m_i_dis = mean_z;
		
		cout << "mean_dis = " << mean_z << "\t" << mean_x << endl;
		cvReleaseImage(&ip_dis);
		
		m_vCP_coat3.clear();
		m_b_getCoat3 = false;
		m_b_getDis3 = true;
		m_vi_state.pop_back();
	}
	else if(m_i_state == 22)
	{
		__SHOW("STATE\t22");
		IplImage* ip_dis = cvCreateImage(cvSize(320, 240), 8, 1);
		getDisImage(ip_dis, 2);
		double mean_x, mean_z;
		getCoatMeanDis(ip_dis, m_vCP_coat2, mean_x, mean_z);
		
		mean_x = -mean_x;
		mean_z = mean_z + 15;
		
		m_Co_man.x = mean_x;
		m_Co_man.z = mean_z;
		
		m_i_dis = mean_z;
		
		cout << "mean_dis = " << mean_z << "\t" << mean_x << endl;
		cvReleaseImage(&ip_dis);
		
		m_vCP_coat2.clear();
		m_b_getCoat2 = false;
		m_b_getDis2 = true;
		m_vi_state.pop_back();
	}
	else if(m_i_state == 12)
	{
		__SHOW("STATE\t12");
		IplImage* ip_dis = cvCreateImage(cvSize(320, 240), 8, 1);
		getDisImage(ip_dis, 1);
		double mean_x, mean_z;
		getCoatMeanDis(ip_dis, m_vCP_coat1, mean_x, mean_z);
		
		mean_x = mean_z + 15;
		mean_z = mean_x;
		
		m_Co_man.x = mean_x;
		m_Co_man.z = mean_z;
		
		m_i_dis = mean_z;
		
		cout << "mean_dis = " << mean_z << "\t" << mean_x << endl;
		cvReleaseImage(&ip_dis);
		
		m_vCP_coat1.clear();
		m_b_getCoat1 = false;
		m_b_getDis1 = true;
		m_vi_state.pop_back();	
	}

	
	// color coat vector get
	else if(m_i_state == 41)
	{
		__SHOW("STATE\t41");
		IplImage* ip_img = cvCreateImage(cvSize(320, 240), 8, 3);
		getColorImage(ip_img, 4);
		getCoatVector(ip_img, m_vCP_coat4);
		m_b_getCoat4 = true;
		//m_b_goon_move = false;
		cvReleaseImage(&ip_img);
		m_vi_state.pop_back();		
	}
	else if(m_i_state == 31)
	{
		__SHOW("STATE\t31");
		IplImage* ip_img = cvCreateImage(cvSize(320, 240), 8, 3);
		getColorImage(ip_img, 3);
		getCoatVector(ip_img, m_vCP_coat3);
		m_b_getCoat3 = true;
		cvReleaseImage(&ip_img);
		m_vi_state.pop_back();
	}
	else if(m_i_state == 21)
	{
		__SHOW("STATE\t21");
		IplImage* ip_img = cvCreateImage(cvSize(320, 240), 8, 3);
		getColorImage(ip_img, 2);
		getCoatVector(ip_img, m_vCP_coat2);
		//cout << "size = " << m_vCP_coat2.size() << endl;
		m_b_getCoat2 = true;
		cvReleaseImage(&ip_img);
		m_vi_state.pop_back();		
	}
	else if(m_i_state == 11)
	{
		__SHOW("STATE\t11");
		IplImage* ip_img = cvCreateImage(cvSize(320, 240), 8, 3);
		getColorImage(ip_img, 1);
		getCoatVector(ip_img, m_vCP_coat1);
		m_b_getCoat1 = true;
		//m_b_goon_move = false;
		cvReleaseImage(&ip_img);
		m_vi_state.pop_back();		
	}
	// away from center
	else if(m_i_state == 15)
	{
		__SHOW("STATE\t15");
		double d_x = 0;
		for(int i = 0; i < m_vCP_coat1.size(); i++)
		{
			d_x += m_vCP_coat1[i].x;
		}
		d_x /= m_vCP_coat1.size();
		m_d_center = d_x - 160;
		m_Co_man.z = d_x - 160;
		m_b_center1 = true;
		m_vi_state.pop_back();
	}
	else if(m_i_state == 25)
	{
		__SHOW("STATE\t25");
		double d_x = 0;
		for(int i = 0; i < m_vCP_coat2.size(); i++)
		{
			d_x += m_vCP_coat2[i].x;
		}
		//data << m_vCP_coat2.size() << endl;
		d_x /= m_vCP_coat2.size();
		m_d_center = d_x;
		m_d_left = m_d_center;
		//cout << "======25======== " << m_d_center << endl;
		//m_Co_man.x = d_x;
		m_b_center2 = true;
		m_vi_state.pop_back();
	}
	else if(m_i_state == 35)
	{
		__SHOW("STATE\t35");
		double d_x = 0;
		for(int i = 0; i < m_vCP_coat3.size(); i++)
		{
			d_x += m_vCP_coat3[i].x;
		}
		//data << "\t\t" << m_vCP_coat3.size() << endl;
		d_x /= m_vCP_coat3.size();
		m_d_center = d_x;
		m_d_right = m_d_center;
		//cout << "======35======== " << m_d_center << endl;
		//m_Co_man.x = d_x;
		m_b_center3 = true;
		m_vi_state.pop_back();
	}
	else if(m_i_state == 45)
	{
		__SHOW("STATE\t45");
		double d_x = 0;
		for(int i = 0; i < m_vCP_coat4.size(); i++)
		{
			d_x += m_vCP_coat4[i].x;
			//cout << m_vCP_coat4[i].x << ", ";
		}
		d_x /= m_vCP_coat4.size();
		m_d_center = -(d_x - 160);
		cout << "m_d_center = " << m_d_center << endl;
		m_Co_man.z = m_d_center;
		m_b_center4 = true;
		m_vi_state.pop_back();
	}
	
	else if(m_i_state == 1)
	{
		__SHOW("STATE\t1");
		IplImage* ip_dis = cvCreateImage(cvSize(320, 240), 8, 1);
		getDisImage(ip_dis, 2);
		cvThreshold(ip_dis, ip_dis, 200, 255, CV_THRESH_BINARY);

		unsigned char* imagedata = (unsigned char*)ip_dis->imageData;
		int step = ip_dis->widthStep / sizeof(unsigned char);
		int i_center_x = 0;
		int i_center_y = 0;
		int i_center_amount = 0;
		for(int i = 0; i < ip_dis->height / 4; i++)
		{
			for(int j = 0; j < ip_dis->width; j++)
			{
				if(imagedata[i * step + j] < 128)
				{
					i_center_x += j;
					i_center_y += i;
					i_center_amount++;
				}
			}
		}
		if(i_center_amount > 0)
		{
			i_center_x /= i_center_amount;
			i_center_y /= i_center_amount;
			cout << i_center_x << "\t" << i_center_y << endl;
		}
		else
		{
			cerr << "i_center_amount = 0" << endl;
			exit(0);
		}
		IplImage* ip_coat = cvCreateImage(cvSize(320, 240), 8, 3);
		getColorImage(ip_coat, 2);
		imagedata = (unsigned char*)ip_coat->imageData;
		step = ip_coat->widthStep / sizeof(unsigned char);
		for(int i = i_center_y - 20; i < i_center_y; i++)
		{
			for(int j = i_center_x - 20; j < i_center_x + 20; j++)
			{
				int b = imagedata[i * step + j * 3];
				int g = imagedata[i * step + j * 3 + 1];
				int r = imagedata[i * step + j * 3 + 2];
				if(b < m_i_coat_bmin)
				{
					m_i_coat_bmin = b;
				}
				if(r < m_i_coat_rmin)
				{
					m_i_coat_rmin = r;
				}
				if(g < m_i_coat_gmin)
				{
					m_i_coat_gmin = g;
				}
				if(b > m_i_coat_bmax)
				{
					m_i_coat_bmax = b;
				}
				if(r > m_i_coat_rmax)
				{
					m_i_coat_rmax = r;
				}
				if(g > m_i_coat_gmax)
				{
					m_i_coat_gmax = g;
				}
				
			}
		}
		cvRectangle(ip_coat, cvPoint(i_center_x - 20, i_center_y - 20), cvPoint(i_center_x + 20, i_center_y), cvScalar(0, 0, 255), 2);
		
		cvSaveImage("color.bmp", ip_coat);
		
		cvReleaseImage(&ip_coat);
		cvReleaseImage(&ip_dis);
		
		m_b_getColor = true;
		m_vi_state.pop_back();
		return 1.0;
	}
	m_i_time++;
  	return TIMEOUT;      
}  
  
void MyController::onRecvMsg(RecvMsgEvent &evt) 
{  
	std::string msg = evt.getMsg();
	if(msg == "Task_start")
	{
		//onInit(NULL);
		init();
		m_b_start = true;
		broadcastMsg("get message: Task_start");
	}
}  

void MyController::onCollision(CollisionEvent &evt) 
{ 
}
void MyController::getCoatMeanDis(IplImage* src, vector<CvPoint> &vCP_coat, double &robot_x, double &robot_z)
{
	unsigned char* imagedata = (unsigned char*)src->imageData;
	int step = src->widthStep / sizeof(unsigned char);
		
	double fovy = m_pRO_robot->getCamFOV() * PI / 180.0;  
	double ar = m_pRO_robot->getCamAS();  
	double fovx = 2 * atan(tan(fovy*0.5)*ar);		
	// Horizontal angle (origin is direction of a camera)  
	double phi = 0.0;
	// Vertical angle (origin is direction of a camera)  
	double theta = 0.0; 

	robot_x = 0;
	robot_z = 0;
	for(int i = 0; i < vCP_coat.size(); i++)
	{
		int i_x = vCP_coat[i].x;
		int i_y = vCP_coat[i].y;
		phi   = fovx*i_x/(320-1.0) - fovx/2.0;
		theta = fovy*i_y/(240-1.0) - fovy/2.0;  
		int distance = imagedata[i_y * step + i_x];
		robot_z += distance * cos(phi) * cos(theta);
		robot_x += distance * sin(phi) * cos(theta);
	}
	robot_x /= vCP_coat.size();
	robot_z /= vCP_coat.size();
}
	
void MyController::getCoatVector(IplImage* src, vector<CvPoint> &vCP_coat)
{
	unsigned char* imagedata = (unsigned char*)src->imageData;
	int step = src->widthStep / sizeof(unsigned char);
	for(int i = 0; i < src->height / 2; i+=3)
	{
		for(int j = 0; j < src->width; j+=3)
		{
			int b = imagedata[i * step + j * 3];
			int g = imagedata[i * step + j * 3 + 1];
			int r = imagedata[i * step + j * 3 + 2];
			if(b >= m_i_coat_bmin && b <= m_i_coat_bmax
			&& g >= m_i_coat_gmin && g <= m_i_coat_gmax
			&& r >= m_i_coat_rmin && r <= m_i_coat_rmax)
			{
				vCP_coat.push_back(cvPoint(j, i));
				/*if(vCP_coat.size() > 5)
				{
					return;
				}*/
			}
		}
	}
}
void MyController::getDisMin(char* src, double &minDis, int &minIdx, double &meanDis)
{
	double d_min = 256;
	int i_minIdx = -1;
	double d_dis = 0;
	for(int i = 0; i < 320; i++)
	{
		d_dis += int((unsigned char)src[i]);
		if(m_b_elevator == true)	data << int((unsigned char)src[i]) << "\t";
		if(d_min > int((unsigned char)src[i]))
		{
			d_min = int((unsigned char)src[i]);
			i_minIdx = i;
		}
	}
	meanDis = d_dis / 320;
	if(m_b_elevator == true)	data << endl;
	minDis = d_min;
	minIdx = i_minIdx;
}
void MyController::getDisLine(char* src, int camID, double d_max)
{
	if(m_b_elevator == true)
	{
		d_max = 600;
	}
	ViewImage *dis_img = m_pVS_view->distanceSensor1D(0.0, d_max, camID);
	char *dis_buf = dis_img->getBuffer();
	memcpy(src, dis_buf, 320);
}
void MyController::getDisImage(IplImage* src, int camID, double d_max)
{
	ViewImage *dis_img = m_pVS_view->distanceSensor2D(0.0, d_max, camID, DEPTHBIT_8, IMAGE_320X240);
	char *dis_buf = dis_img->getBuffer();
	memcpy(src->imageData, dis_buf, src->imageSize);
}
void MyController::getColorImage(IplImage* src, int camID)
{
	ViewImage *dis_img = m_pVS_view->captureView(camID, COLORBIT_24, IMAGE_320X240); 
	char *dis_buf = dis_img->getBuffer();
	memcpy(src->imageData, dis_buf, src->imageSize);
}
extern "C" Controller * createController() 
{  
  	return new MyController;  
}  

