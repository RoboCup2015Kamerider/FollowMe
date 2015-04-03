#include "ControllerEvent.h"  
#include "Controller.h"  
#include "Logger.h"

#define PI 3.1415926535797
#define TIMEOUT 1.0

#define __DEBUG

using namespace std;
  
class MyController : public Controller
{  
public:  
  	void onInit(InitEvent &evt);
  	double onAction(ActionEvent&);
  	void onRecvMsg(RecvMsgEvent &evt);
  	void onCollision(CollisionEvent &evt);
  	
public:
	RobotObj *m_pRO_robot;
	SimObj *m_pSO_man;
	SimObj *m_pSO_man001;
	ViewService *m_pVS_view;
	
	double m_d_wheelRadius;				// 机器人轮子半径
	double m_d_wheelDistance;			// 机器人两轮间距离
	
	Vector3d m_v3d_robotPos;
	Vector3d m_v3d_manPos;
	
	Rotation m_r_robotRot;
	Rotation m_r_manRot;
	
	double m_d_robotRot;
	double m_d_manRot;
	
	vector<Vector3d> m_vv_manPos;
	vector<Rotation> m_vr_manDir;
	vector<double> m_vd_manDir;
	
	Vector3d m_v3d_elevator;
	
	int m_i_pathIdx;
	int m_i_state;
	
	bool m_b_start;
	bool m_b_checkPoint1;
	bool m_b_inElevator;
	
	bool m_b_dir;
};

void MyController::onInit(InitEvent &evt) 
{
	m_pRO_robot = getRobotObj(myname());		// 得到机器人指针
	m_pSO_man = getObj("operator");			// 得到人的指针
	m_pSO_man001 = getObj("man_001");	
	m_d_wheelRadius = 10.0;
	m_d_wheelDistance = 10.0;
	m_b_start = false;
	int m_i_pathIdx = 0;
	m_i_state = 0;
	m_b_dir = true;
	m_b_checkPoint1 = false;
	
	m_v3d_elevator.x(-925);
	m_v3d_elevator.z(125);
	
	m_b_inElevator = false;

	m_pRO_robot->setWheel(m_d_wheelRadius, m_d_wheelDistance);
}

double MyController::onAction(ActionEvent &evt) 
{
	if(m_b_start == true)
	{
		m_pRO_robot->getPosition(m_v3d_robotPos);
		m_pSO_man->getPosition(m_v3d_manPos);
		
		m_pRO_robot->getRotation(m_r_robotRot);
		m_pSO_man->getRotation(m_r_manRot);
		
		double d_robotDir = 2 * asin(m_r_robotRot.qy());
		double d_manDir = 2 * asin(m_r_manRot.qy());
		
		double d_robotMan = sqrt(pow(m_v3d_robotPos.x() - m_v3d_manPos.x(), 2) + pow(m_v3d_robotPos.z() - m_v3d_manPos.z(), 2));
		
		//cout << "robot dir = " << d_robotDir << " man dir = " << d_manDir << endl;
		
		m_vv_manPos.push_back(m_v3d_manPos);
		m_vr_manDir.push_back(m_r_manRot);
		m_vd_manDir.push_back(d_manDir);
		
		Vector3d v3d_man001;
		
		m_pSO_man001->getPosition(v3d_man001);
		double d_disMan001 = sqrt(pow(v3d_man001.x() - m_v3d_robotPos.x(), 2)
		 + pow(v3d_man001.z() - m_v3d_robotPos.z(), 2));
		 
		//cout << "dis man 001 = " << d_disMan001 << endl;
		 
		if(d_disMan001 < 80 && m_b_checkPoint1 == false)
		{
			m_i_state = 10;
			m_b_checkPoint1 = true;
		}
		
		if(m_i_state == 0)
		{
			double d_theta = d_robotDir;
				
			double d_pathX = m_vv_manPos[m_i_pathIdx].x();
			double d_pathZ = m_vv_manPos[m_i_pathIdx].z();
				
			double d_X = d_pathX - m_v3d_robotPos.x();
			double d_Z = d_pathZ - m_v3d_robotPos.z();
			
			double d_pathDir = m_vd_manDir[m_i_pathIdx];
				
			double d_dis = d_X * sin(d_theta) + d_Z * cos(d_theta);
			//cout << "theta = " << d_theta << "man dir = " << d_pathDir << " x = " << d_X << " z = " << d_Z << " dis = " << d_dis << endl;
			double d_dtheta = d_pathDir - d_theta;
			
			if(d_dtheta > PI)
			{
				d_dtheta -= 2 * PI;
			}
			if(d_dtheta < -PI)
			{
				d_dtheta += 2 * PI;
			}
			//cout << "d_dtheta = " << d_dtheta << endl;
			
			if((fabs(d_dtheta - 0) < 0.02 || d_dis > 0) && m_b_dir)
			{
				if(d_dis > 60)
				{
				
					double d_K = 0.5;
					double d_D = 50.0;
					// P control
					double d_vel = d_K * (d_dis - d_D) / m_d_wheelRadius;
				
					m_pRO_robot->setWheelVelocity(d_vel, d_vel);
				}
				else if(d_dis > 0)
				{
					double d_disElevator = sqrt(pow(m_v3d_elevator.x() - m_v3d_robotPos.x(), 2) + pow(m_v3d_elevator.z() - m_v3d_robotPos.z(), 2));
					cout << "dis elevator = " << d_disElevator << endl;
					if(d_disElevator < 300 && d_robotMan < 80 && m_b_inElevator == false)
					{
						m_b_inElevator = true;
						m_pRO_robot->setWheelVelocity(0.0, 0.0);
						m_i_state = 30;
						return TIMEOUT;
						
					}
					double d_vel = d_dis / m_d_wheelRadius;
					m_pRO_robot->setWheelVelocity(d_vel, d_vel);
				}
				m_i_pathIdx++;
			}
			else
			{
				m_b_dir = false;
				
				if(fabs(d_dtheta) > 0.02)
				{
					double d_K = 0.25;
					// P control
					double d_vel = d_K * d_dtheta;
					m_pRO_robot->setWheelVelocity(-d_vel, d_vel);
				}
				else
				{
					m_b_dir = true;
					m_i_pathIdx++;
				}
			}
		}
		else if(m_i_state == 10)
		{
			m_pRO_robot->setWheelVelocity(-20.0, -20.0);
			
			if(d_disMan001 > 80)
			{
				m_i_state = 20;
			}
			
		}
		else if(m_i_state == 20)
		{
			m_pRO_robot->setWheelVelocity(0.0, 0.0);
			sleep(3.0);
			m_vv_manPos.clear();
			m_vr_manDir.clear();
			m_vd_manDir.clear();
			m_i_pathIdx = 0;
			m_i_state = 0;
		}
		else if(m_i_state == 30)
		{
			cout << "in the elevator" << endl;
			broadcastMsg("Door_close");
			sleep(5.0);
			m_pRO_robot->setWheelVelocity(-15.0, -15.0);
			sleep(4.0);
			m_pRO_robot->setWheelVelocity(0.0, 0.0);
			broadcastMsg("Get_off");
			sleep(5.0);
			m_vv_manPos.clear();
			m_vr_manDir.clear();
			m_vd_manDir.clear();
			m_i_pathIdx = 0;
			m_i_state = 40;
			
		}
		else if(m_i_state == 40)
		{
			cout << "out of elevator" << endl;
		}
		
		//cout << "\t\t\trobot pos = " << m_v3d_robotPos.x() << ", " << m_v3d_robotPos.z()
		//<< " man pos = " << m_v3d_manPos.x() << ", " << m_v3d_manPos.z()
		//<< " robot dir = " << m_r_robotRot.qy() << "man dir = " << m_r_manRot.qy() << endl;
	}
  	return TIMEOUT;
}

void MyController::onRecvMsg(RecvMsgEvent &evt) 
{
	std::string msg = evt.getMsg();
	if(msg == "Task_start")
	{
		m_b_start = true;
		broadcastMsg("get message: Task_start");
	}
}

void MyController::onCollision(CollisionEvent &evt) 
{ 
}
extern "C" Controller * createController()
{
  	return new MyController;
}

