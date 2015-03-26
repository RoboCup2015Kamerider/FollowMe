#include "ControllerEvent.h"
#include "Controller.h"
#include "Logger.h"
#include <iostream>
#include <vector>
#include <ViewImage.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define PI 3.1415926535797
#define TIMEOUT 0.5

#define __DEBUG

using namespace std;

double _dotProductXZ(Vector3d &v3d_v1, Vector3d &v3d_v2)
{
	return v3d_v1.x() * v3d_v2.x() + v3d_v1.z() * v3d_v2.z();
}

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
	
	ViewService *m_pVS_view;
	
	Vector3d m_v3d_initPos;				// 机器人初始位置
	Vector3d m_v3d_initDir;				// 机器人初始朝向
	
	Vector3d m_v3d_robotDir;
	
	Vector3d m_v3d_robotPos;	
	Vector3d m_v3d_manPos;
	
	double m_d_runTime;
	
	
	vector<Vector3d> m_vv3d_manPath;
	int i_pathIndex;
	
	double m_d_wheelRadius;				// 机器人轮子半径
	double m_d_wheelDistance;			// 机器人两轮间距离
	
public:
	bool m_b_start;					// 程序开始标志
	bool m_b_checkPoint1;
	bool m_b_checkPoint2;
	int m_i_elevatorTime;
	
public:
	vector<string> m_vs_allEntities;
	
public:
	void turnLeft()
	{
		double SCALE = 2;
		m_pRO_robot->setWheelVelocity(-PI / 4 / SCALE, PI / 4 / SCALE);
		sleep(1.0 * SCALE);
		m_pRO_robot->setWheelVelocity(0, 0);
		Vector3d v3d_oldDir = m_v3d_robotDir;
		m_v3d_robotDir.x(v3d_oldDir.z());
		m_v3d_robotDir.z(-v3d_oldDir.x());
		cout << "turn left" << endl;
	}
	void turnRight()
	{
		double SCALE = 2;
		m_pRO_robot->setWheelVelocity(PI / 4 / SCALE, -PI / 4 / SCALE);
		sleep(1.0 * SCALE);
		m_pRO_robot->setWheelVelocity(0, 0);
		Vector3d v3d_oldDir = m_v3d_robotDir;
		m_v3d_robotDir.x(-v3d_oldDir.z());
		m_v3d_robotDir.z(v3d_oldDir.x());
		cout << "turn right" << endl;
	}
	void turnBack()
	{
		double SCALE = 4;
		m_pRO_robot->setWheelVelocity(-PI / 2 / SCALE, PI / 2 / SCALE);
		sleep(1.0 * SCALE);
		m_pRO_robot->setWheelVelocity(0, 0);
		Vector3d v3d_oldDir = m_v3d_robotDir;
		m_v3d_robotDir.x(-v3d_oldDir.x());
		m_v3d_robotDir.z(-v3d_oldDir.z());
		cout << "turn back" << endl;
	}
	
public:
	Vector3d* GETPLANT(int camID);
	double getMinDistance(int i_camID)
	{
		//Vector3d* pv3d_dis = GETPLANT(i_camID);
		ViewImage* pVI_img = m_pVS_view->distanceSensor2D(0.0, 255.0, i_camID);
		char *dis_buf = pVI_img->getBuffer();
		
 		int i_width = 320;
 		int i_height = 240;
 		
 		double d_minDis = 1e10;
 		for(int i = 0; i < i_height; i++)
 		{
 			for(int j = 0; j < i_width; j++)
 			{
 				//double d_dis = pv3d_dis[i * i_width + j].z();
 				double d_dis = (unsigned char)dis_buf[i * i_width + j];
 				if(d_dis < d_minDis)
 				{
 					d_minDis = d_dis;
 				}
 			}
		}
		cout << "min dis = " << d_minDis;

		return d_minDis;
	}
};  
  
void MyController::onInit(InitEvent &evt)
{
	m_pRO_robot = getRobotObj(myname());		// 得到机器人指针
	m_d_wheelRadius = 10.0;
	m_d_wheelDistance = 10.0;

	m_pRO_robot->setWheel(m_d_wheelRadius, m_d_wheelDistance);
	
	m_pRO_robot->getPosition(m_v3d_initPos);	// 得到机器人的初始位置
	m_pRO_robot->setWheelVelocity(1.0, 1.0);
	sleep(1.0);
	m_pRO_robot->setWheelVelocity(0.0, 0.0);
	m_pRO_robot->getPosition(m_v3d_initDir);
	m_v3d_initDir -= m_v3d_initPos;
	
	if(m_v3d_initDir.x() != 0)
	{
		m_v3d_initDir.x(m_v3d_initDir.x() / m_v3d_initDir.x());
	}
	if(m_v3d_initDir.z() != 0)
	{
		m_v3d_initDir.z(m_v3d_initDir.z() / m_v3d_initDir.z());
	}
	
	m_v3d_robotDir = m_v3d_initDir;
	
	m_b_start = false;
	m_b_checkPoint1 = false;
	m_b_checkPoint2 = false;
	
	i_pathIndex = 0;
	m_d_runTime = 0;
	m_i_elevatorTime = 0;
	
	m_pVS_view = (ViewService*)connectToService("SIGViewer");
	
	bool b_getAllEntities = getAllEntities(m_vs_allEntities);
	
#ifdef __DEBUG
	cout << "m_v3d_initPos = " << m_v3d_initPos.x() << " " << m_v3d_initPos.y() << " " << m_v3d_initPos.z() << endl;
	cout << "m_v3d_initDir = " << m_v3d_initDir.x() << " " << m_v3d_initDir.y() << " " << m_v3d_initDir.z() << endl;
#endif
	
	double d_minAnalyze = 1e10;
	int i_minIndex;
	for(int i = 0; i < m_vs_allEntities.size(); i++)
	{
		string s_entitiesName = m_vs_allEntities[i];
		Vector3d v3d_entities;
		getRobotObj(s_entitiesName.data())->getPosition(v3d_entities);
		v3d_entities -= m_v3d_initPos;
		double d_analyze = v3d_entities.length() / v3d_entities.angle(m_v3d_initDir);
		if(d_analyze < d_minAnalyze && d_analyze > 0 && myname() != m_vs_allEntities[i])
		{
			d_minAnalyze = d_analyze;
			i_minIndex = i;
		}
#ifdef __DEBUG_OLD	
		cout << s_entitiesName << "\t" << v3d_entities.length() / v3d_entities.angle(m_v3d_initDir) << "\t" << v3d_entities.x() << "\t" << v3d_entities.y() << "\t" << v3d_entities.z() << endl;
#endif
	}
	cout << m_vs_allEntities[i_minIndex] << endl;
	m_pSO_man = getObj(m_vs_allEntities[i_minIndex].data());
	m_pSO_man->getPosition(m_v3d_manPos);
	m_vv3d_manPath.push_back(m_v3d_manPos);
	
	cout << m_v3d_manPos.x() << "\t" << m_v3d_manPos.y() << "\t" << m_v3d_manPos.z() << endl;
	
}  
  
double MyController::onAction(ActionEvent &evt) 
{  
	m_d_runTime += TIMEOUT;
	if(m_b_start == false)
	{
		string s_msgStart = "start";  
		broadcastMsg(s_msgStart);
		m_b_start = true;
		//return TIMEOUT;
	}
	//if(m_b_checkPoint1 == false)
	{
		m_pSO_man->getPosition(m_v3d_manPos);
		m_vv3d_manPath.push_back(m_v3d_manPos);
		
		m_pRO_robot->getPosition(m_v3d_robotPos);
		
		Vector3d v3d_diff;
		v3d_diff.x(m_vv3d_manPath[i_pathIndex].x() - m_v3d_robotPos.x());
		v3d_diff.z(m_vv3d_manPath[i_pathIndex].z() - m_v3d_robotPos.z());
		
		Vector3d v3d_robotdiff;
		
		double d_angleWorld = v3d_diff.x() / sqrt(pow(v3d_diff.x(), 2) + pow(v3d_diff.z(), 2));
		
		double d_angleRobot = m_v3d_robotDir.x() / sqrt(pow(m_v3d_robotDir.x(), 2) + pow(m_v3d_robotDir.z(), 2));
		
		double d_angleDiff = (v3d_diff.x() * m_v3d_robotDir.x() + v3d_diff.z() * m_v3d_robotDir.z()) / sqrt(pow(v3d_diff.x(), 2) + pow(v3d_diff.z(), 2));
		double d_arcAngleDiff = acos(d_angleDiff);
		
#ifdef __DEBUG		
		cout << v3d_diff.x() << "\t" << v3d_diff.z() << "\t" << d_angleWorld << "\t" <<  m_v3d_robotDir.x() << "\t" << m_v3d_robotDir.z() << "\t" << d_angleRobot << "\t" << d_angleDiff << "\t" << d_arcAngleDiff << endl;
#endif		
		double d_minDis = getMinDistance(3);
		
		if(d_minDis < 50 && m_d_runTime > 10)
		{
			m_pRO_robot->setWheelVelocity(0, 0);
			//return TIMEOUT;
		}
		
		if(d_minDis < 50 && m_d_runTime > 10 && _dotProductXZ(v3d_diff, m_v3d_robotDir) > 50 && m_b_checkPoint1 == false)
		{
			cout << "in<40#" << d_minDis;
			m_pRO_robot->setWheelVelocity(0, 0);
			if(m_b_checkPoint1 == false)
			{
				m_b_checkPoint1 = true;
				cout << "CHECKPOINT1 =============" << endl;
			}
			return TIMEOUT * 10;
		}
		if(d_minDis < 100 || _dotProductXZ(v3d_diff, m_v3d_robotDir) < 100)
		{
			m_pRO_robot->setWheelVelocity(0, 0);
			cout << "Too Close";
			if(m_b_checkPoint1 == true && m_b_checkPoint2 == false)
			{
				double d_leftDis = getMinDistance(1);
				double d_rightDis = getMinDistance(2);
				cout << "====="  << d_leftDis << " " << d_rightDis << endl;
				if(d_leftDis + d_rightDis < 150)
				{
					if(m_i_elevatorTime < 5)
					{
						m_i_elevatorTime++;
					}
					else
					{
						sleep(5);
						string s_msgElevator = "elevator";
						broadcastMsg(s_msgElevator);
						m_b_checkPoint2 = true;
						sleep(5);
						m_pRO_robot->setWheelVelocity(-10, -10);
						sleep(3);
						m_pRO_robot->setWheelVelocity(0, 0);
						string s_msgOk = "ok";
						broadcastMsg(s_msgOk);
						
						return TIMEOUT;
					}
				}
				else
				{
					cout << "Where am I" << endl;
				}
				//m_b_checkPoint2 = true;
			}
		}
		
		// 检测夹角
		if(fabs(d_arcAngleDiff - 0) < 0.2)
		{
			// same direction
			cout << "in<0.2#";
			v3d_robotdiff.z(_dotProductXZ(v3d_diff, m_v3d_robotDir));
		}
		else if(_dotProductXZ(v3d_diff, m_v3d_robotDir) > 2)
		{
			cout << "in>0.1#";
			v3d_robotdiff.z(_dotProductXZ(v3d_diff, m_v3d_robotDir));
		}
		else if(fabs(d_arcAngleDiff - PI / 2) < 0.2)
		{
			turnRight();
			return TIMEOUT;
		}
		else if(fabs(d_arcAngleDiff + PI / 2) < 0.2)
		{
			turnLeft();
			return TIMEOUT;
		}
		else if(fabs(d_arcAngleDiff - PI) < 0.2)
		{
			turnBack();
			return TIMEOUT;
		}
		
		double d_wheelVelocity;
		if(v3d_robotdiff.z() > 20)
		{
			d_wheelVelocity = v3d_robotdiff.z() / 20;
		}
		else if(v3d_robotdiff.z() > 1 && v3d_diff.length() > 50)
		{
			d_wheelVelocity = v3d_robotdiff.z() / 10;
		}

		m_pRO_robot->setWheelVelocity(d_wheelVelocity, d_wheelVelocity);
		
		i_pathIndex++;
		
	}
  	return TIMEOUT;      
}  
  
void MyController::onRecvMsg(RecvMsgEvent &evt) 
{  
}  

void MyController::onCollision(CollisionEvent &evt) 
{ 
}
Vector3d* MyController::GETPLANT(int camID) 
{
	if(m_pVS_view != NULL) 
	{ 
       		ViewImage *dis_img = m_pVS_view->distanceSensor2D(0.0, 255.0, camID, DEPTHBIT_8, IMAGE_320X240);   
 
  		double fovy = m_pRO_robot->getCamFOV() * PI / 180.0;
  		double ar = m_pRO_robot->getCamAS();  

                double fovx = 2 * atan(tan(fovy * 0.5) * ar);  

                char *dis_buf = dis_img->getBuffer();  
                  
		int width = dis_img->getWidth();
                int height = dis_img->getHeight();
	          
		Vector3d *p = new Vector3d[height * width];
                double *theta = new double[width]; 
		double *phi = new double[height];

	        unsigned char *distance = new unsigned char[height*width];

		double *y = new double[height * width];
		double *x = new double[height * width];
		double *z = new double[height * width];
		//ou << "====" << endl;
		int index;
		for(int i = 0; i < height; i++)
		{
			phi[i] = fovy / 2.0 - fovy * i / (height - 1.0);  
		    	for(int j = 0; j < width; j++)
			{
                        	theta[j] = fovx * j / (width - 1.0) - fovx / 2.0;  
		  		index = i * width + j;
		  		distance[index] = dis_buf[index]; 
				//ou << int(distance[index]) << "\t";
		    	}  
			//ou << endl;
		}
		int *dis = new int[height * width];
		int *disv = new int[height];
		int *dish = new int[width];
		double *angle = new double[height * width];

		
		for(int i = 0; i < height; i++)
		{
		  	disv[i] = distance[i * width + width / 2];
		  	//disv[i] = disv[i];
		  	for(int j = 0;j < width; j++)
			{
				index = i * width + j;
				dis[index]=distance[index];
				//dis[index]=4*dis[index];
		        	dish[j]=distance[width*height/2+j];
		        	//dish[j]=4*dish[j];
				y[index]=disv[i]*sin(phi[i])+30;
				x[index]=dish[j]*sin(theta[j]);
				angle[index]=asin((dish[0]*sin(fovx/2)*(width/2-j))/(dis[index]*width/2));
			
				z[index]=dis[index]*cos(angle[index])*cos(phi[i])+10;
		  	}
		}
		for(int i=0;i<height;i++)
		{
		  	for(int j=0;j<width;j++)
			{
				index=i*width+j;
				p[index]=Vector3d(x[index],y[index],z[index]);
		  	}
		}

		delete theta;
		delete phi;
		delete distance;
		delete dis;
		delete disv;
		delete dish;
		delete x;
		delete y;
		delete z;

		return p;
	}
	return NULL;
}
extern "C" Controller * createController() 
{  
  	return new MyController;  
}  

