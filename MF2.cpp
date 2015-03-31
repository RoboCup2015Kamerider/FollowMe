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

//#define __DEBUG

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
	
	double m_i_state;
	
	
	vector<Vector3d> m_vv3d_manPath;
	int i_pathIndex;
	int m_i_manStop;
	
	double m_d_wheelRadius;				// 机器人轮子半径
	double m_d_wheelDistance;			// 机器人两轮间距离
	
public:
	bool m_b_start;	
	bool m_b_elevator;				// 程序开始标志
	bool m_b_checkPoint1;
	bool m_b_checkPoint2;
	
public:
	vector<string> m_vs_allEntities;
	
	Vector3d v3d_diff;
	double d_diff;
	double d_minDis;
	int m_i_elevator;
	
	int m_i_stop;
	
public:
	void turnLeft();
	void turnRight();
	void turnBack();
	void stop()
	{
		m_pRO_robot->setWheelVelocity(0, 0);
	}
	void moveDis(double d_dis)
	{
		m_pRO_robot->setWheelVelocity(d_dis / 10, d_dis / 10);
		sleep(1.0);
		stop();
	}
	void setVelocity(Vector3d v3d_diff)
	{
		//cout << "set vel = " << v3d_diff.z() << endl;
		double d_wheelVelocity = 0.0;
		if(v3d_diff.z() > 50)
		{
			d_wheelVelocity =  (v3d_diff.z() - 50) / 40;
		}
		else if(v3d_diff.z() > 2)
		{
			d_wheelVelocity =  v3d_diff.z() / 10 < 2 ? v3d_diff.z() / 10 : 2;
		}
		m_pRO_robot->setWheelVelocity(d_wheelVelocity, d_wheelVelocity);
	}
	
public:
	Vector3d* GETPLANT(int camID);
	double getMinDistance(int low, int high, int i_camID);
	bool isInElevator();
	void start()
	{
		//cout << "In start()" << endl;
		if(m_b_start == false)
		{
			string s_msgStart = "start";  
			broadcastMsg(s_msgStart);
			m_b_start = true;
			
	
			m_b_start = false;
			m_b_checkPoint1 = false;
			m_b_checkPoint2 = false;
	
			
			m_d_runTime = 0;
	

		}
	}
	void elevator();
	bool isCheckPoint1(double d_minDis, Vector3d v3d_diff)
	{
		if(d_minDis < 50 && m_d_runTime > 10 && _dotProductXZ(v3d_diff, m_v3d_robotDir) > 50 && m_b_checkPoint1 == false)
		{
			stop();
			if(m_b_checkPoint1 == false)
			{
				m_b_checkPoint1 = true;
				//cout << "--> CHECKPOINT1" << endl;
			}
			return true;
		}
		return false;
	}
	bool followPath(Vector3d v3d_diff, Vector3d &v3d_robotdiff)
	{
		//cout << "In followPath()" << endl;
		double d_angleDiff = (_dotProductXZ(v3d_diff, m_v3d_robotDir)) / sqrt(pow(v3d_diff.x(), 2) + pow(v3d_diff.z(), 2));
		
		double d_arcAngleRobot = atan2(m_v3d_robotDir.x(), m_v3d_robotDir.z());
		double d_arcAngleDiff = atan2(v3d_diff.x(), v3d_diff.z());
		
	#ifdef __DEBUG		
		//cout << m_v3d_manPos.x() << "\t" << m_v3d_manPos.z()  << "\t" <<  m_v3d_robotPos.x() << "\t" << m_v3d_robotPos.z() << endl;
	#endif	
		// 检测夹角
		if(fabs(d_arcAngleDiff - 0) < 0.2)
		{
			// same direction
			v3d_robotdiff.z(_dotProductXZ(v3d_diff, m_v3d_robotDir));
		}
		else if(_dotProductXZ(v3d_diff, m_v3d_robotDir) > 2)
		{
			v3d_robotdiff.z(_dotProductXZ(v3d_diff, m_v3d_robotDir));
		}
		else if(fabs(d_arcAngleDiff - d_arcAngleRobot + PI / 2) < 0.2
			|| fabs(d_arcAngleDiff - d_arcAngleRobot - PI * 3 / 2) < 0.2)
		{
			turnRight();
			return true;
		}
		else if(fabs(d_arcAngleDiff - d_arcAngleRobot - PI / 2) < 0.2
			|| fabs(d_arcAngleDiff - d_arcAngleRobot + PI * 3 / 2) < 0.2)
		{
			turnLeft();
			return true;
		}
		else if(fabs(d_arcAngleDiff - d_arcAngleRobot - PI) < 0.2)
		{
			turnBack();
			return true;
		}
		return false;
	}
	
};
  
void MyController::onInit(InitEvent &evt)
{
	m_pRO_robot = getRobotObj(myname());		// 得到机器人指针
	m_d_wheelRadius = 10.0;
	m_d_wheelDistance = 10.0;

	m_pRO_robot->setWheel(m_d_wheelRadius, m_d_wheelDistance);
	m_pVS_view = (ViewService*)connectToService("SIGViewer");
	m_i_state = 0;
	i_pathIndex = 0;
	m_i_stop = 0;
	m_i_elevator = 0;
	m_b_elevator = false;
}  
  
double MyController::onAction(ActionEvent &evt) 
{
	if(m_i_state == 10)
	{
#ifdef __DEBUG
		cout << "m_i_state = 10" << endl;
#endif
		m_pRO_robot->getPosition(m_v3d_initPos);	// 得到机器人的初始位置
		moveDis(1.0);
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
		i_pathIndex = 0;
		m_i_manStop = 0;
		m_i_state = 15;
	}
	else if(m_i_state == 15)
	{
		bool b_getAllEntities = getAllEntities(m_vs_allEntities);
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
		}
		cout << m_vs_allEntities[i_minIndex] << endl;
		m_pSO_man = getObj(m_vs_allEntities[i_minIndex].data());
		m_pSO_man->getPosition(m_v3d_manPos);
		//m_vv3d_manPath.push_back(m_v3d_manPos);
	
		
		m_i_state = 20;
	}
	else if(m_i_state == 20)
	{
#ifdef __DEBUG
		cout << "m_i_state = 20" << endl;
#endif
		m_pSO_man->getPosition(m_v3d_manPos);
		m_vv3d_manPath.push_back(m_v3d_manPos);
		
		if(m_v3d_manPos.x() == m_vv3d_manPath[m_vv3d_manPath.size() - 2].x() 
		&& m_v3d_manPos.z() == m_vv3d_manPath[m_vv3d_manPath.size() - 2].z())
		{
			m_i_manStop++;
		}
		
		
		m_pRO_robot->getPosition(m_v3d_robotPos);
		//cout << "path pos = " << m_vv3d_manPath[i_pathIndex].x() << "\t" << m_vv3d_manPath[i_pathIndex].z() << endl;
		//cout << "robot pos = " << m_v3d_robotPos.x() << "\t" << m_v3d_robotPos.z() << endl;
		v3d_diff.x(m_vv3d_manPath[i_pathIndex].x() - m_v3d_robotPos.x());
		v3d_diff.z(m_vv3d_manPath[i_pathIndex].z() - m_v3d_robotPos.z());
		i_pathIndex++;
		//cout << "v3d_diff = " << v3d_diff.z() << endl;
		d_diff = sqrt(pow(m_v3d_manPos.x() - m_v3d_robotPos.x(), 2) + pow(m_v3d_manPos.z() - m_v3d_robotPos.z(), 2));
		//cout << "diff = " << d_diff << endl;
		if(m_i_manStop >= 1 && d_diff < 50)
		{
			m_i_state = 40;
			return TIMEOUT;
		}
		double d_minDis1 = getMinDistance(0, 320, 1);
		double d_minDis2 = getMinDistance(0, 320, 2);
		double d_minDis3 = getMinDistance(0, 320, 3);
		double d_minDis4 = getMinDistance(0, 320, 4);
#ifdef __DEBUG
		cout << "dis 1 = " << d_minDis1 << " dis 2 = " << d_minDis2 << 
		"dis 3 = " << d_minDis3 << " dis 4 = " << d_minDis4 << endl;
#endif
		d_minDis = d_minDis2 < d_minDis3 ? d_minDis2 : d_minDis3;
		double d_minThres = 20;
		/*if(i_pathIndex > 5)
		{
			d_minThres = 80;
		}*/
		if(d_minDis1 < 30 || d_minDis4 < 30 || d_minDis2 < 30 || d_minDis3 < 30)
		{
			moveDis(-10);
			stop();
			m_i_state = 20;
		}
		else
		{
			m_i_state = 30;
		}
		if(d_minDis1 + d_minDis4 < 150 && d_minDis1 < 80 && d_minDis4 < 80 && m_b_elevator == false)
		{
			m_i_elevator++;
		}
		else
		{
			m_i_elevator = 0;
		}
		if(m_i_elevator > 4)
		{
			m_b_elevator = true;
			m_i_state = 50;
		}
	}
	else if(m_i_state == 30)
	{	
#ifdef __DEBUG
		cout << "m_i_state = 30" << endl;
#endif
		double d_angleDiff = (_dotProductXZ(v3d_diff, m_v3d_robotDir)) / sqrt(pow(v3d_diff.x(), 2) + pow(v3d_diff.z(), 2));
		
		double d_arcAngleRobot = atan2(m_v3d_robotDir.x(), m_v3d_robotDir.z());
		double d_arcAngleDiff = atan2(v3d_diff.x(), v3d_diff.z());
		
		Vector3d v3d_robotdiff;
		if(fabs(d_arcAngleDiff - d_arcAngleRobot) < 0.2)
		{
			// same direction
			v3d_robotdiff.z(_dotProductXZ(v3d_diff, m_v3d_robotDir));
			//cout << "v3d_robotdiff = " << v3d_robotdiff.z() << endl;
			setVelocity(v3d_robotdiff);
			//moveDis(v3d_robotdiff.z() / 2 - 80);
#ifdef __DEBUG
			cout<<"-80"<<endl;
#endif
		}
		else if(_dotProductXZ(v3d_diff, m_v3d_robotDir) > 0 && d_diff > 80)
		{
			v3d_robotdiff.z(_dotProductXZ(v3d_diff, m_v3d_robotDir));
			//setVelocity(v3d_robotdiff);
			moveDis(v3d_robotdiff.z() / 2 > 10 ? v3d_robotdiff.z() / 2 : 10);
#ifdef __DEBUG
			cout<<"==="<<endl;
#endif
		}
		else if(fabs(d_arcAngleDiff - d_arcAngleRobot + PI / 2) < 0.2
			|| fabs(d_arcAngleDiff - d_arcAngleRobot - PI * 3 / 2) < 0.2)
		{
			turnRight();
		}
		else if(fabs(d_arcAngleDiff - d_arcAngleRobot - PI / 2) < 0.2
			|| fabs(d_arcAngleDiff - d_arcAngleRobot + PI * 3 / 2) < 0.2)
		{
			turnLeft();
		}
		else if(fabs(d_arcAngleDiff - d_arcAngleRobot - PI) < 0.2)
		{
			turnBack();
		}
		m_i_state = 20;
	}
	else if(m_i_state == 40)
	{
		//cout << "m_i_state = 40" << endl;
		stop();
		m_i_state = 20;
		if(m_i_stop > 10)
		{
			m_i_stop = 0;
			m_i_state = 20;
		}
		cout << "something will happen" << endl;
		
	}
	else if(m_i_state == 50)
	{
		cout << "m_i_state = 50" << endl;
		stop();
		broadcastMsg("Door_close");
		sleep(8.0);
		moveDis(-300);
		broadcastMsg("Get_off");
		sleep(5.0);
		m_i_state = 60;
		
	}
		/*
	// checkPoint1
	if(isCheckPoint1(d_minDis, v3d_diff))
	{
		return TIMEOUT * 10;
	}
	// elevator
	if(isInElevator())
	{
		double d_checkDis = _dotProductXZ(v3d_diff, m_v3d_robotDir) - 50;
		moveDis(d_checkDis);
		elevator();
	}*/
		//if(followPath(v3d_diff, v3d_robotdiff))		// else
		//{
		//	return TIMEOUT;
		//}
	
		/*
		if(v3d_diff.length() < 20 && m_d_runTime > 10)
		{
			stop();
			return TIMEOUT;
		}*/
	
	
		
		//
	//}
  	return TIMEOUT;
}  
  
void MyController::onRecvMsg(RecvMsgEvent &evt) 
{
	std::string msg = evt.getMsg();
	if(msg == "Task_start")
	{
		broadcastMsg("get message: Task_start");
		m_i_state = 10;
	}
}  

void MyController::onCollision(CollisionEvent &evt) 
{ 
}
double MyController::getMinDistance(int low, int high, int i_camID)
{
	//Vector3d* pv3d_dis = GETPLANT(i_camID);
	//cout << "In get Min Distance " << i_camID << endl;
	ViewImage* pVI_img = m_pVS_view->distanceSensor2D(0.0, 255.0, i_camID);
	char *dis_buf = pVI_img->getBuffer();
		
 	int i_width = 320;
 	int i_height = 240;
 		
 	double d_minDis = 1e10;
 	for(int i = 0; i < i_height; i++)
 	{
 		for(int j = low; j < high; j++)
 		{
 				//double d_dis = pv3d_dis[i * i_width + j].z();
 			double d_dis = (unsigned char)dis_buf[i * i_width + j];
 			if(d_dis < d_minDis)
 			{
 				d_minDis = d_dis;
 			}
 		}
	}
	//cout << "min distance = " << d_minDis << endl;

	return d_minDis;
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
void MyController::turnLeft()
{
	double SCALE = 1;
	moveDis(10.0);
	m_pRO_robot->setWheelVelocity(-PI / 4 / SCALE, PI / 4 / SCALE);
	sleep(1.0 * SCALE);
	m_pRO_robot->setWheelVelocity(0, 0);
	Vector3d v3d_oldDir = m_v3d_robotDir;
	m_v3d_robotDir.x(v3d_oldDir.z());
	m_v3d_robotDir.z(-v3d_oldDir.x());
	//cout << "turn left" << endl;
}
void MyController::turnRight()
{
	double SCALE = 1;
	moveDis(10.0);
	m_pRO_robot->setWheelVelocity(PI / 4 / SCALE, -PI / 4 / SCALE);
	sleep(1.0 * SCALE);
	m_pRO_robot->setWheelVelocity(0, 0);
	Vector3d v3d_oldDir = m_v3d_robotDir;
	m_v3d_robotDir.x(-v3d_oldDir.z());
	m_v3d_robotDir.z(v3d_oldDir.x());
	//cout << "turn right" << endl;
}
void MyController::turnBack()
{
	double SCALE = 2;
	m_pRO_robot->setWheelVelocity(-PI / 2 / SCALE, PI / 2 / SCALE);
	sleep(1.0 * SCALE);
	m_pRO_robot->setWheelVelocity(0, 0);
	Vector3d v3d_oldDir = m_v3d_robotDir;
	m_v3d_robotDir.x(-v3d_oldDir.x());
	m_v3d_robotDir.z(-v3d_oldDir.z());
	//cout << "turn back" << endl;
}
bool MyController::isInElevator()
{
	if(m_b_checkPoint1 == false || m_b_checkPoint2 == true)
	{
		return false;
	}
	static int i_elevatorTime = 0;
	double d_leftDis = 0;//getMinDistance(1);
	double d_rightDis = 0;//getMinDistance(2);
#ifdef __DEBUG_OLD
	//cout << "====="  << d_leftDis << " " << d_rightDis << endl;
#endif
	if(d_leftDis + d_rightDis < 150)
	{
		if(i_elevatorTime < 6)
		{
			i_elevatorTime++;
		}
		else
		{
			return true;		
		}
	}
	return false;
}
void MyController::elevator()
{
	sleep(3);
	string s_msgElevator = "elevator";
	broadcastMsg(s_msgElevator);
	m_b_checkPoint2 = true;
	sleep(2);
	m_pRO_robot->setWheelVelocity(-20, -20);
	sleep(2);
	m_pRO_robot->setWheelVelocity(0, 0);
	string s_msgOk = "ok";
	broadcastMsg(s_msgOk);
	sleep(8);
	//m_vv3d_manPath.clear();
	i_pathIndex = 0;
}
extern "C" Controller * createController() 
{  
  	return new MyController;  
}  

