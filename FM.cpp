#include "ControllerEvent.h"
#include "Controller.h"
#include "Logger.h"
#include "iostream"
#include "vector"

#define PI 3.1415926

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
	
	Vector3d m_v3d_initPos;				// 机器人初始位置
	Vector3d m_v3d_initDir;				// 机器人初始朝向
	
	Vector3d m_v3d_robotDir;
	
	Vector3d m_v3d_robotPos;	
	Vector3d m_v3d_manPos;
	
	
	vector<Vector3d> m_vv3d_manPath;
	int i_pathIndex;
	
	double m_d_wheelRadius;				// 机器人轮子半径
	double m_d_wheelDistance;			// 机器人两轮间距离
	
public:
	bool m_b_start;					// 程序开始标志
	bool m_b_checkPoint1;
	
public:
	vector<string> m_vs_allEntities;
	
public:
	void turnLeft()
	{
		m_pRO_robot->setWheelVelocity(-PI / 4, PI / 4);
		sleep(1.0);
		m_pRO_robot->setWheelVelocity(0, 0);
		Vector3d v3d_oldDir = m_v3d_robotDir;
		m_v3d_robotDir.x(v3d_oldDir.z());
		m_v3d_robotDir.z(-v3d_oldDir.x());
		cout << "turn left" << endl;
	}
	void turnRight()
	{
		m_pRO_robot->setWheelVelocity(PI / 4, -PI / 4);
		sleep(1.0);
		m_pRO_robot->setWheelVelocity(0, 0);
		Vector3d v3d_oldDir = m_v3d_robotDir;
		m_v3d_robotDir.x(-v3d_oldDir.z());
		m_v3d_robotDir.z(v3d_oldDir.x());
		cout << "turn right" << endl;
	}
	void turnBack()
	{
		m_pRO_robot->setWheelVelocity(PI / 2, -PI / 2);
		sleep(1.0);
		m_pRO_robot->setWheelVelocity(0, 0);
		Vector3d v3d_oldDir = m_v3d_robotDir;
		m_v3d_robotDir.x(-v3d_oldDir.x());
		m_v3d_robotDir.z(-v3d_oldDir.z());
		cout << "turn back" << endl;
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
	
	i_pathIndex = 0;
	
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
	if(m_b_start == false)
	{
		string s_msgStart = "start";  
		broadcastMsg(s_msgStart);
		m_b_start = true;
	}
	if(m_b_checkPoint1 == false)
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
			return 1.0;
		}
		else if(fabs(d_arcAngleDiff + PI / 2) < 0.2)
		{
			turnLeft();
			return 1.0;
		}
		else if(fabs(d_arcAngleDiff - PI) < 0.2)
		{
			turnBack();
			return 1.0;
		}
		
		double d_wheelVelocity;
		if(v3d_robotdiff.z() > 20)
		{
			d_wheelVelocity = v3d_robotdiff.z() / 20;
		}
		else if(v3d_robotdiff.z() > 0.1)
		{
			d_wheelVelocity = v3d_robotdiff.z() / 10;
		}

		m_pRO_robot->setWheelVelocity(d_wheelVelocity, d_wheelVelocity);
		
		i_pathIndex++;
		
	}
  	return 1.0;      
}  
  
void MyController::onRecvMsg(RecvMsgEvent &evt) 
{  
}  

void MyController::onCollision(CollisionEvent &evt) 
{ 
}
  
extern "C" Controller * createController() 
{  
  	return new MyController;  
}  

