#include "ControllerEvent.h"
#include "Controller.h"
#include "Logger.h"
#include "iostream"
#include "vector"

#define PI 3.1415926

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
	}
	void turnRight()
	{
		m_pRO_robot->setWheelVelocity(PI / 4, -PI / 4);
		sleep(1.0);
		m_pRO_robot->setWheelVelocity(0, 0);
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
		
		if(m_v3d_robotDir.x() == 0 && m_v3d_robotDir.z() == 1)
		{
			v3d_robotdiff.x(v3d_diff.x());
			v3d_robotdiff.z(v3d_diff.z());
		}
		else if(m_v3d_robotDir.x() == -1 && m_v3d_robotDir.z() == 0)
		{
			v3d_robotdiff.x(v3d_diff.z());
			v3d_robotdiff.z(-v3d_diff.x());
		}
		else if(m_v3d_robotDir.x() == 0 && m_v3d_robotDir.z() == -1)
		{
			v3d_robotdiff.x(v3d_diff.x());
			v3d_robotdiff.z(-v3d_diff.z());
		}
		else if(m_v3d_robotDir.x() == 1 && m_v3d_robotDir.z() == 0)
		{
			v3d_robotdiff.x(-v3d_diff.z());
			v3d_robotdiff.z(v3d_diff.x());
		}
		
		cout << v3d_diff.x() << "\t" << v3d_diff.z() << endl;
		cout << m_v3d_robotDir.x() << "\t" << m_v3d_robotDir.z() << endl;
		
		
		double d_wheelVelocity;
		if(v3d_robotdiff.z() > 20)
		{
			d_wheelVelocity = v3d_robotdiff.z() / 20;
		}
		else if(v3d_robotdiff.z() > 0.1)
		{
			d_wheelVelocity = v3d_robotdiff.z() / 10;
		}
		else if(v3d_robotdiff.x() > 20)
		{
			turnLeft();
			Vector3d v3d_oldDir = m_v3d_robotDir;
			m_v3d_robotDir.x(v3d_oldDir.z());
			m_v3d_robotDir.z(-v3d_oldDir.x());
			return 1.0;
		}
		else if(v3d_robotdiff.x() < -20)
		{
			turnRight();
			Vector3d v3d_oldDir = m_v3d_robotDir;
			m_v3d_robotDir.x(-v3d_oldDir.z());
			m_v3d_robotDir.z(v3d_oldDir.x());
			return 1.0;
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

