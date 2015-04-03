#include "ControllerEvent.h"  
#include "Controller.h"  
#include "Logger.h"  

#define PI 3.1415926535797
#define TIMEOUT 1.0

#define __DEBUG

using namespace std;

double _distance(Vector3d &v3d_p1, Vector3d &v3d_p2)
{
	return sqrt(pow(v3d_p1.x() - v3d_p2.x(), 2) + pow(v3d_p1.z() - v3d_p2.z(), 2));
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
	SimObj *m_pSO_obj;
	
	double m_d_wheelRadius;				// 机器人轮子半径
	double m_d_wheelDistance;			// 机器人两轮间距离
	
	Vector3d m_v3d_robotPos;
	
	Vector3d m_v3d_trash0;
	Vector3d m_v3d_trash1;
	Vector3d m_v3d_trash2;
	Vector3d m_v3d_wagon;
	
	Rotation m_r_robotRot;
	double m_d_robotRot;
	
	int m_i_state;
	
	bool m_b_start;
	bool m_b_dir;
	
	Vector3d m_v3d_diningTable;
	
	Vector3d m_v3d_dingTableRelayPoint[4];
	
	Vector3d m_v3d_obj;
	
	int m_i_trashClass;
	
	double d_diningTableWidth;
	double d_diningTableLength;
	
	double d_pathWidth;
};  
  
void MyController::onInit(InitEvent &evt) 
{
	m_pRO_robot = getRobotObj(myname());		// 得到机器人指针
	
	m_d_wheelRadius = 10.0;
	m_d_wheelDistance = 10.0;
	m_b_start = false;
	
	m_i_state = 0;
	m_b_dir = true;
	
	m_v3d_diningTable.x(283.338);
	m_v3d_diningTable.y(30.0);
	m_v3d_diningTable.z(29.017);
	
	m_v3d_obj.x(283.338);
	m_v3d_obj.z();
	
	m_v3d_trash0.x(-100);
	m_v3d_trash0.z(-220);
	
	m_v3d_trash1.x(0);
	m_v3d_trash1.z(-220);
	
	m_v3d_trash2.x(100);
	m_v3d_trash2.z(-220);
	
	m_v3d_wagon.x(-200);
	m_v3d_wagon.z(-200);
	
	d_diningTableWidth = 80;
	d_diningTableLength = 140;
	
	d_pathWidth = 30;
	
	m_v3d_dingTableRelayPoint[0].x(m_v3d_diningTable.x() + d_diningTableLength / 2 + d_pathWidth);
	m_v3d_dingTableRelayPoint[0].z(m_v3d_diningTable.z() - d_diningTableWidth / 2 - d_pathWidth);
	
	m_v3d_dingTableRelayPoint[1].x(m_v3d_diningTable.x() - d_diningTableLength / 2 - d_pathWidth);
	m_v3d_dingTableRelayPoint[1].z(m_v3d_diningTable.z() - d_diningTableWidth / 2 - d_pathWidth);
	
	m_v3d_dingTableRelayPoint[2].x(m_v3d_diningTable.x() - d_diningTableLength / 2 - d_pathWidth);
	m_v3d_dingTableRelayPoint[2].z(m_v3d_diningTable.z() + d_diningTableWidth / 2 + d_pathWidth);
	
	m_v3d_dingTableRelayPoint[3].x(m_v3d_diningTable.x() + d_diningTableLength / 2 + d_pathWidth);
	m_v3d_dingTableRelayPoint[3].z(m_v3d_diningTable.z() + d_diningTableWidth / 2 + d_pathWidth);
	
	/*for(int i = 0; i < 4; i++)
	{
		cout << "Relay Point " << i << " x = " << m_v3d_dingTableRelayPoint[i].x()
		<< ", z = " << m_v3d_dingTableRelayPoint[i].z() << endl; 
	}*/
	
	
	
	m_pRO_robot->setWheel(m_d_wheelRadius, m_d_wheelDistance);
}  
  
double MyController::onAction(ActionEvent &evt) 
{
	if(m_b_start == true)
	{
		m_pRO_robot->getPosition(m_v3d_robotPos);
		cout << "robot pos = " << m_v3d_robotPos.x() << " " << m_v3d_robotPos.z() << endl;
		
		if(m_i_state == 0)
		{
			cout << "in state 0" << endl;
			sleep(5.0);
			std::vector<std::string> m_entities;
			getAllEntities(m_entities);
			for(int i = 0; i < m_entities.size(); i++)
			{
				if(m_entities[i] == "can_1")
				{
					m_pSO_obj = getObj("can_1");
					Vector3d v3d_obj;
					m_pSO_obj->getPosition(v3d_obj);
					if(v3d_obj.length() < 1e4)
					{
						m_i_trashClass = 2;
						break;
					}
				}
				if(m_entities[i] == "petbottle_4")
				{
					m_pSO_obj = getObj("petbottle_4");
					Vector3d v3d_obj;
					m_pSO_obj->getPosition(v3d_obj);
					if(v3d_obj.length() < 1e4)
					{
						m_i_trashClass = 1;
						break;
					}
					/*else
					{
						m_pSO_obj = NULL;
					}*/
				}
				
			}
			m_pSO_obj->getPosition(m_v3d_obj);
			cout << "Obj pos = " << m_v3d_obj.x() << ", " << m_v3d_obj.z() << endl;
			m_i_state = 10;
		}
		else if(m_i_state == 10)
		{
			cout << "in state 10" << endl;
			double d_robotRelayPoint[4];
			double d_minDis = 1e10;
			int i_minIdx = 0;
			double d_minObjDis = 1e10;
			int i_minObjIdx = 0;
			for(int i = 0; i < 4; i++)
			{
				d_robotRelayPoint[i] = _distance(m_v3d_robotPos, m_v3d_dingTableRelayPoint[i]);
				double d_objDis = _distance(m_v3d_dingTableRelayPoint[i], m_v3d_obj);
				//cout << "dis " << i << " = " << d_robotRelayPoint[i] << endl;
				//m_pRO_robot->setPosition(m_v3d_dingTableRelayPoint[i]);
				//sleep(1.0);
				if(d_robotRelayPoint[i] < d_minDis)
				{
					d_minDis = d_robotRelayPoint[i];
					i_minIdx = i;
				}
				if(d_objDis < d_minObjDis)
				{
					d_minObjDis = d_objDis;
					i_minObjIdx = i;
				}
			}
			m_pRO_robot->setPosition(m_v3d_dingTableRelayPoint[i_minIdx]);
			sleep(2.0);
			m_pRO_robot->setPosition(m_v3d_dingTableRelayPoint[i_minObjIdx]);
			m_i_state = 20;
		}
		else if(m_i_state == 20)
		{
			double d_dz = m_v3d_obj.z() - m_v3d_diningTable.z();
			double d_dx = m_v3d_obj.x() - m_v3d_diningTable.x();
			if(d_diningTableWidth - fabs(d_dz) <= d_diningTableLength - fabs(d_dx))
			{
				cout << "on the longer side" << endl;
				Vector3d v3d_side = m_v3d_robotPos;
				v3d_side.x(m_v3d_obj.x());
				m_pRO_robot->setPosition(v3d_side);
			}
			else
			{
				cout << "on the shorter side" << endl;
				Vector3d v3d_side = m_v3d_robotPos;
				v3d_side.z(m_v3d_obj.z());
				m_pRO_robot->setPosition(v3d_side);
			}
			m_i_state = 30;
		}
		else if(m_i_state == 30)
		{
			//grasp
			sleep(2.0);
			m_i_state = 40;
		}
		else if(m_i_state == 40)
		{
			Vector3d v3d_pt;
			switch(m_i_trashClass)
			{
			case 0:
				cout << "throw 0" << endl;
				v3d_pt = m_v3d_trash0;
				v3d_pt.z(v3d_pt.z() + d_pathWidth);
				m_pRO_robot->setPosition(v3d_pt);
				break;
			case 1:
				cout << "throw 1" << endl;
				v3d_pt = m_v3d_trash1;
				v3d_pt.z(v3d_pt.z() + d_pathWidth);
				m_pRO_robot->setPosition(v3d_pt);
				break;
			case 2:
				cout << "throw 2" << endl;
				v3d_pt = m_v3d_trash2;
				v3d_pt.z(v3d_pt.z() + d_pathWidth);
				m_pRO_robot->setPosition(v3d_pt);
				break;
			default:
				cout << "throw wagon" << endl;
				v3d_pt = m_v3d_wagon;
				v3d_pt.z(v3d_pt.z() + d_pathWidth);
				m_pRO_robot->setPosition(v3d_pt);
			}
			
		}
	}
  	return TIMEOUT;      
}  
  
void MyController::onRecvMsg(RecvMsgEvent &evt) 
{  
	std::string msg = evt.getMsg();
	if(msg == "Task_start")
	{
		m_b_start = false;
		sleep(1.0);
		m_b_start = true;
		m_pSO_obj = NULL;
		m_i_state = 0;
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

