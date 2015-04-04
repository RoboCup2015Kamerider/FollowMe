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
  	
  	
  	bool turnToObj(Vector3d v3d_robot, Vector3d v3d_obj, Rotation r_robot)
  	{
  		Vector3d v3d_diff;
  		v3d_diff.x(v3d_obj.x() - v3d_robot.x());
  		v3d_diff.z(v3d_obj.z() - v3d_robot.z());
  		double d_diffTheta = atan2(v3d_diff.x(), v3d_diff.z());
  		double d_robotDir = 2 * asin(r_robot.qy());
  		
  		//cout << "Theta = " << d_diffTheta << " robot dir = " << d_robotDir << endl;
  		
  		double d_dtheta = d_diffTheta - d_robotDir;
  		if(fabs(d_dtheta) > 0.02)
		{
			double d_K = 0.25;
			// P control
			double d_vel = d_K * d_dtheta;
			m_pRO_robot->setWheelVelocity(-d_vel, d_vel);
			return false;
		}
		else
		{
			m_pRO_robot->setWheelVelocity(0, 0);
			return true;
		}
  	}
  	
public:
	RobotObj *m_pRO_robot;
	SimObj *m_pSO_obj;
	
	double m_d_wheelRadius;				// 机器人轮子半径
	double m_d_wheelDistance;			// 机器人两轮间距离
	
	Vector3d m_v3d_robotPos;
	string m_graspObjectName;
	
	Vector3d m_v3d_trash0;
	Vector3d m_v3d_trash1;
	Vector3d m_v3d_trash2;
	Vector3d m_v3d_wagon;
	
	Rotation m_r_robotRot;
	double m_d_robotRot;
	
	int m_i_state;
	
	bool m_b_start;
	bool m_b_dir;
	bool m_b_grasp;
	
	Vector3d m_v3d_trash;
	
	Vector3d m_v3d_diningTable;
	
	Vector3d m_v3d_dingTableRelayPoint[4];
	
	Vector3d m_v3d_obj;
	
	int m_i_trashClass;
	
	double d_diningTableWidth;
	double d_diningTableLength;
	
	double d_pathWidth;
	
public:
	bool moveToObj(Vector3d v3d_robot, Vector3d v3d_obj)
  	{
  		double d_diff = sqrt(pow(v3d_robot.x() - v3d_obj.x(), 2) + pow(v3d_robot.z() - v3d_obj.z(), 2));
  		if(d_diff - 20 > 1)
		{
			double d_K = 0.5;
		
			// P control
			double d_vel = d_K * d_diff / m_d_wheelRadius;
				
			m_pRO_robot->setWheelVelocity(d_vel, d_vel);
			return false;
		}
		else if(d_diff > 1)
		{
			double d_vel = d_diff / m_d_wheelRadius;
			m_pRO_robot->setWheelVelocity(d_vel, d_vel);
			sleep(1.0);
			m_pRO_robot->setWheelVelocity(0.0, 0.0);
		}
		else
		{
			m_pRO_robot->setWheelVelocity(0.0, 0.0);
		}
		return true;
  	}
};  
  
void MyController::onInit(InitEvent &evt) 
{
	m_pRO_robot = getRobotObj(myname());		// 得到机器人指针
	
	m_d_wheelRadius = 10.0;
	m_d_wheelDistance = 10.0;
	m_b_start = false;
	m_b_grasp = false;
	
	m_i_state = 0;
	m_b_dir = false;
	
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
	
	d_diningTableWidth = 100;
	d_diningTableLength = 160;
	
	d_pathWidth = 40;
	
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
		m_pRO_robot->getRotation(m_r_robotRot);
		
		//cout << "robot pos = " << m_v3d_robotPos.x() << " " << m_v3d_robotPos.z() << endl;
		
		if(m_i_state == 0)
		{
			//cout << "in state 0" << endl;
			sleep(1.0);
			std::vector<std::string> m_entities;
			getAllEntities(m_entities);
			for(int i = 0; i < m_entities.size(); i++)
			{
				if(m_entities[i] == "can_1")
				{
					m_pSO_obj = getObj("can_1");
					m_graspObjectName = "can_1";
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
					m_graspObjectName = "petbottle_4";
					Vector3d v3d_obj;
					m_pSO_obj->getPosition(v3d_obj);
					if(v3d_obj.length() < 1e4)
					{
						m_i_trashClass = 0;
						break;
					}
				}
				
			}
			// init arm
			m_pRO_robot->setJointVelocity("RARM_JOINT4", PI, PI);
			sleep(1.0);
			m_pRO_robot->setJointVelocity("RARM_JOINT4", 0, 0);
			m_pSO_obj->getPosition(m_v3d_obj);
			//cout << "Obj pos = " << m_v3d_obj.x() << ", " << m_v3d_obj.z() << endl;
			m_i_state = 10;
		}
		else if(m_i_state == 10)
		{
			//cout << "in state 10" << endl;
			double d_robotRelayPoint[4];
			double d_minDis = 1e10;
			int i_minIdx = 0;
			double d_minObjDis = 1e10;
			int i_minObjIdx = 0;
			int i;
			for(i = 0; i < 4; i++)
			{
				double d_objDis = _distance(m_v3d_dingTableRelayPoint[i], m_v3d_obj);
				if(d_objDis < d_minObjDis)
				{
					d_minObjDis = d_objDis;
					i_minObjIdx = i;
				}
			}
			
			for(i = 0; i < 4; i++)
			{
				d_robotRelayPoint[i] = _distance(m_v3d_robotPos, m_v3d_dingTableRelayPoint[i]);
				
				//cout << "dis " << i << " = " << d_robotRelayPoint[i] << endl;
				//m_pRO_robot->setPosition(m_v3d_dingTableRelayPoint[i]);
				//sleep(1.0);
				if(d_robotRelayPoint[i] < d_minDis && fabs(i - i_minObjIdx) != 2)
				{
					d_minDis = d_robotRelayPoint[i];
					i_minIdx = i;
				}
				
			}
			m_v3d_dingTableRelayPoint[i_minIdx].y(30);
			
			Vector3d v3d_p1 = m_v3d_dingTableRelayPoint[i_minIdx];
			
			if(m_b_dir == false)
			{
				bool b_toward = turnToObj(m_v3d_robotPos, v3d_p1, m_r_robotRot);
				if(b_toward == false)
				{
					return TIMEOUT;
				}
				else
				{
					m_b_dir = true;
				}
			}
			bool b_reach = moveToObj(m_v3d_robotPos, v3d_p1);
			if(b_reach == false)
			{
				return TIMEOUT;
			}
			m_b_dir = false;
			//return TIMEOUT;
			//m_pRO_robot->setPosition(m_v3d_dingTableRelayPoint[i_minIdx]);
			//sleep(2.0);
			
			//m_pRO_robot->setPosition(m_v3d_dingTableRelayPoint[i_minObjIdx]);
			m_i_state = 20;
		}
		else if(m_i_state == 20)
		{
			double d_dz = m_v3d_obj.z() - m_v3d_diningTable.z();
			double d_dx = m_v3d_obj.x() - m_v3d_diningTable.x();
			if(d_diningTableWidth - fabs(d_dz) <= d_diningTableLength - fabs(d_dx))
			{
				//cout << "on the longer side" << endl;
				Vector3d v3d_side = m_v3d_robotPos;
				v3d_side.x(m_v3d_obj.x());
				v3d_side.y(30);
				
				if(m_b_dir == false)
				{
					bool b_toward = turnToObj(m_v3d_robotPos, v3d_side, m_r_robotRot);
					if(b_toward == false)
					{
						return TIMEOUT;
					}
					else
					{
						m_b_dir = true;
					}
				}
				bool b_reach = moveToObj(m_v3d_robotPos, v3d_side);
				if(b_reach == false)
				{
					return TIMEOUT;
				}
				//m_b_dir = false;
				
				//m_pRO_robot->setPosition(v3d_side);
				
				Vector3d v3d_objDir;
				v3d_objDir.x(m_v3d_obj.x() - v3d_side.x());
				v3d_objDir.z(m_v3d_obj.z() - v3d_side.z());
				
				double d_objDir = atan2(v3d_objDir.x(), v3d_objDir.z());
				
				m_pRO_robot->getRotation(m_r_robotRot);
				
				double d_robotDir = 2 * asin(m_r_robotRot.qy());
				double d_dtheta = d_robotDir - d_objDir;
				
				if(d_dtheta > PI)
				{
					d_dtheta -= 2 * PI;
				}
				if(d_dtheta < -PI)
				{
					d_dtheta += 2 * PI;
				}
				
				//cout << "robot dir = " << d_robotDir <<" objDir = " << d_objDir << " d_dtheta = " << d_dtheta << endl;
				
				if(fabs(d_dtheta) > 0.02)
				{
					double d_K = 0.25;
					// P control
					double d_vel = d_K * d_dtheta;
					m_pRO_robot->setWheelVelocity(-d_vel, d_vel);
					return TIMEOUT;
				}
			}
			else
			{
				//cout << "on the shorter side" << endl;
				Vector3d v3d_side = m_v3d_robotPos;
				v3d_side.z(m_v3d_obj.z());
				v3d_side.y(30);
				m_pRO_robot->setPosition(v3d_side);
			}
			m_b_dir = false;
			m_i_state = 30;
		}
		else if(m_i_state == 30)
		{
			//grasp
			double d_rarm_joint0 = m_pRO_robot->getJointAngle("RARM_JOINT0");
			double d_rarm_joint1 = m_pRO_robot->getJointAngle("RARM_JOINT1");
			double d_rarm_joint4 = m_pRO_robot->getJointAngle("RARM_JOINT4");
			m_pRO_robot->setJointVelocity("RARM_JOINT0", PI / 8 - d_rarm_joint0, PI / 8 - d_rarm_joint0);
			sleep(1.0);
			m_pRO_robot->setJointVelocity("RARM_JOINT0", 0, 0);
			m_pRO_robot->setJointVelocity("RARM_JOINT1", -PI / 8 - d_rarm_joint1, -PI / 8 - d_rarm_joint1);
			sleep(1.0);
			m_pRO_robot->setJointVelocity("RARM_JOINT1", 0, 0);
			m_pRO_robot->setJointVelocity("RARM_JOINT4", -PI / 2 - d_rarm_joint4, -PI / 2 - d_rarm_joint4);
			sleep(1.0);
			m_pRO_robot->setJointVelocity("RARM_JOINT4", 0, 0);
			
			
			if(m_b_grasp == false)
			{
				m_pRO_robot->setWheelVelocity(0.5, 0.5);
				sleep(1.0);
				m_pRO_robot->setWheelVelocity(0.0, 0.0);
				return TIMEOUT;
			}
			else
			{
				m_pRO_robot->setWheelVelocity(-2.0, -2.0);
				sleep(1.0);
				m_pRO_robot->setWheelVelocity(0.0, 0.0);
			}
			m_i_state = 35;
		}
		else if(m_i_state == 35)
		{
			switch(m_i_trashClass)
			{
			case 0:
				//cout << "throw 0" << endl;
				m_v3d_trash = m_v3d_trash0;
				m_v3d_trash.z(m_v3d_trash.z() + d_pathWidth);
				m_v3d_trash.y(30);
				//m_pRO_robot->setPosition(m_v3d_trash);
				
				
				
				break;
			case 1:
				//cout << "throw 1" << endl;
				m_v3d_trash = m_v3d_trash1;
				m_v3d_trash.z(m_v3d_trash.z() + d_pathWidth);
				m_v3d_trash.y(30);
				//m_pRO_robot->setPosition(m_v3d_trash);
				
				
				break;
			case 2:
				//cout << "throw 2" << endl;
				m_v3d_trash = m_v3d_trash2;
				m_v3d_trash.z(m_v3d_trash.z() + d_pathWidth);
				m_v3d_trash.y(30);
				//m_pRO_robot->setPosition(m_v3d_trash);
				break;
			default:
				//cout << "throw wagon" << endl;
				m_v3d_trash = m_v3d_wagon;
				m_v3d_trash.z(m_v3d_trash.z() + d_pathWidth + 40);
				m_v3d_trash.y(30);
				//m_pRO_robot->setPosition(m_v3d_trash);
			}
			//m_pRO_robot->getPosition(m_v3d_robotPos);
			m_i_state = 42;
		}
		else if(m_i_state == 42)
		{
			double d_trashRelayPoint[4];
			double d_minDis = 1e10;
			int i_minIdx = 0;
			int i;
			for(i = 0; i < 4; i++)
			{
				double d_trashDis = _distance(m_v3d_dingTableRelayPoint[i], m_v3d_trash);
				if(d_trashDis < d_minDis)
				{
					d_minDis = d_trashDis;
					i_minIdx = i;
				}
			}
			
			if(m_b_dir == false)
			{
				bool b_toward = turnToObj(m_v3d_robotPos, m_v3d_dingTableRelayPoint[i_minIdx], m_r_robotRot);
				if(b_toward == false)
				{
					return TIMEOUT;
				}
				else
				{
					m_b_dir = true;
				}
			}
			bool b_reach = moveToObj(m_v3d_robotPos, m_v3d_dingTableRelayPoint[i_minIdx]);
			if(b_reach == false)
			{
				return TIMEOUT;
			}
			m_b_dir = false;
			m_i_state = 44;
		}
		else if(m_i_state == 44)
		{
			if(m_b_dir == false)
			{
				bool b_toward = turnToObj(m_v3d_robotPos, m_v3d_trash, m_r_robotRot);
				if(b_toward == false)
				{
					return TIMEOUT;
				}
				else
				{
					m_b_dir = true;
				}
			}
			bool b_reach = moveToObj(m_v3d_robotPos, m_v3d_trash);
			if(b_reach == false)
			{
				return TIMEOUT;
			}
			m_b_dir = false;
			m_i_state = 48;
		}
		else if(m_i_state == 48)
		{
			double d_objDir = atan2(0, -30);
				
			m_pRO_robot->getRotation(m_r_robotRot);
				
			double d_robotDir = 2 * asin(m_r_robotRot.qy());
			double d_dtheta = d_robotDir - d_objDir;
				
			/*if(d_dtheta > PI)
			{
				d_dtheta -= 2 * PI;
			}
			if(d_dtheta < -PI)
			{
					d_dtheta += 2 * PI;
			}*/
				
			//cout << "robot dir = " << d_robotDir <<" objDir = " << d_objDir << " d_dtheta = " << d_dtheta << endl;
				
			if(fabs(d_dtheta) > 0.02)
			{
				double d_K = 0.25;
				// P control
				double d_vel = d_K * d_dtheta;
				m_pRO_robot->setWheelVelocity(-d_vel, d_vel);
				return TIMEOUT;
			}
			m_i_state = 50;
			
		}
		else if(m_i_state == 50)
		{
			//cout << "Throw" << endl;
			CParts *parts = m_pRO_robot->getParts("RARM_LINK7");

			// release grasping
			parts->releaseObj();

			// wait a bit
			sleep(1);

			// set the grasping flag to neutral
			m_b_grasp = false;
			m_i_state = 60;
		}
		else if(m_i_state == 60)
		{
			m_pRO_robot->setWheelVelocity(0.0, 0.0);
			broadcastMsg("get message: Task_finished");
			m_i_state = 70;
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
	if (m_b_grasp == false) 
	{
		typedef CollisionEvent::WithC C;
		// Get name of entity which is touched by the robot
		const std::vector<std::string> & with = evt.getWith();
		// Get parts of the robot which is touched by the entity
		const std::vector<std::string> & mparts = evt.getMyParts();

		// loop for every collided entities
		for(int i = 0; i < with.size(); i++) 
		{
			if(m_graspObjectName == with[i]) 
			{
				// If the right hand touches the entity
				if(mparts[i] == "RARM_LINK7") 
				{
					SimObj *my = getObj(myname());
					CParts * parts = my->getParts("RARM_LINK7");
					if(parts->graspObj(with[i])) 
					{
						m_b_grasp = true;
					}
				}
			}
		}
	}
}
  
extern "C" Controller * createController() 
{  
  	return new MyController;  
}  

