#include "modeler.h"
#include "sin_torque.h"
#include <fstream>

vector<Solid> m_solid;
vector<Joint> m_joint;
static dWorldID world;
static dSpaceID space;
static dGeomID ground;
static dJointGroupID contactgroup;
vector<Torque> leg_joints;
double curTime = 0;
#define BODY_MASS 62;
#define TIMESTEP 1.0/50.0
#define SPEED 1.0

double energy = 0;
double joint_angle[6];

dsFunctions fn;

void control()
{
	dReal k1 = 3.0, fMax = 10;
	dReal angle = 0;
	for(int j = 0; j < JOINT_NUM;j++)
	{
		if(m_joint[j].type == HINGE)
		{
			dReal curAngle = dJointGetHingeAngle(m_joint[j].joint_id);
			dReal z = angle - curAngle;
			dJointSetHingeParam(m_joint[j].joint_id, dParamVel, k1*z);
			dJointSetHingeParam(m_joint[j].joint_id, dParamFMax, fMax);
		}
	}
}

void init_leg()
{
	ifstream input("../data.txt");
	int num = 0;

	for(int i = 0; i < 3; i++)
	{
		input >> leg_joints[i].A;
		input >> leg_joints[i].T;
		input >> leg_joints[i].phi;
		input >> leg_joints[i].C;
		leg_joints[i + 3].A = leg_joints[i].A;
		leg_joints[i + 3].T = leg_joints[i].T;
		leg_joints[i + 3].phi = leg_joints[i].phi;
		leg_joints[i + 3].C = leg_joints[i].C;
	}
	input>>leg_joints[3].phi;
	input>>leg_joints[4].phi;
	input>>leg_joints[5].phi;
	
  for(int i = 0; i < 6; i++)
	{
		joint_angle[i] = 0.0;
	}
}

void controlWithTorque()
{
	double torque;
	double curAngle;
	int ind[6] = {7, 9, 11, 6, 8, 10};

	for(int i = 0; i < 6; i++)
	{
		torque = leg_joints[i].calculate(curTime);
		dJointAddHingeTorque(m_joint[ind[i]].joint_id,torque);
		curAngle = dJointGetHingeAngle(m_joint[ind[i]].joint_id);
		energy += abs((curAngle - joint_angle[i]) / TIMESTEP * torque);
		joint_angle[i] = curAngle;
	}
}

void load() {
	//load solid
	for(int i = 0; i < SOLID_NUM; i++) {
		m_solid[i].name = solid_name[i];
		m_solid[i].type = solid_type[i];
		m_solid[i].scale[0] = solid_scale[i][0]; 
    m_solid[i].scale[1] = solid_scale[i][1]; 
    m_solid[i].scale[2] = solid_scale[i][2];
		m_solid[i].translation[0] = solid_translation[i][0]; 
    m_solid[i].translation[1] = solid_translation[i][1]; 
    m_solid[i].translation[2] = solid_translation[i][2]+3.2;
		m_solid[i].rotation[0] = solid_rotation[i][0]; 
    m_solid[i].rotation[1] = solid_rotation[i][1]; 
    m_solid[i].rotation[2] = solid_rotation[i][2];
	}

	for(int i = 0; i < SOLID_NUM; i++) {
		dMass m;
		m_solid[i].body = dBodyCreate(world);
		
    dBodySetPosition(
      m_solid[i].body, 
      m_solid[i].translation[0], 
      m_solid[i].translation[1], 
      m_solid[i].translation[2]);
		dMatrix3 R;
		
    dRFromEulerAngles(
      R, 
      m_solid[i].rotation[0], 
      m_solid[i].rotation[1], 
      m_solid[i].rotation[2]);
		dBodySetRotation(m_solid[i].body, R);
		
    double density = DENSITY;
		if(m_solid[i].type == CYLINDER) {
			dMassSetCapsule(&m, density, 3,m_solid[i].scale[0], m_solid[i].scale[2]);
      m_solid[i].geom = dCreateCapsule(space, m_solid[i].scale[0], m_solid[i].scale[2]);
		}
		else {
			dMassSetBox(&m, density,m_solid[i].scale[0], m_solid[i].scale[1], m_solid[i].scale[2]);
			m_solid[i].geom = dCreateBox(
        space, m_solid[i].scale[0], m_solid[i].scale[1], m_solid[i].scale[2]);
		}
	  	
    dBodySetMass(m_solid[i].body, &m);
		dGeomSetBody(m_solid[i].geom, m_solid[i].body);
	}
	
	//load joint
  for (int i=0; i<JOINT_NUM; i++) {
		m_joint[i].type = joint_type[i];
    m_joint[i].body_name[0] = body_name[i][0];
    m_joint[i].body_name[1] = body_name[i][1];
    m_joint[i].translation[0] = joint_translation[i][0]; 
    m_joint[i].translation[1] = joint_translation[i][1]; 
    m_joint[i].translation[2] = joint_translation[i][2]+3.2;

		//attach joint
		dBodyID b_id0;
    dBodyID b_id1;
    for (int k=0; k<SOLID_NUM; k++) {   
			if (m_solid[k].name == m_joint[i].body_name[0]) {
				b_id0 = m_solid[k].body;
      }
      if (m_solid[k].name == m_joint[i].body_name[1]) {
				b_id1 = m_solid[k].body;
      }
		}

		if (m_joint[i].type == HINGE) {
			dJointID j = dJointCreateHinge(world,0);
			m_joint[i].joint_id = j;
			dJointAttach(j,b_id0, b_id1);                
			dJointSetHingeAnchor(
        j,m_joint[i].translation[0],m_joint[i].translation[1],m_joint[i].translation[2]);
			dJointSetHingeAxis(j,0.0,1.0,0.0);
					
				//add joint limit Lo/High
			dJointSetHingeParam(j, dParamLoStop, joint_LoHigh[i][0]);
			dJointSetHingeParam(j, dParamHiStop, joint_LoHigh[i][1]);
		}
		else if (m_joint[i].type == FIXED) {
			dJointID j = dJointCreateFixed(world,0);
			m_joint[i].joint_id = j;
			dJointAttach(j,b_id0, b_id1);
			dJointSetFixed(j);
		}
	}

	leg_joints.resize(6);
	
  init_leg();
}

static void start() 
{
	static float xyz[3] = {5.0,-15.0,5.0};
	static float hpr[3] = {90.0,0,0};
	dsSetViewpoint (xyz,hpr);
}

void drawGeom(dGeomID g, const dReal *pos, const dReal *R, int show_aabb)
{
	if (!g) return;
	if (!pos) pos = dGeomGetPosition (g);
	if (!R) R = dGeomGetRotation (g);

	int type = dGeomGetClass (g);
	if (type == dBoxClass) {
		dVector3 sides;
		dGeomBoxGetLengths (g,sides);
		dsDrawBox(pos,R,sides);
	}
	else if(type == dSphereClass) {
		dReal r = dGeomSphereGetRadius (g);
		dsDrawSphere(pos,R,r);
	}
	else {
		dReal radius,length;
		dGeomCapsuleGetParams (g,&radius,&length);
		dsDrawCapsule(pos,R,length,radius);
	}
}

void nearCallback (void *data, dGeomID o1, dGeomID o2) {
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  dContact contact[MAX_CONTACTS];

	if (b1 && b2 && dAreConnected (b1,b2)) return;

  for (int i = 0; i < MAX_CONTACTS; i++)
  {
    if (o1 == m_solid[12].geom || o1 == m_solid[13].geom || 
        o2 == m_solid[12].geom || o2 == m_solid[13].geom) {
      contact[i].surface.mode = dContactRolling | dContactApprox1;
    }
		else {
			contact[i].surface.mode = dContactBounce | dContactSoftCFM;
    }
    contact[i].surface.mu = 0.3;
    contact[i].surface.soft_cfm = 0.01;
		contact[i].surface.rho = 0.1;
  }
  
  if (int numc = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact))) {
    for (int i = 0; i < numc; i++) {
      dJointID c = dJointCreateContact(world, contactgroup, contact + i);
      dJointAttach(c, b1, b2);
    }
  }
}

static void simLoop (int pause) {
	dBodyAddForce(m_solid[0].body,0,0,600);
	const dReal *pos = dBodyGetPosition(m_solid[0].body);
	controlWithTorque();
	curTime += TIMESTEP;
	dSpaceCollide(space,0,&nearCallback);
	dWorldStep(world, TIMESTEP);
	dJointGroupEmpty(contactgroup);
	
	for(int i = 0; i < SOLID_NUM; i++) {
		drawGeom(m_solid[i].geom, 0, 0, 0);
	}
}

static void command (int cmd) {
	switch (cmd) {
		case 'l':dJointAddHingeTorque(m_joint[6].joint_id,-500);break;
		case 'q':exit(0);break;
	}
}

int main(int argc, char **argv)
{
	dInitODE2(0);
	world = dWorldCreate();
	space = dHashSpaceCreate(0);
	contactgroup = dJointGroupCreate(0);
	m_solid.resize(SOLID_NUM);
	m_joint.resize(JOINT_NUM);
	dWorldSetGravity(world,0,0,-9.8);
	ground = dCreatePlane(space,0,0,1,0);
	load();
	fn.version = DS_VERSION;
	fn.start   = &start;
	fn.step    = &simLoop;
	fn.command = &command;
	fn.stop    = NULL;
	fn.path_to_textures = "..";
	
	dsSimulationLoop (argc,argv,1024,611,&fn);
	dWorldDestroy(world);
	dSpaceDestroy(space);
	dCloseODE();
	
  return 0;
}
