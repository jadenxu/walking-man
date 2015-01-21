#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include<iostream>
#include<string>
#include<vector>
using namespace std;

#define MAX_CONTACTS 10
#ifndef PI
#define PI  3.14159
#endif
#define SOLID_NUM 16
#define JOINT_NUM 15
#define DENSITY  100
#ifdef dDOUBLE
#define dsDrawBox  dsDrawBoxD
#define dsDrawSphere   dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule  dsDrawCapsuleD
#define dsDrawLine     dsDrawLineD
#endif

enum SolidType {BOX, CYLINDER, SPHERE};
enum JointType {UNIVERSAL, HINGE, FIXED};


dReal solid_translation[SOLID_NUM][3] = {
  {0,0,1},
  {0,0.4,2},
  {0,-0.4,2},
  {0,0.8,1.6},
  {0,-0.8,1.6}, 
  {0,0.8,0.8}, 
  {0,-0.8,0.8},
  {0,0,0},
  {0,0.5,-0.75},
  {0,-0.5,-0.75},
  {0,0.5,-2.25},
  {0,-0.5,-2.25},
  {0,0.5,-3.1},
  {0,-0.5,-3.1},
  {0,0,2.4},
  {0,0,3}};

dReal solid_scale[SOLID_NUM][3] = {
  {0.1,0.1,2},
  {0.1,0.1,0.8},
  {0.1,0.1,0.8},
  {0.1,0.1,0.8}, 
  {0.1,0.1,0.8}, 
  {0.1, 0.1, 0.8},
  {0.1, 0.1, 0.8}, 
  {0.1,0.1,1}, 
  {0.1,0.1,1.5}, 
  {0.1,0.1,1.5},
  {0.1,0.1,1.5},
  {0.1,0.1,1.5},
  {0.6,0.6,0.2},
  {0.6,0.6,0.2},
  {0.1,0.1,0.8},
  {0.1,0.1,0.4}};

SolidType solid_type[SOLID_NUM] = {
  CYLINDER, 
  CYLINDER, 
  CYLINDER, 
  CYLINDER, 
  CYLINDER, 
  CYLINDER, 
  CYLINDER, 
  CYLINDER, 
  CYLINDER,
  CYLINDER,
	CYLINDER,
  CYLINDER,
  BOX,
  BOX, 
  CYLINDER, 
  CYLINDER};

dReal solid_rotation[SOLID_NUM][3] = {
  {0,0,0}, 
  {-PI/2,0,0}, 
  {PI/2,0,0}, 
  {0,0,0}, 
  {0,0,0}, 
  {0,0,0}, 
  {0,0,0}, 
  {PI/2,0,0}, 
  {0,0,0}, 
  {0,0,0},
  {0,0,0},
  {0,0,0},
  {0,0,0},
  {0,0,0},
  {0,0,0},
  {0,0,0}};

string solid_name[SOLID_NUM] = {
  "trunk", 
  "right_shoulder", 
  "left_shoulder", 
  "right_arm", 
  "left_arm", 
  "right_forearm", 
  "left_forearm",
  "hip",
  "right_leg", 
  "left_leg", 
  "right_shank", 
	"left_shank",
  "right_foot",
  "left_foot",
  "neck",
  "head"};

JointType joint_type[JOINT_NUM] = {
	FIXED,//0
	FIXED,//1
	FIXED,//2
	FIXED,//3
	FIXED,//4
	FIXED,//5
	HINGE,//6
	HINGE,//7
	HINGE,//8
	HINGE,//9
	HINGE,//10
	HINGE,//11
	FIXED,//12
	FIXED,//13
	FIXED //14
};

dReal joint_translation[JOINT_NUM][3] = {
  {0,0,2.8},
  {0,0,2},
  {0,0.8,2},
  {0,-0.8,2},
  {0,0.8,1.2},
  {0,-0.8,1.2},
  {0,0.5,0},
  {0,-0.5,0},
  {0,0.5,-1.5},
  {0,-0.5,-1.5},
  {0,0.5,-3},
  {0,-0.5,-3},
  {0,0,0},
  {0,0,2},
  {0,0,2}};

string body_name[JOINT_NUM][2] ={
	{"head","neck"},//#0
	{"neck","trunk"},//#1
	{"right_arm","right_shoulder"},//#2
	{"left_arm","left_shoulder"},//#3
  {"right_forearm","right_arm"},//#4
	{"left_forearm","left_arm"},//#5
	{"right_leg","hip"},//#6
	{"left_leg","hip"},//#7
	{"right_leg","right_shank"},//#8
	{"left_leg","left_shank"},//#9
	{"right_shank","right_foot"},//#10
	{"left_shank","left_foot"},//#11
	{"trunk","hip"},//#12
	{"left_shoulder","trunk"},//#13
	{"right_shoulder","trunk"}//#14
};

dReal joint_LoHigh[JOINT_NUM][2] = {
	{- PI / 15,PI / 15},//#0: head and neck
	{- PI / 15, PI / 15},//#1: neck and trunk
	{- PI / 2, PI / 2},//#2: right_arm and rightshoulder
	{- PI / 2, PI / 2},//#3: left_arm and left_shoulder
	{- PI / 2.0 * 3.0, 0},//#4:right_forearm and right_arm
	{- PI / 2.0 * 3.0, 0},//#5:left_forearm and left_arm
	{-165.0 / 180.0 * PI, 60.0 / 180.0 * PI},//#6:right_leg and hip
	{-165.0 / 180.0 * PI, 60.0 / 180.0 * PI},//#7:left_leg and hip
	{-165.0 / 180.0 * PI, 0},//#8:right_leg and right_shank
	{-165.0 / 180.0 * PI, 0},//#9:left_leg and left_shank
	{- 10.0 / 180.0 * PI, 10.0 / 180.0 * PI},//#10:right_shank and right_foot
	{- 10.0 / 180.0 * PI, 10.0 / 180.0 * PI},//#11:left_shank and left_foot
	{- PI / 2, 0},//#12:trunk and hip
	{-dInfinity, dInfinity},//#13:left_shoulder and trunk
	{-dInfinity, dInfinity}//#14:right_shoulder and trunk
};

struct Solid
{
  string name;
	SolidType type;
	dVector3 scale;
	dVector3 translation;
	dVector3 rotation;
	dBodyID body;
	dGeomID geom;
};

struct Joint
{
	dJointID joint_id;
	JointType type;
	string body_name[2];
	dVector3 translation;
};
