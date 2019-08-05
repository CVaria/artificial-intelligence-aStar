#include"defs.h"
#include<iostream>
#include<cmath>
#include <vector>
using namespace :: std;
/*Class Point--*/
Point::Point()
{}
Point::Point(int a, int b){
	x = a;
	y = b;
}
int Point::getX()
{
	return x;
}
int Point::getY()
{
	return y;
}
void Point::setX(int a)
{
	x = a;
	return ;
}
void Point::setY(int a)
{
	y = a;
	return ;
}
bool Point:: operator==(Point a)
{
	if(a.getX()==getX() && a.getY()==getY())
		return true;
	else
		return false;
}
void Point::print()
{
	cout<<"x="<<x+1<<" y="<<y+1;//<<endl;
	return;
}
/*--Class Point*/
/*Plane Point*/
char PlanePoint::getState()
{
	return state;	
}
void PlanePoint::setState(char a)
{
	state = a;
	return;
}  
void PlanePoint:: setNode(Node* tmp)
{
	aStar1 = tmp;
	return;
}
Node* PlanePoint:: getNode()
{
	return aStar1;
}
void PlanePoint::print()
{
	cout<<state;
	cout<<"points:"<<aStar1<<endl;
	return;
}
PlanePoint::PlanePoint ()
{
	state = 'Z';
	aStar1=NULL;
}
/*--Plane Point*/
/*Node--*/
Node::Node()
{}
Node::Node(int temp_g, int temp_status, Point temp_pos, Node* temp_cameFrom,Point goal)
{
	g=temp_g;
	status=temp_status;
	cameFrom=temp_cameFrom;
	pos=Point(temp_pos.getX(), temp_pos.getY());
	h=manhattanDistance(goal);
//h=nonAdmissible(goal);
	f=g+h;
}
Point Node:: getPos()
{
	return pos;
}

Node* Node:: getCameFrom(){
	return cameFrom;
}

void Node:: setCameFrom(Node * tmp)
{
	cameFrom=tmp;
	return;
}
int Node::getStatus()
{
	return status;
}
int Node:: getF()
{
	return f;
}
int Node:: getG()
{
	return g;
}
int Node:: getH()
{
	return h;
}
void Node:: setF(int tmp_f)
{
	 f=tmp_f;
	return;
}
void Node:: setG(int temp_g)
{
	g=temp_g;
	return;
}
void Node:: setStatus(int temp_stat)
{
	status=temp_stat;
	return;
}
int Node:: nonAdmissible(Point target)
{
	int tmp = manhattanDistance(target);
	return tmp*tmp;
}
int Node:: manhattanDistance(Point target)
{
	return abs(pos.getX()-target.getX())+ abs(pos.getY()-target.getY());
}
vector< Point> Node:: find_neighbors(Point limits)
{
	int lim_x=limits.getX(), lim_y=limits.getY();
	int tmp_x=pos.getX(), tmp_y=pos.getY();
	vector< Point > result;

	if(tmp_x-1>=0)
		result.push_back(Point(tmp_x-1, tmp_y));
	if(tmp_x+1<lim_x)
		result.push_back(Point(tmp_x+1, tmp_y));
	if(tmp_y-1>=0)
		result.push_back(Point(tmp_x, tmp_y-1));
	if(tmp_y+1<lim_y)
		result.push_back(Point(tmp_x, tmp_y+1));
	return result;
}
