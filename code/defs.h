#include <vector>
using namespace::std;
class Point{	
	private:
		int x,y;
	public:
		Point (int, int);
		Point (int, int, Point);
		Point();
		int getX();
		int getY();
		void setX(int);
		void setY(int);
		void print();
		bool operator==(Point);
};
class Node{
	private:
		int g, h, f;
		int status;
		Point pos;
		Node* cameFrom;
	public:
		Node(int, int, Point, Node*, Point);
		Node();
		void setCameFrom(Node*);
		Node* getCameFrom();
		int getG();
		int getF();
		int getH();
		void setG(int);
		void setF(int);
		void setStatus(int);
		int getStatus();	
		Point getPos();
		vector<Point> find_neighbors(Point);
		int manhattanDistance(Point);	
		int nonAdmissible(Point);
};
class PlanePoint{
	private:
		char state; /*state ='O' or 'X'*/
		Node* aStar1;/*points to a Node type, if it is created*/
	public: 
		PlanePoint();
		PlanePoint(char, Node*, Node*);
		char getState();
		void setState(char);
		void setNode(Node*);
		Node* getNode();
		void print();
};
