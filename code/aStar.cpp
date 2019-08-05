#include<iostream>
#include"defs.h"
#include<vector>
#include<algorithm>
#include<stdio.h>
#include<queue>

using namespace::std;
bool isMember(Point x, vector< Point> vec);
void print_results(vector< Point > , vector< Point > , int *);
vector <Point> result1, result2;
long long total_nodes=0;
vector<Point> aStar(Point , Point , Point , vector< vector< PlanePoint> >, int , int *);
int main()
{
	int cols, rows, tmp_x, tmp_y;

	/*Read size of plane*/
	scanf("%d %d", &cols, &rows);

	vector< vector< PlanePoint > > my_plane(rows, vector< PlanePoint > (cols));
	/*Read robot's initial positions*/
	scanf("%d %d", &tmp_x, &tmp_y);
	Point robot1(tmp_x-1, tmp_y-1);

	scanf("%d %d", &tmp_x, &tmp_y);
	Point robot2(tmp_x-1, tmp_y-1);

	/*Read goal position*/
	scanf("%d %d", &tmp_x,&tmp_y);
	Point goal(tmp_x-1, tmp_y-1);
	vector <Point> temp1, temp2;
	int t1=0,t2=0;

	/*Read meeting points*/
	int points;
	Point* temp_point;

	vector< Point > target1;
	scanf("%d", &points);

	for(int i = 0;i < points ;i++){
		scanf("%d %d", &tmp_x, &tmp_y);
		temp_point = new Point(tmp_x-1, tmp_y-1);
		target1.push_back(*temp_point);				
	}
	target1.push_back(*(new Point(goal.getX(), goal.getY())));
	
	/*Read plane*/
	char tmp;
	getchar();
	for(int i=0;i<rows;i++)	{
		for(int j=0;j<cols;j++){
			tmp = getchar();
			my_plane[i][j].setState(tmp);
		}
		getchar();/*for the "\n"*/
	}

	int temp_rx, temp_ry;
	temp1 = aStar(robot1, target1[0], Point(cols, rows), my_plane, 1, &t1);
	temp2 = aStar(robot2, target1[0], Point(cols, rows), my_plane, 2, &t2);
	if(temp1.empty()){
		printf("Cant reach target, mission aborted\n");
		exit(1);
		}

	if(t1>t2){
		temp_rx=robot1.getX();
		temp_ry=robot1.getY();
		robot1.setX(robot2.getX());
		robot1.setY(robot2.getY());
		robot2.setX(temp_rx);
		robot2.setY(temp_ry);		
	}
	/*Call A* function for robots...*/
	Point init = Point(robot1.getX(), robot1.getY());
	vector< Point > temp;
	int init_g1=0;
	
	/* A* from initial position till goal*/
	init.setX(robot1.getX());
	init.setY(robot1.getY());
	for(int i = 0; i < (int)target1.size(); i++){
		temp = aStar(init, target1[i], Point(cols, rows), my_plane, 1, &init_g1);
		if(!temp.empty()){
			result1.insert(result1.begin(), temp.begin(), temp.end()-1);
		}
		else{
			printf("Cant reach target, mission aborted\n");
			exit(1);
		}
		init = target1[i];	
	}
	int init_g2=0;

	/*A* for 2nd robot from initial position till goal*/
	init.setX(robot2.getX());
	init.setY(robot2.getY());
	for(int i = 0; i < (int)target1.size(); i++){
		temp = aStar(init, target1[i], Point(cols, rows), my_plane, 2, &init_g2);
		if(!temp.empty()){
			result2.insert(result2.begin(), temp.begin(), temp.end()-1);
		}
		else{
			printf("Cant reach target, mission aborted\n");
			exit(1);
		}
		init = target1[i];	
	}

	result2.erase(result2.begin());
	Point pos_goal;
	vector<Point> temp1_res;
	vector <Point> temp2_res;
	pos_goal.setX(result1[1].getX());
	pos_goal.setY(result1[1].getY());

	/*Store temporarily first A* in vectors*/
	temp1_res.insert(temp1_res.begin(), result1.begin(), result1.end());
	result1.clear();
	temp2_res.insert(temp2_res.begin(), result2.begin(), result2.end());
	result2.clear();

	/* A* for robot 1, returning to initial position*/
	init_g1=1;
	temp = aStar(goal, robot1, Point(cols, rows), my_plane, 1, &init_g1);
	if(!temp.empty())
		result1.insert(result1.begin(), temp.begin(), temp.end());

	/* A* for robot 2, returning to initial position */
	init_g2=1;
	temp = aStar(pos_goal, robot2, Point(cols, rows), my_plane, 2, &init_g2);

	if(!temp.empty())
		result2.insert(result2.begin(), temp.begin(), temp.end());

	printf("\t-------Final results-------\n");
	/*Add robots' initial positions to vectors*/
	temp1_res.push_back(robot1);
	temp2_res.push_back(robot2);

	/*Reverse vectors to be in normal order*/
	reverse(temp1_res.begin(), temp1_res.end());
	reverse(temp2_res.begin(), temp2_res.end());

	/*Print results for robots' 1st A* */
	int time=0;
	print_results(temp1_res, temp2_res, &time);

	/*Erase initial positions (they are in previous vectors)*/
	result1.erase(result1.end());
	result2.erase(result2.end());

	/*Reverse vectors to be in normal order*/
	reverse(result1.begin(), result1.end());
	reverse(result2.begin(), result2.end());	

	/*Print results for robots' 2nd A* */
	print_results(result1, result2, &time);
printf("Total Nodes created: %lld\n", total_nodes);
	return 0;
}
void print_results(vector< Point > r1, vector< Point > r2, int *time)
{
	int i1, i2;
	for(i1 = 0, i2=0; i1 != (int)r1.size()-1 || i2 != (int)r2.size()-1;){
		if(r1[i1]==r2[i2])
			i2--;
		printf("Step %d : Robot 1 ", (*time));
		r1[i1].print();
		printf(" and Robot 2 ");
		r2[i2].print();
		printf("\n");
		if(i1<(int)r1.size()-1)
			i1++;
		if(i2<(int)r2.size()-1)
			i2++;
		(*time)++;
	}
	printf("Step %d : Robot 1 ", (*time));
	r1[i1].print();
	printf(" and Robot 2 ");
	r2[i2].print();
	printf("\n");
	(*time)++;
	return ;
}

/* A* */
vector<Point> aStar(Point start, Point goal, Point limits, vector< vector < PlanePoint> > my_plane, int robot, int * init_g)
{
	vector< Node > open_set, closed_set;
	vector< Point > Pos_Points, Path;
	int BestF, Best_i;
	bool is_conflict=false;
	Node *current, *Goal_Node;
	/*Put initial node in open_set*/
	open_set.push_back(Node(*init_g,1,start, NULL, goal));

	while(!open_set.empty()) {
		/*Find node with minimum  F*/
		BestF=open_set[0].getF();
		Best_i=0;
		for(int i=1; i<(int)open_set.size();i++){
			if(BestF>open_set[i].getF()){
				BestF=open_set[i].getF();
				Best_i=i;
			}
		}

		/*Take node with Fmin from open_set*/
		current = new Node(open_set[Best_i].getG(), open_set[Best_i].getStatus(), open_set[Best_i].getPos() ,open_set[Best_i].getCameFrom(), goal);
		(*current).setStatus(0);		/*Change status to closed*/

		/*If current==goal, return path to start*/
		if(current->getPos()== goal){
			/*Calculate path as Points, and put it in vector "Path"*/
			/*Keep time for robot to reach goal*/
			(*init_g) = current->getG();
			for(Goal_Node = current; Goal_Node != NULL; Goal_Node = Goal_Node->getCameFrom()){
				Path.push_back(Point(Goal_Node->getPos().getX(),Goal_Node->getPos().getY()));
			}	
			total_nodes+=closed_set.size()+open_set.size();
			printf("\n");
			return Path;
		}

		closed_set.push_back(*current);
		open_set.erase(open_set.begin()+Best_i);

		/*Put neighbors points in vector*/
		Pos_Points=(*current).find_neighbors(limits);

		Point temp_curr;/*Temporary Point neighbor*/
		Node * newNode;
		for(int i=0; i<(int)Pos_Points.size(); i++){
			temp_curr=Pos_Points[i];//neighbor
			printf("Robot %d considering ", robot);
			/*Printing "alternative" in case there was conflict in previous step*/
			if(is_conflict){
				printf("alternative ");
				is_conflict=false;
			}
			printf("new position<%d, %d> at step %d", temp_curr.getX()+1, temp_curr.getY()+1, current->getG());
			int temp_size = result1.size();

			PlanePoint *temp_plane_p=&my_plane[temp_curr.getY()][temp_curr.getX()];
			if(robot==2){

				if((current->getG()+1<=temp_size) && (temp_curr == result1[temp_size-current->getG()]) && (current->getPos()== result1[temp_size-current->getG()-1])){
					/*Robot 1 and 2 exchange positions, make all possible neighbors*/
					vector <Point> temp_list;
					temp_list=(*current).find_neighbors(limits);
					int temp_i;
					for(temp_i=0; temp_i<(int)temp_list.size(); temp_i++){	
						PlanePoint *temp_plane_p1=&my_plane[temp_list[temp_i].getY()][temp_list[temp_i].getX()];
						if(!(temp_list[temp_i]==result1[temp_size-current->getG()]) && ((*temp_plane_p1).getState()!='X')){			
							newNode = new Node((*current).getG()+1, 1, temp_list[temp_i], current, goal);
							open_set.push_back(*newNode);
							(*temp_plane_p1).setNode(newNode);
						}
					}
					PlanePoint *temp_plane_p2=&my_plane[(*current).getPos().getY()][(*current).getPos().getX()];
					printf("**Confict**\n Resolving -- ");
					(*temp_plane_p2).setNode(NULL);
					is_conflict=true;
					break;
				}
				if((current->getG()+1<=temp_size) &&  temp_curr == result1[temp_size-current->getG()-1]){
					vector <Point> temp_list;
					temp_list=(*current).find_neighbors(limits);
					int temp_i;
					for(temp_i=0; temp_i<(int)temp_list.size(); temp_i++){	
						PlanePoint *temp_plane_p1=&my_plane[temp_list[temp_i].getY()][temp_list[temp_i].getX()];
						if(!(temp_list[temp_i]==result1[temp_size-current->getG()-1]) && ((*temp_plane_p1).getState()!='X')){			
						newNode = new Node((*current).getG()+1, 1, temp_list[temp_i], current, goal);
			
						open_set.push_back(*newNode);
				
						(*temp_plane_p1).setNode(newNode);
						}
					}
					PlanePoint *temp_plane_p2=&my_plane[(*current).getPos().getY()][(*current).getPos().getX()];
					(*temp_plane_p2).setNode(NULL);
					printf(" **Confict**\n Resolving -- \n");
					is_conflict=true;
					break;
					
				}
			}
			printf("\n");
			/*if planepoint != NULL and status == closed, neighbor already examined, skip*/
			if((((*temp_plane_p).getNode())!=NULL && (*temp_plane_p).getNode()->getStatus()==0 )|| (temp_plane_p->getState() == 'X')){	
				continue;
			}

			/*Node not created yet*/
			if((*temp_plane_p).getNode() == NULL){
				newNode = new Node((*current).getG()+1, 1, temp_curr, current, goal);
				open_set.push_back(*newNode);
				(*temp_plane_p).setNode(newNode);
			}
			else {/*Node is already on open_set, so update if there is better path*/
				Node* already_child = (*temp_plane_p).getNode();
				if((*current).getG()+1 < already_child->getG()){//better node
					already_child->setCameFrom(current);//new father
					already_child->setG((*current).getG()+1);
				}
			}	
		}
	}	
	return Path;
}
