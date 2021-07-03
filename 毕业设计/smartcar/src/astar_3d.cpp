//3D Astar

#include <iostream>
#include <cstring>
#include <algorithm>
#include <cmath>
#include <list>
#include <fstream>
#include <cstdio>
#include <vector>
#include <ctime>
#include <chrono>

using namespace std;

typedef struct node
{
	node()
	{
		x = y = t = 0;
		f = g = h = 0;
		parent = NULL;
	}
	int x, y, t;
	double f, g, h;
	struct node *parent;
} Node;
const double LEN = 10;
const int MAX = 50;
const char START = '0';
const char STOP = '1';
const char ROAD = '*';
const char WALL = '#';

char mmap[MAX][MAX][MAX];     //µØÍŒ
char towd_map[MAX][MAX];
list<Node*> startList, stopList;  //¿ªÆôÁÐ±íºÍ¹Ø±ÕÁÐ±í

int sx, sy, st, ex, ey, et;    //Æðµã×ø±ê(sx,sy,sz)ºÍÖÕµã×ø±ê(ex,ey,ez)
int n, m, T;                //mÐÐnÁÐ
typedef struct Point
{
	double x;
	double y;
	double t;
};
vector<Point> path;  
vector<Point> optPath;
vector<double> s_, v_, a_;

//排除dt=-1情况，若dt=0出现后续会做处理
int dx[17] = { -1,1,0,0,-1,1,-1,1,0,-1,1,0,0,-1,1,-1,1};
int dy[17] = { 0,0,-1,1,-1,-1,1,1,0,0,0,-1,1,-1,-1,1,1};
int dt[17] = { 0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1};

//int dx[26] = { -1,1,0,0,-1,1,-1,1,0,-1,1,0,0,-1,1,-1,1,0,-1,1,0,0,-1,1,-1,1 };
//int dy[26] = { 0,0,-1,1,-1,-1,1,1,0,0,0,-1,1,-1,-1,1,1,0,0,0,-1,1,-1,-1,1,1 };
//int dt[26] = { 0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1 };


double getDis(int x1, int y1, int z1, int x2, int y2, int z2)
{
	double xx1 = x1 * LEN + LEN / 2.0;   //到中心点
	double yy1 = y1 * LEN + LEN / 2.0;
	double zz1 = z1 * LEN + LEN / 2.0;

	double xx2 = x2 * LEN + LEN / 2.0;
	double yy2 = y2 * LEN + LEN / 2.0;
	double zz2 = z2 * LEN + LEN / 2.0;

	return sqrt((xx1 - xx2)*(xx1 - xx2) + (yy1 - yy2)*(yy1 - yy2) + (zz1 - zz2)*(zz1 - zz2));
}


//是否在列表中
bool in_List(Node *pnode, list<Node*> mlist)
{
	for (list<Node*>::iterator it = mlist.begin(); it != mlist.end(); it++)
	{
		if (pnode->x == (*it)->x&&pnode->y == (*it)->y&&pnode->t == (*it)->t)
			return true;
	}
	return false;
}

//从列表删除
bool del(Node *pnode, list<Node*> &mlist)
{
	for (list<Node*>::iterator it = mlist.begin(); it != mlist.end(); it++)
	{
		if (pnode == (*it))
		{
			mlist.erase(it);
			return true;
		}
	}
	return false;
}

//添加到列表
void add(Node *pnode, list<Node*> &mlist)
{
	mlist.push_back(pnode);
	return;
}

//获取F最小的节点
Node* getMin(list<Node*> mlist)
{
	double mmin = 100000000;
	Node *temp = NULL;
	for (list<Node*>::iterator it = mlist.begin(); it != mlist.end(); it++)
	{
		if ((*it)->f < mmin)
		{
			mmin = (*it)->f;
			temp = (*it);
		}
	}
	return temp;
}

//沿父节点寻路
void setRoad(Node *root)
{
	Point p;
	while (root->parent != NULL)
	{
		if (root->x == ex && root->y == ey && root->t == et)
		{
			mmap[root->t][root->x][root->y] = STOP;
		}
		else
		{
			mmap[root->t][root->x][root->y] = ROAD;
		}
		p.x = root->x;
		p.y = root->y;
		p.t = root->t;
		path.push_back(p);
		root = root->parent;
	}
	p.x = root->x;
	p.y = root->y;
	p.t = root->t;
	path.push_back(p);
}

//绘制路线
void printRoad()
{
	for (int kk = 0; kk < T; kk++)
	{
		for (int i = 0; i < m; i++)
		{
			for (int j = 0; j < n; j++)
			{
				if (mmap[kk][i][j] == ROAD || mmap[kk][i][j] == START || mmap[kk][i][j] == STOP)
				{
					std::cout << mmap[kk][i][j] << " ";
				}
				else
				{
					cout << mmap[kk][i][j] << " ";
				}
			}
			cout << endl;
		}
		cout << endl << endl;
	}
	cout << endl << endl << endl;
}

//A*规划
void bfs()
{
	startList.clear();
	stopList.clear();

	Node *preNode = new Node;
	preNode->x = sx;
	preNode->y = sy;
	preNode->t = st;
	preNode->g = 0;
	preNode->h = getDis(sx, sy, st, ex, ey, et);
	preNode->f = preNode->g + preNode->h;
	preNode->parent = NULL;
	add(preNode, startList);

	while (!startList.empty())  //OpenList²»Îª¿Õ
	{
		preNode = getMin(startList);
		if (preNode == NULL)
		{
			cout << "FAILED!" << endl;
			return;
		}
		del(preNode, startList);
		add(preNode, stopList);
		for (int d = 0; d < 26; d++)
		{
			int cx = preNode->x + dx[d];
			int cy = preNode->y + dy[d];
			int ct = preNode->t + dt[d];

			Node *curNode = new Node;
			curNode->x = cx;
			curNode->y = cy;
			curNode->t = ct;
			curNode->g = preNode->g + getDis(cx, cy, ct, preNode->x, preNode->y, preNode->t);
			curNode->h = getDis(cx, cy, ct, ex, ey, et);
			curNode->f = curNode->g + curNode->h;
			curNode->parent = preNode;

			if (cx < 0 || cy < 0 || ct < 0 || cx >= m || cy >= n || ct >= T) continue;   //Ôœœç»òÅöÇœ
			else if (mmap[ct][cx][cy] == WALL) continue;
			else if (in_List(curNode, startList) || in_List(curNode, stopList)) continue;  //ÔÚ¿ªÆô»ò¹Ø±ÕÁÐ±í

			if (cx == ex && cy == ey && ct == et)
			{
				setRoad(curNode);
				//printRoad();
				return;
			}
			add(curNode, startList);
		}
	}
	cout << "FAILED!" << endl;
	return;
}


//计算里程数
void s_calculate(vector<Point> path_)
{
	double sk, s_total = 0;
	int N = path.size();
	s_.push_back(0);
	for (int i = 1; i < N; i++)
	{
		sk = getDis(path[N - i].x, path[N - i].y, 0, path[N - i - 1].x, path[N - i - 1].y, 0);
		s_total = s_total + sk;
		s_.push_back(s_total);
	}
	return;
}

//计算速度加速度
void vel_acc_cal(vector<double> s, vector<Point> path_)
{
	int N = path.size();
	for (int i = 0; i < N - 1; i++)
	{
		v_.push_back((s[i + 1] - s[i]) / (path[i + 1].t - path[i].t));
	}
	for (int j = 0; j < N - 2; j++)
	{
		a_.push_back((s[j + 2] - 2 * s[j + 1] + s[j]) /
			((path[j + 1].t - path[j].t)*(path[j + 1].t - path[j].t)));
	}
}

//计算cost
double cost_cal(vector<double> acc)
{
	double cost = 0;
	for (int i = 0; i < acc.size(); i++)
	{
		cost = cost + acc[i] * acc[i];
	}
	return cost;
}

//检查T是否重复
int ifTrepeat(vector<Point> path_)
{
	vector<int> Tvec;
	for (int k = 0; k < path.size(); k++)  
	{
		Tvec.push_back(path[k].t);
	}
	Tvec.erase(unique(Tvec.begin(), Tvec.end()), Tvec.end());
	if(Tvec.size()==path_.size())
	{
		return 0;
	}
	else
	{
		return path_.size() - Tvec.size();
	}
}

int main()
{
	n = 32;
	m = 15;
	T = 10;
	double time = 15;
	double VMAX = 1;
	double AMAX;
	ifstream myfile("/home/qucd/test_ws/src/smartcar/src/data_3d.txt");
	if (!myfile.is_open())
	{
		cout << "can not open this file" << endl;
		return 0;
	}
	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < n; j++)
		{
			myfile >> towd_map[i][j];
		}
	}
	for (int kk = 0; kk < T; kk++)
	{
		for (int i = 0; i < m; i++)
		{
			for (int j = 0; j < n; j++)
			{
				mmap[kk][i][j] = towd_map[i][j];
			}
		}
	}
	st = 0, sx = 14, sy = 0;
	et = T - 1, ex = 14, ey = 30;
	mmap[st][sx][sy] = START;
	mmap[et][ex][ey] = STOP;

	auto ts = std::chrono::system_clock::now();
	bfs();
	auto te = std::chrono::system_clock::now();
	std::chrono::duration<double> dt = te-ts;
	cout << path.size() << endl;
	cout << dt.count() <<endl;
	for (int k = 0; k < path.size(); k++)  
	{
		cout << path[k].x << " " << path[k].y << " " << path[k].t << endl;
	}
	
	int rep_t = ifTrepeat(path);  //获得T重复的次数，改变采样间隔
	if(rep_t > 0)  //重建地图
	{
		path.clear();
		T = T + rep_t;
		cout << T << endl;
		//判断速度、加速度是否符合约束
		double vmax = LEN * sqrt(2) / (time / T);
		double amax = vmax / (time / T);
		if(vmax > VMAX || amax > AMAX)
		{
			cout << "不满足车辆动力学!FAILED" << endl;
			//return 0;
		}

		for (int kk = 0; kk < T; kk++)
		{
			for (int i = 0; i < m; i++)
			{
				for (int j = 0; j < n; j++)
				{
					mmap[kk][i][j] = towd_map[i][j];
				}
			}
		}
		et = T - 1;
		mmap[st][sx][sy] = START;
	    mmap[et][ex][ey] = STOP;

		auto ts = std::chrono::system_clock::now();
		bfs();
		auto te = std::chrono::system_clock::now();
		std::chrono::duration<double> dt = te-ts;
		cout << path.size() << endl;
		cout << dt.count() <<endl;
		for (int k = 0; k < path.size(); k++)  
		{
			cout << path[k].x << " " << path[k].y << " " << path[k].t << endl;
		}
	}
	
	s_calculate(path);
	for (int k = 0; k < s_.size(); k++)
	{
		cout << s_[k] << endl;
	}
	vel_acc_cal(s_, path);
	double cost = cost_cal(a_);
	cout << "cost is:" << cost << endl;

	return 0;
}
