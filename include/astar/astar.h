#ifndef _ASTAR_H_
#define _ASTAR_H_

/* A*的大致流程
把起始格添加到 "开启（open）列表" 
do 
{ 
		寻找开启列表中F值最低的格子, 我们称它为当前格. 
        把它切换到关闭（close）列表. 
        对当前格相邻的8格中的每一个 
        if (它不可通过 || 已经在 "关闭（close）列表" 中) 
        { 
                什么也不做. 
        } 
        if (它不在开启（open）列表中) 
        { 
                把它添加进 "开启列表", 把当前格作为这一格的父节点, 计算这一格的 FGH 
		}
        if (它已经在开启（open）列表中) 
        { 
                if (用 G 值为参考检查新的路径是否更好, 更低的G值意味着更好的路径) 
                { 
                        把这一格的父节点改成当前格, 并且重新计算这一格的 GF 值. 
                } 
		}
} while( 目标格已经在 "开启列表", 这时候路径被找到) 
如果开启列表已经空了, 说明路径不存在.
最后从目标格开始, 沿着每一格的父节点移动直到回到起始格, 这就是路径.
*/

//定义点，点里含有坐标信息，启发函数信息，父节点信息
#include <vector>
#include <stdlib.h>
#include<iostream>
#include <math.h>
using namespace std;
#define isobstacle 1
#define iswalkable 0
#define isclose 6
#define isopen 7

struct Point  //定义点的数据类型
{
	int x, y; // 位置test2
	int F, G, H; //启发式函数参数
	Point *parent; // A*是回溯的链表
	Point(int _x, int _y, int _F=0,int _G=0,int _H = 0, Point* _parent = NULL) //结构体构造函数
	{
		x=_x; y=_y; F=_F; G=_G; H=_H; parent=_parent;
	}
	
	const bool operator == (const Point &a) const
	{
		return (x==a.x && y==a.y);
	}

	friend ostream& operator << (ostream &Out, Point &a)
	{
		Out <<"x:"<<a.x<<" y:"<<a.y<<" F:"<<a.F<<" G:"<<a.G<<" H:"<<a.H;
		return Out;
	}
};

//A*包含需要的地图信息，close表，open表，计算F、G、H的函数，判断是否超出边界，计算选点周围的点，返回终点地址
class Astar{
private:
	vector<vector<int>> map; // 二维栅格地图
	vector<vector<int>> lab_map; //二维栅格地图记录格子状态
	const int costPa = 10; //直走消耗
	const int costTa = 14; //斜线走消耗
	vector<Point*> open_list;  // open表
	vector<Point*> close_list; // close表
	int calcF(Point *target);  // 计算F
	int calcG(Point *start, Point *target); // 计算G
	int calcH(Point *endpoint, Point *target); // 计算H
	Point* isInLine(vector<Point *> temp, Point *target, int flag); // 判断是否在某个表内
	bool canReach(const Point*point,Point* target); //判断下一个目标是否可达
	vector<Point*> getSurroundingPoint(Point *target);  //对当前位置周围的格子进行处理
	void swapMemory(); //释放内存、指针
public:
	Point *findPath(Point&startPoint, Point&endPoint); //寻找路径链表，返回终点的地址
	// Point *getminipoint();
	void play();
	vector<Point*> getPath(Point&startPoint, Point&endPoint); //获取位置数组
	Astar(vector<vector<int>> _map) :map(_map),lab_map(_map) 
	{
		cout<<"Got map!"<<endl;
		cout<<"Map size: "<<map.size()<<"x"<<map[0].size()<<""<<endl;
	} //构造函数
	~Astar(){swapMemory();}; //析构函数
};

#endif