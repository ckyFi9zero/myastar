#include "astar/astar.h"
#include<vector>
#include<iostream>
#include <math.h>
#define isobstacle 1
#define iswalkable 0
#define isclose 6
#define isopen 7
using namespace std;

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
Point *Astar::findPath(Point&startPoint, Point&endPoint)
{
	swapMemory();
	cout<<"删除旧地图!"<<endl;
	lab_map.assign(map.begin(),map.end()); //拷贝一份相同的地图
	open_list.push_back(new Point(startPoint.x, startPoint.y));
	lab_map[startPoint.y][startPoint.x] = isopen;
	while (!open_list.empty()) 
	{
		int index = 0;
		auto currentP = open_list.front();
		for(int i = 0; i < open_list.size(); i++) //寻找最大F值
		{
			if(open_list[i]->F < currentP->F)
			{
				currentP = open_list[i];
				index = i;
			}
		}
		// cout<< *currentP <<endl;
		if(*currentP == endPoint) //找到终点退出并返回路径链表
		{
			return currentP;
		}

		currentP = new Point(currentP->x, currentP->y, currentP->F, currentP->G, currentP->H, currentP->parent); //向最大值的位置移动
		
		open_list.erase(open_list.begin()+index); //从open表中删除
		close_list.push_back(currentP); //加入close表
		lab_map[currentP->y][currentP->x] = isclose; 
		auto surroundP = getSurroundingPoint(currentP); //获得周围可到达点
		for (int i = 0; i < surroundP.size(); i++)
		{
			if (surroundP[i]->G == 0) //新的点则计算GHF，并指定指向当前位置的地址
			{
				surroundP[i]->G = calcG(currentP, surroundP[i]);
				surroundP[i]->H = calcH(&endPoint, surroundP[i]);
				surroundP[i]->F = calcF(surroundP[i]);
				surroundP[i]->parent = currentP;

				open_list.push_back(surroundP[i]);
				lab_map[surroundP[i]->y][surroundP[i]->x]=isopen;
			}
			else//如果是已在open表中，则比较G值，保存最小G值的点的地址
			{	
				int tempG = calcG(currentP, surroundP[i]);
				//cout<<"tempG:"<<tempG<<" currentP:"<<currentP->G<<endl;
				if (tempG < surroundP[i]->G)
				{
					surroundP[i]->parent = currentP;
					surroundP[i]->G = tempG;
					surroundP[i]->F = calcF(surroundP[i]);
				}
			}
		}

	}
}

vector<Point*> Astar::getPath(Point&startPoint, Point&endPoint)
{
	vector<Point*>tempPath;
	vector<Point*>path;
	if(lab_map[endPoint.y][endPoint.x] == isobstacle)
	{
		
		cout<<"起点或终点为障碍物！"<<endl;
		return tempPath;
	}
	Point *result = findPath(startPoint, endPoint);
	while (result) //将链表中的位置数据取出
	{
		path.push_back(result);
		result = result->parent;
	}

	for(int i = path.size()-1; i >=0; i--)//反转位置
	{
		tempPath.push_back(path[i]);
	}
	cout<<"路径规划成功！"<<endl;
	return tempPath;
}

bool Astar::canReach(const Point*point, Point* target)
{
	if (target->x < 0 || target->x > map[0].size() - 1  //超出地图边界，障碍物，在close表中都不可达
		|| target->y < 0 || target->y > map.size() - 1
		|| lab_map[target->y][target->x] == isobstacle
		|| lab_map[target->y][target->x] == isclose)
		return false;
	else 
	{
		if (abs(point->x - target->x) + abs(point->y - target->y) == 1) //提取上下左右位置
			return true;
		else //斜对角的两侧都为障碍物则不可达，反之有一个无障碍物则可达
			if (map[point->y][target->x] == iswalkable && map[target->y][point->x] == iswalkable)
				return true;
			else
				return false;
	}
	return true;
}

vector<Point *> Astar::getSurroundingPoint(Point *target) //返回目标周围可到达点的地址
{
	vector<Point*> temp;
	int neighber [8][2]= {{-1,-1},{-1,0},{-1,1},{0,-1},{0,1},{1,-1},{1,0},{1,1}};
	for(int i = 0; i < 8; i++)
	{
			Point *new_P = new Point(target->x+neighber[i][0],target->y+neighber[i][1]);
			if(canReach(target, new_P)) 
			{
				Point *exist_P = isInLine(open_list, new_P, isopen); //判断周围的点是否在open表中
				if(exist_P) //如果点在open表内，把表内的地址保存到temp
				{
					temp.push_back(exist_P);
					delete new_P;
				}
				else //如果点不在，则将新地址给temp
				{
					temp.push_back(new_P);
				}	
			}
	}
	return temp;
}

Point* Astar::isInLine(vector<Point *> temp, Point *target, int flag) //如果目标在表内，则返回表内的地址
{
	if(lab_map[target->y][target->x] == flag)
		{
			for (int i = 0; i< temp.size(); i++)
			{
				// if (temp[i]->x == target->x && temp[i]->y == target->y)
				if (*temp[i] == *target)
				{
					return temp[i];
				}	
			}
		}
	return NULL;
}

int Astar::calcF(Point *target)
{
	return target->G + target->H;
}

int Astar::calcG(Point *start, Point *target)
{
	int extraG = (abs(start->x - target->x) + abs(start->y - target->y)) == 1 ? costPa : costTa;
	return extraG + start->G;
}

int Astar::calcH(Point *endpoint, Point *target)
{
	return round(sqrt((endpoint->x - target->x)*(endpoint->x - target->x) + (endpoint->y - target->y)*(endpoint->y - target->y))*costPa);
}


void Astar::swapMemory() //释放内存、指针
{
	for(auto it = open_list.begin(); it != open_list.end(); it++) //释放指针
	{
		if(*it != NULL)
		{
			delete *it;
			*it = NULL;
		}
	}
	for(auto it = close_list.begin(); it != close_list.end(); it++)
	{
		if(*it != NULL)
		{
			delete *it;
			*it = NULL;
		}
	}
	open_list.clear(); //将vector的size置零
	close_list.clear();
	vector<Point*>().swap(open_list);
	vector<Point*>().swap(close_list);
	cout<<"删除旧地图!"<<endl;
}

void Astar::play()
{
	cout<<"Astar"<<endl;
}


int main(int argc, char **argv)
{
	vector<vector<int>> map = {{0,0,0,0,0},{0,0,0,0,0}};
	Astar a(map);
	Point s = Point(0, 0);
	Point e = Point(4, 1);
	vector<Point*>path_v = a.getPath(s, e);
}