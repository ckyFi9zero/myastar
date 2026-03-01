#ifndef MYASTAR_H
#define MYASTAR_H

#include <ros/ros.h>//ROS 基础功能的集成头文件 
//包含常用 ROS 数据类型（如 ros::NodeHandle, ros::Rate）
//在 initialize() 中通过 NodeHandle 读取参数
#include <geometry_msgs/PoseStamped.h>//定义带时间戳的位姿消息类型 
//makePlan() 的 start 和 goal 参数是该类型
//输出的路径点序列 plan 也是 PoseStamped 的集合
#include <costmap_2d/costmap_2d_ros.h>//提供代价地图的访问接口
#include <costmap_2d/costmap_2d.h>
//在 makePlan() 中通过 costmap_ros 查询障碍物信息，避免碰撞
#include <nav_core/base_global_planner.h>//引入 ROS 全局路径规划器的基类接口
#include <vector>

using std::string;
using namespace std;

namespace myastar { //避免命名冲突，将自定义规划器逻辑隔离在global_planner命名空间内
  class MyAstarPlanner : public nav_core::BaseGlobalPlanner { //GlobalPlanner 继承自 ROS 导航栈的标准全局规划器接口
    public://对外接口
      MyAstarPlanner();//默认构造函数：供插件系统动态加载时调用。
      MyAstarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);//带参构造函数：直接初始化时传递规划器名称和代价地图。
      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
      /*功能：初始化规划器。
      参数：
        name：规划器名称（用于 ROS 参数服务器配置）。
        costmap_ros：指向全局代价地图的指针（包含障碍物信息）。
      实现要求：
        必须保存 costmap_ros 供后续路径规划使用（通常赋值给私有成员 costmap_ros_）。
        非阻塞操作，避免长时间初始化。*/
      bool makePlan(const geometry_msgs::PoseStamped& start,
                    const geometry_msgs::PoseStamped& goal,
                    std::vector<geometry_msgs::PoseStamped>& plan);
        /*功能：计算从 start 到 goal 的路径。
    参数：
    start/goal：起点/终点的位姿（包含位置和朝向）。
    plan：输出参数，存储规划的路径点序列。
    返回值：成功返回 true，失败返回 false。
    关键规则：
    路径点必须按顺序从 start 到 goal 填充到 plan 中。*/
    };
    
    float originX;
    float originY;
    float resolution;
    costmap_2d::Costmap2DROS* costmap_ros_;
    costmap_2d::Costmap2D* costmap_;
    int width;
    int height;
    vector<vector<int> > map_v;

    ros::Publisher plan_pub_;  // 路径发布器
    ros::NodeHandle nh_;       // 节点句柄
  }
#endif