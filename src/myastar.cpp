#include "myastar/myastar.h"//导入自己编写的头文件 include内头文件路径（隐式依赖了一些需要头文件）
#include "astar/astar.h"
#include <pluginlib/class_list_macros.h>//注册插件宏头文件
#include <nav_msgs/Path.h>
//作用：定义 nav_msgs::Path 消息类型
//必须场景：需要发布或处理路径数据

PLUGINLIB_EXPORT_CLASS(myastar::MyAstarPlanner, nav_core::BaseGlobalPlanner)//注册插件宏
using namespace std;
namespace myastar { //避免命名冲突，将自定义规划器逻辑隔离在myastar命名空间内

// 默认构造函数
MyAstarPlanner::MyAstarPlanner() {

}

// 带参构造函数（委托给initialize）
MyAstarPlanner::MyAstarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){  
  initialize(name, costmap_ros);
}

// 初始化函数实现
void MyAstarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    setlocale(LC_ALL, "");//设置locale为全局，解决中文乱码问题
  //获取地图信息
    if(!costmap_ros)
    {
        ROS_ERROR("A*路径规划器初始化失败，costmap_ros指针为空！");
        return;
    }
   if(!costmap_ros->getCostmap()) //检查costmap_ros是否有效
   {
      ROS_ERROR("A*路径规划器初始化失败，costmap_ros->getCostmap()返回空指针！");
      return;
   }
   if(!costmap_ros->getCostmap()->getSizeInCellsX() || !costmap_ros->getCostmap()->getSizeInCellsY())
   {
     ROS_ERROR("A*路径规划器初始化失败，地图尺寸为0！");
     return;
    }   
    setlocale(LC_ALL, "");//设置locale为全局，解决中文乱码问题
    costmap_ros_ = costmap_ros;
  //将一维ros地图保存为二维栅格地图
  //  for(int iy = 0; iy<height; iy++)
  //  {
    //    vector<int> temp_v;
    //    for(int ix = 0; ix<width; ix++)
    //    {
      //      int temp = static_cast<int>(costmap_->getCost(ix, iy));
      //      if(temp>=1 && temp<=254)
      //      {
        //         temp_v.push_back(1);
        //      }
        //      else
        //      {
          //         temp_v.push_back(0);
          //      }
          
          //    }
          //    map_v.push_back(temp_v);
          //  }
          plan_pub_ = nh_.advertise<nav_msgs::Path>("/move_base/myastar/plan", 1);
          
          ROS_INFO("A*路径规划器初始化完成！");
        }
        
        
        bool MyAstarPlanner::makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>& plan) {
   //将一维ros地图保存为二维栅格地图
   
  
   costmap_ = costmap_ros_->getCostmap();
   originX = costmap_->getOriginX();
   originY = costmap_->getOriginY();
   width = costmap_->getSizeInCellsX();
   height = costmap_->getSizeInCellsY();
   resolution = costmap_->getResolution();
    //检查costmap是否有效
   if (!costmap_ || !costmap_->getSizeInCellsX() || !costmap_->getSizeInCellsY()) {
    ROS_ERROR("Costmap未初始化或尺寸为0！");
    return false;
 }
   cout<<"地图信息:"<<endl;
   cout<<"地图宽度:"<<width<<"  地图高度:"<<height<<"  地图分辨率:"<<resolution<<endl;
   map_v.clear(); 
   for(int iy = 0; iy<height; iy++)
                {
                  vector<int> temp_v;
                  for(int ix = 0; ix<width; ix++)
                  {
                    int temp = static_cast<int>(costmap_->getCost(ix, iy));
                    if(temp>=128 && temp<=254)
                    {
                        temp_v.push_back(1);
                    }
                    else
                    {
                        temp_v.push_back(0);
                    }
  
                  }
                  map_v.push_back(temp_v);
                }



                ROS_INFO("A*路径规划器收到新的路径规划请求，起点:[%f %f], 终点:[%f %f]", 
                           start.pose.position.x, 
                           start.pose.position.y, 
                           goal.pose.position.x, 
                           goal.pose.position.y);//在ROS日志中记录路径规划的起点和目标点坐标，用于调试和监控
                plan.clear();//清除输出参数 plan（std::vector<geometry_msgs::PoseStamped>）中的旧数据，确保路径是全新的
          float start_P_x = 0;
          float start_P_y = 0;
          float end_P_x = 0;
          float end_P_y = 0;
          start_P_x = start.pose.position.x;
          start_P_y = start.pose.position.y;
          end_P_x = goal.pose.position.x;
          end_P_y = goal.pose.position.y;

          Astar aStart(map_v); //实例化A*算法
          //获取定位位置和终点位置
          Point start_point = Point((start_P_x- originX)/resolution,(start_P_y-originY)/resolution);
          Point end_point = Point((end_P_x-originX)/resolution, (end_P_y-originY)/resolution);

          cout<<"起点:"<<start_point.x<<","<<start_point.y<<endl;
          cout<<"终点:"<<end_point.x<<","<<end_point.y<<endl;
          if (start_point.x < 0 || start_point.x >= width || start_point.y < 0 || start_point.y >= height ||
              end_point.x < 0 || end_point.x >= width || end_point.y < 0 || end_point.y >= height) {
            ROS_ERROR("起点或终点超出地图范围！");
            return false;
          }
          if (map_v[end_point.y][end_point.x] == 1) {
            ROS_ERROR("终点为障碍物！");
            return false;
          }
          cout<<"A*路径规划开始！"<<endl;
          vector<Point*> path_v = aStart.getPath(start_point, end_point); //获取路径
          cout<<"路径长度:"<<path_v.size()<<endl;

          geometry_msgs::PoseStamped pose = start;
          pose.header.frame_id="map";
          if (path_v.size() == 0)
            return false;
          //发布路径信息
          int count=0;
          for(auto &p :path_v)
          {
            count++;
            pose.header.stamp =ros::Time::now();
            pose.pose.position.x = resolution * (p->x) + originX;
            pose.pose.position.y = resolution * (p->y) + originY;
            pose.pose.position.z = 0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            plan.push_back(pose);
          }
          cout<<"路径规划完成，共规划"<<plan.size()<<"个点"<<endl;
          // 发布路径消息
          nav_msgs::Path path_msg;
          path_msg.header.frame_id = "map";
          path_msg.header.stamp = ros::Time::now();
          path_msg.poses = plan;  // 直接使用生成的路径
          plan_pub_.publish(path_msg);

          ROS_INFO("发布路径规划结果，共规划 %lu 个点", plan.size());
          return !plan.empty();
          
          return true;
        }
} // namespace myastar