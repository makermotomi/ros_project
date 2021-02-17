#include <ros/ros.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>  // add goal navi

// Global
int target_pos = 0;
int robot_state = 0; // 0: idle 1:moving 2:goal

//yaml data setting
geometry_msgs::PoseStamped set_point[100];
int data_size=0;//

class Goal {
public:
    Goal(double px, double py, double pz, double ow);
    ~Goal();
 
private:
    ros::Publisher pub;
    ros::NodeHandle nh;
};

Goal::Goal(double px, double py, double pz, double ow){
    pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
 
    ros::Rate one_sec(1);
    one_sec.sleep();
     
    ros::Time time = ros::Time::now();
    geometry_msgs::PoseStamped goal_point;
 
    goal_point.pose.position.x = px;
    goal_point.pose.position.y = py;
//    goal_point.pose.position.z =  pz;
    goal_point.pose.orientation.z =  pz;
    goal_point.pose.orientation.w = ow;
    goal_point.header.stamp = time;
    goal_point.header.frame_id = "map";
 
    pub.publish(goal_point);
 
}
 
Goal::~Goal(){
 
}



void navStatusCallBack(const actionlib_msgs::GoalStatusArray::ConstPtr &status)
{

    int status_id = 0;
    //uint8 PENDING         = 0  
    //uint8 ACTIVE          = 1 
    //uint8 PREEMPTED       = 2
    //uint8 SUCCEEDED       = 3
    //uint8 ABORTED         = 4
    //uint8 REJECTED        = 5
    //uint8 PREEMPTING      = 6
    //uint8 RECALLING       = 7
    //uint8 RECALLED        = 8
    //uint8 LOST            = 9

    if (!status->status_list.empty()){
    actionlib_msgs::GoalStatus goalStatus = status->status_list[0];
    status_id = goalStatus.status;
    }

    ROS_INFO("status=%d\n",status_id);
    if(status_id==1){
    //移動中
    
        ROS_INFO("Target =  x: %2.4f, y: %2.4f, z: %2.4f, w: %2.4f  MOVING!!\n",
                    set_point[target_pos].pose.position.x,
                    set_point[target_pos].pose.position.y,
                    set_point[target_pos].pose.position.z,
                    set_point[target_pos].pose.orientation.w);

        robot_state = 1;
    }

    if((status_id==3)||(status_id==0)){
    //ゴールに到達・もしくはゴールに到達して待機中。
    
        if(status_id==3){
            if(robot_state!=1){
                return;
            }
            robot_state = 2;
            ROS_INFO("Target = %d Goal reached!!!!!\n",target_pos);
            target_pos++;
            if(target_pos <= data_size){
                Goal goal_ob(set_point[target_pos].pose.position.x, set_point[target_pos].pose.position.y, set_point[target_pos].pose.position.z,set_point[target_pos].pose.orientation.w);   
            }
            else{
                ROS_INFO("Final Goal reached!!!!!\n");
            }
            
        }
        else {
            ROS_INFO("Idle!\n");
            robot_state = 0;
        }

    }

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_base_goal_state");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    ros::Subscriber switch_sub;
    ros::Subscriber move_base_status_sub;
    move_base_status_sub = nh.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status", 10, &navStatusCallBack);

    XmlRpc::XmlRpcValue goals_list;    
    //double number_to_get;
    pnh.getParam("goals_list",goals_list);
    data_size= (int)goals_list.size();
    ROS_ASSERT(goals_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_INFO("Number of goal points: %i points", (int)goals_list.size());
    

    for (int32_t i = 0; i < goals_list.size(); ++i)
    {
        //ROS_INFO("read [%i]", i);
        int id = 0;
        std::string name = "";
        if (!goals_list[i]["id"].valid() || !goals_list[i]["name"].valid())
        {
        ROS_WARN("No id or data");
        continue;
        }
        if (goals_list[i]["id"].getType() == XmlRpc::XmlRpcValue::TypeInt)
            id = static_cast<int>(goals_list[i]["id"]);
        if (goals_list[i]["name"].getType() == XmlRpc::XmlRpcValue::TypeString)
            name = static_cast<std::string>(goals_list[i]["name"]);

        if (goals_list[i]["x"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
            set_point[i].pose.position.x = static_cast<double>(goals_list[i]["x"]);
        if (goals_list[i]["y"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
            set_point[i].pose.position.y = static_cast<double>(goals_list[i]["y"]);
        if (goals_list[i]["z"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
            set_point[i].pose.position.z = static_cast<double>(goals_list[i]["z"]);
        if (goals_list[i]["w"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
            set_point[i].pose.orientation.w = static_cast<double>(goals_list[i]["w"]);

        ROS_INFO("id: %i,%s, x: %2.4f, y: %f, z: %f, w: %f \n",id, name.c_str() , set_point[i].pose.position.x, set_point[i].pose.position.y, set_point[i].pose.position.z,set_point[i].pose.orientation.w);
    }

    target_pos = 1;
    Goal goal_ob(set_point[target_pos].pose.position.x, set_point[target_pos].pose.position.y, set_point[target_pos].pose.position.z,set_point[target_pos].pose.orientation.w);    // add goal navi

    ros::spin();

    return 0;
}