#include <stdio.h>
#include <iostream>

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <chrono>

#include <thread>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include<Eigen/Geometry>
#include<Eigen/Core>
#include <math.h>
#include <queue>

using Eigen::MatrixXd;


using namespace ur_rtde;
using namespace std::chrono;

std::vector< std::vector<double> >pose_queue(7,std::vector<double>(10,0.0));

//**回调函数
void pose_Callback(const geometry_msgs::PoseStamped::ConstPtr& message_holder)
{
  //通过message_holder获取并打印订阅到的数据
  //std::cout<<*message_holder<<std::endl;
pose_queue[0].push_back(message_holder->pose.position.x);
pose_queue[0].erase(pose_queue[0].begin());

pose_queue[1].push_back(message_holder->pose.position.y);
pose_queue[1].erase(pose_queue[1].begin());

pose_queue[2].push_back(message_holder->pose.position.z);
pose_queue[2].erase(pose_queue[2].begin());

pose_queue[3].push_back(message_holder->pose.orientation.x);
pose_queue[3].erase(pose_queue[3].begin());

pose_queue[4].push_back(message_holder->pose.orientation.y);
pose_queue[4].erase(pose_queue[4].begin());

pose_queue[5].push_back(message_holder->pose.orientation.z);
pose_queue[5].erase(pose_queue[5].begin());

pose_queue[6].push_back(message_holder->pose.orientation.w);
pose_queue[6].erase(pose_queue[6].begin());


}


//*自定义校验和函数
char checkSum(unsigned char *sendBuffer)
{
  int sum = 0;

  for (size_t i = 2; i < 8; i++)
  {
    sum = sum + sendBuffer[i];
  }

  sendBuffer[8] = sum;
  sum = 0;
  return sum;
}

//***********************************************************************************************************************************/

int main(int argc, char *argv[])
{
    
    //**********************************创建订阅，从aruco_single/pose获取marker的位置姿态信息数据****************************************/
    //初始化 ROS 节点
    ros::init(argc,argv,"sub_pose");
    //创建节点句柄
    ros::NodeHandle n;
    //创建订阅者对象
    ros::Subscriber sub_pose=n.subscribe<geometry_msgs::PoseStamped>("aruco_single/pose",1000,pose_Callback);
    ros::AsyncSpinner spinner(3);
    spinner.start();


  //*************************************** 串口初始化 *******************************************************************************//
  boost::asio::io_service ioService;
  boost::asio::serial_port serialPort(ioService, "/dev/ttyUSB0");

  serialPort.set_option(boost::asio::serial_port::baud_rate(115200));
  serialPort.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
  serialPort.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
  serialPort.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
  serialPort.set_option(boost::asio::serial_port::character_size(8));

  unsigned char sendBuffer[] = {0x55, 0XAA, 0x04, 0x02, 0x03, 0x37, 0x00, 0x00, 0X58};
  //sendBuffer[3]是id号，设定吸取释放溶液的驱动器（偏轴型）为1，推动扔掉吸头的启动器（同轴型）为2。

  //**------------------------------------------------------------------------------------------------------------------------------**//

  //**************************************连接UR***********************************************************************//
  ur_rtde::RTDEControlInterface rtde_control("192.168.3.101");
  ur_rtde::RTDEReceiveInterface rtde_receive("192.168.3.101");
  ur_rtde::RTDEIOInterface rtde_io("192.168.3.101");

  // force_mode初始化
  std::vector<double> task_frame    = {0, 0, 0, 0, 0, 0};
  std::vector<int> selection_vector = {0, 0, 1, 0, 0, 0};
  std::vector<double> wrench_down   = {0, 0, -20, 0, 0, 0};
  std::vector<double> wrench_up     = {0, 0, 1, 0, 0, 0};
  int    force_type = 2;
  double dt         = 1.0/500;  // 2ms
  std::vector<double> limits = {1, 1, 1, 1, 1, 1};
  //*---------------------------------------------------------------------------------------------------------------*//


//设定相机外参
Eigen::Matrix4d T_camera_to_end;
T_camera_to_end.setIdentity();

//设定marker坐标系机器人初始位姿
std::vector<double> initialPointInMarker(6,0.0);
Eigen::Matrix4d T_marker_to_camera_Initial;
T_marker_to_camera_Initial.setIdentity();
T_marker_to_camera_Initial(0,0)= -0.999519;T_marker_to_camera_Initial(0,1)=-0.00182704;T_marker_to_camera_Initial(0,2)=-0.0308467;T_marker_to_camera_Initial(0,3)=0;
T_marker_to_camera_Initial(1,0)= -0.00397905;T_marker_to_camera_Initial(1,1)= 0.99755 ;T_marker_to_camera_Initial(1,2)=0.0698479;T_marker_to_camera_Initial(1,3)=0;
T_marker_to_camera_Initial(2,0)= 0.0306435;T_marker_to_camera_Initial(2,1)=0.0699373;T_marker_to_camera_Initial(2,2)=-0.997077;T_marker_to_camera_Initial(2,3)=0.3;
//******************************************************定义路点***********************************************/

  //初始点

std::vector<double> initialPoint = {-0.7608,0.12105,0.41381,2.152,-2.298,-0.001};

Eigen::Matrix4d T_end_to_base_Initial;
T_end_to_base_Initial.setIdentity();
//机器人末端位置姿态
std::vector<double> TCP_pose(6);
Eigen::Matrix4d T_end_to_base;
T_end_to_base.setIdentity();//先将T_end_to_base初始化为单位矩阵，然后根据获得的实时机器人末端x,y,z,RX,Ry,Rz后再确定
Eigen::Vector3d Translation;
Eigen::Vector3d axis;
Eigen::Matrix3d R_end_to_base;
double alpha;

Eigen::Matrix4d T_marker_to_base;
T_marker_to_base.setIdentity();//先将T_marker_to_base初始化为单位矩阵，等marker坐标系固定后，由T_end_to_base*T_camera_to_end*T_marker_to_camera得到

// 任务空间中的各个目标位置（注意到相机与机器人末端固连，各个T_marker_to_camera可以转化为机器人末端在marker坐标系下的目标路点，而且由于maker坐标系下各个路点是固定的，所以各T_marker_to_camera是定值）
//吸头盒第一排
Eigen::Matrix4d T_Box1_to_Initial;
T_Box1_to_Initial.setIdentity();
T_Box1_to_Initial(0,0)= 0.999568;T_Box1_to_Initial(0,1)=0.0221169;T_Box1_to_Initial(0,2)=0.0193703;T_Box1_to_Initial(0,3)=-0.153886;
T_Box1_to_Initial(1,0)=-0.0221015;T_Box1_to_Initial(1,1)=0.999755;T_Box1_to_Initial(1,2)=-0.00100979;T_Box1_to_Initial(1,3)=-0.0245185;
T_Box1_to_Initial(2,0)=-0.0193879;T_Box1_to_Initial(2,1)=0.000581245;T_Box1_to_Initial(2,2)=0.999812;T_Box1_to_Initial(2,3)=0.12896;
Eigen::Matrix4d T_marker_to_camera_Box1;
T_marker_to_camera_Box1.setIdentity();
Eigen::Matrix4d T_end_to_base_Box1;
T_end_to_base_Box1.setIdentity();

std::vector<double> boxHighPoint1(6,0.0);
std::vector<double> boxMidPoint1(6,0.0);


//吸头盒第二排
Eigen::Matrix4d T_Box2_to_Initial;
T_Box2_to_Initial.setIdentity();
T_Box2_to_Initial<<0.999534,0.0178233,0.0247904,-0.15474,
-0.0177874,0.99984,-0.00166807,-0.0319537,
-0.0248162,0.00122633,0.999691,0.130462,
 0 , 0 , 0 ,  1; 
  Eigen::Matrix4d T_marker_to_camera_Box2;
T_marker_to_camera_Box2.setIdentity();
Eigen::Matrix4d T_end_to_base_Box2;
T_end_to_base_Box2.setIdentity();
std::vector<double> boxHighPoint2(6,0.0);
 std::vector<double> boxMidPoint2(6,0.0);


//工作板第一排
Eigen::Matrix4d T_Well1_to_Initial;
T_Well1_to_Initial.setIdentity();
T_Well1_to_Initial<<0.999842,-0.00389522,0.0173506,0.0755775,
0.00391411,0.999992,-0.00105479,-0.0275238,
-0.0173464,0.00112254,0.999849,0.102617,
0 , 0 , 0 , 1;
Eigen::Matrix4d T_marker_to_camera_Well1;
T_marker_to_camera_Well1.setIdentity();
Eigen::Matrix4d T_end_to_base_Well1;
T_end_to_base_Well1.setIdentity();
std::vector<double> wellHighPoint1(6,0.0);
std::vector<double> wellMidPoint1(6,0.0);
std::vector<double> wellLowPoint1(6,0.0);


//蒸馏水路点
Eigen::Matrix4d T_water_to_Initial;
T_water_to_Initial.setIdentity();
T_water_to_Initial<<0.999841,-0.00401728,0.0173685,-0.17204,
0.00403543,0.999991,-0.00100983,-0.288068,
-0.0173643,0.00107976,0.999849,0.127289,
0,0,0,1;

Eigen::Matrix4d T_marker_to_camera_water;
T_marker_to_camera_water.setIdentity();
Eigen::Matrix4d T_end_to_base_water;
T_end_to_base_water.setIdentity();
 std::vector<double> waterHighPoint(6,0.0);
  std::vector<double> waterLowPoint(6,0.0);

//待稀释溶液路点

Eigen::Matrix4d T_bluewater_to_Initial;
T_bluewater_to_Initial.setIdentity();
T_bluewater_to_Initial<<0.999841,-0.00397746,0.0173614,0.0861026,
0.00399676,0.999991,-0.00107718,-0.307434,
-0.0173569,0.00114639,0.999849,0.119817,
0,0 ,0,1;
Eigen::Matrix4d T_marker_to_camera_blueWater;
T_marker_to_camera_blueWater.setIdentity();
Eigen::Matrix4d T_end_to_base_bluewater;
T_end_to_base_bluewater.setIdentity();
  std::vector<double> bluewaterHighPoint(6,0.0);
  std::vector<double> bluewaterLowPoint(6,0.0);




  // medical waste路点
std::vector<double> trashHighPoint = {-0.614552,-0.186405,0.3,2.08015,-2.28612,-0.0899261};
  std::vector<double> trashPoint = {-0.614552,-0.186405,0.259566,2.08015,-2.28612,-0.0899261};


  //***********************************开始操作****************目标：倍比稀释********************************稀释倍数：4*********//

  //*************************************************到达初始位置，准备开始工作*************************************************/

rtde_control.moveL(initialPoint, 0.5, 0.2);
std::cout<<"开始移动到base初始点"<<std::endl;

std::this_thread::sleep_for(std::chrono::duration<double>{5});


for ( int i = 0; i < 7; i++ )
    {
        for ( int j = 0; j <10; j++ )
            std::cout << "---pose_queue[" << i << "][" << j << "] = " << pose_queue[ i ][ j ]<<std::endl;
        std::cout << std::endl;
    }
//取marker xyz，xyzw十组数据的平均值
std::vector<double> average_pose(7,0.0); 
std::cout<<"average_pose is "<<std::endl;
for(int i = 0;i<7;i++)
{
double sum = 0;

    for (int j = 0;j<10;j++)
    {
sum = sum + pose_queue[i][j];

    }
sum = sum /10;
    average_pose[i] = {sum};
    std::cout<<average_pose[i]<<"  ";
}
std::cout<<std::endl;

//将得到的平均值转换为齐次变换矩阵T_maker_to_camera
Eigen::Quaterniond q = {average_pose[6],average_pose[3],average_pose[4],average_pose[5]};
Eigen::Matrix3d R;
 R= q.toRotationMatrix();
std::cout<<"得到的四元数用旋转矩阵表示为："<< std::endl<<R<<std::endl;
Eigen::Matrix4d T_marker_to_camera;
T_marker_to_camera.setIdentity();
T_marker_to_camera(0,3)=average_pose[0];T_marker_to_camera(1,3)=average_pose[1];T_marker_to_camera(2,3)=average_pose[2];
T_marker_to_camera.block<3,3>(0,0)=R;
std::cout<<"T_marker_to_camera is"<<std::endl<<T_marker_to_camera<<std::endl<<std::endl;



//获得UR的T_end_to_base（从UR controller 的output中读出末端位置和姿态，并将其转换为齐次变换矩阵以便计算）
TCP_pose =rtde_receive.getActualTCPPose();
Translation={TCP_pose[0],TCP_pose[1],TCP_pose[2]};
axis={TCP_pose[3],TCP_pose[4],TCP_pose[5]};
std::cout<<TCP_pose[0]<<","<<TCP_pose[1]<<","<<TCP_pose[2]<<","<<TCP_pose[3]<<","<<TCP_pose[4]<<","<<TCP_pose[5]<<std::endl;
axis.normalize();
alpha=TCP_pose[3]/axis[0];
R_end_to_base=Eigen::AngleAxisd(alpha,axis);
T_end_to_base.block<3,3>(0,0)=R_end_to_base;
T_end_to_base.block<3,1>(0,3)=Translation;
std::cout<<"此时的T_end_to_base is"<<T_end_to_base<<std::endl;


//计算此刻的T_marker_to_base,这个齐次变换矩阵在下次移动marker之前marker坐标系相对于base的位置姿态不变
T_marker_to_base=T_end_to_base*T_camera_to_end*T_marker_to_camera;
std::cout<<"此时的T_marker_to_base is"<<std::endl<<T_marker_to_base<<std::endl;




std::this_thread::sleep_for(std::chrono::duration<double>{1});
//由等式“T_marker_to_base=T_end_to_base*T_cam_to_end*T_marker_to_camera”计算机器人期望的末端姿态在base下的表示，然后将其转为x,y,z,Rx,Ry,Rz的形式给机器人发指令
//接下来要取第一排吸头，然后取蒸馏水，再排到工作板第一排，需要计算boxHighPoint1、waterPoint,wellHighPoint1

//计算marker坐标系下的机器人Initial_pose
T_end_to_base_Initial=T_marker_to_base*T_marker_to_camera_Initial.inverse()*T_camera_to_end.inverse();
std::cout<<"T_end_to_base_Initial"<<std::endl;
std::cout<<T_end_to_base_Initial<<std::endl;
Eigen::Matrix3d R_marker_Initial;
R_marker_Initial=T_end_to_base_Initial.block<3,3>(0,0);
Eigen::AngleAxisd K_marker_Initial;
K_marker_Initial.fromRotationMatrix(R_marker_Initial);
std::cout<<"K_marker_Initial:  "<<std::endl;
Eigen::Vector3d Khat;
Khat=K_marker_Initial.axis();//K^仅代表转轴，没有乘转角
double alpha_marker_Initial;
alpha_marker_Initial=K_marker_Initial.angle();
std::cout<<"Khat"<<std::endl;
initialPointInMarker[0]=T_end_to_base_Initial(0,3);
initialPointInMarker[1]=T_end_to_base_Initial(1,3);
initialPointInMarker[2]=T_end_to_base_Initial(2,3);
std::cout<<"TCP目标平移"<<std::endl;
initialPointInMarker[3]=Khat[0]*alpha_marker_Initial;
initialPointInMarker[4]=Khat[1]*alpha_marker_Initial;
initialPointInMarker[5]=Khat[2]*alpha_marker_Initial;

std::cout<<"开始移动到marker坐标系初始点"<<initialPointInMarker[0]<<","<<initialPointInMarker[1]<<","<<initialPointInMarker[2]<<","<<initialPointInMarker[3]<<","<<initialPointInMarker[4]<<","<<initialPointInMarker[5]<<","<<std::endl;

rtde_control.moveL(initialPointInMarker, 0.5, 0.2);

std::cout<<"已经移动到了marker坐标系的初始点"<<std::endl;




/***************************************************************************************************/

std::this_thread::sleep_for(std::chrono::duration<double>{1});



TCP_pose=rtde_receive.getActualTCPPose();
std::cout<<"又一次获得了TCP位姿"<<std::endl;

//先计算此时的T_end_to_base,再通过T_Box1_to_Initial计算boxHighPoint1
Translation={TCP_pose[0],TCP_pose[1],TCP_pose[2]};
axis={TCP_pose[3],TCP_pose[4],TCP_pose[5]};
axis.normalize();
alpha=TCP_pose[3]/axis[0];
R_end_to_base=Eigen::AngleAxisd(alpha,axis);
T_end_to_base_Initial.block<3,3>(0,0)=R_end_to_base;
T_end_to_base_Initial.block<3,1>(0,3)=Translation;


T_end_to_base_Box1=T_end_to_base_Initial*T_Box1_to_Initial;
std::cout<<"T_end_to_base_Box1 is " <<std::endl<<T_end_to_base_Box1<<std::endl;

Eigen::Matrix3d rotation_matrix;
rotation_matrix=T_end_to_base_Box1.block<3,3>(0,0);
std::cout<<"定义好了rotation_matrix"<<std::endl;
Eigen::AngleAxisd RxRyRz;
RxRyRz.fromRotationMatrix(rotation_matrix);
Eigen::Vector3d K_hat=RxRyRz.axis();
double theta = RxRyRz.angle();
std::cout<<"定义好了轴角"<<std::endl;
boxMidPoint1[0]=T_end_to_base_Box1(0,3);
boxMidPoint1[1]=T_end_to_base_Box1(1,3);
boxMidPoint1[2]=T_end_to_base_Box1(2,3);
boxMidPoint1[3]=K_hat[0]*theta;
boxMidPoint1[4]=K_hat[1]*theta;
boxMidPoint1[5]=K_hat[2]*theta;
std::cout<<"定义好了box1点"<<std::endl;
std::cout<<"目标点是"<<std::endl;
for (int i = 0;i<6;i++)
{
  std::cout<<" "<<boxMidPoint1[i];
}
std::cout<<std::endl;

boxHighPoint1[0]=T_end_to_base_Box1(0,3);
boxHighPoint1[1]=T_end_to_base_Box1(1,3);
boxHighPoint1[2]=T_end_to_base_Box1(2,3)+0.1;
boxHighPoint1[3]=K_hat[0]*theta;
boxHighPoint1[4]=K_hat[1]*theta;
boxHighPoint1[5]=K_hat[2]*theta;

std::cout<<"____________________________________________"<<std::endl;



//计算waterHighPoint
T_end_to_base_water=T_end_to_base_Initial*T_water_to_Initial;
rotation_matrix=T_end_to_base_water.block<3,3>(0,0);
RxRyRz=rotation_matrix;
K_hat=RxRyRz.axis();
theta=RxRyRz.angle();
waterHighPoint[0]=T_end_to_base_water(0,3);
waterHighPoint[1]=T_end_to_base_water(1,3);
waterHighPoint[2]=T_end_to_base_water(2,3)+0.16;
waterHighPoint[3]=K_hat[0]*theta;
waterHighPoint[4]=K_hat[1]*theta;
waterHighPoint[5]=K_hat[2]*theta;
waterLowPoint[0]=T_end_to_base_water(0,3);
waterLowPoint[1]=T_end_to_base_water(1,3);
waterLowPoint[2]=T_end_to_base_water(2,3);
waterLowPoint[3]=K_hat[0]*theta;
waterLowPoint[4]=K_hat[1]*theta;
waterLowPoint[5]=K_hat[2]*theta;
std::cout<<"water点算好了"<<std::endl;
//计算wellHighPoint1
T_end_to_base_Well1=T_end_to_base_Initial*T_Well1_to_Initial;
rotation_matrix=T_end_to_base_Well1.block<3,3>(0,0);
RxRyRz=rotation_matrix;
K_hat=RxRyRz.axis();
theta= RxRyRz.angle();
wellHighPoint1[0]=T_end_to_base_Well1(0,3);
wellHighPoint1[1]=T_end_to_base_Well1(1,3);
wellHighPoint1[2]=T_end_to_base_Well1(2,3)+0.06;
wellHighPoint1[3]=K_hat[0]*theta;
wellHighPoint1[4]=K_hat[1]*theta;
wellHighPoint1[5]=K_hat[2]*theta;

wellMidPoint1[0]=T_end_to_base_Well1(0,3);
wellMidPoint1[1]=T_end_to_base_Well1(1,3);
wellMidPoint1[2]=T_end_to_base_Well1(2,3);
wellMidPoint1[3]=K_hat[0]*theta;
wellMidPoint1[4]=K_hat[1]*theta;
wellMidPoint1[5]=K_hat[2]*theta;
std::cout<<"well1点算好了"<<std::endl;


//*************************************************************************************************************************/



//********************取第一排吸头，然后吸取12*50=600微升蒸馏水到工作板第一排，扔掉吸头并回到初始点准备下一个操作************************/

  //取第一排吸头
  std::cout<<"开始移动到box1的高点"<<std::endl;
  rtde_control.moveL({boxHighPoint1}, 0.5, 0.2);
  std::cout<<"已经移动到了box1的高点"<<std::endl;
  std::cout<<"开始移动到box1的mid点"<<std::endl;
  rtde_control.moveL({boxMidPoint1}, 0.5, 0.2);
 std::cout<<"已经移动到了box1的mid点"<<std::endl;
// Execute 500Hz control loop for a total of 4 seconds, each cycle is ~2ms
  for (unsigned int i=0; i<4000; i++)
  {
    auto t_start = high_resolution_clock::now();
    //First we move the robot down for 4 seconds, then up for 4 seconds
    if (i > 2000)
      rtde_control.forceMode(task_frame, selection_vector, wrench_up, force_type, limits);
    else
      rtde_control.forceMode(task_frame, selection_vector, wrench_down, force_type, limits);
    auto t_stop     = high_resolution_clock::now();
    auto t_duration = std::chrono::duration<double>(t_stop - t_start);

    if (t_duration.count() < dt)
    {
      std::this_thread::sleep_for(std::chrono::duration<double>(dt - t_duration.count()));
    }
  }
  rtde_control.forceModeStop();

rtde_control.moveL({boxHighPoint1}, 0.5, 0.2);

  //吸取蒸馏水

  rtde_control.moveL({waterHighPoint}, 0.5, 0.2);

  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0xD0};
    sendBuffer[7] = {0x07};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
  }

  rtde_control.moveL({waterLowPoint}, 0.5, 0.2);

  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
  }

  rtde_control.moveL({waterHighPoint}, 0.5, 0.2);

  //排到工作板第一排

  rtde_control.moveL({wellHighPoint1}, 0.5, 0.2);
  rtde_control.moveL({wellMidPoint1}, 0.5, 0.2);
  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0xD0};
    sendBuffer[7] = {0x07};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
  }
 rtde_control.moveL({wellHighPoint1}, 0.5, 0.2);

  //扔掉吸头
  rtde_control.moveL({trashHighPoint}, 0.5, 0.2);
  rtde_control.moveL({trashPoint}, 0.5, 0.2);
  {
    sendBuffer[3] = {0x02};
    sendBuffer[6] = {0xD0};
    sendBuffer[7] = {0x07};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    

sendBuffer[3] = {0x02};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
  }

  //*************************************************************************************************************************************/
  //回到初始点

    rtde_control.moveL(initialPoint, 0.5, 0.2);
  
std::this_thread::sleep_for(std::chrono::duration<double>{5});

//取marker xyz，xyzw十组数据的平均值
std::cout<<"average_pose is "<<std::endl;
for(int i = 0;i<7;i++)
{
double sum = 0;

    for (int j = 0;j<10;j++)
    {
sum = sum + pose_queue[i][j];

    }
sum = sum /10;
    average_pose[i] = {sum};
    std::cout<<average_pose[i]<<"  ";
}
std::cout<<std::endl;

//将得到的平均值转换为齐次变换矩阵T_maker_to_camera
q = {average_pose[6],average_pose[3],average_pose[4],average_pose[5]};
 R= q.toRotationMatrix();
std::cout<<"得到的四元数用旋转矩阵表示为："<< std::endl<<R<<std::endl;
T_marker_to_camera.setIdentity();
T_marker_to_camera(0,3)=average_pose[0];
T_marker_to_camera(1,3)=average_pose[1];
T_marker_to_camera(2,3)=average_pose[2];
T_marker_to_camera.block<3,3>(0,0)=R;
std::cout<<"T_marker_to_camera is"<<std::endl<<T_marker_to_camera<<std::endl<<std::endl;

//获得UR的T_end_to_base（从UR controller 的output中读出末端位置和姿态，并将其转换为齐次变换矩阵以便计算）
TCP_pose =rtde_receive.getActualTCPPose();
Translation={TCP_pose[0],TCP_pose[1],TCP_pose[2]};
 axis={TCP_pose[3],TCP_pose[4],TCP_pose[5]};
axis.normalize();
alpha=TCP_pose[3]/axis[0];
R_end_to_base=Eigen::AngleAxisd(alpha,axis);
T_end_to_base.block<3,3>(0,0)=R_end_to_base;
T_end_to_base.block<3,1>(0,3)=Translation;

//计算此刻的T_marker_to_base,这个齐次变换矩阵在下次移动marker之前marker坐标系相对于base的位置姿态不变
T_marker_to_base=T_end_to_base*T_camera_to_end*T_marker_to_camera;
std::cout<<"T_marker_to_base is"<<std::endl<<T_marker_to_base<<std::endl;


std::this_thread::sleep_for(std::chrono::duration<double>{1});

//计算marker坐标系下的机器人Initial_pose
T_end_to_base_Initial=T_marker_to_base*T_marker_to_camera_Initial.inverse()*T_camera_to_end.inverse();
std::cout<<"T_end_to_base_Initial"<<std::endl;
std::cout<<T_end_to_base_Initial<<std::endl;

R_marker_Initial=T_end_to_base_Initial.block<3,3>(0,0);

K_marker_Initial=R_marker_Initial;
std::cout<<"K_marker_Initial:  "<<std::endl;

Khat=K_marker_Initial.axis();//K^仅代表转轴，没有乘转角

alpha_marker_Initial=K_marker_Initial.angle();
std::cout<<"Khat"<<std::endl;
initialPointInMarker[0]=T_end_to_base_Initial(0,3);
initialPointInMarker[1]=T_end_to_base_Initial(1,3);
initialPointInMarker[2]=T_end_to_base_Initial(2,3);
std::cout<<"TCP目标平移"<<std::endl;
initialPointInMarker[3]=Khat[0]*alpha_marker_Initial;
initialPointInMarker[4]=Khat[1]*alpha_marker_Initial;
initialPointInMarker[5]=Khat[2]*alpha_marker_Initial;

std::cout<<"开始移动到marker坐标系初始点"<<initialPointInMarker[0]<<","<<initialPointInMarker[1]<<","<<initialPointInMarker[2]<<","<<initialPointInMarker[3]<<","<<initialPointInMarker[4]<<","<<initialPointInMarker[5]<<","<<std::endl;

rtde_control.moveL(initialPointInMarker, 0.5, 0.2);

std::this_thread::sleep_for(std::chrono::duration<double>{2});



TCP_pose=rtde_receive.getActualTCPPose();
std::cout<<"又一次获得了TCP位姿"<<std::endl;

//先计算此时的T_end_to_base,再通过T_Box1_to_Initial计算boxHighPoint1
Translation={TCP_pose[0],TCP_pose[1],TCP_pose[2]};
axis={TCP_pose[3],TCP_pose[4],TCP_pose[5]};
axis.normalize();
alpha=TCP_pose[3]/axis[0];
R_end_to_base=Eigen::AngleAxisd(alpha,axis);
T_end_to_base_Initial.block<3,3>(0,0)=R_end_to_base;
T_end_to_base_Initial.block<3,1>(0,3)=Translation;


//计算boxHighPoint2
T_end_to_base_Box2=T_end_to_base_Initial*T_Box2_to_Initial;
rotation_matrix=T_end_to_base_Box2.block<3,3>(0,0);
RxRyRz=rotation_matrix;
K_hat=RxRyRz.axis();
theta = RxRyRz.angle();

boxHighPoint2[0]=T_end_to_base_Box2(0,3);
boxHighPoint2[1]=T_end_to_base_Box2(1,3);
boxHighPoint2[2]=T_end_to_base_Box2(2,3)+0.1;
boxHighPoint2[3]=K_hat[0]*theta;
boxHighPoint2[4]=K_hat[1]*theta;
boxHighPoint2[5]=K_hat[2]*theta;

boxMidPoint2[0]=T_end_to_base_Box2(0,3);
boxMidPoint2[1]=T_end_to_base_Box2(1,3);
boxMidPoint2[2]=T_end_to_base_Box2(2,3);
boxMidPoint2[3]=K_hat[0]*theta;
boxMidPoint2[4]=K_hat[1]*theta;
boxMidPoint2[5]=K_hat[2]*theta;

//计算bluewaterHighPoint
T_end_to_base_bluewater=T_end_to_base_Initial*T_bluewater_to_Initial;

rotation_matrix=T_end_to_base_bluewater.block<3,3>(0,0);

RxRyRz=rotation_matrix;

K_hat=RxRyRz.axis();
theta = RxRyRz.angle();


bluewaterHighPoint[0]=T_end_to_base_bluewater(0,3);
bluewaterHighPoint[1]=T_end_to_base_bluewater(1,3);
bluewaterHighPoint[2]=T_end_to_base_bluewater(2,3)+0.1;
bluewaterHighPoint[3]=K_hat[0]*theta;
bluewaterHighPoint[4]=K_hat[1]*theta;
bluewaterHighPoint[5]=K_hat[2]*theta;

bluewaterLowPoint[0]=T_end_to_base_bluewater(0,3);
bluewaterLowPoint[1]=T_end_to_base_bluewater(1,3);
bluewaterLowPoint[2]=T_end_to_base_bluewater(2,3);
bluewaterLowPoint[3]=K_hat[0]*theta;
bluewaterLowPoint[4]=K_hat[1]*theta;
bluewaterLowPoint[5]=K_hat[2]*theta;

//计算wellHighPoint1
T_end_to_base_Well1=T_end_to_base_Initial*T_Well1_to_Initial;
rotation_matrix=T_end_to_base_Well1.block<3,3>(0,0);
RxRyRz=rotation_matrix;
K_hat=RxRyRz.axis();
 theta = RxRyRz.angle();
wellHighPoint1[0]=T_end_to_base_Well1(0,3);
wellHighPoint1[1]=T_end_to_base_Well1(1,3);
wellHighPoint1[2]=T_end_to_base_Well1(2,3)+0.1;
wellHighPoint1[3]=K_hat[0]*theta;
wellHighPoint1[4]=K_hat[1]*theta;
wellHighPoint1[5]=K_hat[2]*theta;

wellMidPoint1[0]=T_end_to_base_Well1(0,3);
wellMidPoint1[1]=T_end_to_base_Well1(1,3);
wellMidPoint1[2]=T_end_to_base_Well1(2,3);
wellMidPoint1[3]=K_hat[0]*theta;
wellMidPoint1[4]=K_hat[1]*theta;
wellMidPoint1[5]=K_hat[2]*theta;




//************************取第二排吸头，然后吸取12*50=600微升蒸馏水到工作板第一排，扔掉吸头并回到初始点准备下一个操作******************//

  //取第二排吸头
  rtde_control.moveL({boxHighPoint2}, 0.5, 0.2);
  rtde_control.moveL({boxMidPoint2}, 0.5, 0.2);

  // force_mode
// Execute 500Hz control loop for a total of 4 seconds, each cycle is ~2ms
  for (unsigned int i=0; i<4000; i++)
  {
    auto t_start = high_resolution_clock::now();
    // First we move the robot down for 2 seconds, then up for 2 seconds
    if (i > 2000)
      rtde_control.forceMode(task_frame, selection_vector, wrench_up, force_type, limits);
    else
      rtde_control.forceMode(task_frame, selection_vector, wrench_down, force_type, limits);
    auto t_stop     = high_resolution_clock::now();
    auto t_duration = std::chrono::duration<double>(t_stop - t_start);

    if (t_duration.count() < dt)
    {
      std::this_thread::sleep_for(std::chrono::duration<double>(dt - t_duration.count()));
    }
  }
  rtde_control.forceModeStop();
  
  rtde_control.moveL({boxHighPoint2}, 0.5, 0.2);

  //吸取蒸馏水
  rtde_control.moveL(bluewaterHighPoint, 0.5, 0.2);
 {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0xD0};
    sendBuffer[7] = {0x07};
    checkSum(sendBuffer);        
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
  }

  rtde_control.moveL(bluewaterLowPoint, 0.5, 0.2);


  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);   
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9)); 
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
  }

  rtde_control.moveL(bluewaterHighPoint, 0.5, 0.2);

  //排到工作板第一排

  rtde_control.moveL({wellHighPoint1},0.5, 0.2);
rtde_control.moveL({wellMidPoint1},0.5, 0.2);

  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0xD0};
    sendBuffer[7] = {0x07};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    
  }
  //混合均匀
for (int i = 0;i<6;i++)
{
  wellLowPoint1[i]=wellMidPoint1[i];
}
wellLowPoint1[2]=wellLowPoint1[2]-0.032;
rtde_control.moveL(wellLowPoint1,0.5,0.2);
  {
    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    

    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0xD0};
    sendBuffer[7] = {0x07};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    

    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    

    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0xD0};
    sendBuffer[7] = {0x07};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

   sendBuffer[3] = {0x01};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    

    sendBuffer[3] = {0x01};
    sendBuffer[6] = {0xD0};
    sendBuffer[7] = {0x07};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    
  }
  
  rtde_control.moveL({wellHighPoint1},0.5,0.2);


  //扔掉吸头
  rtde_control.moveL({trashHighPoint}, 0.5, 0.2);
  rtde_control.moveL({trashPoint}, 0.5, 0.2);
    {
    sendBuffer[3] = {0x02};
    sendBuffer[6] = {0xD0};
    sendBuffer[7] = {0x07};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    

sendBuffer[3] = {0x02};
    sendBuffer[6] = {0x00};
    sendBuffer[7] = {0x00};
    checkSum(sendBuffer);
    serialPort.write_some(boost::asio::buffer(sendBuffer, 9));
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
  }


  //回到初始点
  rtde_control.moveL(initialPoint, 0.5, 0.2);

  //*****************************************************************************************************

  //****实验完成:)

  std::cout << "倍比稀释实验完成：）";


  //****************************************支线任务完成！************************************************//

  return 0;
  
}
