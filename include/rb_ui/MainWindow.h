
#ifndef rb_msgs_MAINWINDOW_H
#define rb_msgs_MAINWINDOW_H


#include "CMsgBox.h"
#include "BaseWindow.h"
#include "qthreadForRos.h"
#include "gloalVal.h"


#include "std_srvs/Empty.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/UInt16MultiArray.h"
#include "roscpp_tutorials/TwoInts.h"
#include "rb_msgAndSrv/rb_ArrayAndBool.h"
#include "rb_msgAndSrv/rb_DoubleBool.h"
#include "rb_msgAndSrv/robotConn.h"
#include "rb_msgAndSrv/robotError.h"
#include "cubeParse/Detection.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include "rb_msgAndSrv/rbImageList.h"
#include <qregion.h>
#include "rb_msgAndSrv/SetEnableSrv.h"
#include "logmanager.h"
#include "hirop_msgs/SetGripper.h"
#include "hirop_msgs/connectGripper.h"
#include "hsr_rosi_device/ClearFaultSrv.h"
#include "hsr_rosi_device/SetEnableSrv.h"
#include "industrial_msgs/RobotStatus.h"
#include "rb_msgAndSrv/rb_StringArray.h"
#include "rb_msgAndSrv/rb_string.h"
//#include "messagehandler.h"

//观察者模式(管理节点重启与关闭)
class observer_rebootUiNode{
public:
    MainWindow* mainwindow;
    ros::AsyncSpinner* sp= nullptr;
public:
    void setparm(MainWindow* mainwindow){
        this->mainwindow=mainwindow;
    }
    //释放内存
    void shutdownNode(){
        ros::shutdown();
        if(sp!= nullptr){
            delete sp;
            sp= nullptr;
        }
    }
    //重启ui节点
    void rebootUiNode();
};

class MainWindow: public BaseWindow {
    Q_OBJECT
public:
    MainWindow(ros::NodeHandle* node,QWidget* parent = Q_NULLPTR);
    ~MainWindow();

private:
    //加入节点观察者
    observer_rebootUiNode ob_node;
    //全局变量
    int index_magicStep;
    int* checkArray;
    bool flag_RvizRun= false;
    CMsgBox* cbox=nullptr;
    QMutex mutex_showImg;
    cv::Mat gl_leftImageMat;
    cv::Mat gl_rightImageMat;
    //全局状态标志
    bool Flag_connOk= false;//连接状态
    bool connFlag_LeftRobot;//左机器人连接状态
    bool connFlag_RightRobot;//右机器人连接状态
    bool errFlag_LeftRobot;//左机器人报警状态
    bool errFlag_RightRobot;//右机器人报警状态
    bool enableFlag_LeftRobot;//左机器人伺服状态
    bool enableFlag_RightRobot;//右机器人伺服状态
    bool connFlag_LeftCamera; //左边相机连接状态
    bool connFlag_RightCamera;//右边相机连接状态
    bool connFlag_LeftGripper;//左夹爪连接状态
    bool connFlag_RightGripper;//右夹爪连接状态
    bool flag_rb1Enable= false;
    bool flag_rb2Enable= false;
    bool flag_gripper1= false;
    bool flag_gripper2= false;
    //holdOnFlag 监控下降沿信号
    bool holdOnFlag_LeftRobotConn= false;
    bool holdOnFlag_RightRobotConn= false;
    bool holdOnFlag_LeftRobotErr= false;
    bool holdOnFlag_RightRobotErr= false;
    bool holdOnFlag_LeftRobotEnable= false;
    bool holdOnFlag_RightRobotEnable= false;
    bool holdOnFlag_LeftCamera= false;
    bool holdOnFlag_RightCamera= false;
    //魔方点位调试页面
    int calibration_mode =0; //魔方复原动作为0, 魔方拍照动作为1
    int calibration_stepNum =0 ; //复原动作执行步骤数
    int calibration_totalStep =0; //复原动作总步骤数
    //rosparam参数
    bool isRunning_solveMagic;
    bool isRunning_grab;
    bool isRunning_d435i;
    //ros节点
    QTimer* updateTimer_listen_roscore;
    QTimer* updateTimer;
    QTimer* updateTimer_com;
    QTimer* updateTimer_rob1status;
    QTimer* updateTimer_rob2status;
    QTimer* updateTimer_LeftCamera;
    QTimer* updateTimer_RightCamera;
    ros::Publisher rbStopCommand_publisher;//机器人停止命令
    ros::Publisher SafetyStop_publisher;//机器人紧急停止
    ros::Subscriber previewImage1_subscriber;//
    ros::Subscriber previewImage2_subscriber;//
    ros::Subscriber Progress_rbSolve;//机器人解魔方进度采集
    ros::Subscriber MagicSolveSolution;//图详解析结果
    ros::Subscriber camera_subscriber;//相机数据采集
    ros::Subscriber rob1Status_subscriber;//机器人1状态数据采集
    ros::Subscriber rob2Status_subscriber;//机器人2状态数据采集
    ros::Subscriber Leftcamera_subscriber;//相机数据采集
    ros::Subscriber Rightcamera_subscriber;//相机数据采集
    ros::Subscriber MagicSolve_subscriber;//魔方解析数据采集
    ros::ServiceClient MagicDataUpdate_client;//魔方数据修改客户端
    ros::ServiceClient rbConnCommand_client;//机器人连接客户端

    ros::ServiceClient LeftGripperSet_client;//左夹爪连接客户端
    ros::ServiceClient RightGripperSet_client;//右夹爪连接客户端
    ros::ServiceClient LeftGripperConn_client;//左夹爪连接客户端
    ros::ServiceClient RightGripperConn_client;//右夹爪连接客户端

    ros::ServiceClient LeftRobReset_client;//左机器人复位
    ros::ServiceClient RightRobReset_client;//右机器人复位
    ros::ServiceClient LeftRobEnable_client;//左机器人使能
    ros::ServiceClient RightRobEnable_client;//右机器人使能

    ros::ServiceClient rbRunCommand_client ;
    ros::ServiceClient rbStopCommand_client ;
    ros::ServiceClient rbSetEnable1_client;
    ros::ServiceClient rbSetEnable2_client;
    ros::ServiceClient rbErrStatus_client;
    ros::ServiceClient rbGrepSetCommand_client;
    ros::ServiceClient MagicStepRunCommand_client ;//魔方分步完成
    ros::ServiceClient ImageGet_client;//获得图像数据
    ros::Subscriber magicGetData_subscriber;//机器人连接状态

    //话题或服务对象初始化头文件部分

    ros::ServiceClient cubeActionClient; //魔方示教点位执行客户端
    ros::ServiceClient cubeApproachClient; //魔法示教寸进客户端
    ros::ServiceClient cubeRecordPoseClient; //魔方示教记录点位客户端
    ros::ServiceClient cubeNewTeachClient; //魔方重新示教客户端
    ros::ServiceClient cubeResetPoseClient; //魔方示教动作点位重置客户端
    ros::Subscriber     CuberRobotPose_sub;  //魔方示教机器人ROS点位采集

    //子线程句柄
    qthreadForRos *thread_forSysCheck;//设备启动自检子线程
    qthreadForRos *thread_forRbConn;//设备连接子线程
    qthreadForRos *thread_forRviz;//设备连接子线程
    qthreadForRos *thread_forCloseRviz;//设备连接子线程
    qthreadForRos *thread_forBeginRun;//开始运行子线程
    qthreadForRos *thread_forSysReset;//系统复位子线程
    qthreadForRos *thread_MagicStepRun;//分步运行子线程
    qthreadForRos *thread_forRbGrepSet;//机器人抓取子线程
    qthreadForRos *thread_forLisionErrInfo;//监听故障子线程

public:
    //ros话题与服务
    void initRosTopic();
private:
    //系统变量初始化
    void SysVarInit();
    //处理所有信号和槽函数
    void signalAndSlot();
    //按钮槽函数
    void dev_connect();//按钮槽函数_设备连接槽函数
    void rviz_statup();//按钮槽函数_rviz启动
    void run_statup();//按钮槽函数_运行启动
    void run_stop();//按钮槽函数_运行停止
    void SysReset();//按钮槽函数_系统复位
    void magicCube_AutoRun();//按钮槽函数_一键解魔方
    void magicCube_get();//按钮槽函数_采集魔方数据
    void magicCube_solve();//按钮槽函数_解算魔方数据
    void magicCube_execute();//按钮槽函数_执行解算魔方数据
    void magicUpdateData();//魔方数据修改
    void robot_grab();//按钮槽函数_机器人抓取
    void safety_sysStop();//按钮槽函数_系统停止
    void safety_rob1Stop();//按钮槽函数_机器人1停止
    void safety_rob2Stop();//按钮槽函数_机器人2停止
    void oputRecord();
    void clearRecord();
    void timer_listen_roscore();
    void timer_onUpdate();
    void timer_LeftCamera();
    void timer_RightCamera();
    void timer_comUpdate();//公共刷新连接状态
    void timer_robot1Status();//公共刷新连接状态
    void timer_robot2Status();//公共刷新连接状态
    void label_tabmp_1_showImage(int mode, int stepNum);
    //魔方点位校准调试页面
    void slot_btn_tabmp_do();
    void slot_btn_tabmp_step();
    void slot_btn_tabmp_recordPose();
    void slot_btn_tabmp_newteach();
    void slot_btn_tabmp_resetPose();
    void slot_btn_rb1_goHomePose();
    void slot_btn_rb2_goHomePose();
    //单步调试页面按钮槽函数
    void slot_btn_rb1SetEnable();
    void slot_btn_rb2SetEnable();
    void slot_btn_rb1Reset();
    void slot_btn_rb2Reset();
    void slot_gripper1();
    void slot_gripper2();
    void slot_rb1putBack();
    void slot_rb2putBack();
    void slot_ResetGrepFun();
    //opencv相关
    QImage cvMat2QImage(const cv::Mat& mat);
    //ros节点回调函数
    void callback_rob1Status_subscriber(const industrial_msgs::RobotStatus::ConstPtr robot_status);
    void callback_rob2Status_subscriber(const industrial_msgs::RobotStatus::ConstPtr robot_status);
    void callback_LeftCamera_subscriber(const sensor_msgs::Image::ConstPtr image);
    void callback_RightCamera_subscriber(const sensor_msgs::Image::ConstPtr image);
    void callback_preview1_subscriber(const sensor_msgs::Image::ConstPtr image);
    void callback_preview2_subscriber(const sensor_msgs::Image::ConstPtr image);
    void callback_camera_subscriber(const sensor_msgs::Image::ConstPtr &msg);
    void callback_magicGetData_subscriber(rb_msgAndSrv::rbImageList rbimageList);
    void callback_magicSolve_subscriber(rb_msgAndSrv::rb_StringArray data_msg);
    void callback_ProgressRbSolve_subscriber(std_msgs::Int8MultiArray data_msg);
    void callback_MagicSolveSolution_subscriber(std_msgs::Bool data_msg);
    //线程处理
    void thread_SysCheck();
    void thread_rbConnCommand();
    void thread_rbRvizCommand();
    void thread_rbCloseRvizCommand();
    void thread_BeginRun();
    void thread_SysReset();
    void thread_RbGrepSet();
//    void thread_MagicStepRunCommand();
    void thread_GagicGetData();
    void thread_GagicSolve();
    void thread_GagicRunSolve();
    void thread_AutoSolveMagic();

signals:
    void emitTextControl(QString text) const;
    void emitQmessageBox(infoLevel level,QString info);
    void emitLightColor(QLabel* label,string color);
    void emitStartTimer(QTimer* timer);
    void emitSwapDataWithCMsgBox(int* array);
private slots:
    void displayTextControl(QString text);
    void showQmessageBox(infoLevel level,QString info);
    void showLightColor(QLabel* label,string color);
    void runTimer(QTimer* timer);
    void slot_cBox_setRunMode(const QString& text);
    void slot_tabWidgetClicked(int index_tab);
    void slot_combox3_Clicked(int index);
    void slot_comboBox_tabmp_1_Clicked(int index);


};




#endif //rb_msgs_MAINWINDOW_H
