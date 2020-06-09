
#ifndef rb_msgs_MAINWINDOW_H
#define rb_msgs_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTableView>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QWidget>
#include <QtWidgets/QLineEdit>
#include <QDialog>

#include <QDir>
#include <QMessageBox>
#include <QProcess>
#include <QTimer>
#include <QThread>
#include <QImage>
#include <iostream>
#include <fstream>
#include "QDebug"

#include "qdebug.h"
#include "ros/ros.h"
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
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include "rb_msgAndSrv/rbImageList.h"
#include <qregion.h>
#include "rb_msgAndSrv/SetEnableSrv.h"
#include "logmanager.h"
#include "hirop_msgs/robotConn.h"
#include "hirop_msgs/robotError.h"
#include "hirop_msgs/SetGripper.h"
#include "hirop_msgs/connectGripper.h"
#include "hsr_rosi_device/ClearFaultSrv.h"
#include "hsr_rosi_device/SetEnableSrv.h"
#include "industrial_msgs/RobotStatus.h"
#include "rb_msgAndSrv/rb_StringArray.h"
#include "rb_msgAndSrv/rb_string.h"
//#include "messagehandler.h"
using namespace std;



#define  BTN_W 150
#define  BTN_H 50
#define  COMBOX_W 200
#define  COMBOX_H 50


class  MainWindow;
extern bool flag_syscheckOk;
extern bool flag_sysckCancel;
extern bool flag_delCbox;


class CMsgBox : public QDialog
{
Q_OBJECT
public:
    explicit CMsgBox(QWidget *parent = nullptr);
    ~CMsgBox();
    static int showMsgBox(QWidget *parent = nullptr);

private:
    void init();
    void slot_timerUpdate();

private slots:
    void slot_SwapDataWithMainwin(int* array);
public:
    enum EnmButton{
        ENM_OK_BTN = 0,
        ENM_CANCEL_BTN,
    };

private:
    QLabel* m_lableTitle;
    QPlainTextEdit* m_plaintext;
    QTimer* cTimer1;
    QPushButton *pBtn_checkCancel;
    QPushButton *pBtn_checkOk;
    int* c_array;
    bool checkOk= false;

};



//给信号与槽函数使用的枚举类型
enum  infoLevel{information,warning};


//多线程,线程与主线程信号与槽通信
class qthreadForRos : public QThread{
    Q_OBJECT
public:
    MainWindow* m;
    void (MainWindow::*f)();
    //函数回调
    void setParm(MainWindow* m,void (MainWindow::*f)()){
        this->m=m;
        this->f=f;
    }
    void run(){
        (m->*f)();
    }
signals:
    void signal_SendMsgBox(infoLevel level,QString info);//infoLevel level,QString info
};

class MainWindow: public QMainWindow {
    Q_OBJECT
public:
    MainWindow(ros::NodeHandle* node,QWidget* parent = Q_NULLPTR);
    ~MainWindow();

private:
    //全局状态标志
    bool flag_rbErrStatus;//机器人连接状态标志
    int index_magicStep;
    int* checkArray;
    bool flag_RvizRun= false;
    CMsgBox* cbox=nullptr;
    QString photoPath;//图片路径
    QString logPath;//图片路径
    QPixmap fitpixmap_redLight;
    QPixmap fitpixmap_greenLight;
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
    //rosparam参数
    bool isRunning_solveMagic;
    bool isRunning_grab;
    bool isRunning_d435i;
    //qt参数
    QString groupBox_qss;
    //ros节点
    ros::NodeHandle* Node;
    QTimer* updateTimer;
    QTimer* updateTimer_com;
    QTimer* updateTimer_rob1status;
    QTimer* updateTimer_rob2status;
    QTimer* updateTimer_LeftCamera;
    QTimer* updateTimer_RightCamera;
    ros::Publisher rbStopCommand_publisher;//机器人停止命令
    ros::Publisher SafetyStop_publisher;//机器人紧急停止
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
    //子线程句柄
    qthreadForRos *thread_forSysCheck;//设备启动自检子线程
    qthreadForRos *thread_forRbConn;//设备连接子线程
    qthreadForRos *thread_forRviz;//设备连接子线程
    qthreadForRos *thread_forCloseRviz;//设备连接子线程
    qthreadForRos *thread_forBeginRun;//开始运行子线程
    qthreadForRos *thread_MagicStepRun;//分步运行子线程
    qthreadForRos *thread_forRbGrepSet;//机器人抓取子线程
    qthreadForRos *thread_forLisionErrInfo;//监听故障子线程
private:
    //系统变量初始化
    void SysVarInit();
    //UI流程
    void initUi(QMainWindow *MainWindow);
    void retranslateUi(QMainWindow *MainWindow);
    //处理所有信号和槽函数
    void signalAndSlot();
    //槽函数
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
    void timer_onUpdate();
    void timer_LeftCamera();
    void timer_RightCamera();
    void timer_comUpdate();//公共刷新连接状态
    void timer_robot1Status();//公共刷新连接状态
    void timer_robot2Status();//公共刷新连接状态

    //单步调试页面按钮槽函数
    void slot_btn_rb1SetEnable();
    void slot_btn_rb2SetEnable();
    void slot_btn_rb1Reset();
    void slot_btn_rb2Reset();
    void slot_gripper1();
    void slot_gripper2();
    //opencv相关
    QImage cvMat2QImage(const cv::Mat& mat);
    //ros节点回调函数
    void callback_rob1Status_subscriber(const industrial_msgs::RobotStatus::ConstPtr robot_status);
    void callback_rob2Status_subscriber(const industrial_msgs::RobotStatus::ConstPtr robot_status);
    void callback_LeftCamera_subscriber(const sensor_msgs::Image::ConstPtr image);
    void callback_RightCamera_subscriber(const sensor_msgs::Image::ConstPtr image);
    void callback_rbConnStatus_subscriber(std_msgs::UInt8MultiArray data_msg);
    void callback_rbErrStatus_subscriber(std_msgs::UInt16MultiArray data_msg);
    void callback_camera_subscriber(const sensor_msgs::Image::ConstPtr &msg);
    void callback_magicGetData_subscriber(rb_msgAndSrv::rbImageList rbimageList);
    void callback_magicSolve_subscriber(rb_msgAndSrv::rb_StringArray data_msg);
    //线程处理
    void thread_SysCheck();
    void thread_rbConnCommand();
    void thread_rbRvizCommand();
    void thread_rbCloseRvizCommand();
    void thread_BeginRun();
    void thread_RbGrepSet();
    void thread_LisionErrInfo();
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

private:
    //qt控件
    QMainWindow* w;
    QWidget *centralWidget;
    QVBoxLayout *verticalLayout_2;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QLabel *label_3;
    QHBoxLayout *horizontalLayout_3;
    QTabWidget *tabWidget;

    QWidget *tab;
    QHBoxLayout *horizontalLayout_4;
    QVBoxLayout *verticalLayout_4;
    QHBoxLayout *horizontalLayout_2;
    QGridLayout *gridLayout;
    QHBoxLayout *horizontalLayout_21;
    QComboBox *comboBox_setRunMode;
    QLabel *label_5;
    QLabel *label_6;
    QLabel *label_7;
    QLabel *label_8;
    QLabel *label_rb1CoonStatus;
    QLabel *label_rb2CoonStatus;
    QLabel *label_rb1ErrStatus;
    QLabel *label_rb2ErrStatus;
    QLabel *label_121;
    QLabel *label_122;
    QLabel *label_123;
    QLabel *label_124;
    QLabel *label_rob1EnableStatus;
    QLabel *label_rob2EnableStatus;
    QLabel *label_LeftCameraConnStatus;
    QLabel *label_RightCameraConnStatus;

    QHBoxLayout *horizontalLayout_5;
    QPushButton *btn_rbConn;
    QPushButton *btn_rvizRun;
    QPushButton *btn_beginRun;
    QPushButton *btn_normalStop;
    QPushButton *btn_SysReset;

    QWidget *tab_2;
    QHBoxLayout *horizontalLayout_6;
    QVBoxLayout *verticalLayout_5;
    QGroupBox *groupBox_tab2_1;
    QHBoxLayout *horizontalLayout_14;
    QPushButton *btn_rb1SetEnable;
    QPushButton *btn_rb2SetEnable;
    QPushButton *btn_rb1Reset;
    QPushButton *btn_rb2Reset;
    QGroupBox *groupBox_tab2_2;
    QHBoxLayout *horizontalLayout_19;
    QPushButton *gripper1;
    QPushButton *gripper2;
    QGroupBox *groupBox_tab3_3;


    QWidget *tab_3;
    QHBoxLayout *horizontalLayout_8;
    QHBoxLayout *horizontalLayout_7;
    QHBoxLayout *horizontalLayout_tab3_1;
    QVBoxLayout *verticalLayout_6;
    QGridLayout* gridLayout1;
    vector<QLabel*> list_label_picture;
    QLabel* label_picture1;
    QLabel* label_picture2;
    QLabel* label_picture3;
    QLabel* label_picture4;
    QLabel* label_picture5;
    QLabel* label_picture6;
    QVBoxLayout *verticalLayout_8;
    QPushButton *btn_magicGetdata;
    QPushButton *btn_magicSolve;
    QPushButton *btn_magicRunSolve;
    QPushButton *btn_magicAutoSolve;
    QPushButton *btn_updateData;
    QLineEdit* line_updataData;

    QWidget *tab_4;
    QHBoxLayout *horizontalLayout_10;
    QHBoxLayout *horizontalLayout_9;
    QVBoxLayout *verticalLayout_11;
    QLabel *label_2;
    QVBoxLayout *verticalLayout_9;
    QGroupBox *groupBox_setMod;
    QHBoxLayout *horizontalLayout_11;
    QComboBox *comboBox;
    QGroupBox *groupBox_selectObject;
    QHBoxLayout *horizontalLayout_12;
    QComboBox *comboBox_2;
    QGroupBox *groupBox_selectRobot;
    QHBoxLayout *horizontalLayout_13;
    QComboBox *comboBox_3;
    QPushButton *btn_rbGrep;
    QWidget *tab_5;
    QHBoxLayout *horizontalLayout_15;
    QHBoxLayout *horizontalLayout_16;
    QVBoxLayout *verticalLayout_1index_magicStep3;
    QPlainTextEdit *plainTextEdit;
    QVBoxLayout *verticalLayout_12;
    QPushButton *btn_oputRecord;
    QPushButton *btn_clearRecord;
    QWidget *tab_6;
    QHBoxLayout *horizontalLayout_18;
    QHBoxLayout *horizontalLayout_17;
    QPushButton *btn_SatetyStop;
    QPushButton *btn_SatetyRb1Reset;
    QPushButton *btn_SatetyRb2Reset;
    QMenuBar *menuBar;
    QStatusBar *statusBar;

    QVBoxLayout* verticalLayout_13;
    QLabel* showMagicStepLable;
    QLabel* isRunning_solveMagic_Lable;
    QLabel* isRunning_grab_Lable;

};


#endif //rb_msgs_MAINWINDOW_H
