#include "MainWindow.h"

bool flag_syscheckOk= false;//系统自检通过标志
bool flag_sysckCancel= false;//系统自检取消
bool flag_delCbox= false;//释放自检弹框


MainWindow::MainWindow(ros::NodeHandle *node, QWidget *parent):QMainWindow(parent),Node(node){
    //系统变量初始化
    SysVarInit();
    //初始化UI
    initUi(this);
    //信号与槽绑定
    signalAndSlot();
}

void MainWindow::SysVarInit() {
    checkArray = new int[9]{0};
    isRunning_solveMagic= false;
    isRunning_grab= false;
    index_magicStep=0;
    //连接状态
    connFlag_LeftCamera= false;
    connFlag_RightCamera= false;
    connFlag_LeftRobot= false;
    connFlag_RightRobot= false;
    errFlag_LeftRobot= false;
    errFlag_RightRobot= false;
    enableFlag_LeftRobot= false;
    enableFlag_RightRobot= false;
    connFlag_LeftGripper= false;
    connFlag_RightGripper= false;
    //定时器实体化
    updateTimer = new QTimer(this);
    updateTimer->setInterval(1000);
    updateTimer_LeftCamera = new QTimer();
    updateTimer_LeftCamera->setInterval(1000);
    updateTimer_RightCamera = new QTimer();
    updateTimer_RightCamera->setInterval(1000);
    updateTimer_com = new QTimer(this);
    updateTimer_com->setInterval(1000);

    updateTimer_rob1status = new QTimer(this);
    updateTimer_rob1status->setInterval(1000);
    updateTimer_rob2status = new QTimer(this);
    updateTimer_rob2status->setInterval(1000);
    //话题或服务对象初始化
    previewImage1_subscriber=Node->subscribe<sensor_msgs::Image>("/UR51/preview_image",1,boost::bind(&MainWindow::callback_preview1_subscriber,this,_1));
    previewImage2_subscriber=Node->subscribe<sensor_msgs::Image>("/UR52/preview_image",1,boost::bind(&MainWindow::callback_preview2_subscriber,this,_1));
    rob1Status_subscriber=Node->subscribe<industrial_msgs::RobotStatus>("/UR51/robot_status",1,boost::bind(&MainWindow::callback_rob1Status_subscriber,this,_1));
    rob2Status_subscriber=Node->subscribe<industrial_msgs::RobotStatus>("/UR52/robot_status",1,boost::bind(&MainWindow::callback_rob2Status_subscriber,this,_1));
    Leftcamera_subscriber=Node->subscribe<sensor_msgs::Image>("/camera_base/color/image_raw",1,boost::bind(&MainWindow::callback_LeftCamera_subscriber,this,_1));
    Rightcamera_subscriber=Node->subscribe<sensor_msgs::Image>("/camera_base_right/color/image_raw",1,boost::bind(&MainWindow::callback_RightCamera_subscriber,this,_1));
    magicGetData_subscriber=Node->subscribe<rb_msgAndSrv::rbImageList>("/cube_image",1,&MainWindow::callback_magicGetData_subscriber,this);
    MagicSolve_subscriber=Node->subscribe<rb_msgAndSrv::rb_StringArray>("changeColor",1000,&MainWindow::callback_magicSolve_subscriber,this);
    Progress_rbSolve=Node->subscribe<std_msgs::Int8MultiArray>("/progress_rbSolveMagic",1000,&MainWindow::callback_ProgressRbSolve_subscriber,this);
    MagicSolveSolution=Node->subscribe<std_msgs::Bool>("solution_situation",1000,&MainWindow::callback_MagicSolveSolution_subscriber,this);

    ImageGet_client = Node->serviceClient<cubeParse::Detection>("cube_detect");

    MagicDataUpdate_client = Node->serviceClient<rb_msgAndSrv::rb_string>("cube_correct");
    LeftGripperSet_client = Node->serviceClient<hirop_msgs::SetGripper>("/UR51/setGripper");
    RightGripperSet_client = Node->serviceClient<hirop_msgs::SetGripper>("/UR52/setGripper");
    LeftGripperConn_client = Node->serviceClient<hirop_msgs::connectGripper>("/UR51/connectGripper");
    RightGripperConn_client = Node->serviceClient<hirop_msgs::connectGripper>("/UR52/connectGripper");

    LeftRobReset_client = Node->serviceClient<hsr_rosi_device::ClearFaultSrv>("/UR51/clear_robot_fault");
    RightRobReset_client = Node->serviceClient<hsr_rosi_device::ClearFaultSrv>("/UR52/clear_robot_fault");
    LeftRobEnable_client = Node->serviceClient<hsr_rosi_device::SetEnableSrv>("/UR51/set_robot_enable");
    RightRobEnable_client = Node->serviceClient<hsr_rosi_device::SetEnableSrv>("/UR52/set_robot_enable");

    rbStopCommand_publisher= Node->advertise<std_msgs::Bool>("/stop_move", 1);
    SafetyStop_publisher=Node->advertise<std_msgs::Bool>("/Safety_stop", 1);
    rbRunCommand_client = Node->serviceClient<rb_msgAndSrv::rb_DoubleBool>("/Rb_runCommand");
    rbSetEnable1_client = Node->serviceClient<rb_msgAndSrv::SetEnableSrv>("/UR51/set_robot_enable");
    rbSetEnable2_client = Node->serviceClient<rb_msgAndSrv::SetEnableSrv>("/UR52/set_robot_enable");
    rbErrStatus_client = Node->serviceClient<rb_msgAndSrv::robotError>("/Rb_errStatus");
    camera_subscriber=Node->subscribe<sensor_msgs::Image>("/usb_cam/image_raw",1,boost::bind(&MainWindow::callback_camera_subscriber, this, _1));
    rbGrepSetCommand_client = Node->serviceClient<rb_msgAndSrv::rb_ArrayAndBool>("/Rb_grepSetCommand");
    MagicStepRunCommand_client = Node->serviceClient<rb_msgAndSrv::rb_ArrayAndBool>("/MagicStepRunCommand");

    qRegisterMetaType<infoLevel>("infoLevel");//信号与槽连接自定义类型需要注册
    //线程句柄初始化
    //给设备连接按钮事件开辟子线程
    thread_forRbConn = new qthreadForRos();
    thread_forRbConn->setParm(this,&MainWindow::thread_rbConnCommand);
    //启动前自检
    thread_forSysCheck = new qthreadForRos();
    thread_forSysCheck->setParm(this,&MainWindow::thread_SysCheck);
    //给启动rviz开辟子线程
    thread_forRviz = new qthreadForRos();
    thread_forRviz->setParm(this,&MainWindow::thread_rbRvizCommand);
    //给关闭rviz开辟子线程
    thread_forCloseRviz = new qthreadForRos();
    thread_forCloseRviz->setParm(this,&MainWindow::thread_rbCloseRvizCommand);
    //开始运行子线程
    thread_forBeginRun= new qthreadForRos();
    thread_forBeginRun->setParm(this,&MainWindow::thread_BeginRun);
    //分步解魔方子线程
    thread_MagicStepRun= new qthreadForRos();
    //机器人抓取子线程
    thread_forRbGrepSet= new qthreadForRos();
    thread_forRbGrepSet->setParm(this,&MainWindow::thread_RbGrepSet);
    //获取工程文件路径
    photoPath= QDir::currentPath() +QString("/src/HsDualAppBridge/rb_ui/photo/");
    logPath= QDir::currentPath();
}

void MainWindow::signalAndSlot() {
/*********************************按钮与槽函数绑定*************************************************/
    //设备连接
    connect(btn_rbConn,&QPushButton::clicked,this,&MainWindow::dev_connect);
    //rviz启动
    connect(btn_rvizRun,&QPushButton::clicked,this,&MainWindow::rviz_statup);
    //运行启动
    connect(btn_beginRun,&QPushButton::clicked,this,&MainWindow::run_statup);
    //运行停止
    connect(btn_normalStop,&QPushButton::clicked,this,&MainWindow::run_stop);
    //系统复位
    connect(btn_SysReset,&QPushButton::clicked,this,&MainWindow::SysReset);
    //采集魔方数据
    connect(btn_magicGetdata,&QPushButton::clicked,this,&MainWindow::magicCube_get);
    //解算魔方数据
    connect(btn_magicSolve,&QPushButton::clicked,this,&MainWindow::magicCube_solve);
    //执行解算魔方
    connect(btn_magicRunSolve,&QPushButton::clicked,this,&MainWindow::magicCube_execute);
    //一键解算魔方
    connect(btn_magicAutoSolve,&QPushButton::clicked,this,&MainWindow::magicCube_AutoRun);
    //更新模仿识别错误数据
    connect(btn_updateData,&QPushButton::clicked,this,&MainWindow::magicUpdateData);
    //机器人抓取
    connect(btn_rbGrep,&QPushButton::clicked,this,&MainWindow::robot_grab);
    //导出日志
    connect(btn_oputRecord,&QPushButton::clicked,this,&MainWindow::oputRecord);
    //清除日志
    connect(btn_clearRecord,&QPushButton::clicked,this,&MainWindow::clearRecord);
    //系统停止
    connect(btn_SatetyStop,&QPushButton::clicked,this,&MainWindow::safety_sysStop);
    //机器人1复位
    connect(btn_SatetyRb1Reset,&QPushButton::clicked,this,&MainWindow::safety_rob1Stop);
    //机器人2复位
    connect(btn_SatetyRb2Reset,&QPushButton::clicked,this,&MainWindow::safety_rob2Stop);
    //下拉框触发事件
    connect(comboBox_setRunMode,SIGNAL(currentIndexChanged(const QString)), this, SLOT(slot_cBox_setRunMode(const QString)));
    //切换页面触发事件
    connect(tabWidget,SIGNAL(tabBarClicked(int)), this, SLOT(slot_tabWidgetClicked(int)));
    //切换左右机器人时间
    connect(comboBox_3,SIGNAL(currentIndexChanged(int)), this, SLOT(slot_combox3_Clicked(int)));
    //单步调试页面信号与槽连接
    connect(btn_rb1SetEnable,&QPushButton::clicked,this,&MainWindow::slot_btn_rb1SetEnable);
    connect(btn_rb2SetEnable,&QPushButton::clicked,this,&MainWindow::slot_btn_rb2SetEnable);
    connect(btn_rb1Reset,&QPushButton::clicked,this,&MainWindow::slot_btn_rb1Reset);
    connect(btn_rb2Reset,&QPushButton::clicked,this,&MainWindow::slot_btn_rb2Reset);
    connect(gripper1,&QPushButton::clicked,this,&MainWindow::slot_gripper1);
    connect(gripper2,&QPushButton::clicked,this,&MainWindow::slot_gripper2);
    connect(btn_rb1putBack,&QPushButton::clicked,this,&MainWindow::slot_rb1putBack);
    connect(btn_rb2putBack,&QPushButton::clicked,this,&MainWindow::slot_rb2putBack);
    connect(btn_ResetGrepFun,&QPushButton::clicked,this,&MainWindow::slot_ResetGrepFun);

    //定时器启动
    connect(updateTimer, &QTimer::timeout, this, &MainWindow::timer_onUpdate);
    updateTimer->start();
    connect(updateTimer_com, &QTimer::timeout, this, &MainWindow::timer_comUpdate);
    updateTimer_com->start();
    connect(updateTimer_rob1status, &QTimer::timeout, this, &MainWindow::timer_robot1Status);
    connect(updateTimer_rob2status, &QTimer::timeout, this, &MainWindow::timer_robot2Status);
    connect(updateTimer_LeftCamera, &QTimer::timeout, this, &MainWindow::timer_LeftCamera);
    connect(updateTimer_RightCamera, &QTimer::timeout, this, &MainWindow::timer_RightCamera);
/****************************************************************************************************/

/*********************************自定义信号与槽函数绑定*************************************************/
    connect(this, &MainWindow::emitTextControl,this, &MainWindow::displayTextControl);
    connect(this, &MainWindow::emitLightColor,this, &MainWindow::showLightColor);
    connect(thread_forRbConn, SIGNAL(signal_SendMsgBox(infoLevel ,QString)), this,SLOT(showQmessageBox(infoLevel,QString)));  //将自定义槽连接到自定义信号
    connect(thread_forBeginRun, SIGNAL(signal_SendMsgBox(infoLevel ,QString)), this,SLOT(showQmessageBox(infoLevel,QString)));  //将自定义槽连接到自定义信号
    connect(thread_forRbGrepSet, SIGNAL(signal_SendMsgBox(infoLevel ,QString)), this,SLOT(showQmessageBox(infoLevel,QString)));  //将自定义槽连接到自定义信号
    connect(thread_MagicStepRun, SIGNAL(signal_SendMsgBox(infoLevel ,QString)), this,SLOT(showQmessageBox(infoLevel,QString)));  //将自定义槽连接到自定义信号
    connect(this, SIGNAL(emitQmessageBox(infoLevel ,QString)), this,SLOT(showQmessageBox(infoLevel,QString)));  //将自定义槽连接到自定义信号
    connect(this, SIGNAL(emitStartTimer(QTimer*)), this,SLOT(runTimer(QTimer*)));  //将自定义槽连接到自定义信号
/****************************************************************************************************/
}

void MainWindow::timer_comUpdate() {
    // 1s更新一次标签状态
    //更新机器人状态
    if(connFlag_LeftRobot){
        emit emitLightColor(label_rb1CoonStatus,"green");
    } else{
        emit emitLightColor(label_rb1CoonStatus,"red");
    }
    if(connFlag_RightRobot){
        emit emitLightColor(label_rb2CoonStatus,"green");
    } else{
        emit emitLightColor(label_rb2CoonStatus,"red");
    }
    if(errFlag_LeftRobot||(!connFlag_LeftRobot)){
        emit emitLightColor(label_rb1ErrStatus,"red");
    } else{
        emit emitLightColor(label_rb1ErrStatus,"green");
    }
    if(errFlag_RightRobot||(!connFlag_RightRobot)){
        emit emitLightColor(label_rb2ErrStatus,"red");
    } else{
        emit emitLightColor(label_rb2ErrStatus,"green");
    }
    if(enableFlag_LeftRobot){
        emit emitLightColor(label_rob1EnableStatus,"green");
    } else{
        emit emitLightColor(label_rob1EnableStatus,"red");
    }
    if(enableFlag_RightRobot){
        emit emitLightColor(label_rob2EnableStatus,"green");
    } else{
        emit emitLightColor(label_rob2EnableStatus,"red");
    }
    //1.更新相机连接状态
    if(connFlag_LeftCamera){
        emit emitLightColor(label_LeftCameraConnStatus,"green");
    } else{
        emit emitLightColor(label_LeftCameraConnStatus,"red");
    }
    if(connFlag_RightCamera){
        emit emitLightColor(label_RightCameraConnStatus,"green");
    } else{
        emit emitLightColor(label_RightCameraConnStatus,"red");
    }
    //2.更新状态栏目信息
    Node->getParam("isRuning_solveMagic",isRunning_solveMagic);
    Node->getParam("isRuning_grab",isRunning_grab);
    Node->getParam("is_publish_d435i_calibration_dual",isRunning_d435i);
    showMagicStepLable->setText(QString("魔方完成第%1步").arg(index_magicStep));
    if(isRunning_solveMagic){
        isRunning_solveMagic_Lable->setText("魔方解算工作中");
    } else{
        isRunning_solveMagic_Lable->setText("魔方停止");
    }
    if(isRunning_grab){
        isRunning_grab_Lable->setText("机器人抓盒子工作中");
    } else{
        isRunning_grab_Lable->setText("机器人抓盒子停止");
    }

    QDateTime time = QDateTime::currentDateTime();
    QString str = time.toString("yyyy-MM-dd hh:mm:ss dddd");
    showtime_Lable->setText(str);

    //当不在工作中时跳过检测
    if(~(isRunning_solveMagic||isRunning_grab)){
        return;
    }
    //状态监控下降沿报警提醒
    if(holdOnFlag_LeftRobotConn){
        if(!connFlag_LeftRobot){
            emit emitQmessageBox(infoLevel::warning ,QString("左机器人连接中断,请重新连接"));
            LOG("ERRINFO")->logWarnMessage("左机器人连接中断");
            holdOnFlag_LeftRobotConn= false;
        }
    }
    if(holdOnFlag_RightRobotConn){
        if(!connFlag_RightRobot){
            emit emitQmessageBox(infoLevel::warning ,QString("右机器人连接中断,请重新连接"));
            LOG("ERRINFO")->logWarnMessage("右机器人连接中断");
            holdOnFlag_RightRobotConn= false;
        }
    }
    if(holdOnFlag_LeftRobotErr){
        if(errFlag_LeftRobot){
            emit emitQmessageBox(infoLevel::warning ,QString("左机器人报警"));
            LOG("ERRINFO")->logWarnMessage("左机器人报警");
            holdOnFlag_LeftRobotErr= false;
        }
    }
    if(holdOnFlag_RightRobotErr){
        if(errFlag_RightRobot){
            emit emitQmessageBox(infoLevel::warning ,QString("右机器人报警"));
            LOG("ERRINFO")->logWarnMessage("右机器人报警");
            holdOnFlag_RightRobotErr= false;
        }
    }
    if(holdOnFlag_LeftRobotEnable){
        if(!enableFlag_LeftRobot){
            emit emitQmessageBox(infoLevel::warning ,QString("左机器人下使能"));
            LOG("ERRINFO")->logWarnMessage("左机器人下使能,请重新连接");
            holdOnFlag_LeftRobotEnable= false;
        }
    }
    if(holdOnFlag_RightRobotEnable){
        if(!enableFlag_RightRobot){
            emit emitQmessageBox(infoLevel::warning ,QString("右机器人下使能"));
            LOG("ERRINFO")->logWarnMessage("右机器人下使能,请重新连接");
            holdOnFlag_RightRobotEnable= false;
        }
    }
    if(holdOnFlag_LeftCamera){
        if(!connFlag_LeftCamera){
            emit emitQmessageBox(infoLevel::warning ,QString("左相机断开连接,请重新连接"));
            LOG("ERRINFO")->logWarnMessage("左相机断开连接");
            holdOnFlag_LeftCamera= false;
        }
    }
    if(holdOnFlag_RightCamera){
        if(!connFlag_RightCamera){
            emit emitQmessageBox(infoLevel::warning ,QString("右相机断开连接,请重新连接"));
            LOG("ERRINFO")->logWarnMessage("右相机断开连接");
            holdOnFlag_RightCamera= false;
        }
    }
}

//定时器回调函数，实时更新状态信息
void MainWindow::timer_onUpdate() {

    if(flag_delCbox){
    if(cbox!= nullptr){
        delete cbox;
        cbox= nullptr;
        flag_delCbox= false;
    }
}
}

void MainWindow::timer_robot1Status() {
    connFlag_LeftRobot= false;
}
void MainWindow::timer_robot2Status() {
    connFlag_RightRobot= false;
}
//定时器1,定时查看连接状态情况
void MainWindow::timer_LeftCamera() {
    connFlag_LeftCamera= false;
}
void MainWindow::timer_RightCamera() {
    connFlag_RightCamera= false;
}

//设备连接按钮-1
void MainWindow::dev_connect() {
    if(cbox==nullptr){
        cbox=new CMsgBox();
        connect(this, SIGNAL(emitSwapDataWithCMsgBox(int*)), cbox,SLOT(slot_SwapDataWithMainwin(int*)),Qt::DirectConnection);  //将自定义槽连接到自定义信号
    }
    thread_forRbConn->start();//运行子线程代码:设备连接按钮中开辟的子线程程序-2
}
//设备连接按钮中开辟的子线程程序-2
void MainWindow::thread_rbConnCommand() {
    cout<<"进入rbConn子线程"<<endl;
    switch (comboBox_setRunMode->currentIndex()){
        case 2:
            cbox->show();
            thread_forSysCheck->start();
           system("rosrun rb_ui decConnect.sh");
            break;
        case 3:
            cbox->show();
            thread_forSysCheck->start();
            system("rosrun rb_ui decConnect.sh");
            break;
    }

}

//启动前自检子线程
void MainWindow::thread_SysCheck() {
    //自检顺序:
    //1.co605_dual_arm_real.launch 机器人连接
    //2.gripper_bridge_dual.launch 夹爪桥连接
    //3.rs_camera_right.launch     右边相机连接
    //4.s_camera.launch             左边相机连接
    //5.publish_d435i_calibration_dual.launch  发布标定参数
    //6.vision_bridge_yolo6d_dual            视觉桥
    while ((!flag_syscheckOk)&&(!flag_sysckCancel)) {
        sleep(1);
        checkArray[0]=connFlag_RightRobot?1:0;
        checkArray[1]=1;
        checkArray[2]=connFlag_LeftCamera?1:0;
        checkArray[3]=connFlag_RightCamera?1:0;
        checkArray[4]=isRunning_d435i?1:0;
        checkArray[5]=1;
        cout<<"发送"<<endl;
        emit emitSwapDataWithCMsgBox(checkArray);
    }
    flag_syscheckOk= false;
    flag_sysckCancel= false;
}


//启动rviz－－－１
void MainWindow::rviz_statup() {
    int index=comboBox_setRunMode->currentIndex();
    //如果不是rviz仿真模式，则返回
    if(index!=1){
        return;
    }
    flag_RvizRun=!flag_RvizRun;
    if(flag_RvizRun){
        //启动rviz
        thread_forCloseRviz->quit();
        thread_forCloseRviz->wait();
        thread_forRviz->start();
        btn_rvizRun->setText( "关闭rviz");
    } else{
        //关闭rviz
        thread_forRviz->terminate();
        thread_forRviz->wait();
        thread_forCloseRviz->start();
        btn_rvizRun->setText( "启动Rviz");
    }
}

//进入子线程－－－２
void MainWindow::thread_rbRvizCommand() {
    system("roslaunch co605_dual_arm_gripper_moveit_config demo.launch");
}

void MainWindow::thread_rbCloseRvizCommand() {
    system("rosrun rb_ui killRviz.sh");
}

//运行启动按钮-1
void MainWindow::run_statup() {
    LOG("RUNINFO")->logInfoMessage("开始运行启动!");
    pProgressBar->setValue(0);  // 当前进度
    pProgressBar->setFormat(QString("当前解魔方进度为：0/0"));
    thread_forBeginRun->start();//转到运行启动按钮开启的子线程-2
}

//运行启动按钮开启的子线程-2
void MainWindow::thread_BeginRun() {
    int index=comboBox_setRunMode->currentIndex();
    if(index==0){
        return;
    }
    if(index==1){
        system("rosrun rb_ui RvizAndTestPoint.sh");
        return;
    }
    //左右夹具设置服务
    hirop_msgs::SetGripper data_setgripper;
    data_setgripper.request.gripperName="SerialGripper";
    if(LeftGripperSet_client.call(data_setgripper))
    {
        if(data_setgripper.response.isSucceeful)
        {
            LOG("RUNINFO")->logInfoMessage("左夹具设置成功!");
        }
        else
        {
            LOG("ERRINFO")->logWarnMessage("左夹具设置失败!");
        }
    }
    else
    {
        emit emitQmessageBox(infoLevel::warning,QString("左夹具设置服务连接失败"));
        LOG("ERRINFO")->logWarnMessage("左夹具设置服务连接失败!");
    }

    if(RightGripperSet_client.call(data_setgripper))
    {
        if(data_setgripper.response.isSucceeful)
        {
            LOG("RUNINFO")->logWarnMessage("右夹具设置成功!");
        }
        else
        {
            LOG("ERRINFO")->logWarnMessage("右夹具设置失败!");
        }
    }
    else
    {
        emit emitQmessageBox(infoLevel::warning,QString("右夹具设置服务连接失败"));
        LOG("ERRINFO")->logWarnMessage("右夹具设置服务连接失败!");
    }
    //左右夹具连接服务
    hirop_msgs::connectGripper data_conngripper;
    if(LeftGripperConn_client.call(data_conngripper))
    {
        if(data_conngripper.response.isConnected)
        {
            LOG("RUNINFO")->logWarnMessage("左夹具连接成功!");
            connFlag_LeftGripper= true;
        }
        else
        {
            LOG("ERRINFO")->logWarnMessage("左夹具连接失败!");
        }
    }
    else
    {
        emit emitQmessageBox(infoLevel::warning,QString("左夹具连接服务连接失败"));
        LOG("ERRINFO")->logWarnMessage("左夹具连接服务连接失败!");
    }

    if(RightGripperConn_client.call(data_conngripper))
    {
        if(data_conngripper.response.isConnected)
        {
            LOG("RUNINFO")->logWarnMessage("右夹具连接成功!");
            connFlag_RightGripper= true;
        }
        else
        {
            LOG("ERRINFO")->logWarnMessage("右夹具连接失败!");

        }
    }
    else
    {
        emit emitQmessageBox(infoLevel::warning,QString("右夹具连接服务连接失败"));
        LOG("ERRINFO")->logWarnMessage("右夹具连接服务连接失败!");
    }

    //左右机器人复位报警服务
    hsr_rosi_device::ClearFaultSrv data_robotClear;
    if(LeftRobReset_client.call(data_robotClear))
    {
        if(data_robotClear.response.finsh)
        {
            LOG("RUNINFO")->logWarnMessage("左机器人清除错误成功!");
        }
        else
        {
            LOG("ERRINFO")->logWarnMessage("左机器人清除错误失败!");
        }
    }
    else
    {
        emit emitQmessageBox(infoLevel::warning,QString("左机器人清除错误服务连接失败"));
        LOG("ERRINFO")->logWarnMessage("左机器人清除错误服务连接失败!");
    }

    if(RightRobReset_client.call(data_robotClear))
    {
        if(data_robotClear.response.finsh)
        {
            LOG("RUNINFO")->logWarnMessage("右机器人清除错误成功!");
        }
        else
        {
            LOG("ERRINFO")->logWarnMessage("右机器人清除错误失败!");
        }
    }
    else
    {
        emit emitQmessageBox(infoLevel::warning,QString("右机器人清除错误服务连接失败"));
        LOG("ERRINFO")->logWarnMessage("右机器人清除错误服务连接失败!");

    }
    //左右机器人上使能服务
    hsr_rosi_device::SetEnableSrv data_robotEnable;
    data_robotEnable.request.enable= true;
    if(LeftRobEnable_client.call(data_robotEnable))
    {
        if(data_robotEnable.response.finsh)
        {
            LOG("RUNINFO")->logWarnMessage("左机器人上使能成功!");
        }
        else
        {
            LOG("ERRINFO")->logWarnMessage("左机器人上使能失败!");
        }
    }
    else
    {
        emit emitQmessageBox(infoLevel::warning,QString("左机器人上使能服务连接失败"));
        LOG("ERRINFO")->logWarnMessage("左机器人上使能服务连接失败!");
    }

    if(RightRobEnable_client.call(data_robotEnable))
    {
        if(data_robotEnable.response.finsh)
        {
            LOG("RUNINFO")->logWarnMessage("右机器人上使能成功!");
        }
        else
        {
            LOG("ERRINFO")->logWarnMessage("右机器人上使能失败!");
        }
    }
    else
    {
        emit emitQmessageBox(infoLevel::warning,QString("右机器人上使能服务连接失败"));
        LOG("ERRINFO")->logWarnMessage("右机器人上使能服务连接失败!");
    }
    //运行魔方解算程序和抓取程序
    system("roslaunch rb_ui dualRobotLaunch.launch");

}


//运行停止
void MainWindow::run_stop() {
    cout<<"点击了运行停止按钮"<<endl;
    std_msgs::Bool msg;
    msg.data=true;
    rbStopCommand_publisher.publish(msg);
}

//系统复位
void MainWindow::SysReset() {
    if(thread_forRbConn->isRunning()){
        thread_forRbConn->terminate();
    }
    if(thread_forBeginRun->isRunning()){
        thread_forBeginRun->terminate();
    }
    pProgressBar->setValue(0);  // 当前进度
    pProgressBar->setFormat(QString("当前解魔方进度为：0/0"));
    flag_syscheckOk= false;
    flag_sysckCancel= false;
    system("rosnode kill $(rosnode list | grep -v /robot_UI)");
}

//点击采集魔方数据按钮－－－１
void MainWindow::magicCube_get() {
    cout<<"点击了采集魔方数据按钮"<<endl;
    if(isRunning_grab){
        return;
    }
    LOG("RUNINFO")->logInfoMessage("机器人开始解魔方!");
    thread_MagicStepRun->setParm(this,&MainWindow::thread_GagicGetData);
        thread_MagicStepRun->start();
}
//进入采集魔方数据子线程－－－２
void MainWindow::thread_GagicGetData() {
    rb_msgAndSrv::rb_ArrayAndBool data_srvs;
    data_srvs.request.data.resize(1);
    data_srvs.request.data[0]=1;
    emit emitQmessageBox(infoLevel::information,QString("发送成功"));
    if(MagicStepRunCommand_client.call(data_srvs)){
        cubeParse::Detection srv;
        ImageGet_client.call(srv);
    } else{
        emit emitQmessageBox(infoLevel::warning,QString("MagicStepRunCommand_client连接失败"));
        LOG("ERRINFO")->logWarnMessage("MagicStepRunCommand_client连接失败!");
    }
    index_magicStep=1;
}

//接收魔方图像数据－－－－－－３
void MainWindow::callback_magicGetData_subscriber(rb_msgAndSrv::rbImageList rbimageList) {
    rb_msgAndSrv::rbImageList data_msg;
    for (int i = 0; i < 6; ++i) {
        const cv_bridge::CvImagePtr &ptr = cv_bridge::toCvCopy(rbimageList.imagelist[i], "bgr8");
        cv::Mat mat = ptr->image;
        QImage qimage = cvMat2QImage(mat);
        QPixmap pixmap = QPixmap::fromImage(qimage);
        QPixmap fitpixmap = pixmap.scaled(label_picture1->width(), label_picture1->height(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);  // 饱满填充
        list_label_picture[i]->setPixmap(fitpixmap);
    }
}

//接受魔方解析数据
void MainWindow::callback_magicSolve_subscriber(rb_msgAndSrv::rb_StringArray data_msg) {
    QString sumString ;
    int length=data_msg.data.size();
    if(length!=54){
        emit emitQmessageBox(infoLevel::warning,QString("接收到数据不等于54个!"));
        return;
    }
    char tmpChar[length];
    for (int i = 0; i < length; ++i) {
        char c_str = data_msg.data[i].data.at(0);
        c_str = (c_str>='a'&& c_str<='z') ?(c_str-32):c_str;
        tmpChar[i]=c_str;
    }
    sumString=QString(QLatin1String(tmpChar,length));
    for (int j = 0; j < 6; ++j) {
        line_updataDataList[j]->setText(sumString.mid(j*9,9));
    }

}

//点击解算魔方数据---1
void MainWindow::magicCube_solve() {
        thread_MagicStepRun->setParm(this,&MainWindow::thread_GagicSolve);
        thread_MagicStepRun->start();
}
//进入解魔方子线程-----2
void MainWindow::thread_GagicSolve() {
    rb_msgAndSrv::rb_ArrayAndBool data_srvs;
    data_srvs.request.data.resize(1);
    data_srvs.request.data[0]=2;
    emit emitQmessageBox(infoLevel::information,QString("发送成功"));
    if(MagicStepRunCommand_client.call(data_srvs)){
        if(data_srvs.response.respond){
            index_magicStep=2;
        }
    } else{
        emit emitQmessageBox(infoLevel::warning,QString("MagicStepRunCommand_client连接失败"));
        LOG("ERRINFO")->logWarnMessage("MagicStepRunCommand_client连接失败!");
    }

}
//点击执行解算魔方动作---1
void MainWindow::magicCube_execute() {
        thread_MagicStepRun->setParm(this,&MainWindow::thread_GagicRunSolve);
        thread_MagicStepRun->start();
}
//进入执行解算魔方子线程---２
void MainWindow::thread_GagicRunSolve() {
    rb_msgAndSrv::rb_ArrayAndBool data_srvs;
    data_srvs.request.data.resize(1);
    data_srvs.request.data[0]=3;
    emit emitQmessageBox(infoLevel::information,QString("发送成功"));
    if(MagicStepRunCommand_client.call(data_srvs)){
        if(data_srvs.response.respond){
            index_magicStep=0;
            LOG("RUNINFO")->logWarnMessage("机器人执行解算魔方成功!");
        }
    } else{
        emit emitQmessageBox(infoLevel::warning,QString("MagicStepRunCommand_client连接失败"));
        LOG("ERRINFO")->logWarnMessage("MagicStepRunCommand_client连接失败!");
    }
    
}
//一键解魔方－－－－1
void MainWindow::magicCube_AutoRun() {
    cout<<"点击了一键解算魔方按钮"<<endl;
    //如果机器人运行中则返回
    if(isRunning_grab){
        return;
    }
        thread_MagicStepRun->setParm(this,&MainWindow::thread_AutoSolveMagic);
        thread_MagicStepRun->start();
}

//进入一键解魔方线程－－－－２
void MainWindow::thread_AutoSolveMagic() {
    rb_msgAndSrv::rb_ArrayAndBool data_srvs;
    data_srvs.request.data.resize(1);
    data_srvs.request.data[0]=4;
    emit emitQmessageBox(infoLevel::information,QString("发送成功"));
    if(MagicStepRunCommand_client.call(data_srvs)){
        if(data_srvs.response.respond){
            LOG("RUNINFO")->logWarnMessage("一键解算魔方成功!");
        } else{
            LOG("ERRINFO")->logWarnMessage("一键解算魔方失败!");
        }
    } else{
        emit emitQmessageBox(infoLevel::warning,QString("MagicStepRunCommand_client连接失败"));
        LOG("ERRINFO")->logWarnMessage("MagicStepRunCommand_client连接失败!");
    }
    index_magicStep=0;
}


void MainWindow::robot_grab() {
//如果机器人运行中则返回
    // if(isRunning_solveMagic){
    //     return;
    // }
//机器人没运行，则开始行动
    thread_forRbGrepSet->start();
}

void MainWindow::safety_sysStop() {
    rb_msgAndSrv::SetEnableSrv data_srvs1;
    rb_msgAndSrv::SetEnableSrv data_srvs2;
    data_srvs1.request.enable= false;
    data_srvs2.request.enable= false;
    if((rbSetEnable1_client.call(data_srvs1))&&(rbSetEnable2_client.call(data_srvs2))){
        if(data_srvs1.response.finsh&&data_srvs2.response.finsh){
            LOG("RUNINFO")->logInfoMessage("机器人伺服停止成功!");
        } else{
            LOG("ERRINFO")->logWarnMessage("机器人伺服停止错误!");
            emit emitQmessageBox(infoLevel::warning,QString("机器人伺服停止错误!"));
        }
    } else{
        LOG("ERRINFO")->logErrorMessage("rbSetEnable_client连接失败!");
        emit emitQmessageBox(infoLevel::warning,QString("rbSetEnable_client连接失败!"));
    }
}

void MainWindow::safety_rob1Stop() {
    cout<<"点击了机器人1复位按钮"<<endl;
}

void MainWindow::safety_rob2Stop() {
    cout<<"点击了机器人2复位按钮"<<endl;
}


//pc相机连接
void MainWindow::callback_camera_subscriber(const sensor_msgs::Image::ConstPtr &msg) {

    const cv_bridge::CvImageConstPtr &ptr = cv_bridge::toCvShare(msg, "bgr8");
    cv::Mat mat = ptr->image;
    QImage image = cvMat2QImage(mat);
    QPixmap pixmap1 = QPixmap::fromImage(image);
    QPixmap fitpixmap1 = pixmap1.scaled(label_picture1->width(), label_picture1->height(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);  // 饱满填充
//    QPixmap fitpixmap1 = pixmap1.scaled(label_picture1->width(), label_picture1->height(), Qt::KeepAspectRatio, Qt::SmoothTransformation);  // 按比例缩放
//    label_picture1->setPixmap(fitpixmap1);
}

void MainWindow::oputRecord() {
    QString file_path = QFileDialog::getOpenFileName(this,"选择文件",logPath, "Files(*.log)");
    QString displayString;
    QFile file(file_path);
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        return;
//        qDebug()<<"Can't open the file!"<<endl;
    }
    while(!file.atEnd())
    {
        QByteArray line = file.readLine();
        QString str(line);
        displayString.append(str);
    }
    file.close();
    plainTextEdit->clear();
    plainTextEdit->setPlainText(displayString);
}

void MainWindow::clearRecord() {
    plainTextEdit->clear();
}

void MainWindow::displayTextControl(QString text) {
    plainTextEdit->appendPlainText(text);
}


void MainWindow::thread_RbGrepSet() {
    LOG("RUNINFO")->logInfoMessage("机器人开始抓取运行中!");
    int index1=comboBox->currentIndex();
    int index2=comboBox_2->currentIndex();
    int index3=comboBox_3->currentIndex();

    rb_msgAndSrv::rb_ArrayAndBool data_msg;
    data_msg.request.data.resize(3);
    data_msg.request.data[0]=index1;
    data_msg.request.data[1]=index2;
    data_msg.request.data[2]=index3;
    if(rbGrepSetCommand_client.call(data_msg)){
    if(data_msg.response.respond){
        LOG("RUNINFO")->logErrorMessage("机器人抓取物品成功!");
    }
    } else{
        LOG("ERRINFO")->logErrorMessage("机器人抓取物品失败!");
        emit emitQmessageBox(infoLevel::warning,QString("机器人抓取物品失败,请将机器人回原点,并复位程序,重新启动!"));

    }
    isRunning_solveMagic=false;
}

MainWindow::~MainWindow() {
    system("rosnode kill -a");
}

QImage MainWindow::cvMat2QImage(const cv::Mat &mat) {
    // 8-bits unsigned, NO. OF CHANNELS = 1
    if(mat.type() == CV_8UC1)
    {
        QImage image(mat.cols, mat.rows, QImage::Format_Indexed8);
        // Set the color table (used to translate colour indexes to qRgb values)
//        image.setNumColors(256);
        for(int i = 0; i < 256; i++)
        {
            image.setColor(i, qRgb(i, i, i));
        }
        // Copy input Mat
        uchar *pSrc = mat.data;
        for(int row = 0; row < mat.rows; row ++)
        {
            uchar *pDest = image.scanLine(row);
            memcpy(pDest, pSrc, mat.cols);
            pSrc += mat.step;
        }
        return image;
    }
        // 8-bits unsigned, NO. OF CHANNELS = 3
    else if(mat.type() == CV_8UC3)
    {
        // Copy input Mat
        const uchar *pSrc = (const uchar*)mat.data;
        // Create QImage with same dimensions as input Mat
        QImage image(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
        return image.rgbSwapped();
    }
    else if(mat.type() == CV_8UC4)
    {
        // Copy input Mat
        const uchar *pSrc = (const uchar*)mat.data;
        // Create QImage with same dimensions as input Mat
        QImage image(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_ARGB32);
        return image.copy();
    }
    else
    {
        return QImage();
    }
}

void MainWindow::showQmessageBox(infoLevel level,QString info) {
    switch (level){
        case infoLevel ::information:
                QMessageBox::information(this,"提示",info,QMessageBox::Ok);break;
        case infoLevel ::warning:
                QMessageBox::warning(this,"警告",info,QMessageBox::Ok);break;
    }
}

void MainWindow::initUi(QMainWindow *MainWindow) {
//    if (MainWindow->objectName().isEmpty())
//        MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
//    MainWindow->resize(967, 645);
//    设置背景和背景颜色
//    QImage _image;
//    _image.load("/home/wangneng/catkin_ws/src/HsDualAppBridge/rb_msgs/photo/a.jpg");
//    setAutoFillBackground(true);   // 这个属性一定要设置
//    QPalette pal(palette());
////    pal.setBrush(QPalette::Window, QBrush(_image.scaled(size(), Qt::IgnoreAspectRatio)));
//    pal.setBrush(QPalette::Window, QBrush(_image.scaled(size(), Qt::IgnoreAspectRatio,
//                                                        Qt::SmoothTransformation)));
//    setPalette(pal);
//    设置背景颜色:
//    this->setStyleSheet("background-color:rgb(255,34,198)");
    MainWindow->resize(967, 824);
//    QLabel* qlabel = new QLabel(this);
//    qlabel->setGeometry(0, 0, this->width(), this->height());
//    QPixmap bgImage("a.jpg");
//    qlabel->setStyleSheet("background-color:black");
//    qlabel->setAlignment(Qt::AlignCenter);
//    qlabel->setPixmap(bgImage.scaled(this->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));

    centralWidget = new QWidget(MainWindow);
    centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
    verticalLayout_2 = new QVBoxLayout(centralWidget);
    verticalLayout_2->setSpacing(6);
    verticalLayout_2->setContentsMargins(11, 11, 11, 11);
    verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
    verticalLayout = new QVBoxLayout();
    verticalLayout->setSpacing(6);
    verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
    horizontalLayout = new QHBoxLayout();
    horizontalLayout->setSpacing(6);
    horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
    label_3 = new QLabel(centralWidget);
    label_3->setObjectName(QString::fromUtf8("label_3"));
    label_3->setPixmap(QPixmap(photoPath+"logo.png"));
//    label_3->setPixmap(QPixmap(QString::fromUtf8("./rb_ui/photo/logo.png")));

    horizontalLayout->addWidget(label_3);

    label = new QLabel(centralWidget);
    label->setObjectName(QString::fromUtf8("label"));
    //全局空间属性初始化
    QFont font;
    font.setPointSize(20);
    font.setBold(true);
    font.setItalic(false);
    font.setWeight(75);
    QString tab_qss=
            "QTabBar::tab{width:100}\n"
            "QTabBar::tab{height:40}";
    groupBox_qss=
            "QGroupBox{\n""\n"
            "border-width:2px;\n""\n"
            "border-style:solid;\n""\n"
            "border-radius: 10px;\n""\n"
            "border-color:gray;\n""\n"
            "margin-top:0.5ex;\n""\n""}\n""\n"
            "QGroupBox::title{\n""\n"
            "subcontrol-origin:margin;\n""\n"
            "subcontrol-position:top left;\n""\n"
            "left:10px;\n""\n"
            "margin-left:0px;\n""\n"
            "padding:0 1px;\n""\n""}"
            ;
    label->setFont(font);
    label->setAlignment(Qt::AlignCenter);

    horizontalLayout->addWidget(label);

    horizontalLayout->setStretch(0, 1);
    horizontalLayout->setStretch(1, 6);

    verticalLayout->addLayout(horizontalLayout);

    horizontalLayout_3 = new QHBoxLayout();
    horizontalLayout_3->setSpacing(6);
    horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
    tabWidget = new QTabWidget(centralWidget);
    tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
    tabWidget->setStyleSheet(tab_qss);
    tab = new QWidget();
    tab->setObjectName(QString::fromUtf8("tab"));
    horizontalLayout_4 = new QHBoxLayout(tab);
    horizontalLayout_4->setSpacing(6);
    horizontalLayout_4->setContentsMargins(11, 11, 11, 11);
    horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
    verticalLayout_4 = new QVBoxLayout();
    verticalLayout_4->setSpacing(30);
    verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
    verticalLayout_4->setSizeConstraint(QLayout::SetDefaultConstraint);
//    verticalLayout_4->setContentsMargins(-1, -1, -1, 100);


    horizontalLayout_2 = new QHBoxLayout();
    horizontalLayout_2->setSpacing(6);
    horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
    horizontalLayout_2->setContentsMargins(-1, -1, 0, 50);


    gridLayout = new QGridLayout();
    gridLayout->setSpacing(6);
    gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
    //tab1页面字体
    QFont ft("Microsoft YaHei", 15);
    QPixmap tmp_pixmap=QPixmap(photoPath+"light_red.png");
    QPixmap tmp_pixmap1=QPixmap(photoPath+"light_green.png");
    fitpixmap_redLight = tmp_pixmap.scaled(30,30, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);  // 饱满填充
    fitpixmap_greenLight = tmp_pixmap1.scaled(30,30, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);  // 饱满填充

    label_5 = new QLabel(tab);
    label_5->setObjectName(QString::fromUtf8("label_5"));
    label_5->setFixedSize(160,50);
    label_5->setFont(ft);
    label_5->setAlignment(Qt::AlignVCenter|Qt::AlignRight);
    gridLayout->addWidget(label_5, 1, 0, 1, 1);

    label_rb1CoonStatus = new QLabel(tab);
    label_rb1CoonStatus->setObjectName(QString::fromUtf8("label_rb1CoonStatus"));
    label_rb1CoonStatus->setPixmap(fitpixmap_redLight);
    label_rb1CoonStatus->setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
    gridLayout->addWidget(label_rb1CoonStatus, 1, 1, 1, 1);

    label_6 = new QLabel(tab);
    label_6->setObjectName(QString::fromUtf8("label_6"));
    label_6->setFixedSize(160,50);
    label_6->setFont(ft);
    label_6->setAlignment(Qt::AlignVCenter|Qt::AlignRight);
    gridLayout->addWidget(label_6, 1, 2, 1, 1);
    label_rb2CoonStatus = new QLabel(tab);
    label_rb2CoonStatus->setObjectName(QString::fromUtf8("label_rb2CoonStatus"));
    label_rb2CoonStatus->setPixmap(fitpixmap_redLight);
    label_rb2CoonStatus->setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
    gridLayout->addWidget(label_rb2CoonStatus, 1, 3, 1, 1);

    label_7 = new QLabel(tab);
    label_7->setObjectName(QString::fromUtf8("label_7"));
    label_7->setFixedSize(160,50);
    label_7->setFont(ft);
    label_7->setAlignment(Qt::AlignVCenter|Qt::AlignRight);
    gridLayout->addWidget(label_7, 1, 4, 1, 1);
    label_rb1ErrStatus = new QLabel(tab);
    label_rb1ErrStatus->setObjectName(QString::fromUtf8("label_rb1ErrStatus"));
    label_rb1ErrStatus->setPixmap(fitpixmap_redLight);
    label_rb1ErrStatus->setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
    gridLayout->addWidget(label_rb1ErrStatus, 1, 5, 1, 1);

    label_8 = new QLabel(tab);
    label_8->setObjectName(QString::fromUtf8("label_8"));
    label_8->setFixedSize(160,50);
    label_8->setFont(ft);
    label_8->setAlignment(Qt::AlignVCenter|Qt::AlignRight);
    gridLayout->addWidget(label_8, 1, 6, 1, 1);
    label_rb2ErrStatus = new QLabel(tab);
    label_rb2ErrStatus->setObjectName(QString::fromUtf8("label_rb2ErrStatus"));
    label_rb2ErrStatus->setPixmap(fitpixmap_redLight);
    label_rb2ErrStatus->setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
    gridLayout->addWidget(label_rb2ErrStatus, 1, 7, 1, 1);

    label_123 = new QLabel(tab);
    label_123->setObjectName(QString::fromUtf8("label_123"));
    label_123->setFixedSize(160,50);
    label_123->setFont(ft);
    label_123->setAlignment(Qt::AlignVCenter|Qt::AlignRight);
    label_123->setText(QApplication::translate("MainWindow", "机器人1伺服状态", nullptr));
    gridLayout->addWidget(label_123, 2, 0, 1, 1);

    label_rob1EnableStatus = new QLabel(tab);
    label_rob1EnableStatus->setFixedSize(50,50);
    label_rob1EnableStatus->setObjectName(QString::fromUtf8("label_rob1EnableStatus"));
    label_rob1EnableStatus->setPixmap(fitpixmap_redLight);
    label_rob1EnableStatus->setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
    gridLayout->addWidget(label_rob1EnableStatus, 2, 1, 1, 1);

    label_124 = new QLabel(tab);
    label_124->setObjectName(QString::fromUtf8("label_124"));
    label_124->setFixedSize(160,50);
    label_124->setFont(ft);
    label_124->setAlignment(Qt::AlignVCenter|Qt::AlignRight);
    label_124->setText(QApplication::translate("MainWindow", "机器人2伺服状态", nullptr));
    gridLayout->addWidget(label_124, 2, 2, 1, 1);

    label_rob2EnableStatus = new QLabel(tab);
    label_rob2EnableStatus->setFixedSize(50,50);
    label_rob2EnableStatus->setObjectName(QString::fromUtf8("label_rob2EnableStatus"));
    label_rob2EnableStatus->setPixmap(fitpixmap_redLight);
    label_rob2EnableStatus->setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
    gridLayout->addWidget(label_rob2EnableStatus, 2, 3, 1, 1);

    label_121 = new QLabel(tab);
    label_121->setObjectName(QString::fromUtf8("label_121"));
    label_121->setFixedSize(160,50);
    label_121->setFont(ft);
    label_121->setAlignment(Qt::AlignVCenter|Qt::AlignRight);
    label_121->setText(QApplication::translate("MainWindow", "左相机连接状态", nullptr));
    gridLayout->addWidget(label_121, 2, 4, 1, 1);

    label_LeftCameraConnStatus = new QLabel(tab);
    label_LeftCameraConnStatus->setFixedSize(50,50);
    label_LeftCameraConnStatus->setObjectName(QString::fromUtf8("label_LeftCameraConnStatus"));
    label_LeftCameraConnStatus->setPixmap(fitpixmap_redLight);
    label_LeftCameraConnStatus->setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
    gridLayout->addWidget(label_LeftCameraConnStatus, 2, 5, 1, 1);
    label_122 = new QLabel(tab);
    label_122->setObjectName(QString::fromUtf8("label_122"));
    label_122->setFixedSize(160,50);
    label_122->setFont(ft);
    label_122->setAlignment(Qt::AlignVCenter|Qt::AlignRight);
    label_122->setText(QApplication::translate("MainWindow", "右相机连接状态", nullptr));
    gridLayout->addWidget(label_122, 2, 6, 1, 1);
    label_RightCameraConnStatus = new QLabel(tab);
    label_RightCameraConnStatus->setObjectName(QString::fromUtf8("label_RightCameraConnStatus"));
    label_RightCameraConnStatus->setPixmap(fitpixmap_redLight);
    label_RightCameraConnStatus->setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
    gridLayout->addWidget(label_RightCameraConnStatus, 2, 7, 1, 1);

    horizontalLayout_2->addLayout(gridLayout);

    groupBox_tab1_status = new QGroupBox(tab);
    groupBox_tab1_status->setObjectName(QString::fromUtf8("groupBox_tab1_status"));
    groupBox_tab1_status->setStyleSheet(groupBox_qss);
    groupBox_tab1_status->setTitle(QApplication::translate("MainWindow", "系统状态显示", nullptr));
    groupBox_tab1_status->setLayout(horizontalLayout_2);
//    horizontalLayout_2->addWidget(groupBox_tab1_status);

//    verticalLayout_4->addLayout(horizontalLayout_2);
    verticalLayout_4->addWidget(groupBox_tab1_status);

    horizontalLayout_21 = new QHBoxLayout();
    horizontalLayout_21->setSpacing(6);
    horizontalLayout_21->setObjectName(QString::fromUtf8("horizontalLayout_21"));
    comboBox_setRunMode=new QComboBox();
    comboBox_setRunMode->addItem(QString());
    comboBox_setRunMode->addItem(QString());
    comboBox_setRunMode->addItem(QString());
    comboBox_setRunMode->addItem(QString());
    comboBox_setRunMode->setFixedSize(COMBOX_W,COMBOX_H);
    comboBox_setRunMode->setObjectName(QString::fromUtf8("comboBox_setRunMode"));
    horizontalLayout_21->addWidget(comboBox_setRunMode);

    groupBox_tab1_mod = new QGroupBox();
    groupBox_tab1_mod->setObjectName(QString::fromUtf8("groupBox_tab1_mod"));
    groupBox_tab1_mod->setStyleSheet(groupBox_qss);
    groupBox_tab1_mod->setTitle(QApplication::translate("MainWindow", "运行模式选择", nullptr));
    groupBox_tab1_mod->setLayout(horizontalLayout_21);
//    horizontalLayout_21->addWidget(groupBox_tab1_mod);

    verticalLayout_4->addWidget(groupBox_tab1_mod);
//    verticalLayout_4->addLayout(horizontalLayout_21);

    horizontalLayout_5 = new QHBoxLayout();
    horizontalLayout_5->setSpacing(6);
    horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
    btn_rbConn = new QPushButton(tab);
    btn_rbConn->setObjectName(QString::fromUtf8("btn_rbConn"));
    btn_rbConn->setFixedSize(BTN_W,BTN_H);
    //根据窗口，去设置按钮的位置
    button_style(*btn_rbConn); //设置按钮的样式

    btn_rvizRun = new QPushButton(tab);
    btn_rvizRun->setObjectName(QString::fromUtf8("btn_rvizRun"));
    btn_rvizRun->setFixedSize(BTN_W,BTN_H);


    btn_beginRun = new QPushButton(tab);
    btn_beginRun->setObjectName(QString::fromUtf8("btn_beginRun"));
    btn_beginRun->setFixedSize(BTN_W,BTN_H);


    btn_normalStop = new QPushButton(tab);
    btn_normalStop->setObjectName(QString::fromUtf8("btn_normalStop"));
    btn_normalStop->setFixedSize(BTN_W,BTN_H);


    btn_SysReset= new QPushButton(tab);
    btn_SysReset->setObjectName(QString::fromUtf8("btn_SysReset"));
    btn_SysReset->setFixedSize(BTN_W,BTN_H);


    btn_rbConn->setEnabled(false);
    btn_rvizRun->setEnabled(false);
    btn_beginRun->setEnabled(false);
    btn_normalStop->setEnabled(false);
    btn_SysReset->setEnabled(false);

    groupBox_tab1_func = new QGroupBox();
    groupBox_tab1_func->setObjectName(QString::fromUtf8("groupBox_tab1_func"));
    groupBox_tab1_func->setStyleSheet(groupBox_qss);
    groupBox_tab1_func->setTitle(QApplication::translate("MainWindow", "系统功能", nullptr));

    horizontalLayout_5->addWidget(btn_rbConn);
    horizontalLayout_5->addWidget(btn_rvizRun);
    horizontalLayout_5->addWidget(btn_beginRun);
    horizontalLayout_5->addWidget(btn_normalStop);
    horizontalLayout_5->addWidget(btn_SysReset);

    groupBox_tab1_func->setLayout(horizontalLayout_5);

//    verticalLayout_4->addLayout(horizontalLayout_5);
    verticalLayout_4->addWidget(groupBox_tab1_func);
//    verticalLayout_4->setStretch(0, 1);
//    verticalLayout_4->setStretch(1, 1);

    verticalLayout_4->setStretch(0,2);
    verticalLayout_4->setStretch(1,1);
    verticalLayout_4->setStretch(2,2);
    horizontalLayout_4->addLayout(verticalLayout_4);

    tabWidget->addTab(tab, QString());
    tab_2 = new QWidget();
    tab_2->setObjectName(QString::fromUtf8("tab_2"));
    horizontalLayout_6 = new QHBoxLayout(tab_2);
    horizontalLayout_6->setSpacing(6);
    horizontalLayout_6->setContentsMargins(11, 11, 11, 11);
    horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
    verticalLayout_5 = new QVBoxLayout();
    verticalLayout_5->setSpacing(6);
    verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
    groupBox_tab2_1 = new QGroupBox(tab_2);
    groupBox_tab2_1->setObjectName(QString::fromUtf8("groupBox_tab2_1"));
    groupBox_tab2_1->setStyleSheet(groupBox_qss);

    horizontalLayout_14 = new QHBoxLayout(groupBox_tab2_1);
    horizontalLayout_14->setSpacing(6);
    horizontalLayout_14->setContentsMargins(11, 11, 11, 11);
    horizontalLayout_14->setObjectName(QString::fromUtf8("horizontalLayout_14"));
    btn_rb1SetEnable = new QPushButton(groupBox_tab2_1);
    btn_rb1SetEnable->setObjectName(QString::fromUtf8("btn_rb1SetEnable"));
    btn_rb1SetEnable->setFixedSize(BTN_W,BTN_H);
    horizontalLayout_14->addWidget(btn_rb1SetEnable);

    btn_rb2SetEnable = new QPushButton(groupBox_tab2_1);
    btn_rb2SetEnable->setObjectName(QString::fromUtf8("btn_rb2SetEnable"));
    btn_rb2SetEnable->setFixedSize(BTN_W,BTN_H);

    horizontalLayout_14->addWidget(btn_rb2SetEnable);

    btn_rb1Reset = new QPushButton(groupBox_tab2_1);
    btn_rb1Reset->setObjectName(QString::fromUtf8("btn_rb1Reset"));
    btn_rb1Reset->setFixedSize(BTN_W,BTN_H);

    horizontalLayout_14->addWidget(btn_rb1Reset);

    btn_rb2Reset = new QPushButton(groupBox_tab2_1);
    btn_rb2Reset->setObjectName(QString::fromUtf8("btn_rb2Reset"));
    btn_rb2Reset->setFixedSize(BTN_W,BTN_H);

    horizontalLayout_14->addWidget(btn_rb2Reset);


    verticalLayout_5->addWidget(groupBox_tab2_1);

    groupBox_tab2_2 = new QGroupBox(tab_2);
    groupBox_tab2_2->setObjectName(QString::fromUtf8("groupBox_tab2_2"));
    groupBox_tab2_2->setStyleSheet(groupBox_qss);

    horizontalLayout_19 = new QHBoxLayout(groupBox_tab2_2);
    horizontalLayout_19->setSpacing(6);
    horizontalLayout_19->setContentsMargins(11, 11, 11, 11);
    horizontalLayout_19->setObjectName(QString::fromUtf8("horizontalLayout_19"));
    gripper1 = new QPushButton(groupBox_tab2_2);
    gripper1->setObjectName(QString::fromUtf8("gripper1"));
    gripper1->setFixedSize(BTN_W,BTN_H);
    horizontalLayout_19->addWidget(gripper1);


    gripper2 = new QPushButton(groupBox_tab2_2);
    gripper2->setObjectName(QString::fromUtf8("gripper2"));
    gripper2->setFixedSize(BTN_W,BTN_H);

    horizontalLayout_19->addWidget(gripper2);

    verticalLayout_5->addWidget(groupBox_tab2_2);

    groupBox_tab3_3 = new QGroupBox(tab_2);
    groupBox_tab3_3->setObjectName(QString::fromUtf8("groupBox_tab3_3"));
    groupBox_tab3_3->setStyleSheet(groupBox_qss);
    horizontalLayout_20 = new QHBoxLayout(groupBox_tab3_3);
    horizontalLayout_20->setSpacing(6);
    horizontalLayout_20->setContentsMargins(11, 11, 11, 11);
    horizontalLayout_20->setObjectName(QString::fromUtf8("horizontalLayout_20"));
    btn_rb1putBack = new QPushButton(groupBox_tab3_3);
    btn_rb1putBack->setText("左机器人放回魔方");
    btn_rb1putBack->setObjectName(QString::fromUtf8("btn_rb1putBack"));
    btn_rb1putBack->setFixedSize(BTN_W,BTN_H);
    horizontalLayout_20->addWidget(btn_rb1putBack);

    btn_rb2putBack = new QPushButton(groupBox_tab3_3);
    btn_rb2putBack->setObjectName(QString::fromUtf8("btn_rb2putBack"));
    btn_rb2putBack->setText("右边机器人放回魔方");
    btn_rb2putBack->setFixedSize(BTN_W,BTN_H);
    horizontalLayout_20->addWidget(btn_rb2putBack);

    btn_ResetGrepFun = new QPushButton(groupBox_tab3_3);
    btn_ResetGrepFun->setObjectName(QString::fromUtf8("btn_ResetGrepFun"));
    btn_ResetGrepFun->setText("重置抓取功能");
    btn_ResetGrepFun->setFixedSize(BTN_W,BTN_H);

    horizontalLayout_20->addWidget(btn_ResetGrepFun);




    verticalLayout_5->addWidget(groupBox_tab3_3);


    horizontalLayout_6->addLayout(verticalLayout_5);


    tabWidget->addTab(tab_2, QString());



    tab_3 = new QWidget();
    tab_3->setObjectName(QString::fromUtf8("tab_3"));
    horizontalLayout_8 = new QHBoxLayout(tab_3);
    horizontalLayout_8->setSpacing(6);
    horizontalLayout_8->setContentsMargins(11, 11, 11, 11);
    horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
    horizontalLayout_7 = new QHBoxLayout();
    horizontalLayout_7->setSpacing(6);
    horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
    verticalLayout_6 = new QVBoxLayout();
    verticalLayout_6->setSpacing(6);
    verticalLayout_6->setObjectName(QString::fromUtf8("verticalLayout_6"));
//    tableView = new QTableView(tab_3);
//    tableView->setObjectName(QString::fromUtf8("tableView"));
//
//    verticalLayout_6->addWidget(tableView);

    gridLayout1 = new QGridLayout();
    gridLayout1->setSpacing(6);
    gridLayout1->setObjectName(QString::fromUtf8("gridLayout1"));
    label_picture1=new QLabel(tab_3);
    label_picture2=new QLabel(tab_3);
    label_picture3=new QLabel(tab_3);
    label_picture4=new QLabel(tab_3);
    label_picture5=new QLabel(tab_3);
    label_picture6=new QLabel(tab_3);

    label_picture1->setObjectName(QString::fromUtf8("label_picture1"));
    label_picture2->setObjectName(QString::fromUtf8("label_picture2"));
    label_picture3->setObjectName(QString::fromUtf8("label_picture3"));
    label_picture4->setObjectName(QString::fromUtf8("label_picture4"));
    label_picture5->setObjectName(QString::fromUtf8("label_picture5"));
    label_picture6->setObjectName(QString::fromUtf8("label_picture6"));
    label_picture1->setFixedSize(400,200);
    label_picture2->setFixedSize(400,200);
    label_picture3->setFixedSize(400,200);
    label_picture4->setFixedSize(400,200);
    label_picture5->setFixedSize(400,200);
    label_picture6->setFixedSize(400,200);
    label_picture1->setAlignment(Qt::AlignVCenter|Qt::AlignHCenter);
    label_picture2->setAlignment(Qt::AlignVCenter|Qt::AlignHCenter);
    label_picture3->setAlignment(Qt::AlignVCenter|Qt::AlignHCenter);
    label_picture4->setAlignment(Qt::AlignVCenter|Qt::AlignHCenter);
    label_picture5->setAlignment(Qt::AlignVCenter|Qt::AlignHCenter);
    label_picture6->setAlignment(Qt::AlignVCenter|Qt::AlignHCenter);
    list_label_picture.push_back(label_picture1);
    list_label_picture.push_back(label_picture2);
    list_label_picture.push_back(label_picture3);
    list_label_picture.push_back(label_picture4);
    list_label_picture.push_back(label_picture5);
    list_label_picture.push_back(label_picture6);
    QPixmap tmp_pixmap2=QPixmap(photoPath+"question1.png");
    QPixmap tmp_fixmap2 = tmp_pixmap2.scaled(list_label_picture[0]->width(),list_label_picture[0]->height(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);  // 饱满填充
    for (int i = 0; i < 6; ++i) {
        list_label_picture[i]->setPixmap(tmp_fixmap2);
    }
    gridLayout1->addWidget(label_picture1, 0, 0, 1, 1);
    gridLayout1->addWidget(label_picture2, 0, 1, 1, 1);
    gridLayout1->addWidget(label_picture3, 1, 0, 1, 1);
    gridLayout1->addWidget(label_picture4, 1, 1, 1, 1);
    gridLayout1->addWidget(label_picture5, 2, 0, 1, 1);
    gridLayout1->addWidget(label_picture6, 2, 1, 1, 1);

    horizontalLayout_tab3_1=new QHBoxLayout();
    for (int k = 0; k < 6; ++k) {
        line_updataDataList.push_back(new QLineEdit());
    }
    vector<QString> stringlist{QString("右面颜色序列"),
                               QString("上面颜色序列"),
                               QString("下面颜色序列"),
                               QString("左面颜色序列"),
                               QString("前面颜色序列"),
                               QString("后面颜色序列")
                               };
    for (int h = 0; h < 6; ++h) {
        line_updataDataList[h]->setPlaceholderText(stringlist[h]);
    }

    btn_updateData=new QPushButton(tab_3);
    btn_updateData->setObjectName(QString::fromUtf8("btn_updateData"));
    btn_updateData->setFixedSize(BTN_W,BTN_H);
    btn_updateData->setText("修改魔方采集数据");
//    horizontalLayout_tab3_1->addWidget(line_updataData);
    for (int j = 0; j < 6; ++j) {
        horizontalLayout_tab3_1->addWidget(line_updataDataList[j]);
    }

    horizontalLayout_tab3_1->addWidget(btn_updateData);


    verticalLayout_6->addLayout(gridLayout1);
    verticalLayout_6->addLayout(horizontalLayout_tab3_1);

    horizontalLayout_7->addLayout(verticalLayout_6);

    verticalLayout_8 = new QVBoxLayout();
    verticalLayout_8->setSpacing(6);
    verticalLayout_8->setObjectName(QString::fromUtf8("verticalLayout_8"));
    btn_magicGetdata = new QPushButton(tab_3);
    btn_magicGetdata->setObjectName(QString::fromUtf8("btn_magicGetdata"));
    btn_magicSolve = new QPushButton(tab_3);
    btn_magicSolve->setObjectName(QString::fromUtf8("btn_magicSolve"));
    btn_magicRunSolve = new QPushButton(tab_3);
    btn_magicRunSolve->setObjectName(QString::fromUtf8("btn_magicRunSolve"));
    btn_magicAutoSolve = new QPushButton(tab_3);
    btn_magicAutoSolve->setObjectName(QString::fromUtf8("btn_magicAutoSolve"));
    btn_magicGetdata->setFixedSize(BTN_W,BTN_H);
    btn_magicSolve->setFixedSize(BTN_W,BTN_H);
    btn_magicRunSolve->setFixedSize(BTN_W,BTN_H);
    btn_magicAutoSolve->setFixedSize(BTN_W,BTN_H);
    verticalLayout_8->addWidget(btn_magicGetdata);
    verticalLayout_8->addWidget(btn_magicSolve);
    verticalLayout_8->addWidget(btn_magicRunSolve);
    verticalLayout_8->addWidget(btn_magicAutoSolve);


    horizontalLayout_7->addLayout(verticalLayout_8);

    horizontalLayout_8->addLayout(horizontalLayout_7);

    tabWidget->addTab(tab_3, QString());
    tab_4 = new QWidget();
    tab_4->setObjectName(QString::fromUtf8("tab_4"));
    horizontalLayout_10 = new QHBoxLayout(tab_4);
    horizontalLayout_10->setSpacing(6);
    horizontalLayout_10->setContentsMargins(11, 11, 11, 11);
    horizontalLayout_10->setObjectName(QString::fromUtf8("horizontalLayout_10"));
    horizontalLayout_9 = new QHBoxLayout();
    horizontalLayout_9->setSpacing(6);
    horizontalLayout_9->setObjectName(QString::fromUtf8("horizontalLayout_9"));
    verticalLayout_11 = new QVBoxLayout();
    verticalLayout_11->setSpacing(6);
    verticalLayout_11->setObjectName(QString::fromUtf8("verticalLayout_11"));
    label_processImag = new QLabel(tab_4);
    label_processImag->setObjectName(QString::fromUtf8("label_processImag"));
    label_processImag->setAlignment(Qt::AlignCenter);
    label_processImag->setFixedSize(600,400);
    label_processImag->setText("深度匹配效果图");
    label_preImag = new QLabel(tab_4);
    label_preImag->setObjectName(QString::fromUtf8("label_preImag"));
    label_preImag->setAlignment(Qt::AlignCenter);
    label_preImag->setFixedSize(600,400);
    label_preImag->setText("实时画面展示");
    verticalLayout_11->addWidget(label_preImag);
    verticalLayout_11->addWidget(label_processImag);

    horizontalLayout_9->addLayout(verticalLayout_11);

    verticalLayout_9 = new QVBoxLayout();
    verticalLayout_9->setSpacing(6);
    verticalLayout_9->setObjectName(QString::fromUtf8("verticalLayout_9"));

    groupBox_setMod = new QGroupBox(tab_4);
    groupBox_setMod->setObjectName(QString::fromUtf8("groupBox_setMod"));
    groupBox_setMod->setStyleSheet(groupBox_qss);


    horizontalLayout_11 = new QHBoxLayout(groupBox_setMod);
    horizontalLayout_11->setSpacing(6);
    horizontalLayout_11->setContentsMargins(11, 11, 11, 11);
    horizontalLayout_11->setObjectName(QString::fromUtf8("horizontalLayout_11"));
    comboBox = new QComboBox(groupBox_setMod);
    comboBox->setFixedSize(COMBOX_W,COMBOX_H);

    comboBox->addItem(QString());
    comboBox->addItem(QString());
    comboBox->setObjectName(QString::fromUtf8("comboBox"));

    horizontalLayout_11->addWidget(comboBox);


    verticalLayout_9->addWidget(groupBox_setMod);

    groupBox_selectObject = new QGroupBox(tab_4);
    groupBox_selectObject->setObjectName(QString::fromUtf8("groupBox_selectObject"));
    groupBox_selectObject->setStyleSheet(groupBox_qss);

    horizontalLayout_12 = new QHBoxLayout(groupBox_selectObject);
    horizontalLayout_12->setSpacing(6);
    horizontalLayout_12->setContentsMargins(11, 11, 11, 11);
    horizontalLayout_12->setObjectName(QString::fromUtf8("horizontalLayout_12"));
    comboBox_2 = new QComboBox(groupBox_selectObject);
    comboBox_2->setFixedSize(COMBOX_W,COMBOX_H);

    comboBox_2->addItem(QString());
    comboBox_2->addItem(QString());
    comboBox_2->setObjectName(QString::fromUtf8("comboBox_2"));

    horizontalLayout_12->addWidget(comboBox_2);


    verticalLayout_9->addWidget(groupBox_selectObject);

    groupBox_selectRobot = new QGroupBox(tab_4);
    groupBox_selectRobot->setObjectName(QString::fromUtf8("groupBox_selectRobot"));
    groupBox_selectRobot->setStyleSheet(groupBox_qss);

    horizontalLayout_13 = new QHBoxLayout(groupBox_selectRobot);
    horizontalLayout_13->setSpacing(6);
    horizontalLayout_13->setContentsMargins(11, 11, 11, 11);
    horizontalLayout_13->setObjectName(QString::fromUtf8("horizontalLayout_13"));
    comboBox_3 = new QComboBox(groupBox_selectRobot);
    comboBox_3->setFixedSize(COMBOX_W,COMBOX_H);

    comboBox_3->addItem(QString());
    comboBox_3->addItem(QString());
    comboBox_3->setObjectName(QString::fromUtf8("comboBox_3"));

    horizontalLayout_13->addWidget(comboBox_3);


    verticalLayout_9->addWidget(groupBox_selectRobot);

    btn_rbGrep = new QPushButton(tab_4);
    btn_rbGrep->setObjectName(QString::fromUtf8("btn_rbGrep"));
    btn_rbGrep->setFixedSize(BTN_W,BTN_H);

    verticalLayout_9->addWidget(btn_rbGrep,0,Qt::AlignHCenter);

    horizontalLayout_9->addLayout(verticalLayout_9);
    horizontalLayout_9->setStretch(0,3);
    horizontalLayout_9->setStretch(1,1);

    horizontalLayout_10->addLayout(horizontalLayout_9);

    tabWidget->addTab(tab_4, QString());
    tab_5 = new QWidget();
    tab_5->setObjectName(QString::fromUtf8("tab_5"));
    horizontalLayout_15 = new QHBoxLayout(tab_5);
    horizontalLayout_15->setSpacing(6);
    horizontalLayout_15->setContentsMargins(11, 11, 11, 11);
    horizontalLayout_15->setObjectName(QString::fromUtf8("horizontalLayout_15"));
    horizontalLayout_16 = new QHBoxLayout();
    horizontalLayout_16->setSpacing(6);
    horizontalLayout_16->setObjectName(QString::fromUtf8("horizontalLayout_16"));
    verticalLayout_13 = new QVBoxLayout();
    verticalLayout_13->setSpacing(6);
    verticalLayout_13->setObjectName(QString::fromUtf8("verticalLayout_13"));
    plainTextEdit = new QPlainTextEdit(tab_5);
    plainTextEdit->setObjectName(QString::fromUtf8("plainTextEdit"));

    verticalLayout_13->addWidget(plainTextEdit);


    horizontalLayout_16->addLayout(verticalLayout_13);

    verticalLayout_12 = new QVBoxLayout();
    verticalLayout_12->setSpacing(6);
    verticalLayout_12->setObjectName(QString::fromUtf8("verticalLayout_12"));
    btn_oputRecord = new QPushButton(tab_5);
    btn_oputRecord->setObjectName(QString::fromUtf8("btn_oputRecord"));
    btn_oputRecord->setFixedSize(BTN_W,BTN_H);
    btn_clearRecord = new QPushButton(tab_5);
    btn_clearRecord->setObjectName(QString::fromUtf8("btn_clearRecord"));
    btn_clearRecord->setFixedSize(BTN_W,BTN_H);

    verticalLayout_12->addWidget(btn_oputRecord);
    verticalLayout_12->addWidget(btn_clearRecord);


    horizontalLayout_16->addLayout(verticalLayout_12);


    horizontalLayout_15->addLayout(horizontalLayout_16);

    tabWidget->addTab(tab_5, QString());
    tab_6 = new QWidget();
    tab_6->setObjectName(QString::fromUtf8("tab_6"));
    horizontalLayout_18 = new QHBoxLayout(tab_6);
    horizontalLayout_18->setSpacing(6);
    horizontalLayout_18->setContentsMargins(11, 11, 11, 11);
    horizontalLayout_18->setObjectName(QString::fromUtf8("horizontalLayout_18"));
    horizontalLayout_17 = new QHBoxLayout();
    horizontalLayout_17->setSpacing(6);
    horizontalLayout_17->setObjectName(QString::fromUtf8("horizontalLayout_17"));
    btn_SatetyStop = new QPushButton(tab_6);
    btn_SatetyStop->setObjectName(QString::fromUtf8("btn_SatetyStop"));
    btn_SatetyStop->setFixedSize(BTN_W,BTN_H);

    horizontalLayout_17->addWidget(btn_SatetyStop);

    btn_SatetyRb1Reset = new QPushButton(tab_6);
    btn_SatetyRb1Reset->setObjectName(QString::fromUtf8("btn_SatetyRb1Reset"));
    btn_SatetyRb1Reset->setFixedSize(BTN_W,BTN_H);

    horizontalLayout_17->addWidget(btn_SatetyRb1Reset);

    btn_SatetyRb2Reset = new QPushButton(tab_6);
    btn_SatetyRb2Reset->setObjectName(QString::fromUtf8("btn_SatetyRb2Reset"));
    btn_SatetyRb2Reset->setFixedSize(BTN_W,BTN_H);

    horizontalLayout_17->addWidget(btn_SatetyRb2Reset);


    horizontalLayout_18->addLayout(horizontalLayout_17);

    tabWidget->addTab(tab_6, QString());

    horizontalLayout_3->addWidget(tabWidget);

    verticalLayout->addLayout(horizontalLayout_3);

    verticalLayout->setStretch(0, 1);
    verticalLayout->setStretch(1, 9);

    verticalLayout_2->addLayout(verticalLayout);

    MainWindow->setCentralWidget(centralWidget);
    menuBar = new QMenuBar(MainWindow);
    menuBar->setObjectName(QString::fromUtf8("menuBar"));
//    menuBar->setGeometry(QRect(0, 0, 967, 31));
    MainWindow->setMenuBar(menuBar);
    statusBar = new QStatusBar(MainWindow);
    statusBar->setObjectName(QString::fromUtf8("statusBar"));
    showMagicStepLable =new QLabel();
    isRunning_solveMagic_Lable=new QLabel();
    isRunning_grab_Lable=new QLabel();
    showMagicStepLable->setVisible(false);
    statusBar->addWidget(showMagicStepLable);
    statusBar->addWidget(isRunning_solveMagic_Lable);
    statusBar->addWidget(isRunning_grab_Lable);

    pProgressBar = new QProgressBar(this);
//    pProgressBar->move(100,60);
    pProgressBar->setOrientation(Qt::Horizontal);  // 水平方向
    pProgressBar->setMinimum(0);  // 最小值
    pProgressBar->setMaximum(100);  // 最大值
    pProgressBar->setValue(0);  // 当前进度
    pProgressBar->setFormat(QString("当前解魔方进度为：0/0"));
    pProgressBar->setVisible(false);  // 不可见
    statusBar->addWidget(pProgressBar);

    showtime_Lable=new QLabel();
    statusBar->addWidget(showtime_Lable);

    MainWindow->setStatusBar(statusBar);
    tabWidget->setCurrentIndex(0);
//    QMetaObject::connectSlotsByName(this);
    //ui控件属性设置
    retranslateUi(this);
}

void MainWindow::retranslateUi(QMainWindow *MainWindow) {
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", nullptr));
        label_3->setText(QString());
        label->setText(QApplication::translate("MainWindow", "\345\217\214\346\234\272\345\231\250\344\272\272\344\272\222\345\212\250\344\270\216\345\215\217\344\275\234\345\271\263\345\217\260", nullptr));
        label_5->setText(QApplication::translate("MainWindow", "机器人1连接状态", nullptr));
        label_6->setText(QApplication::translate("MainWindow", "机器人2连接状态", nullptr));
        label_7->setText(QApplication::translate("MainWindow", "机器人1故障状态", nullptr));
        label_8->setText(QApplication::translate("MainWindow", "机器人2故障状态", nullptr));
//        label_picture1->setText(QApplication::translate("MainWindow", "图片", nullptr));
        label_rb1CoonStatus->setText(QString());
        btn_rbConn->setText(QApplication::translate("MainWindow", "\350\256\276\345\244\207\350\277\236\346\216\245", nullptr));
        btn_rvizRun->setText(QApplication::translate("MainWindow", "\345\220\257\345\212\250rviz", nullptr));
        btn_beginRun->setText(QApplication::translate("MainWindow", "\345\274\200\345\247\213\350\277\220\350\241\214", nullptr));
        btn_normalStop->setText(QApplication::translate("MainWindow", "运行停止", nullptr));
        btn_SysReset->setText(QApplication::translate("MainWindow", "系统复位", nullptr));

        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("MainWindow", "\344\270\273\347\225\214\351\235\242", nullptr));
        groupBox_tab2_1->setTitle(QApplication::translate("MainWindow", "\346\234\272\345\231\250\344\272\272\350\260\203\350\257\225", nullptr));
        btn_rb1SetEnable->setText(QApplication::translate("MainWindow", "\345\267\246\346\234\272\345\231\250\344\272\272\344\270\212\344\275\277\350\203\275", nullptr));
        btn_rb2SetEnable->setText(QApplication::translate("MainWindow", "\345\217\263\346\234\272\345\231\250\344\272\272\344\270\212\344\275\277\350\203\275", nullptr));
        btn_rb1Reset->setText(QApplication::translate("MainWindow", "\345\267\246\346\234\272\345\231\250\344\272\272\345\244\215\344\275\215", nullptr));
        btn_rb2Reset->setText(QApplication::translate("MainWindow", "\345\217\263\346\234\272\345\231\250\344\272\272\345\244\215\344\275\215", nullptr));
        groupBox_tab2_2->setTitle(QApplication::translate("MainWindow", "\345\244\271\345\205\267\350\260\203\350\257\225", nullptr));
        gripper1->setText(QApplication::translate("MainWindow", "\345\267\246\345\244\271\345\205\267\345\274\240\345\274\200", nullptr));
        gripper2->setText(QApplication::translate("MainWindow", "\345\217\263\345\244\271\345\205\267\345\274\240\345\274\200", nullptr));
        groupBox_tab3_3->setTitle(QApplication::translate("MainWindow", "\345\205\266\344\273\226\350\260\203\350\257\225", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("MainWindow", "单步调试界面", nullptr));
        btn_magicGetdata->setText(QApplication::translate("MainWindow", "\351\207\207\351\233\206\351\255\224\346\226\271\346\225\260\346\215\256", nullptr));
        btn_magicSolve->setText(QApplication::translate("MainWindow", "\350\247\243\347\256\227", nullptr));
        btn_magicRunSolve->setText(QApplication::translate("MainWindow", "\346\211\247\350\241\214\350\247\243\347\256\227", nullptr));
        btn_magicAutoSolve->setText(QApplication::translate("MainWindow", "\344\270\200\351\224\256\350\247\243\347\256\227", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_3), QApplication::translate("MainWindow", "\351\255\224\346\226\271\347\225\214\351\235\242", nullptr));
        groupBox_setMod->setTitle(QApplication::translate("MainWindow", "\346\250\241\345\274\217\350\256\276\347\275\256", nullptr));
        comboBox->setItemText(0, QApplication::translate("MainWindow", "从货架抓,放桌子上", nullptr));
        comboBox_setRunMode->setItemText(0, QApplication::translate("MainWindow", "请选择运行模式", nullptr));
        comboBox_setRunMode->setItemText(1, QApplication::translate("MainWindow", "RVIZ虚拟点位测试模式", nullptr));
        comboBox_setRunMode->setItemText(2, QApplication::translate("MainWindow", "真机真实点位运行模式", nullptr));
        comboBox_setRunMode->setItemText(3, QApplication::translate("MainWindow", "真机虚拟点位测试模式", nullptr));

        comboBox->setItemText(0, QApplication::translate("MainWindow", "从货架抓,放桌子上", nullptr));
        comboBox->setItemText(1, QApplication::translate("MainWindow", "从桌子抓,放货架上", nullptr));
        groupBox_selectObject->setTitle(QApplication::translate("MainWindow", "\346\212\223\345\217\226\345\257\271\350\261\241", nullptr));
        comboBox_2->setItemText(0, QApplication::translate("MainWindow", "牛奶盒", nullptr));
        comboBox_2->setItemText(1, QApplication::translate("MainWindow", "可乐盒", nullptr));

        groupBox_selectRobot->setTitle(QApplication::translate("MainWindow", "\346\234\254\344\275\223\351\200\211\346\213\251", nullptr));
        comboBox_3->setItemText(0, QApplication::translate("MainWindow", "\345\267\246\346\234\272\345\231\250\344\272\272\346\212\223", nullptr));
        comboBox_3->setItemText(1, QApplication::translate("MainWindow", "\345\217\263\346\234\272\345\231\250\344\272\272\346\212\223", nullptr));

        btn_rbGrep->setText(QApplication::translate("MainWindow", "\346\211\247\350\241\214\346\212\223\345\217\226", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_4), QApplication::translate("MainWindow", "\346\212\223\345\217\226\347\225\214\351\235\242", nullptr));
        btn_oputRecord->setText(QApplication::translate("MainWindow", "\346\227\245\345\277\227\345\257\274\345\207\272", nullptr));
        btn_clearRecord->setText(QApplication::translate("MainWindow", "日志清除", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_5), QApplication::translate("MainWindow", "\346\227\245\345\277\227\347\225\214\351\235\242", nullptr));
        btn_SatetyStop->setText(QApplication::translate("MainWindow", "\347\263\273\347\273\237\346\200\245\345\201\234", nullptr));
        btn_SatetyRb1Reset->setText(QApplication::translate("MainWindow", "机器人1复位", nullptr));
        btn_SatetyRb2Reset->setText(QApplication::translate("MainWindow", "机器人2复位", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_6), QApplication::translate("MainWindow", "\345\256\211\345\205\250\347\225\214\351\235\242", nullptr));
    }
}

void MainWindow::showLightColor(QLabel* label,string color) {
if(color=="red"){
    label->setPixmap(fitpixmap_redLight);
} else if(color=="green"){
    label->setPixmap(fitpixmap_greenLight);
}
}

void MainWindow::callback_LeftCamera_subscriber(sensor_msgs::Image::ConstPtr image) {
    connFlag_LeftCamera= true;
    holdOnFlag_LeftCamera= true;
    emit emitStartTimer(updateTimer_LeftCamera);
    if(comboBox_3->currentIndex()==1){
        return;
    }
    mutex_showImg.lock();
    //显示图片
    const cv_bridge::CvImageConstPtr &ptr = cv_bridge::toCvShare(image, "bgr8");
    cv::Mat mat = ptr->image;
    QImage qimage = cvMat2QImage(mat);
    QPixmap tmp_pixmap = QPixmap::fromImage(qimage);
    QPixmap new_pixmap = tmp_pixmap.scaled(label_preImag->width(), label_preImag->height(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);  // 饱满填充
//    QPixmap tmp_pixmap = pixmap1.scaled(label_picture1->width(), label_picture1->height(), Qt::KeepAspectRatio, Qt::SmoothTransformation);  // 按比例缩放
    label_preImag->setPixmap(new_pixmap);
    mutex_showImg.unlock();
}

void MainWindow::callback_RightCamera_subscriber(const sensor_msgs::Image::ConstPtr image) {
    connFlag_RightCamera= true;
    holdOnFlag_RightCamera=true;
    emit emitStartTimer(updateTimer_RightCamera);
    if(comboBox_3->currentIndex()==0){
        return;
    }
    mutex_showImg.lock();
    //显示图片
    const cv_bridge::CvImageConstPtr &ptr = cv_bridge::toCvShare(image, "bgr8");
    cv::Mat mat = ptr->image;
    QImage qimage = cvMat2QImage(mat);
    QPixmap tmp_pixmap = QPixmap::fromImage(qimage);
    QPixmap new_pixmap = tmp_pixmap.scaled(label_preImag->width(), label_preImag->height(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);  // 饱满填充
//    QPixmap tmp_pixmap = pixmap1.scaled(label_picture1->width(), label_picture1->height(), Qt::KeepAspectRatio, Qt::SmoothTransformation);  // 按比例缩放
    label_preImag->setPixmap(new_pixmap);
    mutex_showImg.unlock();
}

void MainWindow::callback_rob1Status_subscriber(const industrial_msgs::RobotStatus::ConstPtr robot_status) {
    connFlag_LeftRobot= true;
    holdOnFlag_LeftRobotConn= true;
    emit emitStartTimer(updateTimer_rob1status);
    if(robot_status->in_error.val==0){
        errFlag_LeftRobot= false;
        holdOnFlag_LeftRobotErr= true;
    } else{
        errFlag_LeftRobot=true;
    }

    if(robot_status->drives_powered.val==1){
        enableFlag_LeftRobot= true;
        holdOnFlag_LeftRobotEnable= true;
    } else{
        enableFlag_LeftRobot= false;
    }
}

void MainWindow::callback_rob2Status_subscriber(const industrial_msgs::RobotStatus::ConstPtr robot_status) {
    connFlag_RightRobot= true;
    holdOnFlag_RightRobotConn=true;
    emit emitStartTimer(updateTimer_rob2status);
    if(robot_status->in_error.val==0){
        errFlag_RightRobot= false;
        holdOnFlag_RightRobotErr= true;
    } else{
        errFlag_RightRobot=true;
    }

    if(robot_status->drives_powered.val==1){
        enableFlag_RightRobot= true;
        holdOnFlag_RightRobotEnable= true;
    } else{
        enableFlag_RightRobot= false;
    }
}

void MainWindow::runTimer(QTimer* timer) {
    timer->start();
}

void MainWindow::slot_btn_rb1SetEnable() {
    flag_rb1Enable=!flag_rb1Enable;
    if(flag_rb1Enable){
        btn_rb1SetEnable->setText("左机器人下使能");
        system("rosservice call /UR51/set_robot_enable \"enable: true\"");
    } else{
        btn_rb1SetEnable->setText("左机器人上使能");
        system("rosservice call /UR51/set_robot_enable \"enable: false\"");
    }
}
void MainWindow::slot_btn_rb2SetEnable() {
    flag_rb2Enable=!flag_rb2Enable;
    if(flag_rb2Enable){
        btn_rb2SetEnable->setText("右机器人下使能");
        system("rosservice call /UR52/set_robot_enable \"enable: true\"");
    } else{
        btn_rb2SetEnable->setText("右机器人上使能");
        system("rosservice call /UR52/set_robot_enable \"enable: false\"");
    }
}

void MainWindow::slot_btn_rb1Reset() {
    system("rosservice call /UR51/clear_robot_fault \"{}\"");
}

void MainWindow::slot_btn_rb2Reset() {
    system("rosservice call /UR52/clear_robot_fault \"{}\"");
}

void MainWindow::slot_gripper1() {
    flag_gripper1=!flag_gripper1;
    if(flag_gripper1){
        gripper1->setText("左夹具关闭");
        system("rosservice call /UR51/openGripper \"{}\"");
    } else{
        gripper1->setText("左夹具张开");
        system("rosservice call /UR51/closeGripper \"{}\"");
    }
}


void MainWindow::slot_gripper2() {
    flag_gripper2=!flag_gripper2;
    if(flag_gripper2){
        gripper2->setText("右夹具关闭");
        system("rosservice call /UR52/openGripper \"{}\"");
    } else{
        gripper2->setText("右夹具张开");
        system("rosservice call /UR52/closeGripper \"{}\"");
    }
}


void MainWindow::slot_cBox_setRunMode(const QString& text) {
    //设置模式按钮显示更新
    switch (comboBox_setRunMode->currentIndex()){
        case 0:
            btn_rbConn->setEnabled(false);
            btn_rvizRun->setEnabled(false);
            btn_beginRun->setEnabled(false);
            btn_normalStop->setEnabled(false);
            btn_SysReset->setEnabled(false);
            break;
        case 1:
            btn_rbConn->setEnabled(true);
            btn_rvizRun->setEnabled(true);
            btn_beginRun->setEnabled(true);
            btn_normalStop->setEnabled(true);
            btn_SysReset->setEnabled(true);
            btn_rbConn->setVisible(false);
            btn_rvizRun->setVisible(true);
            break;
        case 2:
            btn_rbConn->setEnabled(true);
            btn_rvizRun->setEnabled(true);
            btn_beginRun->setEnabled(true);
            btn_normalStop->setEnabled(true);
            btn_SysReset->setEnabled(true);
            btn_rbConn->setVisible(true);
            btn_rvizRun->setVisible(false);
        case 3:
            btn_rbConn->setEnabled(true);
            btn_rvizRun->setEnabled(true);
            btn_beginRun->setEnabled(true);
            btn_normalStop->setEnabled(true);
            btn_SysReset->setEnabled(true);
            btn_rbConn->setVisible(true);
            btn_rvizRun->setVisible(false);
    }
}

void MainWindow::magicUpdateData() {
    cout<<"按下按钮"<<endl;
    QString newString;
    vector<int> v{1,0,4,2,3,5};
    foreach(int a,v)
    {
        newString+=line_updataDataList[a]->text();
    }
    int num_E=newString.toStdString().size();
    if(num_E==6*9)
    {
        rb_msgAndSrv::rb_string msg_data;
        msg_data.request.data.data=newString.toStdString();
        emit emitQmessageBox(infoLevel::information,QString(newString));
        MagicDataUpdate_client.call(msg_data);
    } else
    {
        emit emitQmessageBox(infoLevel::warning,QString("当前字符数:%1,不是54个字母").arg(num_E));
    }
}

void MainWindow::slot_rb1putBack() {
    cout<<"左边放置魔方"<<endl;
    system("rosservice call /placeMagicCube \"data:\n- 0\"");

}

void MainWindow::slot_rb2putBack() {
    cout<<"右边放置魔方"<<endl;
    system("rosservice call /placeMagicCube \"data:\n- 1\"");
}

//是机器人解魔方进度展示
void MainWindow::callback_ProgressRbSolve_subscriber(std_msgs::Int8MultiArray data_msg) {
    if(pProgressBar== nullptr){
        return;
    }
    float sumStep_rbSolve=static_cast<float >(data_msg.data[0]);
    float curStep_rbSolve=static_cast<float >(data_msg.data[1]);
    float curProcess=(curStep_rbSolve/sumStep_rbSolve)*100;
    pProgressBar->setValue(curProcess);  // 当前进度
    pProgressBar->setFormat(QString("当前解魔方进度为：%1/%2").arg(data_msg.data[1]).arg(data_msg.data[0]));
}

void MainWindow::slot_tabWidgetClicked(int index_tab) {
    if(index_tab!=2){
        pProgressBar->setVisible(false);
        showMagicStepLable->setVisible(false);
    } else{
        pProgressBar->setVisible(true);
        showMagicStepLable->setVisible(true);
    }
}

void MainWindow::callback_MagicSolveSolution_subscriber(std_msgs::Bool data_msg) {
    if(data_msg.data){
        emit emitQmessageBox(infoLevel::information,QString("魔方图像解析成功"));
    } else{
        emit emitQmessageBox(infoLevel::information,QString("魔方图像解析失败"));
    }
}

void MainWindow::slot_combox3_Clicked(int index) {
//    switch (index){
//        case 0:
//            label_leftCamera->setVisible(true);
//            label_preImag->setVisible(false);
//            break;
//        case 1:
//            label_leftCamera->setVisible(false);
//            label_preImag->setVisible(true);
//            break;
//    }
}

void MainWindow::callback_preview1_subscriber(const sensor_msgs::Image::ConstPtr image) {
//    const cv_bridge::CvImageConstPtr &ptr = cv_bridge::toCvShare(image, "bgr8");
//    cv::Mat mat = ptr->image;
//    cv::pyrDown(mat,mat,cv::Size(mat.cols / 2, mat.rows / 2));
//    cv::imshow("UR51_previewImage",mat);
//    cv::waitKey();
//    cv::destroyWindow("UR51_previewImage");
    //显示图片
    const cv_bridge::CvImageConstPtr &ptr = cv_bridge::toCvShare(image, "bgr8");
    cv::Mat mat = ptr->image;
    QImage qimage = cvMat2QImage(mat);
    QPixmap tmp_pixmap = QPixmap::fromImage(qimage);
    QPixmap new_pixmap = tmp_pixmap.scaled(label_processImag->width(), label_processImag->height(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);  // 饱满填充
//    QPixmap tmp_pixmap = pixmap1.scaled(label_picture1->width(), label_picture1->height(), Qt::KeepAspectRatio, Qt::SmoothTransformation);  // 按比例缩放
    label_processImag->setPixmap(new_pixmap);
}

void MainWindow::callback_preview2_subscriber(const sensor_msgs::Image::ConstPtr image) {
//    const cv_bridge::CvImageConstPtr &ptr = cv_bridge::toCvShare(image, "bgr8");
//    cv::Mat mat = ptr->image;
//    cv::pyrDown(mat,mat,cv::Size(mat.cols / 2, mat.rows / 2));
//    cv::imshow("UR52_previewImage",mat);
//    cv::waitKey();
//    cv::destroyWindow("UR52_previewImage");
    //显示图片
    const cv_bridge::CvImageConstPtr &ptr = cv_bridge::toCvShare(image, "bgr8");
    cv::Mat mat = ptr->image;
    QImage qimage = cvMat2QImage(mat);
    QPixmap tmp_pixmap = QPixmap::fromImage(qimage);
    QPixmap new_pixmap = tmp_pixmap.scaled(label_processImag->width(), label_processImag->height(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);  // 饱满填充
//    QPixmap tmp_pixmap = pixmap1.scaled(label_picture1->width(), label_picture1->height(), Qt::KeepAspectRatio, Qt::SmoothTransformation);  // 按比例缩放
    label_processImag->setPixmap(new_pixmap);

}

void MainWindow::button_style(QPushButton& btn) {
    QFile file("/home/wangneng/catkin_ws/src/HsDualAppBridge/rb_ui/config/button.qss"); //通过文件路径创建文件对象
    file.open(QFile::ReadOnly); //文件打开方式
    QString str = file.readAll(); //获取qss中全部字符
    btn.setStyleSheet(str); //设置样式表
}

void MainWindow::slot_ResetGrepFun() {
    if(thread_forRbGrepSet->isRunning()){
        thread_forRbGrepSet->terminate();
        emit emitQmessageBox(infoLevel::information,QString("退出线程成功!"));
    }

}

CMsgBox::CMsgBox(QWidget *parent):QDialog(parent)
{
    init();
}

int CMsgBox::showMsgBox(QWidget *parent)
{
    CMsgBox msgBox(parent);
    return msgBox.exec();
}

void CMsgBox::init()
{
    this->setFixedSize(533,300);
    this->setWindowFlags(Qt::FramelessWindowHint | Qt::Dialog); //隐藏标题栏
    this->setWindowModality(Qt::ApplicationModal); //窗口模态

    QWidget *pWidget = new QWidget(this);
    pWidget->resize(this->size());
    QVBoxLayout *pVLayout=new QVBoxLayout;
    QHBoxLayout *pHLayout1=new QHBoxLayout;
    QHBoxLayout *pHLayout2=new QHBoxLayout;
    QHBoxLayout *pHLayout3=new QHBoxLayout;

    m_lableTitle=new QLabel("系统自检中,请等待完成...");
    QFont font;
    font.setPointSize(16);
    font.setBold(true);
    font.setItalic(false);
    font.setWeight(75);
    m_lableTitle->setFont(font);
    pHLayout1->addWidget(m_lableTitle,0,Qt::AlignHCenter);

    m_plaintext=new QPlainTextEdit();
    pHLayout2->addWidget(m_plaintext);

    pBtn_checkCancel = new QPushButton(this);
    pBtn_checkCancel->setFixedSize(111,46);
    pBtn_checkCancel->setText("取消自检");
    connect(pBtn_checkCancel,&QPushButton::clicked,[=]{
        done(ENM_OK_BTN);
        flag_sysckCancel= true;
        flag_delCbox= true;
    });

    pBtn_checkOk = new QPushButton(this);
    pBtn_checkOk->setFixedSize(111,46);
    pBtn_checkOk->setText("自检通过");
    connect(pBtn_checkOk,&QPushButton::clicked,[=]{
        done(ENM_CANCEL_BTN);
        flag_delCbox= true;
    });
    pBtn_checkOk->setVisible(false);
    pHLayout3->addWidget(pBtn_checkCancel);
    pHLayout3->addWidget(pBtn_checkOk);
    pVLayout->addLayout(pHLayout1);
    pVLayout->addLayout(pHLayout2);
    pVLayout->addLayout(pHLayout3);
    this->setLayout(pVLayout);

    cTimer1 = new QTimer(this);
    cTimer1->setInterval(1000);
    //定时器启动
    QObject::connect(cTimer1, &QTimer::timeout, this, &CMsgBox::slot_timerUpdate);
    cTimer1->start();
}

//定时检测有自检过程无完成
void CMsgBox::slot_timerUpdate() {
    cout<<"定时"<<endl;
    if(!checkOk){
        return;
    }
    m_plaintext->clear();
    //1.co605_dual_arm_real.launch 机器人连接
    //2.gripper_bridge_dual.launch 夹爪桥连接
    //3.rs_camera_right.launch     右边相机连接
    //4.s_camera.launch             左边相机连接
    //5.publish_d435i_calibration_dual.launch  发布标定参数启动
    //6.vision_bridge_yolo6d_dual            视觉桥启动
    QString arrayString[]{"机器人连接:",
                          "夹爪桥连接:",
                          "左边相机连接:",
                          "右边相机连接:",
                          "发布标定参数启动:",
                          "视觉桥启动:"
    };
    int sum=0;
    for (int i = 0; i <6; ++i) {
        if(c_array[i]==1){
            arrayString[i]+="成功";
        } else{
            arrayString[i]+="失败";
        }
        sum+=c_array[i];
        m_plaintext->appendPlainText(arrayString[i]);
    }

    if(sum==6){
        pBtn_checkCancel->setVisible(false);
        pBtn_checkOk->setVisible(true);
        flag_syscheckOk= true;
    }
}

CMsgBox::~CMsgBox(){
    cout<<"析构了"<<endl;
}

void CMsgBox::slot_SwapDataWithMainwin(int* array) {
    checkOk= true;
    c_array=array;
}























