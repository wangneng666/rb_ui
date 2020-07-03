#include "MainWindow.h"

MainWindow::MainWindow(ros::NodeHandle *node, QWidget *parent):BaseWindow(node,parent){
    //系统变量初始化
    SysVarInit();
    //ros话题和服务生成
    initRosTopic();
    //初始化UI
    initUi(this);
    //信号与槽绑定
    signalAndSlot();
}

void MainWindow::SysVarInit() {
    ob_node.setparm(this);
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

    updateTimer_showImage=new QTimer(this);
    updateTimer_showImage->setInterval(50);

    updateTimer_listen_roscore = new QTimer(this);
    updateTimer_listen_roscore->setInterval(1000);

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
    //系统复位子线程
    thread_forSysReset= new qthreadForRos();
    thread_forSysReset->setParm(this,&MainWindow::thread_SysReset);
    //监听故障子线程
    thread_lisionRbErrInfo= new qthreadForRos();
    thread_lisionRbErrInfo->setParm(this,&MainWindow::thread_LisionRbErrInfo);
    //分步解魔方子线程
    thread_MagicStepRun= new qthreadForRos();
    //魔方点位示教子线程
    thread_MagicPoseTeach=new rbQthread();
    //机器人抓取子线程
    thread_forRbGrepSet= new qthreadForRos();
    thread_forRbGrepSet->setParm(this,&MainWindow::thread_RbGrepSet);
    //获取工程文件路径
    photoPath= QDir::currentPath() +QString("/src/HsDualAppBridge/rb_ui/photo/");
    logPath= QDir::currentPath();
    QPixmap tmp_pixmap_red=QPixmap(photoPath+"light_red.png");
    QPixmap tmp_pixmap_green=QPixmap(photoPath+"light_green.png");
    fitpixmap_redLight = tmp_pixmap_red.scaled(30,30, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);  // 饱满填充
    fitpixmap_greenLight = tmp_pixmap_green.scaled(30,30, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);  // 饱满填充

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
    //切换左右机器人事件
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
    connect(btn_rb1_goHomePose,&QPushButton::clicked,this,&MainWindow::slot_btn_rb1_goHomePose);
    connect(btn_rb2_goHomePose,&QPushButton::clicked,this,&MainWindow::slot_btn_rb2_goHomePose);

    //魔方点位校准页面
    connect(btn_tabmp_do,&QPushButton::clicked,this,&MainWindow::slot_btn_tabmp_do);
    connect(btn_tabmp_step,&QPushButton::clicked,this,&MainWindow::slot_btn_tabmp_step);
    connect(btn_tabmp_recordPose,&QPushButton::clicked,this,&MainWindow::slot_btn_tabmp_recordPose);
    connect(btn_tabmp_newteach,&QPushButton::clicked,this,&MainWindow::slot_btn_tabmp_newteach);
    connect(btn_tabmp_resetPose,&QPushButton::clicked,this,&MainWindow::slot_btn_tabmp_resetPose);
    connect(comboBox_tabmp_1,SIGNAL(currentIndexChanged(int)), this, SLOT(slot_comboBox_tabmp_1_Clicked(int)));


    //定时器启动
    connect(updateTimer_listen_roscore, &QTimer::timeout, this, &MainWindow::timer_listen_roscore);
    connect(updateTimer, &QTimer::timeout, this, &MainWindow::timer_onUpdate);
    updateTimer->start();
    connect(updateTimer_com, &QTimer::timeout, this, &MainWindow::timer_comUpdate);
    updateTimer_com->start();
    connect(updateTimer_rob1status, &QTimer::timeout, this, &MainWindow::timer_robot1Status);
    connect(updateTimer_rob2status, &QTimer::timeout, this, &MainWindow::timer_robot2Status);
    connect(updateTimer_LeftCamera, &QTimer::timeout, this, &MainWindow::timer_LeftCamera);
    connect(updateTimer_RightCamera, &QTimer::timeout, this, &MainWindow::timer_RightCamera);
    connect(updateTimer_showImage, &QTimer::timeout, this, &MainWindow::timer_showImage);

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

void MainWindow::initRosTopic(){
    //话题或服务对象初始化
    tmp_publisher= Node->advertise<std_msgs::Int8>("/back_home", 1);
    previewImage1_subscriber=Node->subscribe<sensor_msgs::Image>("/UR51/preview_image",1,boost::bind(&MainWindow::callback_preview1_subscriber,this,_1));
    previewImage2_subscriber=Node->subscribe<sensor_msgs::Image>("/UR52/preview_image",1,boost::bind(&MainWindow::callback_preview2_subscriber,this,_1));
    rob1Status_subscriber=Node->subscribe<industrial_msgs::RobotStatus>("/UR51/robot_status",1,boost::bind(&MainWindow::callback_rob1Status_subscriber,this,_1));
    rob2Status_subscriber=Node->subscribe<industrial_msgs::RobotStatus>("/UR52/robot_status",1,boost::bind(&MainWindow::callback_rob2Status_subscriber,this,_1));
    Leftcamera_subscriber=Node->subscribe<sensor_msgs::Image>("/camera_base/color/image_raw",1,boost::bind(&MainWindow::callback_LeftCamera_subscriber,this,_1));
    Rightcamera_subscriber=Node->subscribe<sensor_msgs::Image>("/camera_base_right/color/image_raw",1,boost::bind(&MainWindow::callback_RightCamera_subscriber,this,_1));
    magicGetData_subscriber=Node->subscribe<rb_msgAndSrv::rbImageList>("/cube_image",1,&MainWindow::callback_magicGetData_subscriber,this);
    MagicSolve_subscriber=Node->subscribe<rb_msgAndSrv::rb_StringArray>("changeColor",1000,&MainWindow::callback_magicSolve_subscriber,this);
    cubeTeachPose_subscriber=Node->subscribe<geometry_msgs::PoseStamped>("cubeTeachPose",1000,&MainWindow::callback_cubeTeachPose_subscriber,this);
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
    camera_subscriber=Node->subscribe<sensor_msgs::Image>("/usb_cam/image_raw",1,boost::bind(&MainWindow::callback_camera_subscriber, this, _1));
    rbGrepSetCommand_client = Node->serviceClient<rb_msgAndSrv::rb_ArrayAndBool>("/Rb_grepSetCommand");
    MagicStepRunCommand_client = Node->serviceClient<rb_msgAndSrv::rb_ArrayAndBool>("/MagicStepRunCommand");
    //魔方点位调试页面
    cubeActionClient  = Node->serviceClient<rb_msgAndSrv::rb_ArrayAndBool>("magic_move_to_point");
    cubeApproachClient  = Node->serviceClient<std_srvs::Empty>("magic_step_move");
    cubeRecordPoseClient = Node->serviceClient<std_srvs::Empty>("magic_recordPose");
    cubeNewTeachClient = Node->serviceClient<std_srvs::Empty>("magic_newTeach");
    cubeResetPoseClient = Node->serviceClient<std_srvs::Empty>("magic_resetPose");
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

    //获取报警信息
    if(connFlag_RightRobot){
        if(!thread_lisionRbErrInfo->isRunning()){
            thread_lisionRbErrInfo->start();
        }
    }

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
    if (cbox == nullptr) {
        cbox = new CMsgBox();
        connect(this, SIGNAL(emitSwapDataWithCMsgBox(int * )), cbox, SLOT(slot_SwapDataWithMainwin(int * )),
                Qt::DirectConnection);  //将自定义槽连接到自定义信号
    }
    if (thread_forRbConn->isRunning()) {
        emit emitQmessageBox(infoLevel::warning, QString("功能程序正在运行中,请不要重复启动!如要再次启动,请先复位程序!"));

    } else {
        thread_forRbConn->start();//运行子线程代码:设备连接按钮中开辟的子线程程序-2
    }
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
    if(thread_forBeginRun->isRunning()){
        emit emitQmessageBox(infoLevel::warning,QString("功能程序正在运行中,请不要重复启动!"));
    } else{
        thread_forBeginRun->start();//转到运行启动按钮开启的子线程-2
    }
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


void MainWindow::safety_rob1Stop() {

    cout<<"点击了机器人1复位按钮"<<endl;
}

void MainWindow::safety_rob2Stop() {
    cout<<"点击了机器人2复位按钮"<<endl;
}
//运行停止
void MainWindow::run_stop() {
    cout<<"点击了运行停止按钮"<<endl;
    hsr_rosi_device::SetEnableSrv srv1;
    hsr_rosi_device::SetEnableSrv srv2;
    srv1.request.enable= false;
    srv2.request.enable= false;
    LeftRobEnable_client.call(srv1);
    RightRobEnable_client.call(srv2);
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
    if(thread_forSysReset->isRunning()){
        thread_forSysReset->terminate();
        sleep(1);
        thread_forSysReset->start();
    } else{
        thread_forSysReset->start();
    }

}


void MainWindow::thread_SysReset() {
    hsr_rosi_device::SetEnableSrv srv1;
    hsr_rosi_device::SetEnableSrv srv2;
    srv1.request.enable= false;
    srv2.request.enable= false;
    LeftRobEnable_client.call(srv1);
    RightRobEnable_client.call(srv2);
    ob_node.shutdownNode();
    sleep(0.5);
    cout<<"杀死节点"<<endl;
//    system("rosnode kill $(rosnode list | grep -v /robot_UI)");
    system("rosnode kill $(rosnode list)");
    sleep(1);
    cout<<"杀死所有关于ros的进程"<<endl;
    system("kill $(ps -ef | grep ros|awk '{print  $2}')");
//    system("killall -9 roscore");
//    system("killall -9 rosmaster");
    sleep(2);
    emit emitStartTimer(updateTimer_listen_roscore);
    cout<<"重启roscore"<<endl;
    system("roscore");
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
     if(isRunning_grab||isRunning_solveMagic){
         emit emitQmessageBox(infoLevel::warning, QString("抓取程序或解魔方正在运行中,请不要重复执行"));
         return;
     }
//机器人没运行，则开始行动
    if (thread_forRbGrepSet->isRunning()) {
        emit emitQmessageBox(infoLevel::warning, QString("抓取程序正在运行中,请不要重复执行"));
    } else {
        thread_forRbGrepSet->start();
    }
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


//pc相机连接
void MainWindow::callback_camera_subscriber(const sensor_msgs::Image::ConstPtr &msg) {
    const cv_bridge::CvImageConstPtr &ptr = cv_bridge::toCvShare(msg, "bgr8");
    cv::Mat mat = ptr->image;
    QImage image = cvMat2QImage(mat);
    QPixmap pixmap1 = QPixmap::fromImage(image);
    QPixmap fitpixmap1 = pixmap1.scaled(label_picture1->width(), label_picture1->height(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);  // 饱满填充
//    QPixmap fitpixmap1 = pixmap1.scaled(label_picture1->width(), label_picture1->height(), Qt::KeepAspectRatio, Qt::SmoothTransformation);  // 按比例缩放
    label_picture1->setPixmap(fitpixmap1);
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
    rbGrepSetCommand_client.call(data_msg);//返回结果无效,不能表明机器人控制模块抓取完成,因此不使用返回值!
//    if(rbGrepSetCommand_client.call(data_msg))
//    {
//        if(data_msg.response.respond)
//        {
//            LOG("RUNINFO")->logErrorMessage("机器人抓取物品成功!");
//        } else
//        {
//            LOG("RUNINFO")->logErrorMessage("机器人抓取物品失败!");
//        }
//    }
}

MainWindow::~MainWindow() {
    system("rosnode kill -a");
    //关闭ros相关进程
    system("kill $(ps -ef | grep ros|awk '{print  $2}')");
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

    const cv_bridge::CvImageConstPtr &ptr = cv_bridge::toCvShare(image, "bgr8");
    cv::Mat mat = ptr->image;
    gl_leftImageMat=mat.clone();
    if(comboBox_3->currentIndex()==1){
        return;
    }
    mutex_showImg.lock();
    //显示图片

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

    const cv_bridge::CvImageConstPtr &ptr = cv_bridge::toCvShare(image, "bgr8");
    cv::Mat mat = ptr->image;
    gl_rightImageMat=mat.clone();
    if(comboBox_3->currentIndex()==0){
        return;
    }
    mutex_showImg.lock();
    //显示图片

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
    if(index_tab!=3){
        pProgressBar->setVisible(false);
        showMagicStepLable->setVisible(false);
    } else{
        pProgressBar->setVisible(true);
        showMagicStepLable->setVisible(true);
    }
    if(index_tab==2){
        updateTimer_showImage->start();
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
    //显示图片
    const cv_bridge::CvImageConstPtr &ptr = cv_bridge::toCvShare(image, "bgr8");
    cv::Mat mat = ptr->image;
    QImage qimage = cvMat2QImage(mat);
    QPixmap tmp_pixmap = QPixmap::fromImage(qimage);
    QPixmap new_pixmap = tmp_pixmap.scaled(label_processImag->width(), label_processImag->height(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);  // 饱满填充
//    QPixmap tmp_pixmap = pixmap1.scaled(label_picture1->width(), label_picture1->height(), Qt::KeepAspectRatio, Qt::SmoothTransformation);  // 按比例缩放
    label_processImag->setPixmap(new_pixmap);

}


void MainWindow::slot_ResetGrepFun() {
    if(thread_forRbGrepSet->isRunning()){
        thread_forRbGrepSet->quit();
        thread_forRbGrepSet->wait();
        if(thread_forRbGrepSet->isRunning()){
        emit emitQmessageBox(infoLevel::information,QString("退出线程成功!"));
        }
    } else{
        emit emitQmessageBox(infoLevel::information,QString("抓取功能线程空闲中!"));
    }
}

void MainWindow::slot_btn_tabmp_do() {
    if(thread_MagicPoseTeach->isRunning()){
        emit emitQmessageBox(infoLevel::information,QString("线程正在工作中,请等待结束"));
        return;
    }
    thread_MagicPoseTeach->setParm2([&]{
//        调用阿忠的服务，名字待定
        rb_msgAndSrv::rb_ArrayAndBool srv;
        srv.request.data.resize(2);
        srv.request.data[0]=calibration_mode;
        srv.request.data[1]=calibration_stepNum;

        if (calibration_mode == 0)
        {
            calibration_stepNum++;
            if (calibration_stepNum == 18)
                calibration_stepNum = 0;
        }

        if (calibration_mode == 1)
        {
            calibration_stepNum++;
            if (calibration_stepNum == 8)
                calibration_stepNum = 0;
        }

        if (cubeActionClient.call(srv))
            ROS_INFO("Mode: %d, StepNum: %d", calibration_mode, calibration_stepNum);
        else
            ROS_INFO("Fialed to call service move_to_point");
    });
    thread_MagicPoseTeach->start();



}

void MainWindow::slot_btn_tabmp_step() {
    if(thread_MagicPoseTeach->isRunning()){
        emit emitQmessageBox(infoLevel::information,QString("线程正在工作中,请等待结束"));
        return;
    }
    thread_MagicPoseTeach->setParm2([&]{
        //调用服务
        std_srvs::Empty srv;
        if (cubeApproachClient.call(srv))
            ROS_INFO("success to call service cubeApproachClient");
        else
            ROS_INFO("Fialed to call service cubeApproachClient");
        flag_showImg=true;
    });
    thread_MagicPoseTeach->start();
}

void MainWindow::slot_btn_tabmp_recordPose() {
    if(thread_MagicPoseTeach->isRunning()){
        emit emitQmessageBox(infoLevel::information,QString("线程正在工作中,请等待结束"));
        return;
    }
    thread_MagicPoseTeach->setParm2([&]{
        //调用服务
        std_srvs::Empty srv;
        if (cubeRecordPoseClient.call(srv))
            ROS_INFO("success to call service cubeRecordPoseClient");
        else
            ROS_INFO("Fialed to call service cubeRecordPoseClient");
    });
    thread_MagicPoseTeach->start();

}

void MainWindow::slot_btn_tabmp_newteach() {
    if(thread_MagicPoseTeach->isRunning()){
        emit emitQmessageBox(infoLevel::information,QString("线程正在工作中,请等待结束"));
        return;
    }
    thread_MagicPoseTeach->setParm2([&]{
        std_srvs::Empty srv;
        if (cubeNewTeachClient.call(srv))
            ROS_INFO("success to call service cubeNewTeachClient");
        else
            ROS_INFO("Fialed to call service cubeNewTeachClient");
//    先调用放下魔方的服务函数;
        // calibration_mode = 0;
        calibration_stepNum = 0;
    });
    thread_MagicPoseTeach->start();


}

void MainWindow::slot_btn_tabmp_resetPose() {
    if(thread_MagicPoseTeach->isRunning()){
        emit emitQmessageBox(infoLevel::information,QString("线程正在工作中,请等待结束"));
        return;
    }
    thread_MagicPoseTeach->setParm2([&]{
        //调用服务
        std_srvs::Empty srv;
        if (cubeResetPoseClient.call(srv))
            ROS_INFO("success to call service cubeResetPoseClient");
        else
            ROS_INFO("Fialed to call service cubeResetPoseClient");
    });
    thread_MagicPoseTeach->start();


}

void MainWindow::label_tabmp_1_showImage() {
    //图片保存的路径
    std::string readpath = "";
    QPixmap new_pixmap;
    if (calibration_mode == 0)
    {
        if(!flag_showImg){
            return;
        }
        flag_showImg=false;
        //读取示教点位的模板效果图片
        // cv::Mat image = cv::imread(readpath + to_string(calibration_stepNum) + ".jpg", 1);
        cv::Mat image = cv::imread("/home/de/catkin_ws/src/HsDualAppBridge/rb_ui/photo/question.jpg", 1);
        QImage qimage = cvMat2QImage(image);
        QPixmap tmp_pixmap = QPixmap::fromImage(qimage);
        new_pixmap = tmp_pixmap.scaled(label_preImag->width(), label_preImag->height(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
    }

    if (calibration_mode == 1)
    {
        //此处假设把订阅到的照片设为全局变量来供其他函数使用
        cv::Point Upper_Left(738, 318);
        cv::Point Bottom_Right(1182, 762);
        cv::Mat image;
        if ( calibration_stepNum < 5)
        {
            image = gl_leftImageMat.clone();
            cv::rectangle(image, Upper_Left, Bottom_Right, cv::Scalar(0, 0, 255), 5, cv::LINE_8, 0);
        }
        else
        {
            image = gl_rightImageMat.clone();
            cv::rectangle(image, Upper_Left, Bottom_Right, cv::Scalar(0, 0, 255), 5, cv::LINE_8, 0);
        }

         QImage qimage = cvMat2QImage(image);
         QPixmap tmp_pixmap = QPixmap::fromImage(qimage);
         new_pixmap = tmp_pixmap.scaled(label_preImag->width(), label_preImag->height(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
    }
    label_tabmp_1->setPixmap(new_pixmap);

}

void MainWindow::slot_btn_rb1_goHomePose() {
    std_msgs::Int8 msg;
    msg.data=0;
    tmp_publisher.publish(msg);
    // sleep(1);
    // tmp_publisher.shutdown();
}

void MainWindow::slot_btn_rb2_goHomePose() {
    std_msgs::Int8 msg;
    msg.data=1;
    tmp_publisher.publish(msg);
    // sleep(1);
    // tmp_publisher.shutdown();
}

void MainWindow::slot_comboBox_tabmp_1_Clicked(int index) {
    calibration_mode=index;
}

void MainWindow::timer_listen_roscore() {
    cout<<"定时中"<<endl;
    sleep(3);
    ob_node.rebootUiNode();
    updateTimer_listen_roscore->stop();
}


void observer_rebootUiNode::rebootUiNode(){
    sp=new ros::AsyncSpinner(1);
    sp->start();
    ros::start();
    mainwindow->initRosTopic();
}

void MainWindow::timer_showImage(){
    label_tabmp_1_showImage();
}

void MainWindow::callback_cubeTeachPose_subscriber(geometry_msgs::PoseStamped data_msg) {
    double a1 = data_msg.pose.position.x;
    double a2 =data_msg.pose.position.y;
    double a3 =data_msg.pose.position.z;
    double o1 = data_msg.pose.orientation.x;
    double o2 = data_msg.pose.orientation.y;
    double o3 = data_msg.pose.orientation.z;
    double o4 = data_msg.pose.orientation.w;
    QString tmp=QString("当前示教点坐标:\n位置:[%1,%2,%3]\n姿态:[%4,%5,%6,%7]").arg(a1).arg(a2).arg(a3).arg(o1).arg(o2).arg(o3).arg(o4);
    textEdit_tabmp_1->setText(tmp);

}

void MainWindow::thread_LisionRbErrInfo() {
    hirop_msgs::robotError srv;
    ros::ServiceClient client = Node->serviceClient<hirop_msgs::robotError>("getRobotErrorFaultMsg");
    client.call(srv);
    uint64_t level=srv.response.errorLevel;
    int errorLevel=level;
    string errorMsg=srv.response.errorMsg;
    string isError=srv.response.isError?"true":"false";
    string dealMsg=srv.response.dealMsg;
    QString tmp=QString("errorLevel:%1\nerrorMsg:%2\nisError:%3\ndealMsg:%4").arg(errorLevel).arg(QString().fromStdString(errorMsg)).arg(QString().fromStdString(isError)).arg(QString().fromStdString(dealMsg));
    if(srv.response.isError){
        emit emitQmessageBox(infoLevel::information,tmp);
    }
}
























