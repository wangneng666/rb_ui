#include "MainWindow.h"

MainWindow::MainWindow(ros::NodeHandle *node, QWidget *parent):QMainWindow(parent),Node(node){
    //系统变量初始化
    SysVarInit();
    //初始化UI
    initUi(this);
    //信号与槽绑定
    signalAndSlot();
}

void MainWindow::SysVarInit() {
    flag_sysRun= true; //设备启动允许标志
    flag_rbConnStatus= false;//机器人连接标志
    flag_rbErrStatus= false;//机器人故障标志
    isRunning_solveMagic= false;
    isRunning_grab= false;
    index_magicStep=0;
    index_RvizCount=0;
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
    updateTimer->setInterval(1);
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
    rob1Status_subscriber=Node->subscribe<industrial_msgs::RobotStatus>("/UR51/robot_status",1000,boost::bind(&MainWindow::callback_rob1Status_subscriber,this,_1));
    rob2Status_subscriber=Node->subscribe<industrial_msgs::RobotStatus>("/UR52/robot_status",1000,boost::bind(&MainWindow::callback_rob2Status_subscriber,this,_1));
    Leftcamera_subscriber=Node->subscribe<sensor_msgs::Image>("/camera_base/color/image_raw",1000,boost::bind(&MainWindow::callback_LeftCamera_subscriber,this,_1));
    Rightcamera_subscriber=Node->subscribe<sensor_msgs::Image>("/camera_base_right/color/image_raw",1000,boost::bind(&MainWindow::callback_RightCamera_subscriber,this,_1));
    magicGetData_subscriber=Node->subscribe<rb_msgAndSrv::rbImageList>("/cube_image",1,&MainWindow::callback_magicGetData_subscriber,this);
    MagicSolve_subscriber=Node->subscribe<std_msgs::UInt8MultiArray>("/cube_solution",1000,&MainWindow::callback_magicSolve_subscriber,this);
    ImageGet_client = Node->serviceClient<cubeParse::Detection>("cube_detect");

    rbStopCommand_publisher= Node->advertise<std_msgs::Bool>("/stop_move", 1);
    SafetyStop_publisher=Node->advertise<std_msgs::Bool>("/Safety_stop", 1);
    rbConnCommand_client = Node->serviceClient<hirop_msgs::robotConn>("getRobotConnStatus");
    rbRunCommand_client = Node->serviceClient<rb_msgAndSrv::rb_DoubleBool>("/Rb_runCommand");
    rbSetEnable1_client = Node->serviceClient<rb_msgAndSrv::SetEnableSrv>("/UR51/set_robot_enable");
    rbSetEnable2_client = Node->serviceClient<rb_msgAndSrv::SetEnableSrv>("/UR52/set_robot_enable");
    rbErrStatus_client = Node->serviceClient<rb_msgAndSrv::robotError>("/Rb_errStatus");
//    camera_subscriber=Node->subscribe<sensor_msgs::Image>("/usb_cam/image_raw",1000,boost::bind(&MainWindow::callback_camera_subscriber, this, _1));
    rbGrepSetCommand_client = Node->serviceClient<rb_msgAndSrv::rb_ArrayAndBool>("/Rb_grepSetCommand");
    MagicStepRunCommand_client = Node->serviceClient<rb_msgAndSrv::rb_ArrayAndBool>("/MagicStepRunCommand");

    qRegisterMetaType<infoLevel>("infoLevel");//信号与槽连接自定义类型需要注册
    //线程句柄初始化
    //给设备连接按钮事件开辟子线程
    thread_forRbConn = new qthreadForRos();
    thread_forRbConn->setParm(this,&MainWindow::thread_rbConnCommand);
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
    //监听机器人故障子线程
    thread_forLisionErrInfo= new qthreadForRos();
    thread_forLisionErrInfo->setParm(this,&MainWindow::thread_LisionErrInfo);
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
    //机器人抓取
    connect(btn_rbGrep,&QPushButton::clicked,this,&MainWindow::robot_grab);
    //导出日志
    connect(btn_oputRecord,&QPushButton::clicked,this,&MainWindow::oputRecord);
    //清除日志
    connect(btn_clearRecord,&QPushButton::clicked,this,&MainWindow::clearRecord);
    //系统停止
    connect(btn_SatetyStop,&QPushButton::clicked,this,&MainWindow::safety_sysStop);
    //机器人1停止
    connect(btn_SatetyRb1Reset,&QPushButton::clicked,this,&MainWindow::safety_rob1Stop);
    //机器人2停止
    connect(btn_SatetyRb2Reset,&QPushButton::clicked,this,&MainWindow::safety_rob2Stop);
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


}

//定时器回调函数，实时更新状态信息
void MainWindow::timer_onUpdate() {

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
        thread_forRbConn->start();//运行子线程代码:设备连接按钮中开辟的子线程程序-2
}
//设备连接按钮中开辟的子线程程序-2
void MainWindow::thread_rbConnCommand() {
    int index=comboBox_setRunMode->currentIndex();
    switch (index){
        case 1:system("rosrun rb_ui decConnect.sh");break;
        case 2:system("rosrun rb_ui decConnect.sh");break;
    }
    updateTimer_RightCamera->start();
}
//启动rviz－－－１
void MainWindow::rviz_statup() {
    int index=comboBox_setRunMode->currentIndex();
    //如果不是rviz仿真模式，则返回
    if(index!=0){
        return;
    }
    index_RvizCount++;
    cout<<index_RvizCount<<endl;
    if(index_RvizCount%2==1){
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
    //启动urdf模型
    system("roslaunch co605_dual_arm_gripper_moveit_config demo.launch ");
}

void MainWindow::thread_rbCloseRvizCommand() {
    system("rosrun rb_ui killRviz.sh");
}

//运行启动按钮-1
void MainWindow::run_statup() {
    updateTimer_rob1status->start();
    return;
    cout<<"点击了运行启动按钮"<<endl;
    thread_forBeginRun->start();//转到运行启动按钮开启的子线程-2
}

//运行启动按钮开启的子线程-2
void MainWindow::thread_BeginRun() {
    int index=comboBox_setRunMode->currentIndex();
    switch (index){
        case 0:system("rosrun rb_ui RvizAndTestPoint.sh");break;
        case 1:system("rosrun rb_ui RealRbAndReadPoint.sh");break;
        case 2:system("rosrun rb_ui RealRbAndTestPoint.sh");break;
    }
}

//监听故障状态子线程-3
void MainWindow::thread_LisionErrInfo() {
    //每隔一秒监听一次报警信息,并在机器人报警状态显示上刷新
    rb_msgAndSrv::robotError errorMsg;
    while (!flag_rbErrStatus){
        sleep(1);
        if(rbErrStatus_client.call(errorMsg)){
        //如果报警
            if(errorMsg.response.isError)
            {
                flag_rbErrStatus= true;
                emit thread_forLisionErrInfo->signal_SendMsgBox(infoLevel::warning,QString("机器人故障!"));
            }
        }
        else
        {
            emit thread_forLisionErrInfo->signal_SendMsgBox(infoLevel::warning,QString("rbErrStatus_client接收消息失败!"));
            return;
        }
    }
}

//运行停止
void MainWindow::run_stop() {
    updateTimer_LeftCamera->start();
    cout<<"点击了运行停止按钮"<<endl;
    std_msgs::Bool msg;
    msg.data=true;
    rbStopCommand_publisher.publish(msg);
}

//系统复位
void MainWindow::SysReset() {
    system("rosnode kill $(rosnode list | grep -v /robot_UI)");
}

//点击采集魔方数据按钮－－－１
void MainWindow::magicCube_get() {
    cout<<"点击了采集魔方数据按钮"<<endl;
    if(isRunning_grab){
        return;
    }
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
        LOG("ROS_NODE")->logWarnMessage("MagicStepRunCommand_client接收消息失败!");
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
void MainWindow::callback_magicSolve_subscriber(std_msgs::UInt8MultiArray data_msg) {
    QString sumString ;
//    for (int i = 0; i < data_msg.data.size(); ++i) {
//        sumString+=QString("%1").arg(data_msg.data[i]);
            ;
//    }
    label_magicSolveData->setText(sumString);
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
        LOG("ROS_NODE")->logWarnMessage("MagicStepRunCommand_client接收消息失败!");
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
            cout<<"执行解算魔方成功"<<endl;
        }
    } else{
        LOG("ROS_NODE")->logWarnMessage("MagicStepRunCommand_client接收消息失败!");
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
            cout<<"一键解算魔方成功"<<endl;
        } else{
            cout<<"一键解算魔方失败"<<endl;
        }
    } else{
        LOG("ROS_NODE")->logWarnMessage("MagicStepRunCommand_client接收消息失败!");
    }
    index_magicStep=0;
}


void MainWindow::robot_grab() {
//如果机器人运行中则返回
    if(isRunning_solveMagic|isRunning_grab){
        return;
    }
//机器人没运行，则开始行动
    cout<<"点击了机器人抓取按钮"<<endl;
    thread_forRbGrepSet->start();

}

void MainWindow::safety_sysStop() {
    rb_msgAndSrv::SetEnableSrv data_srvs1;
    rb_msgAndSrv::SetEnableSrv data_srvs2;
    data_srvs1.request.enable= false;
    data_srvs2.request.enable= false;
    if((rbSetEnable1_client.call(data_srvs1))&&(rbSetEnable2_client.call(data_srvs2))){
        if(data_srvs1.response.finsh&&data_srvs2.response.finsh){
            LOG("ROBOT")->logInfoMessage("机器人伺服停止成功!");
        } else{
            LOG("ROBOT")->logInfoMessage("机器人伺服停止错误!");
            emit emitQmessageBox(infoLevel::warning,QString("机器人伺服停止错误!"));
        }
    } else{
        LOG("ROS_NODE")->logErrorMessage("rbRunCommand_client接收消息失败!");
        emit emitQmessageBox(infoLevel::warning,QString("rbRunCommand_client接收消息失败!"));
        return;
    }
    cout<<"点击了系统停止按钮"<<endl;
}

void MainWindow::safety_rob1Stop() {
    cout<<"点击了机器人1复位按钮"<<endl;
}

void MainWindow::safety_rob2Stop() {
    cout<<"点击了机器人2复位按钮"<<endl;
}

void MainWindow::callback_rbConnStatus_subscriber(std_msgs::UInt8MultiArray data_msg) {
    //两台机器人均连上了才表示连接标志成功
    sleep(1);
    if(data_msg.data[0]==1&&data_msg.data[1]==1){
        flag_rbConnStatus= true;
        cout<<"接收到连接成功数据"<<endl;
    } else{
        flag_rbConnStatus= false;
    }
}

void MainWindow::callback_rbErrStatus_subscriber(std_msgs::UInt16MultiArray data_msg) {
    sleep(1);
    if(data_msg.data[0]==1){
        label_rb1ErrStatus->setPixmap(QPixmap(QString::fromUtf8("/home/wangneng/catkin_ws/src/HsDualAppBridge/rb_msgs/photo/light_red.png")));
        LOG("Robot")->logErrorMessage("机器人1故障");
    }
    if(data_msg.data[1]==1){
        label_rb2ErrStatus->setPixmap(QPixmap(QString::fromUtf8("/home/wangneng/catkin_ws/src/HsDualAppBridge/rb_msgs/photo/light_red.png")));
        LOG("Robot")->logErrorMessage("机器人2故障");
    }
}

void MainWindow::callback_camera_subscriber(const sensor_msgs::Image::ConstPtr &msg) {
    const cv_bridge::CvImageConstPtr &ptr = cv_bridge::toCvShare(msg, "bgr8");
    cv::Mat mat = ptr->image;
    QImage image = cvMat2QImage(mat);
    QPixmap pixmap1 = QPixmap::fromImage(image);
//    QPixmap fitpixmap1 = pixmap1.scaled(label_picture1->width(), label_picture1->height(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);  // 饱满填充
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
        cout<<"抓取成功"<<endl;
    }
    } else{
        LOG("Warning")->logErrorMessage("rbGrepSetCommand_client接收消息失败!");
    }
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
    QFont font;
    font.setPointSize(20);
    font.setBold(true);
    font.setItalic(false);
    font.setWeight(75);
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
//    verticalLayout_4->setStretch(0,1);
//    verticalLayout_4->setStretch(1,1);
    verticalLayout_4->setContentsMargins(-1, -1, -1, 100);
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

    comboBox_setRunMode=new QComboBox();
    comboBox_setRunMode->addItem(QString());
    comboBox_setRunMode->addItem(QString());
    comboBox_setRunMode->addItem(QString());
    comboBox_setRunMode->setFixedHeight(50);
    comboBox_setRunMode->setObjectName(QString::fromUtf8("comboBox_setRunMode"));
    gridLayout->addWidget(comboBox_setRunMode, 3, 0, 1, 2);

    horizontalLayout_2->addLayout(gridLayout);
    verticalLayout_4->addLayout(horizontalLayout_2);

    horizontalLayout_5 = new QHBoxLayout();
    horizontalLayout_5->setSpacing(6);
    horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
    btn_rbConn = new QPushButton(tab);
    btn_rbConn->setObjectName(QString::fromUtf8("btn_rbConn"));

    horizontalLayout_5->addWidget(btn_rbConn);

    btn_rvizRun = new QPushButton(tab);
    btn_rvizRun->setObjectName(QString::fromUtf8("btn_rvizRun"));

    horizontalLayout_5->addWidget(btn_rvizRun);

    btn_beginRun = new QPushButton(tab);
    btn_beginRun->setObjectName(QString::fromUtf8("btn_beginRun"));

    horizontalLayout_5->addWidget(btn_beginRun);

    btn_normalStop = new QPushButton(tab);
    btn_normalStop->setObjectName(QString::fromUtf8("btn_normalStop"));

    horizontalLayout_5->addWidget(btn_normalStop);

    btn_SysReset= new QPushButton(tab);
    btn_SysReset->setObjectName(QString::fromUtf8("btn_SysReset"));
    horizontalLayout_5->addWidget(btn_SysReset);


    verticalLayout_4->addLayout(horizontalLayout_5);

    verticalLayout_4->setStretch(0, 1);
    verticalLayout_4->setStretch(1, 1);

    horizontalLayout_4->addLayout(verticalLayout_4);

    tabWidget->addTab(tab, QString());
    tab_2 = new QWidget();
    tab_2->setObjectName(QString::fromUtf8("tab_2"));
//    QVBoxLayout *verticalLayout_3=new QVBoxLayout(tab_2);
//    QGridLayout* gridLayout3=new QGridLayout();
//    gridLayout3->setSpacing(6);
//    gridLayout3->setObjectName(QString::fromUtf8("gridLayout3"));


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

    label_magicSolveData=new QLabel(tab_3);
    label_magicSolveData->setObjectName(QString::fromUtf8("label_magicSolveData"));
//    label_magicSolveData->setFixedSize(400,200);
    label_magicSolveData->setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
    gridLayout1->addWidget(label_magicSolveData, 3, 0, 1, 2);
    label_magicSolveData->setFont(ft);
    label_magicSolveData->setText(QApplication::translate("MainWindow", "魔方解析数据:", nullptr));
    verticalLayout_6->addLayout(gridLayout1);


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
    btn_magicGetdata->setFixedSize(200,50);
    btn_magicSolve->setFixedSize(200,50);
    btn_magicRunSolve->setFixedSize(200,50);
    btn_magicAutoSolve->setFixedSize(200,50);
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
    label_2 = new QLabel(tab_4);
    label_2->setObjectName(QString::fromUtf8("label_2"));
    label_2->setAlignment(Qt::AlignCenter);

    verticalLayout_11->addWidget(label_2);


    horizontalLayout_9->addLayout(verticalLayout_11);

    verticalLayout_9 = new QVBoxLayout();
    verticalLayout_9->setSpacing(6);
    verticalLayout_9->setObjectName(QString::fromUtf8("verticalLayout_9"));

    groupBox_setMod = new QGroupBox(tab_4);
    groupBox_setMod->setObjectName(QString::fromUtf8("groupBox_setMod"));
    horizontalLayout_11 = new QHBoxLayout(groupBox_setMod);
    horizontalLayout_11->setSpacing(6);
    horizontalLayout_11->setContentsMargins(11, 11, 11, 11);
    horizontalLayout_11->setObjectName(QString::fromUtf8("horizontalLayout_11"));
    comboBox = new QComboBox(groupBox_setMod);
    comboBox->addItem(QString());
    comboBox->addItem(QString());
    comboBox->setObjectName(QString::fromUtf8("comboBox"));

    horizontalLayout_11->addWidget(comboBox);


    verticalLayout_9->addWidget(groupBox_setMod);

    groupBox_selectObject = new QGroupBox(tab_4);
    groupBox_selectObject->setObjectName(QString::fromUtf8("groupBox_selectObject"));
    horizontalLayout_12 = new QHBoxLayout(groupBox_selectObject);
    horizontalLayout_12->setSpacing(6);
    horizontalLayout_12->setContentsMargins(11, 11, 11, 11);
    horizontalLayout_12->setObjectName(QString::fromUtf8("horizontalLayout_12"));
    comboBox_2 = new QComboBox(groupBox_selectObject);
    comboBox_2->addItem(QString());
    comboBox_2->addItem(QString());
    comboBox_2->setObjectName(QString::fromUtf8("comboBox_2"));

    horizontalLayout_12->addWidget(comboBox_2);


    verticalLayout_9->addWidget(groupBox_selectObject);

    groupBox_selectRobot = new QGroupBox(tab_4);
    groupBox_selectRobot->setObjectName(QString::fromUtf8("groupBox_selectRobot"));
    horizontalLayout_13 = new QHBoxLayout(groupBox_selectRobot);
    horizontalLayout_13->setSpacing(6);
    horizontalLayout_13->setContentsMargins(11, 11, 11, 11);
    horizontalLayout_13->setObjectName(QString::fromUtf8("horizontalLayout_13"));
    comboBox_3 = new QComboBox(groupBox_selectRobot);
    comboBox_3->addItem(QString());
    comboBox_3->addItem(QString());
    comboBox_3->setObjectName(QString::fromUtf8("comboBox_3"));

    horizontalLayout_13->addWidget(comboBox_3);


    verticalLayout_9->addWidget(groupBox_selectRobot);

    btn_rbGrep = new QPushButton(tab_4);
    btn_rbGrep->setObjectName(QString::fromUtf8("btn_rbGrep"));

    verticalLayout_9->addWidget(btn_rbGrep);


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

    btn_clearRecord = new QPushButton(tab_5);
    btn_clearRecord->setObjectName(QString::fromUtf8("btn_clearRecord"));
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

    horizontalLayout_17->addWidget(btn_SatetyStop);

    btn_SatetyRb1Reset = new QPushButton(tab_6);
    btn_SatetyRb1Reset->setObjectName(QString::fromUtf8("btn_SatetyRb1Reset"));

    horizontalLayout_17->addWidget(btn_SatetyRb1Reset);

    btn_SatetyRb2Reset = new QPushButton(tab_6);
    btn_SatetyRb2Reset->setObjectName(QString::fromUtf8("btn_SatetyRb2Reset"));

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
    menuBar->setGeometry(QRect(0, 0, 967, 31));
    MainWindow->setMenuBar(menuBar);
    statusBar = new QStatusBar(MainWindow);
    statusBar->setObjectName(QString::fromUtf8("statusBar"));
    showMagicStepLable =new QLabel();
    isRunning_solveMagic_Lable=new QLabel();
    isRunning_grab_Lable=new QLabel();
    statusBar->addWidget(showMagicStepLable);
    statusBar->addWidget(isRunning_solveMagic_Lable);
    statusBar->addWidget(isRunning_grab_Lable);
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
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("MainWindow", "魔方调试界面", nullptr));
        btn_magicGetdata->setText(QApplication::translate("MainWindow", "\351\207\207\351\233\206\351\255\224\346\226\271\346\225\260\346\215\256", nullptr));
        btn_magicSolve->setText(QApplication::translate("MainWindow", "\350\247\243\347\256\227", nullptr));
        btn_magicRunSolve->setText(QApplication::translate("MainWindow", "\346\211\247\350\241\214\350\247\243\347\256\227", nullptr));
        btn_magicAutoSolve->setText(QApplication::translate("MainWindow", "\344\270\200\351\224\256\350\247\243\347\256\227", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_3), QApplication::translate("MainWindow", "\351\255\224\346\226\271\347\225\214\351\235\242", nullptr));
        label_2->setText(QApplication::translate("MainWindow", "\345\233\276\345\203\217\346\230\276\347\244\272", nullptr));
        groupBox_setMod->setTitle(QApplication::translate("MainWindow", "\346\250\241\345\274\217\350\256\276\347\275\256", nullptr));
        comboBox->setItemText(0, QApplication::translate("MainWindow", "从货架抓,放桌子上", nullptr));
        comboBox_setRunMode->setItemText(0, QApplication::translate("MainWindow", "RVIZ虚拟点位测试模式", nullptr));
        comboBox_setRunMode->setItemText(1, QApplication::translate("MainWindow", "真机真实点位运行模式", nullptr));
        comboBox_setRunMode->setItemText(2, QApplication::translate("MainWindow", "真机虚拟点位测试模式", nullptr));

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
    emit emitStartTimer(updateTimer_LeftCamera);
}

void MainWindow::callback_RightCamera_subscriber(const sensor_msgs::Image::ConstPtr image) {
    connFlag_RightCamera= true;
    emit emitStartTimer(updateTimer_RightCamera);
//    const cv_bridge::CvImageConstPtr &ptr = cv_bridge::toCvShare(msg, "bgr8");
//    cv::Mat mat = ptr->image;
//    QImage image = cvMat2QImage(mat);
//    QPixmap pixmap1 = QPixmap::fromImage(image);
//    QPixmap fitpixmap1 = pixmap1.scaled(label_picture1->width(), label_picture1->height(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);  // 饱满填充
//    QPixmap fitpixmap1 = pixmap1.scaled(label_picture1->width(), label_picture1->height(), Qt::KeepAspectRatio, Qt::SmoothTransformation);  // 按比例缩放
//    label_picture1->setPixmap(fitpixmap1);
}

void MainWindow::callback_rob1Status_subscriber(const industrial_msgs::RobotStatus::ConstPtr robot_status) {
    connFlag_LeftRobot= true;
    emit emitStartTimer(updateTimer_rob1status);
    if(robot_status->in_error.val==0){
        errFlag_LeftRobot= false;
    } else{
        errFlag_LeftRobot=true;
    }

    if(robot_status->drives_powered.val==1){
        enableFlag_LeftRobot= true;
    } else{
        enableFlag_LeftRobot= false;
    }
}

void MainWindow::callback_rob2Status_subscriber(const industrial_msgs::RobotStatus::ConstPtr robot_status) {
    connFlag_RightRobot= true;
    emit emitStartTimer(updateTimer_rob2status);
    if(robot_status->in_error.val==0){
        errFlag_RightRobot= false;
    } else{
        errFlag_RightRobot=true;
    }

    if(robot_status->drives_powered.val==1){
        enableFlag_RightRobot= true;
    } else{
        enableFlag_RightRobot= false;
    }
}

void MainWindow::runTimer(QTimer* timer) {
    timer->start();
}





























