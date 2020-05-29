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
    //定时器实体化
    updateTimer = new QTimer(this);
    updateTimer->setInterval(1);
    //话题或服务对象初始化
    rbStopCommand_publisher= Node->advertise<std_msgs::Bool>("/Rb_stopCommand", 1000);
    SafetyStop_publisher=Node->advertise<std_msgs::Bool>("/Safety_stop", 1000);
    rbConnCommand_client = Node->serviceClient<rb_msgs::robotConn>("/Rb_connCommand");
    rbSetEnable1_client = Node->serviceClient<rb_msgs::SetEnableSrv>("/UR51/set_robot_enable");
    rbSetEnable2_client = Node->serviceClient<rb_msgs::SetEnableSrv>("/UR52/set_robot_enable");
    rbErrStatus_client = Node->serviceClient<rb_msgs::robotError>("/Rb_errStatus");
//    ImageGet_client = Node->serviceClient<rb_msgs::rbImageList>("/MagicCubeImage");
//    camera_subscriber=Node->subscribe<sensor_msgs::Image>("/usb_cam/image_raw",1000,boost::bind(&MainWindow::callback_camera_subscriber, this, _1));
    rbAutoSolveMagicCommand_client = Node->serviceClient<rb_msgs::rb_DoubleBool>("/Rb_AutoSolveMagic");
    rbGrepSetCommand_client = Node->serviceClient<rb_msgs::rb_ArrayAndBool>("/Rb_grepSetCommand");
    rbRunCommand_client = Node->serviceClient<rb_msgs::rb_DoubleBool>("/Rb_runCommand");
    MagicGetDataCommand_client = Node->serviceClient<rb_msgs::rb_DoubleBool>("/MagicGetDataCommand");
    MagicSolveCommand_client= Node->serviceClient<rb_msgs::rb_DoubleBool>("/MagicSolveCommand");
    MagicRunSolveCommand_client = Node->serviceClient<rb_msgs::rb_DoubleBool>("/MagicRunSolveCommand");

    qRegisterMetaType<infoLevel>("infoLevel");//信号与槽连接自定义类型需要注册
    //线程句柄初始化
    //给设备连接按钮事件开辟子线程
    thread_forRbConn = new qthreadForRos();
    thread_forRbConn->setParm(this,&MainWindow::thread_rbConnCommand);
    //开始运行子线程
    thread_forBeginRun= new qthreadForRos();
    thread_forBeginRun->setParm(this,&MainWindow::thread_BeginRun);
    //一键解魔方子线程
    thread_forAutoSolveMagic= new qthreadForRos();
    thread_forAutoSolveMagic->setParm(this,&MainWindow::thread_AutoSolveMagic);
    //采集魔方数据子线程
    thread_forGagicGetData= new qthreadForRos();
    thread_forGagicGetData->setParm(this,&MainWindow::thread_GagicGetData);
    //解算魔方子线程
    thread_forGagicSolve= new qthreadForRos();
    thread_forGagicSolve->setParm(this,&MainWindow::thread_GagicSolve);
    //运行解算子线程
    thread_forGagicRunSolve= new qthreadForRos();
    thread_forGagicRunSolve->setParm(this,&MainWindow::thread_GagicRunSolve);
    //机器人抓取子线程
    thread_forRbGrepSet= new qthreadForRos();
    thread_forRbGrepSet->setParm(this,&MainWindow::thread_RbGrepSet);
    //监听机器人故障子线程
    thread_forLisionErrInfo= new qthreadForRos();
    thread_forLisionErrInfo->setParm(this,&MainWindow::thread_LisionErrInfo);
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
    connect(btn_SatetyRb1Stop,&QPushButton::clicked,this,&MainWindow::safety_rob1Stop);
    //机器人2停止
    connect(btn_SatetyRb2Stop,&QPushButton::clicked,this,&MainWindow::safety_rob2Stop);
    //定时器启动
    connect(updateTimer, &QTimer::timeout, this, &MainWindow::timer_onUpdate);
    updateTimer->start();
/****************************************************************************************************/

/*********************************自定义信号与槽函数绑定*************************************************/
    connect(this, &MainWindow::emitTextControl,this, &MainWindow::displayTextControl);
    connect(thread_forRbConn, SIGNAL(signal_SendMsgBox(infoLevel ,QString)), this,SLOT(showQmessageBox(infoLevel,QString)));  //将自定义槽连接到自定义信号
    connect(thread_forBeginRun, SIGNAL(signal_SendMsgBox(infoLevel ,QString)), this,SLOT(showQmessageBox(infoLevel,QString)));  //将自定义槽连接到自定义信号
    connect(thread_forGagicGetData, SIGNAL(signal_SendMsgBox(infoLevel ,QString)), this,SLOT(showQmessageBox(infoLevel,QString)));  //将自定义槽连接到自定义信号
    connect(thread_forGagicSolve, SIGNAL(signal_SendMsgBox(infoLevel ,QString)), this,SLOT(showQmessageBox(infoLevel,QString)));  //将自定义槽连接到自定义信号
    connect(thread_forGagicRunSolve, SIGNAL(signal_SendMsgBox(infoLevel ,QString)), this,SLOT(showQmessageBox(infoLevel,QString)));  //将自定义槽连接到自定义信号
    connect(thread_forRbGrepSet, SIGNAL(signal_SendMsgBox(infoLevel ,QString)), this,SLOT(showQmessageBox(infoLevel,QString)));  //将自定义槽连接到自定义信号
    connect(thread_forAutoSolveMagic, SIGNAL(signal_SendMsgBox(infoLevel ,QString)), this,SLOT(showQmessageBox(infoLevel,QString)));  //将自定义槽连接到自定义信号
    connect(this, SIGNAL(emitQmessageBox(infoLevel ,QString)), this,SLOT(showQmessageBox(infoLevel,QString)));  //将自定义槽连接到自定义信号

/****************************************************************************************************/
}

//定时器回调函数，实时获取状态信息
void MainWindow::timer_onUpdate() {
    Node->getParam("isRunning_solveMagic",isRunning_solveMagic);
    Node->getParam("isRunning_grab",isRunning_grab);
}

//设备连接按钮-1
void MainWindow::dev_connect() {
    bool isRunning;
    Node->getParam("isRuning",isRunning);
    //把耗时操作放在子线程中避免堵塞主线程,若按钮多次调用start,而子线程未执行完,则自动拦截之后的请求,随便多次按.
    cout<<"点击了设备连接按钮"<<endl;
    if(!flag_rbConnStatus)
    {
        thread_forRbConn->start();//运行子线程代码:设备连接按钮中开辟的子线程程序-2
    }
}
//设备连接按钮中开辟的子线程程序-2
void MainWindow::thread_rbConnCommand() {
    rb_msgs::robotConn data_srvs;
    if(rbConnCommand_client.call(data_srvs)){
        if(data_srvs.response.ret){
            flag_rbConnStatus= true;
            label_rb1CoonStatus->setPixmap(QPixmap(QString::fromUtf8("~/catkin_ws/src/HsDualAppBridge/rb_msgs/photo/light_green.png")));
            label_rb2CoonStatus->setPixmap(QPixmap(QString::fromUtf8("~/catkin_ws/src/HsDualAppBridge/rb_msgs/photo/light_green.png")));
        } else{
            LOG("Warning")->logErrorMessage("机器人连接失败!");
            emit thread_forRbConn->signal_SendMsgBox(infoLevel::warning,QString("机器人连接失败!"));
        }
    } else{
        LOG("Warning")->logErrorMessage("rbConnCommand_client接收消息失败!");
        emit thread_forRbConn->signal_SendMsgBox(infoLevel::warning,QString("rbConnCommand_client接收消息失败!"));
    }
}


//运行启动按钮-1
void MainWindow::run_statup() {
    cout<<"点击了运行启动按钮"<<endl;
    thread_forBeginRun->start();//转到运行启动按钮开启的子线程-2
}
//运行启动按钮开启的子线程-2
void MainWindow::thread_BeginRun() {
    system("roslaunch co605_dual_arm_gripper_moveit_config demo.launch ");

    //1.机器人上使能
    //2.启动仕忠的launch文件
    //3.开辟线程监听机器人状态(故障状态)
//    rb_msgs::SetEnableSrv data_srvs1;
//    rb_msgs::SetEnableSrv data_srvs2;
//    data_srvs1.request.enable= true;
//    data_srvs2.request.enable= true;
//    if((rbSetEnable1_client.call(data_srvs1))&&(rbSetEnable2_client.call(data_srvs2))){
//        if(data_srvs1.response.finsh&&data_srvs2.response.finsh){
//            cout<<"机器人上使能成功"<<endl;
//        } else{
//            emit thread_forBeginRun->signal_SendMsgBox(infoLevel::warning,QString("rbConnCommand_client接收消息失败!"));
//        }
//    } else{
//        LOG("Warning")->logErrorMessage("rbRunCommand_client接收消息失败!");
//        emit thread_forBeginRun->signal_SendMsgBox(infoLevel::warning,QString("rbRunCommand_client接收消息失败!"));
//        return;
//    }
//    system("roslaunch rubik_cube_solve solve.launch");
//    //开辟监听故障状态子线程
//    thread_forLisionErrInfo->start();//转到监听故障状态子线程-3
}
//监听故障状态子线程-3
void MainWindow::thread_LisionErrInfo() {
    //每隔一秒监听一次报警信息,并在机器人报警状态显示上刷新
    rb_msgs::robotError errorMsg;
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

//启动rviz
void MainWindow::rviz_statup() {
//    QProcess* proc=new QProcess(this);
//    proc->start("rosrun rviz rviz");
}

//运行停止
void MainWindow::run_stop() {
    cout<<"点击了运行停止按钮"<<endl;
    rb_msgs::SetEnableSrv data_srvs1;
    rb_msgs::SetEnableSrv data_srvs2;
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
    //关掉thread_forBeginRun线程
    thread_forBeginRun->quit();
}

void MainWindow::magicCube_get() {
    cout<<"点击了采集魔方数据按钮"<<endl;
    thread_forGagicGetData->start();
}

void MainWindow::thread_GagicGetData() {
    magicGetData_subscriber=Node->subscribe<rb_msgs::rbImageList>("/camera",1,&MainWindow::callback_magicGetData_subscriber,this);
}

//接收魔方数据
void MainWindow::callback_magicGetData_subscriber(rb_msgs::rbImageList rbimageList) {
    rb_msgs::rbImageList data_msg;
    cout<<"size"<<data_msg.imagelist.size()<<endl;
    data_msg.imagelist.reserve(6);
    for (int i = 0; i < 6; ++i) {
        sensor_msgs::Image image = data_msg.imagelist[0];
        const cv_bridge::CvImagePtr &ptr = cv_bridge::toCvCopy(image, "bgr8");
        cv::Mat mat = ptr->image;
        QImage qimage = cvMat2QImage(mat);
        QPixmap pixmap = QPixmap::fromImage(qimage);
        QPixmap fitpixmap = pixmap.scaled(label_picture1->width(), label_picture1->height(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);  // 饱满填充
        list_label_picture[i]->setPixmap(fitpixmap);
    }
    magicGetData_subscriber.shutdown();
}

void MainWindow::magicCube_solve() {
    cout<<"点击了解算魔方数据按钮"<<endl;
    thread_forGagicSolve->start();
}

void MainWindow::magicCube_execute() {
    cout<<"点击了执行解算魔方数据按钮"<<endl;
    thread_forGagicRunSolve->start();
}

//启动一键解魔方线程－－－－１
void MainWindow::magicCube_AutoRun() {
    cout<<"点击了一键解算魔方按钮"<<endl;
    //如果机器人运行中则返回
    if(isRunning_grab|isRunning_grab){
        return;
    }
    thread_forAutoSolveMagic->start();

}

//进入一键解魔方线程－－－－２
void MainWindow::thread_AutoSolveMagic() {
    rb_msgs::rb_DoubleBool data_srvs;
    data_srvs.request.request=true;
    if(rbAutoSolveMagicCommand_client.call(data_srvs)){
        if(data_srvs.response.respond){
            cout<<"运行一键解魔方成功"<<endl;
        }
    } else{
        LOG("Warning")->logErrorMessage("rbAutoSolveMagicCommand_client接收消息失败!");

    }
}

void MainWindow::robot_grab() {
//如果机器人运行中则返回
    if(isRunning_grab|isRunning_grab){
        return;
    }
//机器人没运行，则开始行动
    cout<<"点击了机器人抓取按钮"<<endl;
    thread_forRbGrepSet->start();

}

void MainWindow::safety_sysStop() {
    cout<<"点击了系统停止按钮"<<endl;
}

void MainWindow::safety_rob1Stop() {
    cout<<"点击了机器人1停止按钮"<<endl;
}

void MainWindow::safety_rob2Stop() {
    cout<<"点击了机器人2停止按钮"<<endl;
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

//void MainWindow::callback_camera_subscriber(const sensor_msgs::Image::ConstPtr &msg) {
//    const cv_bridge::CvImageConstPtr &ptr = cv_bridge::toCvShare(msg, "bgr8");
//    cv::Mat mat = ptr->image;
//    QImage image = cvMat2QImage(mat);
//    QPixmap pixmap1 = QPixmap::fromImage(image);
//    QPixmap fitpixmap1 = pixmap1.scaled(label_picture1->width(), label_picture1->height(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);  // 饱满填充
////    QPixmap fitpixmap1 = pixmap1.scaled(label_picture1->width(), label_picture1->height(), Qt::KeepAspectRatio, Qt::SmoothTransformation);  // 按比例缩放
//    label_picture1->setPixmap(fitpixmap1);
//    label_picture2->setPixmap(fitpixmap1);
//    label_picture3->setPixmap(fitpixmap1);
//    label_picture4->setPixmap(fitpixmap1);
//    label_picture5->setPixmap(fitpixmap1);
//    label_picture6->setPixmap(fitpixmap1);
//}

void MainWindow::oputRecord() {
    QString displayString;
    QFile file("./robotLog.log");
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

void MainWindow::thread_GagicSolve() {
    rb_msgs::rb_DoubleBool data_srvs;
    data_srvs.request.request=true;
    if(MagicSolveCommand_client.call(data_srvs)){
        if(data_srvs.response.respond){
            cout<<"解算魔方数据成功"<<endl;
        }
    } else{
        LOG("")->logWarnMessage("MagicSolveCommand_client接收消息失败!");
    }
}

void MainWindow::thread_GagicRunSolve() {
    rb_msgs::rb_DoubleBool data_srvs;
    data_srvs.request.request=true;
    if(MagicRunSolveCommand_client.call(data_srvs)){
        if(data_srvs.response.respond){
            cout<<"运行魔方解算数据成功"<<endl;
        }
    } else{
        LOG("Warning")->logErrorMessage("rbGrepSetCommand_client接收消息失败!");

    }
}

void MainWindow::thread_RbGrepSet() {
    int index1=comboBox->currentIndex();
    int index2=comboBox_2->currentIndex();
    int index3=comboBox_3->currentIndex();

    rb_msgs::rb_ArrayAndBool data_msg;
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
    label_3->setPixmap(QPixmap(QString::fromUtf8("/home/wangneng/catkin_ws/src/HsDualAppBridge/rb_ui/photo/logo.png")));
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

    label_5 = new QLabel(tab);
    label_5->setObjectName(QString::fromUtf8("label_5"));
    label_5->setAlignment(Qt::AlignVCenter|Qt::AlignRight);
    gridLayout->addWidget(label_5, 0, 0, 1, 1);
    label_rb1CoonStatus = new QLabel(tab);
    label_rb1CoonStatus->setObjectName(QString::fromUtf8("label_rb1CoonStatus"));
    label_rb1CoonStatus->setPixmap(QPixmap(QString::fromUtf8("/home/wangneng/catkin_ws/src/HsDualAppBridge/rb_ui/photo/light_red.png")));
    label_rb1CoonStatus->setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
    gridLayout->addWidget(label_rb1CoonStatus, 0, 1, 1, 1);

    label_6 = new QLabel(tab);
    label_6->setObjectName(QString::fromUtf8("label_6"));
    label_6->setAlignment(Qt::AlignVCenter|Qt::AlignRight);
    gridLayout->addWidget(label_6, 0, 2, 1, 1);
    label_rb2CoonStatus = new QLabel(tab);
    label_rb2CoonStatus->setObjectName(QString::fromUtf8("label_rb2CoonStatus"));
    label_rb2CoonStatus->setPixmap(QPixmap(QString::fromUtf8("/home/wangneng/catkin_ws/src/HsDualAppBridge/rb_ui/photo/light_red.png")));
    label_rb2CoonStatus->setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
    gridLayout->addWidget(label_rb2CoonStatus, 0, 3, 1, 1);

    label_7 = new QLabel(tab);
    label_7->setObjectName(QString::fromUtf8("label_7"));
    label_7->setAlignment(Qt::AlignVCenter|Qt::AlignRight);
    gridLayout->addWidget(label_7, 0, 4, 1, 1);
    label_rb1ErrStatus = new QLabel(tab);
    label_rb1ErrStatus->setObjectName(QString::fromUtf8("label_rb1ErrStatus"));
    label_rb1ErrStatus->setPixmap(QPixmap(QString::fromUtf8("/home/wangneng/catkin_ws/src/HsDualAppBridge/rb_ui/photo/light_red.png")));
    label_rb1ErrStatus->setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
    gridLayout->addWidget(label_rb1ErrStatus, 0, 5, 1, 1);

    label_8 = new QLabel(tab);
    label_8->setObjectName(QString::fromUtf8("label_8"));
    label_8->setAlignment(Qt::AlignVCenter|Qt::AlignRight);
    gridLayout->addWidget(label_8, 0, 6, 1, 1);
    label_rb2ErrStatus = new QLabel(tab);
    label_rb2ErrStatus->setObjectName(QString::fromUtf8("label_rb2ErrStatus"));
    label_rb2ErrStatus->setPixmap(QPixmap(QString::fromUtf8("/home/wangneng/catkin_ws/src/HsDualAppBridge/rb_ui/photo/light_red.png")));
    label_rb2ErrStatus->setAlignment(Qt::AlignVCenter|Qt::AlignLeft);
    gridLayout->addWidget(label_rb2ErrStatus, 0, 7, 1, 1);

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


    verticalLayout_4->addLayout(horizontalLayout_5);

    verticalLayout_4->setStretch(0, 1);
    verticalLayout_4->setStretch(1, 1);

    horizontalLayout_4->addLayout(verticalLayout_4);

    tabWidget->addTab(tab, QString());
    tab_2 = new QWidget();
    tab_2->setObjectName(QString::fromUtf8("tab_2"));
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
    label_picture1->setFixedSize(300,200);
    label_picture2->setFixedSize(300,200);
    label_picture3->setFixedSize(300,200);
    label_picture4->setFixedSize(300,200);
    label_picture5->setFixedSize(300,200);
    label_picture6->setFixedSize(300,200);
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
    gridLayout1->addWidget(label_picture1, 0, 0, 1, 1);
    gridLayout1->addWidget(label_picture2, 0, 1, 1, 1);
    gridLayout1->addWidget(label_picture3, 1, 0, 1, 1);
    gridLayout1->addWidget(label_picture4, 1, 1, 1, 1);
    gridLayout1->addWidget(label_picture5, 2, 0, 1, 1);
    gridLayout1->addWidget(label_picture6, 2, 1, 1, 1);
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
    btn_magicGetdata->setFixedSize(200,50);
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

    btn_SatetyRb1Stop = new QPushButton(tab_6);
    btn_SatetyRb1Stop->setObjectName(QString::fromUtf8("btn_SatetyRb1Stop"));

    horizontalLayout_17->addWidget(btn_SatetyRb1Stop);

    btn_SatetyRb2Stop = new QPushButton(tab_6);
    btn_SatetyRb2Stop->setObjectName(QString::fromUtf8("btn_SatetyRb2Stop"));

    horizontalLayout_17->addWidget(btn_SatetyRb2Stop);


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
        label_7->setText(QApplication::translate("MainWindow", "机器人1报警状态", nullptr));
        label_8->setText(QApplication::translate("MainWindow", "机器人2报警状态", nullptr));
        label_picture1->setText(QApplication::translate("MainWindow", "图片", nullptr));
        label_rb1CoonStatus->setText(QString());
        btn_rbConn->setText(QApplication::translate("MainWindow", "\350\256\276\345\244\207\350\277\236\346\216\245", nullptr));
        btn_rvizRun->setText(QApplication::translate("MainWindow", "\345\220\257\345\212\250rviz", nullptr));
        btn_beginRun->setText(QApplication::translate("MainWindow", "\345\274\200\345\247\213\350\277\220\350\241\214", nullptr));
        btn_normalStop->setText(QApplication::translate("MainWindow", "\345\215\225\346\254\241\345\201\234\346\255\242", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("MainWindow", "\344\270\273\347\225\214\351\235\242", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("MainWindow", "\344\273\277\347\234\237\347\225\214\351\235\242", nullptr));
        btn_magicGetdata->setText(QApplication::translate("MainWindow", "\351\207\207\351\233\206\351\255\224\346\226\271\346\225\260\346\215\256", nullptr));
        btn_magicSolve->setText(QApplication::translate("MainWindow", "\350\247\243\347\256\227", nullptr));
        btn_magicRunSolve->setText(QApplication::translate("MainWindow", "\346\211\247\350\241\214\350\247\243\347\256\227", nullptr));
        btn_magicAutoSolve->setText(QApplication::translate("MainWindow", "\344\270\200\351\224\256\350\247\243\347\256\227", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_3), QApplication::translate("MainWindow", "\351\255\224\346\226\271\347\225\214\351\235\242", nullptr));
        label_2->setText(QApplication::translate("MainWindow", "\345\233\276\345\203\217\346\230\276\347\244\272", nullptr));
        groupBox_setMod->setTitle(QApplication::translate("MainWindow", "\346\250\241\345\274\217\350\256\276\347\275\256", nullptr));
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
        btn_SatetyRb1Stop->setText(QApplication::translate("MainWindow", "\346\234\272\345\231\250\344\272\2721\345\201\234\346\255\242", nullptr));
        btn_SatetyRb2Stop->setText(QApplication::translate("MainWindow", "\346\234\272\345\231\250\344\272\2722\345\201\234\346\255\242", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_6), QApplication::translate("MainWindow", "\345\256\211\345\205\250\347\225\214\351\235\242", nullptr));
    }
}






