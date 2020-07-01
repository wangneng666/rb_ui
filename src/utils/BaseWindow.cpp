
#include "BaseWindow.h"

BaseWindow::BaseWindow(ros::NodeHandle *node, QWidget *parent):QMainWindow(parent),Node(node) {
    initQtVal();
}

BaseWindow::~BaseWindow() {

}

void BaseWindow::initQtVal() {
    tab_qss=
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
}

void BaseWindow::initUi(QMainWindow *MainWindow) {
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

    btn_rb1_goHomePose = new QPushButton(groupBox_tab3_3);
    btn_rb1_goHomePose->setText("左机器人回原点");
    btn_rb1_goHomePose->setObjectName(QString::fromUtf8("btn_rb1_goHomePose"));
    btn_rb1_goHomePose->setFixedSize(BTN_W,BTN_H);
    horizontalLayout_20->addWidget(btn_rb1_goHomePose);

    btn_rb2_goHomePose = new QPushButton(groupBox_tab3_3);
    btn_rb2_goHomePose->setText("右机器人回原点");
    btn_rb2_goHomePose->setObjectName(QString::fromUtf8("btn_rb2_goHomePose"));
    btn_rb2_goHomePose->setFixedSize(BTN_W,BTN_H);
    horizontalLayout_20->addWidget(btn_rb2_goHomePose);

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

    //魔方点位调试页面
    tab_magicPose = new QWidget();
    tab_magicPose->setObjectName(QString::fromUtf8("tab_magicPose"));
    horizontalLayout_22 = new QHBoxLayout(tab_magicPose);
    horizontalLayout_22->setSpacing(6);
    horizontalLayout_22->setContentsMargins(11, 11, 11, 11);
    horizontalLayout_22->setObjectName(QString::fromUtf8("horizontalLayout_22"));
    horizontalLayout_tabmp_1 = new QHBoxLayout();
    horizontalLayout_tabmp_1->setSpacing(6);
    horizontalLayout_tabmp_1->setObjectName(QString::fromUtf8("horizontalLayout_tabmp_1"));
    verticalLayout_tabmp_11 = new QVBoxLayout();
    verticalLayout_tabmp_11->setSpacing(6);
    verticalLayout_tabmp_11->setObjectName(QString::fromUtf8("verticalLayout_tabmp_11"));
    label_tabmp_1 = new QLabel(tab_magicPose);
    label_tabmp_1->setObjectName(QString::fromUtf8("label_tabmp_1"));

    verticalLayout_tabmp_11->addWidget(label_tabmp_1);

//    horizontalLayout_tabmp_1->addLayout(verticalLayout_tabmp_11);
    gBox_tabmp_1 = new QGroupBox(tab_magicPose);
    gBox_tabmp_1->setObjectName(QString::fromUtf8("gBox_tabmp_1"));
    gBox_tabmp_1->setStyleSheet(groupBox_qss);
    gBox_tabmp_1->setTitle("图像显示");
    gBox_tabmp_2 = new QGroupBox(tab_magicPose);
    gBox_tabmp_2->setObjectName(QString::fromUtf8("gBox_tabmp_2"));
    gBox_tabmp_2->setStyleSheet(groupBox_qss);
    gBox_tabmp_2->setTitle("操作流程");
    horizontalLayout_tabmp_1->addWidget(gBox_tabmp_1);
    horizontalLayout_tabmp_1->addWidget(gBox_tabmp_2);

    gBox_tabmp_1->setLayout(verticalLayout_tabmp_11);


    vLayout_tabmp_12 = new QVBoxLayout();
    vLayout_tabmp_12->setSpacing(6);
    vLayout_tabmp_12->setObjectName(QString::fromUtf8("vLayout_tabmp_12"));

    gBox_tabmp_2->setLayout(vLayout_tabmp_12);

    vLayout_tabmp_121 = new QVBoxLayout();
    vLayout_tabmp_121->setSpacing(6);
    vLayout_tabmp_121->setObjectName(QString::fromUtf8("vLayout_tabmp_121"));
    comboBox_tabmp_1 = new QComboBox(tab_magicPose);
    comboBox_tabmp_1->addItem(QString());
    comboBox_tabmp_1->addItem(QString());
    comboBox_tabmp_1->setObjectName(QString::fromUtf8("comboBox_tabmp_1"));
    comboBox_tabmp_1->setFixedSize(BTN_W,BTN_H);

    vLayout_tabmp_121->addWidget(comboBox_tabmp_1,0,Qt::AlignHCenter);

    btn_tabmp_do = new QPushButton(tab_magicPose);
    btn_tabmp_do->setObjectName(QString::fromUtf8("btn_tabmp_do"));
    btn_tabmp_do->setFixedSize(BTN_W,BTN_H);
    vLayout_tabmp_121->addWidget(btn_tabmp_do,0,Qt::AlignHCenter);

    btn_tabmp_step = new QPushButton(tab_magicPose);
    btn_tabmp_step->setObjectName(QString::fromUtf8("btn_tabmp_step"));
    btn_tabmp_step->setFixedSize(BTN_W,BTN_H);

    vLayout_tabmp_121->addWidget(btn_tabmp_step,0,Qt::AlignHCenter);

    btn_tabmp_recordPose = new QPushButton(tab_magicPose);
    btn_tabmp_recordPose->setObjectName(QString::fromUtf8("btn_tabmp_recordPose"));
    btn_tabmp_recordPose->setFixedSize(BTN_W,BTN_H);

    vLayout_tabmp_121->addWidget(btn_tabmp_recordPose,0,Qt::AlignHCenter);


    vLayout_tabmp_12->addLayout(vLayout_tabmp_121);

    hLayout_tabmp_122 = new QHBoxLayout();
    hLayout_tabmp_122->setSpacing(6);
    hLayout_tabmp_122->setObjectName(QString::fromUtf8("hLayout_tabmp_122"));
    btn_tabmp_newteach = new QPushButton(tab_magicPose);
    btn_tabmp_newteach->setObjectName(QString::fromUtf8("btn_tabmp_newteach"));
    btn_tabmp_newteach->setFixedSize(BTN_W,BTN_H);
    hLayout_tabmp_122->addWidget(btn_tabmp_newteach,0,Qt::AlignHCenter);

    btn_tabmp_resetPose = new QPushButton(tab_magicPose);
    btn_tabmp_resetPose->setObjectName(QString::fromUtf8("btn_tabmp_resetPose"));
    btn_tabmp_resetPose->setFixedSize(BTN_W,BTN_H);

    hLayout_tabmp_122->addWidget(btn_tabmp_resetPose,0,Qt::AlignHCenter);


    vLayout_tabmp_12->addLayout(hLayout_tabmp_122);

    hLayout_tabmp_123 = new QHBoxLayout();
    hLayout_tabmp_123->setSpacing(6);
    hLayout_tabmp_123->setObjectName(QString::fromUtf8("hLayout_tabmp_123"));
    textEdit_tabmp_1 = new QTextEdit(tab_magicPose);
    textEdit_tabmp_1->setObjectName(QString::fromUtf8("textEdit_tabmp_1"));
    textEdit_tabmp_1->setText("当前示教点坐标:");
    hLayout_tabmp_123->addWidget(textEdit_tabmp_1,0,Qt::AlignHCenter);

    vLayout_tabmp_12->addLayout(hLayout_tabmp_123);

//    horizontalLayout_tabmp_1->addLayout(vLayout_tabmp_12);

    horizontalLayout_tabmp_1->setStretch(0, 2);
    horizontalLayout_tabmp_1->setStretch(1, 1);

    horizontalLayout_22->addLayout(horizontalLayout_tabmp_1);

    tabWidget->addTab(tab_magicPose, QString());





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

void BaseWindow::retranslateUi(QMainWindow *MainWindow) {
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
    tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("MainWindow", "设备调试界面", nullptr));

    label_tabmp_1->setText(QString());
    comboBox_tabmp_1->setItemText(0, QApplication::translate("MainWindow", "\351\255\224\346\226\271\345\244\215\345\216\237\345\212\250\344\275\234", nullptr));
    comboBox_tabmp_1->setItemText(1, QApplication::translate("MainWindow", "\351\255\224\346\226\271\346\213\215\347\205\247\345\212\250\344\275\234", nullptr));

    btn_tabmp_do->setText(QApplication::translate("MainWindow", "\346\211\247\350\241\214", nullptr));
    btn_tabmp_step->setText(QApplication::translate("MainWindow", "\345\257\270\350\277\233", nullptr));
    btn_tabmp_recordPose->setText(QApplication::translate("MainWindow", "\350\256\260\345\275\225\347\202\271\344\275\215", nullptr));
    btn_tabmp_newteach->setText(QApplication::translate("MainWindow", "\351\207\215\346\226\260\347\244\272\346\225\231", nullptr));
    btn_tabmp_resetPose->setText(QApplication::translate("MainWindow", "动作点位重置", nullptr));
    tabWidget->setTabText(tabWidget->indexOf(tab_magicPose), QApplication::translate("MainWindow", "\351\255\224\346\226\271\347\202\271\344\275\215\346\240\241\345\207\206", nullptr));

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
    comboBox_2->setItemText(0, QApplication::translate("MainWindow", "维他奶", nullptr));
    comboBox_2->setItemText(1, QApplication::translate("MainWindow", "可乐罐", nullptr));
    comboBox_2->setItemText(2, QApplication::translate("MainWindow", "旺仔牛奶", nullptr));
    comboBox_2->setItemText(3, QApplication::translate("MainWindow", "公仔", nullptr));

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
