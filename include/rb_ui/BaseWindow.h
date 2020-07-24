

#ifndef RB_UI_BASEWINDOW_H
#define RB_UI_BASEWINDOW_H
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
#include <QtWidgets/QProgressBar>
#include <QDateTime>
#include <QDialog>
#include <QDir>
#include <QMessageBox>
#include <QProcess>
#include <QTimer>
#include <QThread>
#include <QImage>
#include <fstream>
#include <QMutex>
#include "QDebug"
#include "qdebug.h"
#include "ros/ros.h"
#include <iostream>
using namespace std;

#define  BTN_W 150
#define  BTN_H 50
#define  COMBOX_W 200
#define  COMBOX_H 50


class BaseWindow: public QMainWindow {
public:
    BaseWindow(ros::NodeHandle* node,QWidget* parent = Q_NULLPTR);
    ~BaseWindow();

public:
    //UI流程
    void initQtVal();
    void initUi(QMainWindow *MainWindow);
    void retranslateUi(QMainWindow *MainWindow);

public:
    //ros节点
    ros::NodeHandle* Node;
    //全局变量
    QString tab_qss;
    QString groupBox_qss;
    QString photoPath;
    QString logPath;
    QPixmap fitpixmap_redLight ;
    QPixmap fitpixmap_greenLight;
public:
    //qt控件
//    QMainWindow* w;
    QWidget *centralWidget;
    QVBoxLayout *verticalLayout_2;
    QVBoxLayout *vLayout_centralWidget_1;
    QHBoxLayout *hLayout_centralwidget_1;
    QHBoxLayout *hLayout_centralwidget_2;
    QLabel *label_centralWidget_title;
    QLabel *label_centralWidget_logo;
    QTabWidget *tabWidget;

    //主界面
    QWidget *tab_main;
    QGroupBox *gBox_tabmain_status;
    QGroupBox *gBox_tabmain_mode;
    QGroupBox *gBox_tabmain_func;
    QHBoxLayout *horizontalLayout_4;
    QVBoxLayout *vLayout_tabmain_1;
    QHBoxLayout *hLayout_tabmain_1;
    QHBoxLayout *hLayout_tabmain_2;
    QHBoxLayout *hLayout_tabmain_3;
    QGridLayout *gridLayout;
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

    QPushButton *btn_rbConn;
    QPushButton *btn_rvizRun;
    QPushButton *btn_beginRun;
    QPushButton *btn_normalStop;
    QPushButton *btn_SysReset;

    //单步调试界面
    QWidget *tab_stepDebug;
    QHBoxLayout *horizontalLayout_6;
    QVBoxLayout *vLayout_tabStepDebug_1;
    QGroupBox *groupBox_tabStepDebug_1;
    QHBoxLayout *horizontalLayout_14;
    QPushButton *btn_rb1SetEnable;
    QPushButton *btn_rb2SetEnable;
    QPushButton *btn_rb1Reset;
    QPushButton *btn_rb2Reset;
    QGroupBox *groupBox_tabStepDebug_2;
    QHBoxLayout *horizontalLayout_19;
    QPushButton *gripper1;
    QPushButton *gripper2;
    QGroupBox *groupBox_tabStepDebug_3;
    QHBoxLayout *horizontalLayout_20;
    QPushButton *btn_rb1_goHomePose;
    QPushButton *btn_rb2_goHomePose;
    QPushButton *btn_rb1putBack;
    QPushButton *btn_rb2putBack;
    QPushButton *btn_ResetGrepFun;

    //魔方点位校准界面
    QWidget *tab_checkMagicPose;
    QHBoxLayout *horizontalLayout_22;
    QHBoxLayout *hLayout_tabCheckMagicPose_1;
    QVBoxLayout *vLayout_tabCheckMagicPose_1;
    QVBoxLayout *vLayout_tabCheckMagicPose_2;
    QGroupBox* gBox_tabCheckMagicPose_1;
    QGroupBox* gBox_tabCheckMagicPose_2;
    QLabel *label_tabmp_1;
    QVBoxLayout *vLayout_tabmp_121;
    QComboBox *comboBox_tabmp_1;
    QPushButton *btn_tabmp_do;
    QPushButton *btn_tabmp_step;
    QPushButton *btn_tabmp_recordPose;
    QHBoxLayout *hLayout_tabmp_122;
    QPushButton *btn_tabmp_newteach;
    QPushButton *btn_tabmp_resetPose;
    QHBoxLayout *hLayout_tabmp_123;
    QTextEdit *textEdit_tabmp_1;

    //魔方功能界面
    QWidget *tab_magicFun;
    QHBoxLayout *horizontalLayout_8;
    QHBoxLayout *hLayout_tabmagicFun_1;
    QHBoxLayout *horizontalLayout_tab3_1;
    QVBoxLayout *vLayout_tabmagicFun_1;
    QGridLayout* gridLayout1;
    vector<QLabel*> list_label_picture;
    QLabel* label_picture1;
    QLabel* label_picture2;
    QLabel* label_picture3;
    QLabel* label_picture4;
    QLabel* label_picture5;
    QLabel* label_picture6;
    QVBoxLayout *vLayout_tabmagicFun_2;
    QPushButton *btn_magicGetdata;
    QPushButton *btn_magicSolve;
    QPushButton *btn_magicRunSolve;
    QPushButton *btn_magicAutoSolve;
    QPushButton *btn_updateData;
    vector<QLineEdit*> line_updataDataList;
    QProgressBar* pProgressBar= nullptr;

    //抓取界面
    QWidget *tab_grabFun;
    QHBoxLayout *horizontalLayout_10;
    QHBoxLayout *hLayout_tabgrabFun_1;
    QVBoxLayout *vLayout_tabgrabFun_1;
    QLabel *label_processImag;
    QLabel *label_preImag;
    QVBoxLayout *vLayout_tabgrabFun_2;
    QGroupBox *gBox_tabgrabFun_setMod;
    QHBoxLayout *horizontalLayout_11;
    QComboBox *comboBox_0;
    QComboBox *comboBox;
    QGroupBox *gBox_tabgrabFun_targetConfig;
    QVBoxLayout *vLayout_12;
    QComboBox *comboBox_2;
    QGroupBox *gBox_tabgrabFun_doWork;
    QHBoxLayout *horizontalLayout_13;
    QComboBox *comboBox_3;
    QPushButton *btn_rbGrep;
    QPushButton *btn_rbGrepStop;

    //日志界面
    QWidget *tab_recoder;
    QHBoxLayout *horizontalLayout_15;
    QHBoxLayout *hLayout_tabrecoder_1;
    QPlainTextEdit *plainTextEdit;
    QVBoxLayout* vLayout_tabrecoder_1;
    QVBoxLayout* vLayout_tabrecoder_2;
    QPushButton* btn_oputRecord;
    QPushButton* btn_clearRecord;

    //安全界面
    QWidget *tab_safety;
    QHBoxLayout *hLayout_tabsafety_1;
    QPushButton *btn_SatetyStop;
    QPushButton *btn_SatetyRb1Reset;
    QPushButton *btn_SatetyRb2Reset;
    QMenuBar *menuBar;
    QStatusBar *statusBar;

    QLabel* showMagicStepLable;
    QLabel* isRunning_solveMagic_Lable;
    QLabel* isRunning_grab_Lable;
    QLabel* showtime_Lable;
};


#endif //RB_UI_BASEWINDOW_H
