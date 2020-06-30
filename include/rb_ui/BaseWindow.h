

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
    QGroupBox *groupBox_tab1_status;
    QGroupBox *groupBox_tab1_mod;
    QGroupBox *groupBox_tab1_func;
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
    QHBoxLayout *horizontalLayout_20;
    QPushButton *btn_rb1_goHomePose;
    QPushButton *btn_rb2_goHomePose;
    QPushButton *btn_rb1putBack;
    QPushButton *btn_rb2putBack;
    QPushButton *btn_ResetGrepFun;

    QWidget *tab_magicPose;
    QHBoxLayout *horizontalLayout_22;
    QHBoxLayout *horizontalLayout_tabmp_1;
    QVBoxLayout *verticalLayout_tabmp_11;
    QLabel *label_tabmp_1;
    QVBoxLayout *vLayout_tabmp_12;
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
    vector<QLineEdit*> line_updataDataList;
    QProgressBar* pProgressBar= nullptr;

    QWidget *tab_4;
    QHBoxLayout *horizontalLayout_10;
    QHBoxLayout *horizontalLayout_9;
    QVBoxLayout *verticalLayout_11;
    QLabel *label_processImag;
    QLabel *label_preImag;
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
    QLabel* showtime_Lable;
};


#endif //RB_UI_BASEWINDOW_H
