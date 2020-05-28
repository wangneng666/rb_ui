#include "rb_ui/MainWindow.h"
#include "logmanager.h"
#include "globalvar.h"
#include "messagehandler.h"
using namespace std;
using namespace cv;


int main(int args,char** argv){
    //ros节点
    std::string nodeName = "robot_UI";
    ros::init(args, argv, nodeName);
    //创建节点
    log4cplus::Initializer initializer;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node;
    //应用程序
    QApplication app(args, argv);
    MainWindow mainwindow(&node);
    mainwindow.show();
    gloalMain = &mainwindow;
    qInstallMessageHandler(customMessageHandler);
    return app.exec();
}
