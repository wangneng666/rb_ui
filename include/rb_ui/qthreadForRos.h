//
// Created by wangneng on 6/28/20.
//

#ifndef RB_UI_QTHREADFORROS_H
#define RB_UI_QTHREADFORROS_H

#include "MainWindow.h"

#include <QThread>


//给信号与槽函数使用的枚举类型
enum  infoLevel{information,warning};

class MainWindow;
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


#endif //RB_UI_QTHREADFORROS_H
