
#ifndef HANDRB_UI_RBQTHREAD_H
#define HANDRB_UI_RBQTHREAD_H


#include <QThread>
#include <iostream>



class MainWindow;
//多线程,线程与主线程信号与槽通信
class rbQthread : public QThread{
public:
    int mod=0;//模式
    MainWindow* m;
    void (MainWindow::*f1)();
//    void (*f2)() ;
    std::function<void ()>  f2;
    //函数回调
    void setParm(MainWindow* m,void (MainWindow::*f1)()){
        this->m=m;
        this->f1=f1;
        mod=1;
    }

    void setParm2(std::function<void ()> f2){
        this->f2=f2;
        mod=2;
    }
    void run(){
        switch (mod){
            case 1:(m->*f1)();break;
            case 2:f2();break;
        }
    }

public:
    void deleteself(){
        delete this;
        std::cout<<"析构自身"<<std::endl;
    }
};


#endif
