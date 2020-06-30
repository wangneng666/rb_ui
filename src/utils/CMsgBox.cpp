
#include "CMsgBox.h"


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
