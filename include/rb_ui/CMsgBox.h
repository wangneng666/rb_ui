
#ifndef RB_UI_CMSGBOX_H
#define RB_UI_CMSGBOX_H
#include "gloalVal.h"
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
#include <QTimer>
#include <iostream>
using namespace std;


class CMsgBox : public QDialog {
    Q_OBJECT
public:
    explicit CMsgBox(QWidget *parent = nullptr);
    ~CMsgBox();
    static int showMsgBox(QWidget *parent = nullptr);

private:
    void init();
    void slot_timerUpdate();

private slots:
            void slot_SwapDataWithMainwin(int* array);
public:
    enum EnmButton{
        ENM_OK_BTN = 0,
        ENM_CANCEL_BTN,
    };

private:
    QLabel* m_lableTitle;
    QPlainTextEdit* m_plaintext;
    QTimer* cTimer1;
    QPushButton *pBtn_checkCancel;
    QPushButton *pBtn_checkOk;
    int* c_array;
    bool checkOk= false;

};


#endif //RB_UI_CMSGBOX_H
