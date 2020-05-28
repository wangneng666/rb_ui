#ifndef MESSAGEHANDLER_H
#define MESSAGEHANDLER_H

#include <QString>
#include <QTime>
#include "globalvar.h"
#define _TIME_ qPrintable(QTime::currentTime ().toString ("hh:mm:ss"))
void customMessageHandler(QtMsgType type, const QMessageLogContext &, const QString & msg){
    QString qstrText;
    switch (type)
    {
        case QtDebugMsg:
            qstrText = QString("%1: %2").arg(_TIME_, msg);
            break;
        case QtInfoMsg:
            qstrText = QString("%1: %2 %3").arg(_TIME_, QString("[Info] : "),msg);
            break;
        case QtWarningMsg:
            qstrText = QString("%1: %2 %3").arg(_TIME_,QString("[Warn] :"), msg);
            break;
        case QtCriticalMsg:
            qstrText = QString("%1: %2 %3").arg(_TIME_, QString("[Error] :"),msg);
            break;
        case QtFatalMsg:
            qstrText = QString("%1: %2").arg(_TIME_, msg);
            exit(0);
    }
    qstrText.replace(QRegExp("\""), "");
    gloalMain->emitTextControl(qstrText);
}

#endif // MESSAGEHANDLER_H
