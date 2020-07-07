#ifndef LOG4QTFUN_H
#define LOG4QTFUN_H
#include <QString>
#include "log4fun.h"
class log4QtFun :public logBase
{
public:
    log4QtFun(std::shared_ptr<logBase> ptr);
    virtual void logInfoMessage(const std::string& msg);
    virtual void logWarnMessage(const std::string& msg);
    virtual void logErrorMessage(const std::string& msg);
    virtual void logFatelMessage(const std::string& msg);
    virtual string &getModouleName();
private:
    std::shared_ptr<logBase> ptr;
    QString moudleName;
};

#endif // LOG4QTFUN_H
