#ifndef LOG4FUN_H
#define LOG4FUN_H
#include <log4cplus/log4cplus.h>
#include "logbase.h"
class log4Fun :public logBase
{
public:
    log4Fun(const string &name);
    virtual void logInfoMessage(const std::string& msg);
    virtual void logWarnMessage(const std::string& msg);
    virtual void logErrorMessage(const std::string& msg);
    virtual void logFatelMessage(const std::string& msg);
    virtual string &getModouleName();
private:
    void initParam();
private:
    log4cplus::SharedAppenderPtr consoleAppender,fileAppender;
    log4cplus::Logger logger;
protected:
    std::string logMoudleName;
};

#endif // LOG4FUN_H
