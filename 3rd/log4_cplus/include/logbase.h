#ifndef LOGBASE_H
#define LOGBASE_H
#include <iostream>
#include <string>
using namespace std;
class logBase{
public:
    virtual void logInfoMessage(const std::string& msg)=0;
    virtual void logWarnMessage(const std::string& msg)=0;
    virtual void logErrorMessage(const std::string& msg)=0;
    virtual void logFatelMessage(const std::string& msg)=0;
    virtual string &getModouleName()=0;
};
#endif // LOGBASE_H
