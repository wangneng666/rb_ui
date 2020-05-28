#pragma once
#include "log4fun.h"
#include <map>
#include <memory>
#define LOG(__VAR__) logSingleManager::getInstance()->getLog(__VAR__)
class logSingleManager
{
private:
    logSingleManager();
public:
    static logSingleManager *getInstance(){
        if(ptr == nullptr)
            ptr = new logSingleManager();
        return ptr;
    }
public:
    std::shared_ptr<logBase> getLog(const std::string & moudle);
    void addLogMember(std::shared_ptr<logBase> &ptr, std::string &name);
private:
    std::map<string, std::shared_ptr<logBase>> logGroup;
    static logSingleManager *ptr;
};


