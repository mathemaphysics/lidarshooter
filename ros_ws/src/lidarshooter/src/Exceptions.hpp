#pragma once

#include <exception>
#include <stdexcept>

namespace lidarshooter
{

class BaseException
{
public:
    BaseException() = default;
    ~BaseException() = default;

    virtual std::string getError();

private:
    std::string errorLocation;
    std::string errorString;
    long errorCode;
};

class ConfigurationException : public BaseException
{
public:
    ConfigurationException() = default;
    ~ConfigurationException() = default;

private:

};

}
