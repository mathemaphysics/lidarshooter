#pragma once

#include <exception>
#include <stdexcept>

#include <fmt/format.h>

namespace lidarshooter
{

class BaseException
{
public:
    BaseException(std::string _errorLocation, std::string _errorString, long _errorCode)
        : errorLocation(_errorLocation), errorString(_errorString), errorCode(_errorCode) {}
    ~BaseException() = default;

    virtual std::string getError() = 0;

    std::string getErrorLocation()
    {
        return errorLocation;
    }

    std::string getErrorString()
    {
        return errorString;
    }

    long getErrorCode()
    {
        return errorCode;
    }

private:
    std::string errorLocation;
    std::string errorString;
    long errorCode;
};

class ConfigurationException : public BaseException
{
public:
    ConfigurationException() = default;
    ConfigurationException(std::string _fileName, std::string _errorLocation, std::string _errorString, long _errorCode)
        : BaseException(_errorLocation, _errorString, _errorCode), fileName(_fileName) {}
    ~ConfigurationException() = default;

    virtual std::string getError() override
    {
        return fmt::format("Configuration error: {} in {} (code {})", getErrorString(), getErrorLocation(), getErrorCode());
    }

private:
    std::string fileName;
};

class CloudNotSetException : public BaseException
{
public:
    CloudNotSetException() = default;
    CloudNotSetException(std::string _errorLocation, std::string _errorString, long _errorCode)
        : BaseException(_errorLocation, _errorString, _errorCode) {}
    ~CloudNotSetException() = default;

    virtual std::string getError() override
    {
        return fmt::format("CloudNotSet error: {} in {} (code {})", getErrorString(), getErrorLocation(), getErrorCode());
    }

private:
    std::string fileName;
};

class TraceException : public BaseException
{
public:
    TraceException() = default;
    TraceException(std::string _errorLocation, std::string _errorString, long _errorCode)
        : BaseException(_errorLocation, _errorString, _errorCode) {}
    ~TraceException() = default;

    virtual std::string getError() override
    {
        return fmt::format("Trace error: {} in {} (code {})", getErrorString(), getErrorLocation(), getErrorCode());
    }

private:
    std::string fileName;
};

}
