#pragma once

#include "LidarShooter.hpp"

#include <exception>
#include <stdexcept>
#include <iostream>
#include <sstream>
#include <string>

#include <fmt/format.h>
#include <embree3/rtcore.h>

#ifdef LIDARSHOOTER_OPTIX_FOUND
#include <cuda_runtime_api.h>
#include <driver_types.h>
#include <optix.h>
#endif

// TODO: Get rid of preprocessor macros and use throws directly

/**
 * @brief OptiX error handling macros; try to use exceptions directly
 */

#pragma region OptixErrorChecking

#define OPTIX_CHECK( call )                                                    \
    ::lidarshooter::optix::optixCheck( call, #call, __FILE__, __LINE__ )

#define OPTIX_CHECK_LOG( call )                                                \
    do                                                                         \
    {                                                                          \
        char   LOG[2048];                                                      \
        size_t LOG_SIZE = sizeof( LOG );                                       \
        ::lidarshooter::optix::optixCheckLog( call, LOG, sizeof( LOG ),        \
                                LOG_SIZE, #call, __FILE__, __LINE__ );         \
    } while( false )

#define OPTIX_CHECK_NOTHROW( call )                                            \
    ::lidarshooter::optix::optixCheckNoThrow( call, #call, __FILE__, __LINE__ )

#pragma endregion OptixErrorChecking

#pragma region CUDAErrorChecking

/**
 * @brief Simple CUDA error handling functions (debug use; try to use exceptions)
 */
#define CUDA_CHECK( call ) ::lidarshooter::optix::cudaCheck( call, #call, __FILE__, __LINE__ )

#define CUDA_SYNC_CHECK() ::lidarshooter::optix::cudaSyncCheck( __FILE__, __LINE__ )

#define CUDA_CHECK_NOTHROW( call )                                             \
    ::lidarshooter::optix::cudaCheckNoThrow( call, #call, __FILE__, __LINE__ )

#pragma endregion CUDAErrorChecking

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

class MeshNotSetException : public BaseException
{
public:
    MeshNotSetException() = default;
    MeshNotSetException(std::string _errorLocation, std::string _errorString, long _errorCode)
        : BaseException(_errorLocation, _errorString, _errorCode) {}
    ~MeshNotSetException() = default;

    virtual std::string getError() override
    {
        return fmt::format("MeshNotSet error: {} in {} (code {})", getErrorString(), getErrorLocation(), getErrorCode());
    }

private:
    std::string fileName;
};

class BadGeometryException : public BaseException
{
public:
    BadGeometryException() = default;
    BadGeometryException(std::string _errorLocation, std::string _errorString, long _errorCode, RTCGeometryType _geometryType)
        : BaseException(_errorLocation, _errorString, _errorCode), geometryType(_geometryType) {}
    ~BadGeometryException() = default;

    virtual std::string getError() override
    {
        return fmt::format("BadGeometry error (type {}): {} in {} (code {})", geometryType, getErrorString(), getErrorLocation(), getErrorCode());
    }

private:
    std::string fileName;
    RTCGeometryType geometryType;
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

#ifdef LIDARSHOOTER_OPTIX_FOUND

namespace optix {

class Exception : public std::runtime_error
{
public:
    Exception(const char *msg)
        : std::runtime_error(msg)
    {}

    Exception(OptixResult res, const char *msg)
        : std::runtime_error(createMessage(res, msg).c_str())
    {}

private:
    std::string createMessage(OptixResult res, const char *msg)
    {
        std::ostringstream out;
        out << optixGetErrorName(res) << ": " << msg;
        return out.str();
    }
};

inline void optixCheck(OptixResult res, const char *call, const char *file, unsigned int line)
{
    if (res != OPTIX_SUCCESS)
    {
        std::stringstream ss;
        ss << "Optix call '" << call << "' failed: " << file << ':' << line << ")\n";
        throw Exception(res, ss.str().c_str());
    }
}

inline void optixCheckLog(OptixResult res,
                          const char *log,
                          size_t sizeof_log,
                          size_t sizeof_log_returned,
                          const char *call,
                          const char *file,
                          unsigned int line)
{
    if (res != OPTIX_SUCCESS)
    {
        std::stringstream ss;
        ss << "Optix call '" << call << "' failed: " << file << ':' << line << ")\nLog:\n"
           << log << (sizeof_log_returned > sizeof_log ? "<TRUNCATED>" : "") << '\n';
        throw Exception(res, ss.str().c_str());
    }
}

inline void optixCheckNoThrow(OptixResult res, const char *call, const char *file, unsigned int line) noexcept
{
    if (res != OPTIX_SUCCESS)
    {
        std::cerr << "Optix call '" << call << "' failed: " << file << ':' << line << ")\n";
        std::terminate();
    }
}

inline void cudaCheck(cudaError_t error, const char *call, const char *file, unsigned int line)
{
    if (error != cudaSuccess)
    {
        std::stringstream ss;
        ss << "CUDA call (" << call << " ) failed with error: '"
           << cudaGetErrorString(error) << "' (" << file << ":" << line << ")\n";
        throw Exception(ss.str().c_str());
    }
}

inline void cudaSyncCheck(const char *file, unsigned int line)
{
    cudaDeviceSynchronize();
    cudaError_t error = cudaGetLastError();
    if (error != cudaSuccess)
    {
        std::stringstream ss;
        ss << "CUDA error on synchronize with error '"
           << cudaGetErrorString(error) << "' (" << file << ":" << line << ")\n";
        throw Exception(ss.str().c_str());
    }
}

inline void cudaCheckNoThrow(cudaError_t error, const char *call, const char *file, unsigned int line) noexcept
{
    if (error != cudaSuccess)
    {
        std::cerr << "CUDA call (" << call << " ) failed with error: '"
                  << cudaGetErrorString(error) << "' (" << file << ":" << line << ")\n";
        std::terminate();
    }
}

} // End namespace optix

#endif

} // End namespace lidarshooter