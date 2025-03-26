// shared_logger.h
#pragma once
#include "spdlog/spdlog.h"

namespace Logging {
    // Function to get the shared logger instance
    std::shared_ptr<spdlog::logger> GetLogger();
    
    // Function to initialize the shared logger with a file
    void InitialiseLogger(const std::string& filename);
    
    // Function to create a child logger with a specific class name
    std::shared_ptr<spdlog::logger> CreateClassLogger(const std::string& class_name);
    
    // Optional: convenience logging functions
    void LogInfo(const std::string& message);
    void LogWarning(const std::string& message);
    void LogError(const std::string& message);
}