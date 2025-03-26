// shared_logger.cpp
#include "rotor_tm_sim/base/lib_logger.hpp"
#include "spdlog/sinks/basic_file_sink.h"
#include <iostream>

namespace Logging {
    // The actual shared logger instance
    std::shared_ptr<spdlog::logger> shared_logger;
    // The shared file sink - all child loggers will use this same sink
    std::shared_ptr<spdlog::sinks::basic_file_sink_mt> file_sink;
    
    std::shared_ptr<spdlog::logger> GetLogger() {
        return shared_logger;
    }
    
    void InitialiseLogger(const std::string& filename) {
        try {
            // Create a file sink that all loggers will share
            file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(filename, true);
            
            // Create the main logger with this sink
            shared_logger = std::make_shared<spdlog::logger>("Main", file_sink);
            
            // Set the log pattern to include timestamp, logger name, and log level
            shared_logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%n] [%l] %v");
            
            // Optional: set log level
            shared_logger->set_level(spdlog::level::info);
            
            // Optional: flush on info level
            shared_logger->flush_on(spdlog::level::info);
            
            std::cout << "Shared logger initialized with file: " << filename << std::endl;
        }
        catch (const spdlog::spdlog_ex& ex) {
            std::cerr << "Shared logger initialization failed: " << ex.what() << std::endl;
        }
    }
    
    std::shared_ptr<spdlog::logger> CreateClassLogger(const std::string& class_name) {
        if (!file_sink) {
            std::cerr << "Error: Trying to create a class logger before initializing the shared logger." << std::endl;
            return nullptr;
        }
        
        try {
            // Create a new logger with the class name but using the same sink
            auto class_logger = std::make_shared<spdlog::logger>(class_name, file_sink);
            
            // Copy settings from the shared logger
            class_logger->set_level(shared_logger->level());
            class_logger->flush_on(shared_logger->flush_level());
            
            // Register the logger with spdlog's registry
            spdlog::register_logger(class_logger);
            
            return class_logger;
        }
        catch (const spdlog::spdlog_ex& ex) {
            std::cerr << "Class logger creation failed: " << ex.what() << std::endl;
            return nullptr;
        }
    }
    
    // Convenience logging functions
    void LogInfo(const std::string& message) {
        if (shared_logger) {
            shared_logger->info(message);
        }
    }
    
    void LogWarning(const std::string& message) {
        if (shared_logger) {
            shared_logger->warn(message);
        }
    }
    
    void LogError(const std::string& message) {
        if (shared_logger) {
            shared_logger->error(message);
        }
    }
}