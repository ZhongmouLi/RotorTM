// shared_logger.cpp
#include "rotor_tm_sim/base/lib_logger.hpp"
#include "spdlog/sinks/basic_file_sink.h"
#include <iostream>


MyLogger::MyLogger(const std::string& class_name, const std::string& log_file) 
    : logger(nullptr), file_sink(nullptr) {
    try {
        // Create the file sink
        file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(log_file, true);
        
        // Check if a logger with this name already exists
        logger = spdlog::get(class_name);
        
        if (logger) {
            // If logger exists but with a different sink, it's a problem
            std::cerr << "Warning: Logger with name '" << class_name 
                      << "' already exists. Using existing logger." << std::endl;
            return;
        }
        
        // Create a new logger with the provided class name and file sink
        logger = std::make_shared<spdlog::logger>(class_name, file_sink);
        
        // Configure the logger
        logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%n] [%l] %v");
        logger->set_level(spdlog::level::info);
        logger->flush_on(spdlog::level::info);
        
        // Register the logger with spdlog's registry
        spdlog::register_logger(logger);
    }
    catch (const spdlog::spdlog_ex& ex) {
        std::cerr << "Logger creation failed for " << class_name << ": " << ex.what() << std::endl;
    }
}

MyLogger::~MyLogger(){

    if(logger)
    {
        logger->flush();
        
    // Remove the logger from spdlog's registry
    spdlog::drop(logger->name());
    
    // Release our reference to the logger
    logger.reset();
    }
}

// void MyLogger::info(const std::string& message) {
//     if (logger) {
//         logger->info(message);
//     }
// }

// void MyLogger::warn(const std::string& message) {
//     if (logger) {
//         logger->warn(message);
//     }
// }

// void MyLogger::error(const std::string& message) {
//     if (logger) {
//         logger->error(message);
//     }
// }

// void MyLogger::debug(const std::string& message) {
//     if (logger) {
//         logger->debug(message);
//     }
// }


bool MyLogger::is_valid() const {
    return logger != nullptr;
}

std::shared_ptr<spdlog::logger> MyLogger::get_logger() const {
    if(logger)
    {
        return logger;
    }
    else
    {
        return nullptr;
    }
}

void MyLogger::set_level(spdlog::level::level_enum level) {
    if (logger) {
        logger->set_level(level);
    }
}

void MyLogger::set_pattern(const std::string& pattern) {
    if (logger) {
        logger->set_pattern(pattern);
    }
}