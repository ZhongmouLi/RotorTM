// MyLogger.hpp
#pragma once
#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"
#include <memory>
#include <string>

class MyLogger {
    private:
        std::shared_ptr<spdlog::logger> logger;
        std::shared_ptr<spdlog::sinks::basic_file_sink_mt> file_sink;
        
    public:
        // Constructor now takes just the class name and log file path;
        MyLogger(const std::string& class_name, const std::string& log_file);
        ~MyLogger();
        
        // // Basic logging methods
        // void info(const std::string& message);
        // void warn(const std::string& message);
        // void error(const std::string& message);
        // void debug(const std::string& message);
        
        // Utility methods
        bool is_valid() const;
        std::shared_ptr<spdlog::logger> get_logger() const;
        
        // Configuration methods
        void set_level(spdlog::level::level_enum level);
        void set_pattern(const std::string& pattern);
    };