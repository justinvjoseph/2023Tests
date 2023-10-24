

#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <string>
#include <iostream>
#include <memory>
#include <fstream>

namespace rms_utils {

enum class LogLevel {
	NONE = 0,
	ERROR = 1,
	WARN = 2,
	INFO = 3,
	DEBUG = 4
};

enum class LogOutput {
	CONSOLE,
	FILE
};

/**
* Logger Class Used to Output Details of Current Application Flow
*/
class Logger {
public:
	static std::shared_ptr<Logger> GetInstance();

	void SetLogPreferences(std::string logFileName,
						   LogLevel level,
						   LogOutput output);

	void Log(std::string codeFile, int codeLine, std::string message, LogLevel messageLevel = LogLevel::DEBUG);

	LogOutput GetLogOutput(const std::string& logOutput);
	LogLevel GetLogLevel(const std::string& logLevel);

private:
	LogLevel logLevel;
	LogOutput logOutput;
	std::ofstream logFile;

	static std::shared_ptr<Logger> loggerInstance;

	void LogMessage(const std::string& message);

};

} //namespace rms_utils

#endif
