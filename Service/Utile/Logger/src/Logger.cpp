#include "Logger.hpp"

#include <iostream>
#include <fstream>

using namespace std;
using namespace Log;

namespace fs = std::filesystem;

Logger::Logger(string rootdir, bool showConsole)
	:rootdir_(rootdir) , showConsole_(showConsole) , isRunning_(true)
{
	if (!fs::exists(rootdir))
	{
		fs::create_directories(rootdir);
	}

	loggerLoop_ = thread(&Logger::loggerLoop, this);
}

Logger::~Logger()
{
	isRunning_ = false;

	queueCondVar_.notify_all();
	if (loggerLoop_.joinable()) 
	{
		loggerLoop_.join();
	}
}

std::string Logger::getCurrentDate()
{
	auto now = std::chrono::system_clock::now();
	std::time_t t = std::chrono::system_clock::to_time_t(now);
	std::tm tm{};
#ifdef _WIN32
	localtime_s(&tm, &t);
#else
	localtime_r(&t, &tm);
#endif
	std::ostringstream oss;
	oss << std::put_time(&tm, "%Y-%m-%d");
	return oss.str();
}

std::string Logger::getCurrentTime()
{
	auto now = std::chrono::system_clock::now();
	std::time_t t = std::chrono::system_clock::to_time_t(now);
	std::tm tm{};
#ifdef _WIN32
	localtime_s(&tm, &t);
#else
	localtime_r(&t, &tm);
#endif
	std::ostringstream oss;
	oss << std::put_time(&tm, "%H");
	return oss.str();
}

std::string Logger::getTimestamp()
{
	auto now = std::chrono::system_clock::now();
	std::time_t t = std::chrono::system_clock::to_time_t(now);
	std::tm tm{};
#ifdef _WIN32
	localtime_s(&tm, &t);
#else
	localtime_r(&t, &tm);
#endif
	std::ostringstream oss;
	oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
	return oss.str();
}

std::string Logger::levelToString(const LogLevel level)
{
	switch (level)
	{
	case LogLevel::DEBUG:   return "DEBUG";
	case LogLevel::INFO:    return "INFO";
	case LogLevel::WARN:	return "WARN";
	case LogLevel::ERROR:   return "ERROR";
	case LogLevel::FATAL:   return "FATAL";
	default:                return "UNKNOWN";
	}
}

void Logger::loggerLoop() 
{
	while (isRunning_)
	{
		std::unique_lock<std::mutex> lock(loggerlock_);
		queueCondVar_.wait(lock, [&] { return !LogQueue_.empty(); });

		while (!LogQueue_.empty())
		{
			LogMsg msg = LogQueue_.front();
			LogQueue_.pop();
			lock.unlock();

			string date = getCurrentDate();
			string time = getCurrentTime();
			fs::path dirPath = fs::path(rootdir_) / date;
			fs::path filePath = dirPath / (time + ".txt");

			if (!fs::exists(dirPath))
			{
				fs::create_directories(dirPath);
			}

			if (!fs::exists(filePath)) {
				std::ofstream createFile(filePath);
				createFile.close();
			}

			std::ofstream ofs(filePath, std::ios::app);
			if (ofs.is_open())
			{
				std::ostringstream oss;
				oss << "[" << msg.timestamp << "] "
					<< "[" << levelToString(msg.level) << "] "
					<< "[" << msg.funcName << "] "
					<< msg.logmsg << std::endl;

				ofs << oss.str();

				if (showConsole_) std::cout << oss.str();
			}

			lock.lock();
		}
	}

	return;
}

bool Logger::Log(
	const LogLevel level,
	const std::string msg, 
	const std::source_location& loc)
{
	if (msg.empty()) return false;

	LogMsg Msg;
	Msg.level = level;
	Msg.logmsg = msg;
	Msg.funcName = loc.function_name();
	Msg.timestamp = getTimestamp();

	{
		lock_guard<mutex> lock(loggerlock_);

		LogQueue_.push(Msg);
		queueCondVar_.notify_one();
	}
	
	return true;
}