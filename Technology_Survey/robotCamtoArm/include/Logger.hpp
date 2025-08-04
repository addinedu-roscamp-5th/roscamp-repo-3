#pragma once
#include <thread>
#include <mutex>
#include <condition_variable>
#include <string>
#include <queue>
#include <filesystem>
#include <chrono>
#include <source_location>

namespace Log 
{
	enum LogLevel
	{
		DEBUG = 0,			// 디버깅 목적의 로그
		INFO,				// 정보성 메시지 로그
		WARN,				// 경고성 메시지 로그
		ERROR,				// 오류 메시지 로그
		FATAL,				// 가장 심각한 오류 메시지 로그
		LogLevelCnt,
	};

	typedef struct LogMsg
	{
		LogLevel level;
		std::string funcName;
		std::string timestamp; //YYYY - MM - DD - HH-MM
		std::string logmsg;
	}LogMsg;

	class Logger
	{
	private:
		std::thread					loggerLoop_;
		std::mutex					loggerlock_;
		std::condition_variable		queueCondVar_;
		bool						isRunning_;

		std::string					rootdir_;


		std::queue<LogMsg>			LogQueue_;

		bool						showConsole_;

		//---------------------------------------------private Fuctins
		void						loggerLoop();

		std::string					getCurrentDate();
		std::string					getCurrentTime();
		std::string					getTimestamp();

		std::string					levelToString(const LogLevel level);

	public:
		using Ptr = std::shared_ptr<Logger>; //alias

		Logger(std::string rootdir, bool showConsole = false);
		~Logger();

		bool Log(const LogLevel level , const std::string msg, const std::source_location& loc = std::source_location::current());
	};
}

