
#include "Logger.hpp"

using namespace rms_utils;

int main() {

    auto logger = rms_utils::Logger::GetInstance();

    logger->SetLogPreferences("textLog.txt", LogLevel::DEBUG, LogOutput::FILE);

    logger->Log("File", 1, "Testing 1");
    logger->Log("File", 1, "Testing 2");
    logger->Log("File", 1, "Testing 3");
    logger->Log("File", 1, "Testing 4");
    logger->Log("File", 1, "Testing 55");
    logger->Log("File", 1, "Testing 66");
    logger->Log("File", 1, "Testing 1345");
    logger->Log("File", 1, "Testing 1123124");

    return 0;
}