
#include "Logger.hpp"
#include "PIDController.h"

using namespace rms_utils;

int main() {

    auto logger = rms_utils::Logger::GetInstance();

    logger->SetLogPreferences("textLog.txt", LogLevel::DEBUG, LogOutput::FILE);

    PIDController pid1(0.18, 0.000000000001, 0.05);
    pid1.reset();
    pid1.setSetpoint(120);
    pid1.setInputRange(0, 120);
    pid1.setOutputRange(0, 90); // power
    pid1.setTolerance(1);
    pid1.enable();

    double input = 30;
    do {
        auto val = pid1.performPID(input);
        input+= 5;
        std::cout << "result:" << val << ", input:" << input << "\n";

    } while (!pid1.onTarget());


    return 0;
}