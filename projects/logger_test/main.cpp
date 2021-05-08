// Copyright 2021 UBC Sailbot

#include "Connection.h"
#include "Logger.h"

int main() {
    std::string file_path = "../logs/logger_1.log";
    NetworkTable::Logger logger(file_path);
    logger.Trace("Trace from test client 1\n");
    logger.Debug("Debug from test client 1\n");
    logger.Info("Info from test client 1\n");
    logger.Warning("Warning from test client 1\n");
    logger.Error("Error from test client 1\n");
    logger.Fatal("Fatal from test client 1\n");
    return 0;
}

