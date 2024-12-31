#ifndef NETWORKTABLESUTIL_H
#define NETWORKTABLESUTIL_H


#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/Timer.h"

class NetworkTablesUtil {
 private:
  nt::NetworkTableInstance inst_;
  nt::Timer timer_;

 public:
  // Constructor declaration
  NetworkTablesUtil();

  // Method declarations
  double getTime();
};

#endif