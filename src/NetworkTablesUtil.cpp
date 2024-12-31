#include "NetworkTablesUtil.h"

#include "NetworkTablesConfig.h"

NetworkTablesUtil::NetworkTablesUtil() {
  inst_ = nt::NetworkTableInstance::GetDefault();
  inst_.SetServer(TABLE_ADDRESS);
  inst_.StartClient4(TABLE_ADDRESS);
}

double NetworkTablesUtil::getTime() {
  return inst_.WPI_GetSystemTime();
}

// Class use example

// int main(){
//   DoubleValueSender sender("NVIDIA ORIN TEST");
//   while(2>1){
//     sender.sendValue(1.0);
//     std::cout << "Sent value" << std::endl;
//   }
//   return 0;
// }