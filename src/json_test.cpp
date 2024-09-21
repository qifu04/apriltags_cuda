#include <stdio.h>

#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;
int main(void) {
  std::ifstream f("/home/nvidia/code/OrinVisionSystem/calibrationmatrix.json");
  json data = json::parse(f);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      std::cout << data["matrix"][i][j] << " ";
    }
    std::cout << std::endl;
  }
  return 0;
}