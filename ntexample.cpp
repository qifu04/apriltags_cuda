#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

#include <iostream>

int main() {
  // Initialize NetworkTables
  auto ntinst = nt::NetworkTableInstance::GetDefault();

  // Start client
  ntinst.StartClient3("localhost");  // Replace "localhost" with the IP address
                                     // of your server if needed

  // Get the table and key
  auto table = ntinst.GetTable("datatable");
  nt::NetworkTableEntry entry = table->GetEntry("myKey");

  // Retrieve value (assuming it is a double, change as needed)
  double value = entry.GetDouble(0.0);

  // Print the value
  std::cout << "Value from NetworkTables: " << value << std::endl;

  return 0;
}
