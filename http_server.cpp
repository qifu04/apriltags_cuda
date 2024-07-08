#include "httplib.h"

int main() {
  // Create an HTTP server
  httplib::Server svr;

  // Define a GET endpoint /hi that responds with "Hello World"
  svr.Get("/hi", [](const httplib::Request& req, httplib::Response& res) {
    res.set_content("Hello World", "text/plain");
  });

  // Start the server on port 8080
  svr.listen("0.0.0.0", 8080);

  return 0;
}
