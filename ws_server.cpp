#include <seasocks/Server.h>
#include <seasocks/WebSocket.h>
#include <seasocks/Response.h>
#include <seasocks/PrintfLogger.h>
#include <iostream>
#include <vector>
#include <memory>
#include <filesystem>
#include "base64.h"

using namespace seasocks;
namespace fs = std::filesystem;

class ImageWebSocket : public WebSocket::Handler {
public:
    ImageWebSocket(Server* server) : server_(server) {}

    void onConnect(WebSocket* connection) override {
        std::cout << "Client connected" << std::endl;
    }

    void onData(WebSocket* connection, const uint8_t* data, size_t length) override {
        std::vector<uint8_t> imageData(data, data + length);
        std::cout << "Received data of size: " << length << std::endl;
        std::string base64Image = base64_encode(imageData.data(), imageData.size());
        std::string imageHtml = "<img src='data:image/jpeg;base64," + base64Image + "' />";
        connection->send(imageHtml);
    }

    void onDisconnect(WebSocket* connection) override {
        std::cout << "Client disconnected" << std::endl;
    }

private:
    Server* server_;
};

int main() {
    auto logger = std::make_shared<PrintfLogger>(Logger::Level::Debug);


    Server server(logger);

    auto handler = std::make_shared<ImageWebSocket>(&server);
    server.addWebSocketHandler("/ws", handler);

    server.serve("web", 9090);
    return 0;
}
