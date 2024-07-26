#include <seasocks/Client.h>
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace seasocks;
using namespace cv;

class ImageClient : public WebSocket::Client {
public:
    void onConnect(WebSocket* ws) override {
        std::cout << "Connected to server" << std::endl;
        std::vector<uint8_t> image = readImage("image.jpg");
        if (!image.empty()) {
            ws->sendBinary(reinterpret_cast<const char*>(image.data()), image.size());
        }
    }

    void onDisconnect(WebSocket* ws) override {
        std::cout << "Disconnected from server" << std::endl;
    }

    void onData(WebSocket* ws, const char* data, size_t length) override {
        std::cout << "Received data of size: " << length << std::endl;
    }

private:
    std::vector<uint8_t> readImage(const std::string& path) {
        Mat img = imread(path, IMREAD_COLOR);
        if (img.empty()) {
            std::cerr << "Could not read the image: " << path << std::endl;
            return {};
        }
        std::vector<uint8_t> buf;
        imencode(".jpg", img, buf);
        return buf;
    }
};

int main() {
    Client client;
    auto clientSock = client.connect("ws://localhost:9090/ws");
    auto imgClient = std::make_shared<ImageClient>();
    clientSock->addClient(imgClient);
    client.run();
    return 0;
}
