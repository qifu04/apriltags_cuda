#include <seasocks/Server.h>
#include <seasocks/WebSocket.h>
#include <seasocks/StringUtil.h>
#include <seasocks/PrintfLogger.h>
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <memory>
#include <set>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>

using json = nlohmann::json;

class AprilTagHandler : public seasocks::WebSocket::Handler {
public:
    AprilTagHandler(const std::string& imagePath, std::shared_ptr<seasocks::Server> server)
        : imagePath_(imagePath), server_(server) {
        // Load the image once to check if it's valid
        cv::Mat testImage = cv::imread(imagePath_);
        if (testImage.empty()) {
            throw std::runtime_error("Could not open image file: " + imagePath_);
        }
    }

    void onConnect(seasocks::WebSocket *socket) override {
        std::lock_guard<std::mutex> lock(mutex_);
        clients_.insert(socket);
    }

    void onDisconnect(seasocks::WebSocket *socket) override {
        std::lock_guard<std::mutex> lock(mutex_);
        clients_.erase(socket);
    }

    void onData(seasocks::WebSocket *socket, const char *data) override {
        try {
            auto j = json::parse(data);
            if (j["type"] == "control") {
                if (j.contains("brightness")) {
                    brightness_ = j["brightness"].get<int>();
                }
                if (j.contains("exposure")) {
                    exposure_ = j["exposure"].get<int>();
                }
                std::cout << "Received settings - Brightness: " << brightness_ 
                          << ", Exposure: " << exposure_ << std::endl;
            }
        } catch (const json::parse_error& e) {
            std::cerr << "JSON parse error: " << e.what() << std::endl;
        }
    }

    void broadcast(const std::vector<uint8_t>& data) {
        server_->execute([this, data]{
            std::lock_guard<std::mutex> lock(mutex_);
            for (auto client : clients_) {
                client->send(data.data(), data.size());
            }
        });
    }

    void readAndSend() {
        while (running_) {
            cv::Mat frame = cv::imread(imagePath_);
            if (frame.empty()) {
                std::cerr << "Error: Could not read image file." << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }

            // Apply brightness (simulated)
            frame.convertTo(frame, -1, 1, brightness_ - 50);

            // Simulate AprilTag detection
            cv::rectangle(frame, cv::Point(100, 100), cv::Point(200, 200), cv::Scalar(0, 255, 0), 2);

            // Encode the image to JPEG
            std::vector<uchar> buffer;
            cv::imencode(".jpg", frame, buffer);

            // Broadcast the image
            broadcast(buffer);

            std::this_thread::sleep_for(std::chrono::milliseconds(33)); // ~30 fps
        }
    }

    void stop() { running_ = false; }

private:
    std::set<seasocks::WebSocket*> clients_;
    std::mutex mutex_;
    std::string imagePath_;
    std::shared_ptr<seasocks::Server> server_;
    std::atomic<bool> running_{true};
    std::atomic<int> brightness_{50};
    std::atomic<int> exposure_{50};
};

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <path_to_image_file>" << std::endl;
        return 1;
    }

    auto logger = std::make_shared<seasocks::PrintfLogger>();
    auto server = std::make_shared<seasocks::Server>(logger);

    try {
        auto handler = std::make_shared<AprilTagHandler>(argv[1], server);
        server->addWebSocketHandler("/ws", handler);

        std::thread read_thread(&AprilTagHandler::readAndSend, handler);

        server->serve("", 8080);

        handler->stop();
        if (read_thread.joinable()) {
            read_thread.join();
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}



