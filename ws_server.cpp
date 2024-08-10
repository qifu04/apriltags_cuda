#include <seasocks/PrintfLogger.h>
#include <seasocks/Server.h>
#include <seasocks/StringUtil.h>
#include <seasocks/WebSocket.h>

#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <set>
#include <thread>

using json = nlohmann::json;
using namespace std;

class AprilTagHandler : public seasocks::WebSocket::Handler {
 public:
  AprilTagHandler(std::shared_ptr<seasocks::Server> server) : server_(server) {}

  void onConnect(seasocks::WebSocket* socket) override {
    std::lock_guard<std::mutex> lock(mutex_);
    clients_.insert(socket);
  }

  void onDisconnect(seasocks::WebSocket* socket) override {
    std::lock_guard<std::mutex> lock(mutex_);
    clients_.erase(socket);
  }

  void onData(seasocks::WebSocket* socket, const char* data) override {
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
    server_->execute([this, data] {
      std::lock_guard<std::mutex> lock(mutex_);
      for (auto client : clients_) {
        client->send(data.data(), data.size());
      }
    });
  }

  void readAndSend(const int camera_idx) {
    cout << "Enabling video capture" << endl;
    cv::VideoCapture cap(camera_idx, cv::CAP_V4L);
    if (!cap.isOpened()) {
      cerr << "Couldn't open video capture device" << endl;
      return;
    }
    cap.set(cv::CAP_PROP_CONVERT_RGB, false);
    // cap.set(CAP_PROP_MODE, CV_CAP_MODE_YUYV);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);

    cout << "  " << cap.get(cv::CAP_PROP_FRAME_WIDTH) << "x"
         << cap.get(cv::CAP_PROP_FRAME_HEIGHT) << " @"
         << cap.get(cv::CAP_PROP_FPS) << "FPS" << endl;

    cv::Mat bgr_img, yuyv_img;
    while (running_) {
      cap >> yuyv_img;
      cv::cvtColor(yuyv_img, bgr_img, cv::COLOR_YUV2BGR_YUYV);

      // Encode the image to JPEG
      std::vector<uchar> buffer;
      cv::imencode(".jpg", bgr_img, buffer);

      // Broadcast the image
      broadcast(buffer);
    }
  }

  void stop() { running_ = false; }

 private:
  std::set<seasocks::WebSocket*> clients_;
  std::mutex mutex_;
  std::shared_ptr<seasocks::Server> server_;
  std::atomic<bool> running_{true};
  std::atomic<int> brightness_{50};
  std::atomic<int> exposure_{50};
};

int main(int argc, char* argv[]) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <camera index>" << std::endl;
    return 1;
  }

  int camera_idx = atoi(argv[1]);
  if (camera_idx < 0) {
    std::cerr << "Invalid camera index: " << camera_idx << std::endl;
    return 1;
  }

  auto logger = std::make_shared<seasocks::PrintfLogger>();
  auto server = std::make_shared<seasocks::Server>(logger);

  try {
    auto handler = std::make_shared<AprilTagHandler>(server);
    server->addWebSocketHandler("/ws", handler);

    std::thread read_thread(
        bind(&AprilTagHandler::readAndSend, handler, camera_idx));

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
