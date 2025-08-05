#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "apriltag_gpu.h"
#include "apriltag_utils.h"

extern "C" {
#include "apriltag.h"
}

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " image_path\n";
    return 1;
  }
  const char *tag_family = "tag36h11";

  cv::Mat bgr = cv::imread(argv[1], cv::IMREAD_COLOR);
  if (bgr.empty()) {
    std::cerr << "Failed to load image: " << argv[1] << "\n";
    return 1;
  }
  int width = bgr.cols;
  int height = bgr.rows;
  std::cout << "Image size: " << width << "x" << height << std::endl;
  if (width % 8 || height % 8) {
    std::cerr << "Image dimensions must be multiples of 8\n";
    return 1;
  }

  cv::Mat gray;
  cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
  image_u8_t im{gray.cols, gray.rows, gray.cols, gray.data};

  apriltag_family_t *tf = nullptr;
  if (!setup_tag_family(&tf, tag_family)) {
    std::cerr << "Could not setup tag family\n";
    return 1;
  }
  apriltag_detector_t *td = apriltag_detector_create();
  apriltag_detector_add_family(td, tf);
  td->quad_decimate = 1.0;
  td->quad_sigma = 0.0;
  td->nthreads = 1;
  td->debug = false;
  td->refine_edges = true;
  td->wp = workerpool_create(1);

  auto cpu_start = std::chrono::steady_clock::now();
  zarray_t *cpu_detections = apriltag_detector_detect(td, &im);
  auto cpu_end = std::chrono::steady_clock::now();
  std::cout << "CPU detections: " << zarray_size(cpu_detections) << " time "
            << std::chrono::duration_cast<std::chrono::milliseconds>(cpu_end -
                                                                     cpu_start)
                   .count()
            << " ms" << std::endl;

  cv::Mat yuyv;
  cv::cvtColor(bgr, yuyv, cv::COLOR_BGR2YUV_YUYV);

  frc971::apriltag::CameraMatrix cam{};
  cam.fx = width;
  cam.fy = height;
  cam.cx = width / 2.0;
  cam.cy = height / 2.0;
  frc971::apriltag::DistCoeffs dist{};

  frc971::apriltag::GpuDetector gpu(width, height, td, cam, dist);

  auto gpu_start = std::chrono::steady_clock::now();
  gpu.Detect(yuyv.data);
  auto gpu_end = std::chrono::steady_clock::now();
  const zarray_t *gpu_detections = gpu.Detections();
  std::cout << "GPU detections: " << zarray_size(gpu_detections) << " time "
            << std::chrono::duration_cast<std::chrono::milliseconds>(gpu_end -
                                                                     gpu_start)
                   .count()
            << " ms" << std::endl;

  bool match = zarray_size(cpu_detections) == zarray_size(gpu_detections);
  if (match) {
    for (int i = 0; i < zarray_size(cpu_detections); ++i) {
      apriltag_detection_t *cd;
      apriltag_detection_t *gd;
      zarray_get(cpu_detections, i, &cd);
      zarray_get(gpu_detections, i, &gd);
      if (cd->id != gd->id || std::abs(cd->c[0] - gd->c[0]) > 0.5 ||
          std::abs(cd->c[1] - gd->c[1]) > 0.5) {
        match = false;
        break;
      }
    }
  }
  std::cout << "Results " << (match ? "match" : "do not match") << std::endl;

  apriltag_detections_destroy(cpu_detections);
  apriltag_detector_destroy(td);
  teardown_tag_family(&tf, tag_family);
  return match ? 0 : 2;
}
