// gpu_detector_test.cpp
#include <gtest/gtest.h>

#include "apriltag_gpu.h"
#include "apriltag_utils.h"
#include "opencv2/opencv.hpp"

extern "C" {
#include "apriltag.h"
}

using namespace cv;

// Fixture for the GpuDetector tests
class GpuDetectorTest : public ::testing::Test {
 protected:
  Mat yuyv_img, bgr_img;

  apriltag_family_t *tf = NULL;
  apriltag_detector_t *td = NULL;
  const char *tag_family = "tag36h11";
  frc971::apriltag::CameraMatrix cam;
  frc971::apriltag::DistCoeffs dist;

  void SetUp() override {
    // Read in the image
    bgr_img = cv::imread("../data/colorimage.jpg", cv::IMREAD_COLOR);
    cvtColor(bgr_img, yuyv_img, COLOR_BGR2YUV_YUYV);

    // Setup Tag Family and tag detector
    setup_tag_family(&tf, tag_family);
    td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);

    // Setup Tag Detector
    td->quad_decimate = 2.0;
    td->quad_sigma = 0.0;
    td->nthreads = 1;
    td->debug = false;
    td->refine_edges = true;
    td->wp = workerpool_create(1);

    // Setup Camera Matrix
    cam.fx = 905.495617;
    cam.fy = 907.909470;
    cam.cx = 609.916016;
    cam.cy = 352.682645;

    // Setup Distortion Coefficients
    dist.k1 = 0.059238;
    dist.k2 = -0.075154;
    dist.p1 = -0.003801;
    dist.p2 = 0.001113;
    dist.k3 = 0.0;
  }

  void TearDown() override {
    // Cleanup code here if needed
    teardown_tag_family(&tf, tag_family);
    apriltag_detector_destroy(td);
  }
};

// Test to ensure GpuDetector detects an AprilTag correctly
TEST_F(GpuDetectorTest, GpuDetectsAprilTag) {
  int width = yuyv_img.cols;
  int height = yuyv_img.rows;
  frc971::apriltag::GpuDetector detector(width, height, td, cam, dist);
  detector.Detect(yuyv_img.data);
  const zarray_t *detections = detector.Detections();

  ASSERT_EQ(1, zarray_size(detections));
}

TEST_F(GpuDetectorTest, CpuDetectsAprilTag) {
  Mat gray;
  cvtColor(bgr_img, gray, COLOR_BGR2GRAY);
  image_u8_t im = {gray.cols, gray.rows, gray.cols, gray.data};
  zarray_t *detections = apriltag_detector_detect(td, &im);

  ASSERT_EQ(1, zarray_size(detections));
}

// Main function to run the tests
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
