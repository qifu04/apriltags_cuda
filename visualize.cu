
#include <iomanip>
#include <iostream>

#include "apriltag_gpu.h"
#include "opencv2/opencv.hpp"

extern "C" {
#include "apriltag.h"
#include "common/getopt.h"
#include "common/matd.h"
#include "tag16h5.h"
#include "tag25h9.h"
#include "tag36h11.h"
#include "tagCircle21h7.h"
#include "tagCircle49h12.h"
#include "tagCustom48h12.h"
#include "tagStandard41h12.h"
#include "tagStandard52h13.h"
}

using namespace std;
using namespace cv;

int main(int argc, char *argv[]) {
  TickMeter meter;
  meter.start();

  // Initialize tag detector with options
  apriltag_family_t *tf = NULL;
  const char *famname = "tag36h11";
  if (!strcmp(famname, "tag36h11")) {
    tf = tag36h11_create();
  } else if (!strcmp(famname, "tag25h9")) {
    tf = tag25h9_create();
  } else if (!strcmp(famname, "tag16h5")) {
    tf = tag16h5_create();
  } else if (!strcmp(famname, "tagCircle21h7")) {
    tf = tagCircle21h7_create();
  } else if (!strcmp(famname, "tagCircle49h12")) {
    tf = tagCircle49h12_create();
  } else if (!strcmp(famname, "tagStandard41h12")) {
    tf = tagStandard41h12_create();
  } else if (!strcmp(famname, "tagStandard52h13")) {
    tf = tagStandard52h13_create();
  } else if (!strcmp(famname, "tagCustom48h12")) {
    tf = tagCustom48h12_create();
  } else {
    printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
    exit(-1);
  }

  apriltag_detector_t *td = apriltag_detector_create();
  apriltag_detector_add_family(td, tf);

  if (errno == ENOMEM) {
    printf(
        "Unable to add family to detector due to insufficient memory to "
        "allocate the tag-family decoder with the default maximum hamming "
        "value of 2. Try choosing an alternative tag family.\n");
    exit(-1);
  }

  td->quad_decimate = 2.0;
  td->quad_sigma = 0.0;
  td->nthreads = 1;
  td->debug = 0;
  td->refine_edges = 1;
  td->wp = workerpool_create(td->nthreads);

  meter.stop();
  cout << "Detector " << famname << " initialized in " << std::fixed
       << std::setprecision(3) << meter.getTimeSec() << " seconds" << endl;

  meter.reset();

  frc971::apriltag::CameraMatrix cam;
  cam.fx = 905.495617;
  cam.fy = 907.909470;
  cam.cx = 609.916016;
  cam.cy = 352.682645;

  frc971::apriltag::DistCoeffs dist;
  dist.k1 = 0.059238;
  dist.k2 = -0.075154;
  dist.p1 = -0.003801;
  dist.p2 = 0.001113;
  dist.k3 = 0.0;

  Mat bgr_img, yuyv_img;
  bgr_img = cv::imread("../data/colorimage.jpg", cv::IMREAD_COLOR);
  cvtColor(bgr_img, yuyv_img, COLOR_BGR2YUV_YUYV);
  int width = bgr_img.cols;
  int height = bgr_img.rows;

  // Make an image_u8_t header for the Mat data
  // image_u8_t im = {gray.cols, gray.rows, gray.cols, gray.data};

  // zarray_t *detections = apriltag_detector_detect(td, &im);

  frc971::apriltag::GpuDetector detector(width, height, td, cam, dist);
  detector.Detect(yuyv_img.data);
  const zarray_t *detections = detector.Detections();

  if (errno == EAGAIN) {
    printf("Unable to create the %d threads requested.\n", td->nthreads);
    exit(-1);
  }

  for (int i = 0; i < zarray_size(detections); i++) {
    apriltag_detection_t *det;
    zarray_get(detections, i, &det);
    std::cout << "tag #: " << det->id << std::endl;
    std::cout << "hamming: " << det->hamming << std::endl;
    std::cout << "margin: " << det->decision_margin << std::endl;
    std::cout << "center: " << det->c[0] << "," << det->c[1] << std::endl;
    for (size_t j = 0; j < det->H->ncols; ++j) {
      std::cout << std::endl;
      for (size_t k = 0; k < det->H->nrows; ++k) {
        std::cout << matd_get(det->H, j, k) << " ";
      }
    }
    std::cout << std::endl;
  }

  // Draw detection outlines
  for (int i = 0; i < zarray_size(detections); i++) {
    apriltag_detection_t *det;
    zarray_get(detections, i, &det);
    line(bgr_img, Point(det->p[0][0], det->p[0][1]),
         Point(det->p[1][0], det->p[1][1]), Scalar(0, 0xff, 0), 2);
    line(bgr_img, Point(det->p[0][0], det->p[0][1]),
         Point(det->p[3][0], det->p[3][1]), Scalar(0, 0, 0xff), 2);
    line(bgr_img, Point(det->p[1][0], det->p[1][1]),
         Point(det->p[2][0], det->p[2][1]), Scalar(0xff, 0, 0), 2);
    line(bgr_img, Point(det->p[2][0], det->p[2][1]),
         Point(det->p[3][0], det->p[3][1]), Scalar(0xff, 0, 0), 2);

    stringstream ss;
    ss << det->id;
    String text = ss.str();
    int fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
    double fontscale = 1.0;
    int baseline;
    Size textsize = getTextSize(text, fontface, fontscale, 2, &baseline);
    putText(
        bgr_img, text,
        Point(det->c[0] - textsize.width / 2, det->c[1] + textsize.height / 2),
        fontface, fontscale, Scalar(0xff, 0x99, 0), 2);
  }
  // apriltag_detections_destroy(detections);

  imshow("Tag Detections", bgr_img);
  waitKey(0);

  cv::Mat gray_cuda(cv::Size(width, height), CV_8UC1);
  detector.CopyGrayTo(gray_cuda.data);
  imshow("gray_cuda", gray_cuda);
  waitKey(0);

  cv::Mat decimated_cuda(gray_cuda.size() / 2, CV_8UC1);
  detector.CopyDecimatedTo(decimated_cuda.data);
  imshow("decimated_cuda", decimated_cuda);
  waitKey(0);

  cv::Mat thresholded_cuda(decimated_cuda.size(), CV_8UC1);
  detector.CopyThresholdedTo(thresholded_cuda.data);
  imshow("thresholded_cuda", thresholded_cuda);
  waitKey(0);

  apriltag_detector_destroy(td);

  if (!strcmp(famname, "tag36h11")) {
    tag36h11_destroy(tf);
  } else if (!strcmp(famname, "tag25h9")) {
    tag25h9_destroy(tf);
  } else if (!strcmp(famname, "tag16h5")) {
    tag16h5_destroy(tf);
  } else if (!strcmp(famname, "tagCircle21h7")) {
    tagCircle21h7_destroy(tf);
  } else if (!strcmp(famname, "tagCircle49h12")) {
    tagCircle49h12_destroy(tf);
  } else if (!strcmp(famname, "tagStandard41h12")) {
    tagStandard41h12_destroy(tf);
  } else if (!strcmp(famname, "tagStandard52h13")) {
    tagStandard52h13_destroy(tf);
  } else if (!strcmp(famname, "tagCustom48h12")) {
    tagCustom48h12_destroy(tf);
  }

  return 0;
}
