#include "capture.hpp"
#include "detection.hpp"
#include "common.hpp"
#include "preprocess.hpp"

int main() {
  bool AIFlag = false;
  auto cap = std::make_shared<Capture>(0);
  auto detection = std::make_shared<Detection>();
  detection->score = 0.5;
  std::future<void> future;
  std::vector<PredictResult> AIresults;
  cap->open();

  while (true) {

    cv::Mat img = cap->read();
    auto start = std::chrono::steady_clock::now();
    if (!future.valid()) {

      future = std::async(std::launch::async,
                          std::bind(&Detection::inference, detection, img));
    }
    if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      AIresults = detection->results;
      AIFlag = true;
      future = std::async(
          std::launch::async,
          [&detection](const cv::Mat &image) {
            detection->inference(image);
          },
          img);
    } else {
      AIFlag = false;
    }
    detection->drawBox(img);
    auto end = std::chrono::steady_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    // std::cout << duration.count() << std::endl;
    cv::imshow("aa", img);
    cv::waitKey(20);
  }

  return 0;
}
