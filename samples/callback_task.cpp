#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>

#include "cepton_sdk2.h"
using namespace std;

void check_api_error(int err, char const *api) {
  if (err != CEPTON_SUCCESS) {
    cout << "API Error for" << api << ":" << CeptonGetErrorCodeName(err)
         << "\n";
    exit(1);
  }
}

void sensorErrorCallback(CeptonSensorHandle handle, int error_code,
                         const char *error_msg, const void *error_data,
                         size_t error_data_size) {
  cout << "Got error:" << error_msg << "\n";
}

class User {
 public:
  std::mutex mtx;
  std::condition_variable cv;
  int n_frames = 0;

  // Initializing n_frames
  void frameCallback(CeptonSensorHandle handle, int64_t start_timestamp,
                     size_t n_points, size_t stride, const uint8_t *points) {
    cout << "Got " << ++n_frames << "frames:" << (int)n_points << "points\n";
    if (n_frames >= 10) {
      cv.notify_all();
    }
  }

  // Wait
  void wait() {
    std::unique_lock<std::mutex> lck(mtx);
    cout << "waiting..\n";
    cv.wait_for(lck, std::chrono::seconds(1));
  }
};

// reinterpret_cast
void frameCallback(CeptonSensorHandle handle, int64_t start_timestamp,
                   size_t n_points, size_t stride, const uint8_t *points,
                   void *user_data) {
  reinterpret_cast<User *>(user_data)->frameCallback(handle, start_timestamp,
                                                     n_points, stride, points);
}

int main(int argc, char **argv) {
  User user;
  int ret;

  // Initialize
  ret = CeptonInitialize(CEPTON_API_VERSION, sensorErrorCallback);
  check_api_error(ret, "CeptonInitialize");
  cout << "db initialize \n";

  // Start networking listener thread
  ret = CeptonStartNetworking();
  check_api_error(ret, "CeptonStartNetworking");
  cout << "db start networking \n";

  // Listen for frames
  ret =
      CeptonListenFrames(CEPTON_AGGREGATION_MODE_NATURAL, frameCallback, &user);
  check_api_error(ret, "CeptonListenFrames");
  cout << "db listen for frames \n";

  // User::sleep();
  user.wait();

  // Deinitialize
  ret = CeptonDeinitialize();
  check_api_error(ret, "CeptonDeinitialize");
  return 0;
}
