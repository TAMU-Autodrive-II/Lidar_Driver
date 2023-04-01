#include <stdio.h>
#include <stdlib.h>

#include "cepton_sdk2.h"

void check_api_error(int err, char const *api) {
  if (err != CEPTON_SUCCESS) {
    printf("API Error for %s: %s\n", api, CeptonGetErrorCodeName(err));
    exit(1);
  }
}

void sensorErrorCallback(CeptonSensorHandle handle, int error_code,
                         const char *error_msg, const void *error_data,
                         size_t error_data_size) {
  printf("Got error: %s\n", error_msg);
}

int n_frames = 0;

void frameCallback(CeptonSensorHandle handle, int64_t start_timestamp,
                   size_t n_points, size_t stride, const uint8_t *points,
                   void *user_data) {
  printf("Got %d frames: %d points\n", ++n_frames, (int)n_points);
}

int main() {
  int ret;

  // Initialize
  ret = CeptonInitialize(CEPTON_API_VERSION, sensorErrorCallback);
  check_api_error(ret, "CeptonInitialize");

  ret = CeptonEnableLegacyTranslation();
  check_api_error(ret, "EnableLegacyTranslation");

  // Start networking listener thread
  ret = CeptonStartNetworking();
  check_api_error(ret, "CeptonStartNetworking");

  // Listen for frames
  ret = CeptonListenFrames(CEPTON_AGGREGATION_MODE_NATURAL, frameCallback, 0);
  check_api_error(ret, "CeptonListenFrames");

  // Sleep
  while (n_frames < 10)
    ;

  // Deinitialize
  ret = CeptonDeinitialize();
  check_api_error(ret, "CeptonDeinitialize");
  return 0;
}
