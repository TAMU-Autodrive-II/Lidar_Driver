#include <cepton_sdk2.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>

char global_buffer[2000];

/* Networking code */

int create_socket() {
  int socket_desc = socket(AF_INET, SOCK_DGRAM, 0);

  if (socket_desc == -1) {
    printf("Could not create UDP socket\n");
    exit(-1);
  }

  return socket_desc;
}

int connect_to_sensor() {
  int socket_desc;
  struct sockaddr_in sensor_addr;

  sensor_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  sensor_addr.sin_family = AF_INET;
  sensor_addr.sin_port = htons(8808);

  socket_desc = create_socket();

  if (bind(socket_desc, (struct sockaddr *)&sensor_addr, sizeof(sensor_addr)) <
      0) {
    printf("Bind error\n");
    exit(-1);
  }

  return socket_desc;
}

int receive_udp_data(int socket_desc, unsigned long *from) {
  int received;
  struct sockaddr_in from_addr;
  socklen_t addrlen = sizeof(from_addr);
  received = recvfrom(socket_desc, global_buffer, sizeof(global_buffer), 0,
                      (struct sockaddr *)&from_addr, &addrlen);
  if (received < 0) {
    printf("Recv failed\n");
    return 0;
  }
  *from = htonl(from_addr.sin_addr.s_addr);
  return received;
}

/* Get high precision time */
long long get_hp_time() {
  struct timespec time;
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time);
  return time.tv_sec * 1000000 +
         (time.tv_nsec + 500) / 1000;  // convert to microseconds
}

void check_api_error(int err, char const *api) {
  if (err != CEPTON_SUCCESS) {
    printf("API Error for %s: %s\n", api, CeptonGetErrorCodeName(err));
    exit(1);
  }
}

int n_frames = 0;

void frameCallback(CeptonSensorHandle handle, int64_t start_timestamp,
                   size_t n_points, size_t stride, const uint8_t *points,
                   void *user_data) {
  printf("Got %d frames\n", ++n_frames);
}

void sensorErrorCallback(CeptonSensorHandle handle, int error_code,
                         const char *error_msg, const void *error_data,
                         size_t error_data_size) {
  printf("Got error: %s\n", error_msg);
}

int main() {
  int ret;
  int received;
  int socket_desc = connect_to_sensor();

  // Initialize cepton SDK
  ret = CeptonInitialize(CEPTON_API_VERSION, sensorErrorCallback);
  check_api_error(ret, "CeptonInitialize");

  // Listen for frames
  ret = CeptonListenFrames(CEPTON_AGGREGATION_MODE_NATURAL, frameCallback, 0);
  check_api_error(ret, "CeptonListenFrames");

  // Use direct networking
  while (n_frames < 10) {
    unsigned long handle;
    received = receive_udp_data(socket_desc, &handle);
    if (received <= 0) break;
    ret = CeptonReceiveData(handle, get_hp_time(),
                            (const uint8_t *)global_buffer, received);
    check_api_error(ret, "CeptonReceiveData");
  }

  // Deinitialize
  ret = CeptonDeinitialize();
  check_api_error(ret, "CeptonDeinitialize");
  return 0;
}
