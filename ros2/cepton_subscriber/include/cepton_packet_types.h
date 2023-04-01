#pragma once
typedef uint64_t CeptonSensorHandle;

// Cepton Point flags, copied from cepton_sdk2.h
enum {
  CEPTON_POINT_SATURATED = 1 << 0,
  CEPTON_POINT_LOW_SNR = 1 << 1,
  CEPTON_POINT_FRAME_PARITY = 1 << 2,
  CEPTON_POINT_FRAME_BOUNDARY = 1 << 3,
  CEPTON_POINT_SECOND_RETURN = 1 << 4,
  CEPTON_POINT_NO_RETURN = 1 << 5,
  CEPTON_POINT_NOISE = 1 << 6,
};

#pragma pack(push, 1)
// Cepton Point structure, copied from cepton_sdk2.h
struct CeptonPoint {
  int16_t x;
  uint16_t y;
  int16_t z;
  uint8_t reflectivity;
  uint8_t relative_timestamp;
  uint8_t channel_id;
  uint8_t flags;
};

// Cepton Info structure, copied from cepton_sdk2.h
struct CeptonSensor {
  /// Size of the CeptonSensor struct
  /// plus any consecutive sensor info blocks
  uint32_t info_size;

  // per sensor info
  uint32_t serial_number;
  CeptonSensorHandle handle;

  // Model
  char model_name[28];
  uint16_t model;
  uint16_t model_reserved;
  uint32_t part_number;

  // FW
  uint32_t firmware_version;  // LSB->MSB major/minor/build/patch

  // Time
  int64_t power_up_timestamp;
  int64_t time_sync_offset;
  int64_t time_sync_drift;

  /// Config
  uint8_t return_count;
  uint8_t channel_count;
  uint8_t reserved[2];
  uint32_t status_flags;

  // Unit in 0.01 Kelvin
  uint16_t temperature;
};
#pragma pack(pop)
