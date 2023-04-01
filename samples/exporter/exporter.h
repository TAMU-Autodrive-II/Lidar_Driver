#pragma once

#define NOMINMAX

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <fstream>
#include <iostream>
#include <mutex>

#if defined(__cplusplus) && __cplusplus >= 201703L && defined(__has_include)
#if __has_include(<filesystem>)
#define GHC_USE_STD_FS
#include <filesystem>
namespace fs {
using namespace std::filesystem;
using ifstream = std::ifstream;
using ofstream = std::ofstream;
using fstream = std::fstream;
}  // namespace fs
#endif
#endif
#ifndef GHC_USE_STD_FS
#include <ghc/filesystem.hpp>
namespace fs {
using namespace ghc::filesystem;
using ifstream = ghc::filesystem::ifstream;
using ofstream = ghc::filesystem::ofstream;
using fstream = ghc::filesystem::fstream;
}  // namespace fs
#endif

#include "cepton_sdk2.h"
#include "cxxopts.hpp"

using namespace std;

class CeptonExporter {
 public:
  CeptonExporter();
  ~CeptonExporter();

  int InitializeSDK();
  int CleanupSDK();
  int ParseOptions(int argc, char **argv);

 private:
  static void ReportError(int err, string when, bool quit = false) {
    string err_str = CeptonGetErrorCodeName(err);
    ReportError(err_str, when, quit);
  }

  static void ReportError(string err_str, string when = "", bool quit = false) {
    if (when.empty())
      cout << "ERROR: " << err_str << endl;
    else
      cout << "ERROR: (" << when << ") " << err_str << endl;
    if (quit) exit(-1);
  }

  cxxopts::Options options;
  bool sdk_initialized = false;

  string base_dir = ".";
  fs::path capture_file;
  CeptonReplayHandle capture_handle = nullptr;
  bool streaming = false;
  unsigned frame_count = 5;
  uint64_t target_sn = 0;
  bool include_invalid = false;
  bool single_file = false;
  bool split_timestamp = false;
  string output_name = "";
  int64_t file_ts = 0;
  int aggregation_mode = CEPTON_AGGREGATION_MODE_NATURAL;

  unsigned completed_sensors = 0;
  bool got_frame = false;

  enum class Format { UNKNOWN = 0, CSV, CR };

  Format format = Format::UNKNOWN;

  struct SensorData {
    uint64_t serial_number;
    unsigned frames_recorded;
    uint64_t last_ts;
  };

  map<CeptonSensorHandle, SensorData> sensor;
  mutable mutex waiter_mutex;
  mutable condition_variable waiter;

  void FrameData(CeptonSensorHandle handle, int64_t timestamp, size_t n_points,
                 size_t stride, uint8_t const *c_points);
  static void PrintVersion(const string &fw_name, uint32_t fw_version);
  void SensorInfo(CeptonSensorHandle handle, const struct CeptonSensor *info);
  void WriteCSVFrame(SensorData &sd, int64_t timestamp, size_t size,
                     size_t stride, uint8_t const *points);
  void WriteCRFrame(SensorData &sd, int64_t timestamp, size_t size,
                    size_t stride, uint8_t const *points);

  static void CbCeptonSensorImageData(CeptonSensorHandle handle,
                                      int64_t timestamp, size_t n_points,
                                      size_t stride, uint8_t const *c_points,
                                      void *user_data);
  static void CbCeptonSensorInfo(CeptonSensorHandle handle,
                                 const struct CeptonSensor *info,
                                 void *user_data);
};

class CsvExporter {
 private:
  shared_ptr<ostream> export_stream;
  bool split_timestamp = false;

  void ExportPoint(CeptonPoint const &p, int64_t timestamp);

 public:
  int Export(uint8_t const *points, int64_t timestamp, size_t size,
             size_t stride, bool include_invalid, bool append);

  // Subclass interfaces (with default implementations)
  int OpenFile(fs::path &file_name, bool append = false) {
    auto file_stream = make_shared<fs::ofstream>();
    auto openmode = ios_base::out | ios_base::binary | ios_base::trunc;
    if (append) openmode = ios_base::out | ios_base::binary | ios_base::app;
    file_stream->open(file_name, openmode);
    if (!file_stream->bad()) export_stream = file_stream;
    return file_stream->bad() ? -1 : 0;
  }

  void SetSplitTimestamp(bool st) { split_timestamp = st; }

  int CloseFile() {
    if (export_stream) export_stream->flush();
    export_stream.reset();
    return 0;
  }
};
