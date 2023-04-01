#include "exporter.h"

#include <cctype>
#include <regex>

#include "cr_exporter.h"

// https://stackoverflow.com/questions/874134/find-out-if-string-ends-with-another-string-in-c
bool hasEnding(std::string const &fullString, std::string const &ending) {
  if (fullString.length() >= ending.length()) {
    return (0 == fullString.compare(fullString.length() - ending.length(),
                                    ending.length(), ending));
  } 
  return false;
}

void CeptonExporter::WriteCRFrame(SensorData &sd, int64_t timestamp,
                                  size_t size, size_t stride,
                                  uint8_t const *points) {
  string fname = output_name;
  if (fname.empty()) {
    CeptonPoint const &p0 = *reinterpret_cast<CeptonPoint const *>(points);
    file_ts = timestamp + p0.relative_timestamp;
    fname = to_string(file_ts) + ".cr";
  }
  CrExporter exp;
  exp.OpenFile(fname);
  exp.ExportFrameXYZI_Distance(points, timestamp, size, stride);
  exp.CloseFile();
}

void CeptonExporter::WriteCSVFrame(SensorData &sd, int64_t timestamp,
                                   size_t size, size_t stride,
                                   uint8_t const *points) {
  CsvExporter exp;
  exp.SetSplitTimestamp(split_timestamp);
  fs::path p = fs::absolute(base_dir);
  p.append(to_string(sd.serial_number));
  if (!fs::create_directories(p)) {
    cout << "ERROR: Failed to create directory " << p.string() << endl;
  }
  bool append = single_file && file_ts != 0;
  CeptonPoint const &p0 = *reinterpret_cast<CeptonPoint const *>(points);
  if (!append) {
    if (include_invalid)
      file_ts = timestamp + p0.relative_timestamp;
    else {
      size_t i;
      file_ts = timestamp;
      for (i = 0; i < size; i++) {
        CeptonPoint const &pi =
            *reinterpret_cast<CeptonPoint const *>(points + i * stride);
        file_ts += pi.relative_timestamp;
        if ((pi.flags & CEPTON_POINT_NO_RETURN) == 0) break;
      }
    }
  }

  if (output_name.empty()) {
    p.append(to_string(file_ts) + ".csv");
  } else {
    if (single_file) {
      p.append(output_name);
      // Add csv extension if not exist
      if (!p.has_extension()) {
        p.replace_extension("csv");
      }
    } else {
      p.append(output_name);
      // Create folder if not exist
      if (!fs::create_directories(p)) {
        cout << "ERROR: Failed to create directory " << p.string() << endl;
      }
      p.append(to_string(file_ts) + ".csv");
    }
  }

  if (!single_file)
    cout << "Writing Frame: (" << sd.frames_recorded + 1 << "/" << frame_count
         << ") " << p.string() << endl;
  else if (!append)
    cout << "Writing single file: " << p.string() << endl;
  exp.OpenFile(p, append);
  exp.Export(points, timestamp, size, stride, include_invalid, append);
  exp.CloseFile();

  sd.frames_recorded++;
  if (!capture_file.empty() && CeptonReplayIsFinished(capture_handle)) {
    waiter.notify_one();
  }
  if (!streaming && sd.frames_recorded >= frame_count) {
    unique_lock<mutex> lock(waiter_mutex);
    completed_sensors++;
    if (sensor.size() == completed_sensors) {
      waiter.notify_one();
    }
  }
}

int parseAggregationMode(const string &m) {
  static const regex hzMode(R"(^\s*(\d+(\.\d+)?)hz\s*$)", regex::icase);
  static const regex msMode(R"(^\s*(\d+(\.\d+)?)ms\s*$)", regex::icase);
  std::smatch cm;
  if (regex_match(m, cm, hzMode)) {
    auto hz = stod(cm[0]);
    return (int)(1000000 / hz);
  }
  if (regex_match(m, cm, msMode)) {
    auto ms = stod(cm[0]);
    return (int)(1000 * ms);
  }
  return -1;
}

string GetOptionsHelp() {
  return "Cepton Exporter Tool (version " + to_string(VERSION_MAJOR) + '.' +
         to_string(VERSION_MINOR) + '.' + to_string(VERSION_BUILD) + ")";
}

CeptonExporter::CeptonExporter()
    : options("cepton_exporter", GetOptionsHelp()) {
  options.add_options()
      // clang-format off
    ("s,serial-number", "Target sensor serial number, dump all connected "
      "sensors if unspecified", cxxopts::value<uint64_t>(), "")
    ("format", "Output file format. Supported CR/CSV. If output file "
      "specified with -o has .cr extension, default to CR files, otherwise "
      "default to CSV", cxxopts::value<string>(), "")
    ("b,base-dir", "Base folder to store frame data",
      cxxopts::value<string>(), "")
    ("n,frames", "Number of frames to dump, default 5. For PCAPs, use 0 "
      "to mean all frames.", cxxopts::value<unsigned>(), "")
    ("f,fixed-frame-size", "Use fixed frame size, e.g. '40hz', '25ms'",
      cxxopts::value<string>(), "")
    ("c,capture", "Specify pcap file to read from", cxxopts::value<string>(), "")
    ("o,output", "Specify csv file name", cxxopts::value<string>(), "")
    ("all-points", "Export all points, including the invalid ones. CSV only.")
    ("single-file", "Export points to a single file")
    ("split-timestamp", "Split timestamp into seconds and microsecond parts. CSV only.")
    ("overwrite", "Force overwrite of exiting output")
    ("h,help", "Print Help")
      // clang-format on
      ;

  // options.add_options("internal")
  //   // clang-format off
  //   // clang-format on
  //   ;
}

CeptonExporter::~CeptonExporter() {
  if (sdk_initialized) {
    int ret = CeptonDeinitialize();
    if (ret != CEPTON_SUCCESS) ReportError(ret, "CeptonDeinitialize");
    sdk_initialized = false;
  }
}

void CeptonExporter::CbCeptonSensorImageData(CeptonSensorHandle handle,
                                             int64_t timestamp, size_t n_points,
                                             size_t stride,
                                             uint8_t const *c_points,
                                             void *user_data) {
  reinterpret_cast<CeptonExporter *>(user_data)->FrameData(
      handle, timestamp, n_points, stride, c_points);
}

void CeptonExporter::CbCeptonSensorInfo(CeptonSensorHandle handle,
                                        const struct CeptonSensor *info,
                                        void *user_data) {
  reinterpret_cast<CeptonExporter *>(user_data)->SensorInfo(handle, info);
}

void CeptonExporter::FrameData(CeptonSensorHandle handle, int64_t timestamp,
                               size_t n_points, size_t stride,
                               uint8_t const *c_points) {
  if (sensor.count(handle)) {
    got_frame = true;
    auto &sd = sensor[handle];
    if (streaming || sd.frames_recorded < frame_count) {
      if (format == Format::CSV)
        WriteCSVFrame(sd, timestamp, n_points, stride, c_points);
      else if (format == Format::CR)
        WriteCRFrame(sd, timestamp, n_points, stride, c_points);
    } else {
      // Do nothing
    }
  }
}

void CeptonExporter::PrintVersion(const string &fw_name, uint32_t fw_version) {
  uint8_t major = fw_version & 0xFF;
  uint8_t minor = (fw_version >> 8) & 0xFF;
  uint8_t build = (fw_version >> 16) & 0xFF;
  uint8_t patch = (fw_version >> 24) & 0xFF;
  cout << fw_name << " FW version #" << to_string(major) << "."
       << to_string(minor) << "." << to_string(build) << "." << to_string(patch)
       << endl;
}

void CeptonExporter::SensorInfo(CeptonSensorHandle handle,
                                const struct CeptonSensor *info) {
  if (target_sn && target_sn != info->serial_number) return;
  if (sensor.count(handle)) return;  // No need to update if we already have it
  cout << "Detected sensor #" << to_string(info->serial_number) << endl;

  // Special action to roll back the replay for first sensor detect
  if (sensor.empty() && !capture_file.empty()) {
    CeptonReplaySeek(capture_handle, 0);
  }
  SensorData sd = {info->serial_number, 0, 0};
  sensor.emplace(handle, sd);
  waiter.notify_one();
}

void CbCeptonSensorError(CeptonSensorHandle handle, int error_code,
                         const char *error_msg, const void *error_data,
                         size_t error_data_size) {
  cout << "ERROR (Sensor): " << error_msg << endl;
}

int CeptonExporter::InitializeSDK() {
  int err;
  err = CeptonInitialize(CEPTON_API_VERSION, CbCeptonSensorError);
  if (err != CEPTON_SUCCESS) ReportError(err, "CeptonInitialize", true);
  sdk_initialized = true;

  // Enable legacy (even if this fails it is fine)
  CeptonEnableLegacyTranslation();

  // Listen to point data
  if (streaming) {
    err = CeptonListenPoints(CbCeptonSensorImageData, this);
    if (err != CEPTON_SUCCESS) ReportError(err, "CeptonListenPoints", true);
  } else {
    err = CeptonListenFrames(aggregation_mode, CbCeptonSensorImageData, this);
    if (err != CEPTON_SUCCESS) ReportError(err, "CeptonListenFrames", true);
  }

  // Listen to sensor detection
  CeptonListenSensorInfo(CbCeptonSensorInfo, this);

  if (!capture_file.empty()) {
    // err = CeptonStartReplay(capture_file.c_str(), 0, 100);
    err = CeptonReplayLoadPcap(capture_file.c_str(), 0, &capture_handle);
    if (err != CEPTON_SUCCESS) ReportError(err, "CeptonReplayLoadPcap", true);
    err = CeptonReplaySetSpeed(capture_handle, 0);  // No delay replay
    if (err != CEPTON_SUCCESS) ReportError(err, "CeptonReplaySetSpeed", true);
    err = CeptonReplayPlay(capture_handle);
    if (err != CEPTON_SUCCESS) ReportError(err, "CeptonStartReplay", true);
  } else {
    err = CeptonStartNetworking();
    if (err != CEPTON_SUCCESS) ReportError(err, "CeptonStartNetworking", true);
  }
  return 0;
}

int CeptonExporter::CleanupSDK() {
  int ret;
  if (!capture_file.empty()) {
    ret = CeptonReplayUnloadPcap(capture_handle);
    if (ret != CEPTON_SUCCESS) ReportError(ret, "CeptonStopReplay", true);
  } else {
    ret = CeptonStopNetworking();
    if (ret != CEPTON_SUCCESS) ReportError(ret, "CeptonStopNetworking", true);
  }

  if (streaming) {
    ret = CeptonUnlistenPoints(CbCeptonSensorImageData, this);
    if (ret != CEPTON_SUCCESS) ReportError(ret, "CeptonUnlistenPoints", true);
  } else {
    ret = CeptonUnlistenFrames(CbCeptonSensorImageData, this);
    if (ret != CEPTON_SUCCESS) ReportError(ret, "CeptonUnlistenFrames", true);
  }
  ret = CeptonDeinitialize();
  if (ret != CEPTON_SUCCESS) ReportError(ret, "CeptonDeinitialize", true);
  sdk_initialized = false;
  return 0;
}

void PrintHelp(cxxopts::Options &options) {
  cout << options.help({"", "Group"});
  cout << R"NOTES(Notes:
  By default, points are stored in <base-dir>/<serial-number>/<timestamp>.csv
  where timestamp is the microsecond value of linux epoch time. Only full
  frames are dumped. In --single-file mode, points are stored in
  <base-dir>/<serial-number>/<timestamp>.csv
)NOTES" << endl;
  cout << R"HELP(Examples:
  # Dump all sensors 10 frames each to c:/tmp/<serial_number>/<timestamp>.csv
  cepton_exporter -n 10 --base-dir=c:/tmp

  # Dump 5 frames sensor #8883 to ./8883/<timestamp>.csv
  cepton_exporter -s 8883

  # Use fixed frame size instead of the "natural" frame boundaries
  cepton_exporter -n 5 -f 40Hz
  cepton_exporter -n 10 -f 25ms

  # Convert .pcap file to .cr file
  cepton_exporter -c input.pcap -o output.cr
)HELP" << endl;
}

int CeptonExporter::ParseOptions(int argc, char **argv) {
  if (argc == 1) {
    PrintHelp(options);
    return 0;
  }

  try {
    cxxopts::ParseResult result = options.parse(argc, argv);

    for (int i = 1; i < argc; i++) {
      ReportError("Extra argument ignored: " + string(argv[i]));
    }

    if (result.count("help")) {
      PrintHelp(options);
      return 0;
    }

    if (result.count("serial-number")) {
      target_sn = result["serial-number"].as<uint64_t>();
    } else {
      target_sn = 0;
    }

    if (result.count("base-dir")) {
      base_dir = result["base-dir"].as<string>();
    } else {
      base_dir = ".";
    }

    if (result.count("format")) {
      auto fmt = result["format"].as<string>();
      transform(fmt.begin(), fmt.end(), fmt.begin(),
                [](unsigned char c) { return std::toupper(c); });
      if (fmt == "CSV")
        format = Format::CSV;
      else if (fmt == "CR")
        format = Format::CR;
      else {
        ReportError("Unknonw format: ", fmt);
      }
    }

    if (result.count("output")) {
      output_name = result["output"].as<string>();
      if (fs::exists(output_name)) {
        if (result.count("overwrite")) {
          cout << "Replacing existing file: " << output_name << endl;
          fs::remove(output_name);
        } else {
          ReportError("Output file exists. Use --overwrite to force", "", true);
        }
      }
    } else {
      output_name = "";
    }

    if (format == Format::UNKNOWN) {
      if (hasEnding(output_name, "cr"))
        format = Format::CR;
      else
        format = Format::CSV;
    }

    if (result.count("frames")) {
      frame_count = result["frames"].as<unsigned>();
    } else {
      frame_count = 5;
    }

    if (result.count("fixed-frame-size")) {
      string input = result["fixed-frame-size"].as<string>();
      aggregation_mode = parseAggregationMode(input);
      if (aggregation_mode == -1) {
        ReportError(
            "Invalid fixed_frame_size, expect unit in hz or ms: " + input, "",
            true);
      }
    }

    if (result.count("capture")) {
      capture_file = fs::absolute(result["capture"].as<string>());
      if (!fs::exists(capture_file)) {
        ReportError("Capture file doesn't exist " + capture_file.string(), "",
                    true);
      }
    }

    if (result.count("single-file")) {
      if (capture_file.empty()) {
        ReportError("--single-file can only be used when loading captures.", "",
                    true);
      }
      if (!result.count("frames")) {
        streaming = true;
      }
      single_file = true;
    }

    if (result.count("split-timestamp")) {
      split_timestamp = true;
    }

    include_invalid = result.count("all-points") > 0;

    if (InitializeSDK() == 0) {
      unique_lock<mutex> lock(waiter_mutex);
      waiter.wait_for(lock, chrono::milliseconds(15000000));
      if (sensor.empty()) {
        ReportError("No sensor found", "", true);
      }

      int timeouts = 0;
      while (completed_sensors < sensor.size()) {
        // We do a constant 0.5 second wait, regardless of when the condition
        // variable receives notify_one/notify_all. This because when running on
        // windows, the waiter would occasionaly be triggered early, resulting
        // in the program not running to completion on all frames
        waiter.wait_for(lock, chrono::milliseconds(500));
        if (!capture_file.empty()) {
          if (CeptonReplayIsFinished(capture_handle)) break;
        }
        if (!got_frame) {
          timeouts++;
        }
        got_frame = false;
        if (timeouts > 4) {
          ReportError("Timed out waiting for " + to_string(frame_count) +
                      " frames");
          break;
        }
      }
    }
    CleanupSDK();
  } catch (const cxxopts::OptionException &e) {
    ReportError(CEPTON_SUCCESS, e.what(), true);
  }
  return 0;
}

int main(int argc, char **argv) {
  CeptonExporter ex;
  return ex.ParseOptions(argc, argv);
}
