#pragma once
#include <cstdint>
#include <memory>

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

class CrExporter {
 public:
  int OpenFile(const fs::path &file_name);
  int CloseFile();
  int ExportFrameCEPP(uint8_t const *points, int64_t timestamp, size_t count,
                      size_t stride);
  int ExportFrameXYZI_Distance(uint8_t const *points, int64_t timestamp,
                               size_t count, size_t stride);

 private:
  std::unique_ptr<fs::fstream> crfile;
  std::unique_ptr<fs::fstream> crindex;
  uint32_t record_count = 0;
  uint32_t data_size = 0;
};
