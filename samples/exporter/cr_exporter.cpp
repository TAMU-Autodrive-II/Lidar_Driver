#include "cr_exporter.h"

#include <cmath>

#include "cepton_sdk2.h"

using namespace std;

constexpr auto outbin = fstream::out | fstream::in | fstream::binary;
constexpr uint8_t CR_HEADER[16] = {'C', 'R', '0', '\0', 16, 0, 0, 0,
                                   0,   0,   0,   0,    0,  0, 0, 0};
constexpr uint8_t CR_IDX_HEADER[16] = {'C', 'R', 'I', '\0', 16, 0, 0, 0,
                                       0,   0,   0,   0,    0,  0, 0, 0};

struct CRBlockHeader {
  uint32_t token;  // 4cc format, check below for currently supported tokens
  uint32_t data_size;
  int64_t timestamp;
  uint32_t name_id;
  uint32_t sequence_number;  // Frame number
};

struct CRIndexBlock {
  int64_t timestamp;  // Microseconds since epoch
  uint64_t file_offset;
};
constexpr uint32_t TOKEN_CEPP = 0x50504543;  // CEPP

int CrExporter::OpenFile(const fs::path& file_name) {
  if (!fs::exists(file_name)) {
    crfile.reset(new fs::fstream(file_name, outbin | fstream::trunc));
    crindex.reset(
        new fs::fstream(file_name.string() + ".idx", outbin | fstream::trunc));
    // Write headers
    crfile->write((const char*)CR_HEADER, sizeof(CR_HEADER));
    crindex->write((const char*)CR_IDX_HEADER, sizeof(CR_IDX_HEADER));
    record_count = 0;
    data_size = 0;
  } else {
    crfile.reset(new fs::fstream(file_name, outbin));
    crfile->seekg(8);
    crfile->read((char*)&data_size, 4);

    crindex.reset(new fs::fstream(file_name.string() + ".idx", outbin));
    crindex->seekg(8);
    crindex->read((char*)&record_count, 4);
  }
  return 0;
}

int CrExporter::CloseFile() {
  crindex->seekg(8);
  crindex->write((const char*)&record_count, 4);
  crindex->close();
  crfile->seekg(8);
  crfile->write((const char*)&data_size, 4);
  crfile->close();
  return 0;
}

int CrExporter::ExportFrameCEPP(uint8_t const* points, int64_t timestamp,
                                size_t count, size_t stride) {
  CRBlockHeader hdr{TOKEN_CEPP, (uint32_t)(count * 10), timestamp, 0,
                    record_count};
  CRIndexBlock ib{timestamp, 16 + data_size};

  crfile->seekg(16 + data_size);
  crfile->write((const char*)&hdr, sizeof(hdr));
  if (stride == 10) {
    crfile->write((const char*)points, count * stride);
  } else {
    for (uint32_t i = 0; i < count; i++) {
      crfile->write((const char*)(points + i * stride), 10);
    }
  }
  data_size += (uint32_t)(sizeof(hdr) + count * 10);

  crindex->seekg(16 + record_count * 16);
  crindex->write((const char*)&ib, sizeof(ib));
  record_count++;
  return 0;
}

constexpr uint32_t TOKEN_XYZI = 0x495A5958;  // XYZI
struct XYZI {
  float x, y, z, i;
};
int CrExporter::ExportFrameXYZI_Distance(uint8_t const* points,
                                         int64_t timestamp, size_t count,
                                         size_t stride) {
  CRBlockHeader hdr{TOKEN_XYZI, (uint32_t)(count * sizeof(XYZI)), timestamp, 0,
                    record_count};
  CRIndexBlock ib{timestamp, 16 + data_size};

  crfile->seekg(16 + data_size);
  crfile->write((const char*)&hdr, sizeof(hdr));
  uint32_t actual = 0;
  for (uint32_t i = 0; i < count; i++) {
    CeptonPoint const& p =
        *reinterpret_cast<CeptonPoint const*>(points + i * stride);
    if (p.flags & CEPTON_POINT_NO_RETURN) continue;  // Skip no_return
    XYZI r{p.x * 0.005f, p.y * 0.005f, p.z * 0.005f, 0};
    r.i = sqrt(r.x * r.x + r.y * r.y + r.z * r.z);
    crfile->write((const char*)&r, sizeof(r));
    actual++;
  }
  // Update data size again.
  hdr.data_size = actual * sizeof(XYZI);
  crfile->seekg(16 + data_size + 4);
  crfile->write((const char*)&hdr.data_size, 4);
  data_size += (uint32_t)(sizeof(hdr) + actual * sizeof(XYZI));

  crindex->seekg(16 + record_count * 16);
  crindex->write((const char*)&ib, sizeof(ib));
  record_count++;
  return 0;
}
