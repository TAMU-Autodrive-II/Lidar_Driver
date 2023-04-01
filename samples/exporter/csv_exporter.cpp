#include <cmath>

#include "exporter.h"

static char buffer[4000];

// Reflectivity look up table
static const float reflectivity_LUT[256] = {
    0.000f,  0.010f,  0.020f,  0.030f,  0.040f,  0.050f,  0.060f,  0.070f,
    0.080f,  0.090f,  0.100f,  0.110f,  0.120f,  0.130f,  0.140f,  0.150f,
    0.160f,  0.170f,  0.180f,  0.190f,  0.200f,  0.210f,  0.220f,  0.230f,
    0.240f,  0.250f,  0.260f,  0.270f,  0.280f,  0.290f,  0.300f,  0.310f,
    0.320f,  0.330f,  0.340f,  0.350f,  0.360f,  0.370f,  0.380f,  0.390f,
    0.400f,  0.410f,  0.420f,  0.430f,  0.440f,  0.450f,  0.460f,  0.470f,
    0.480f,  0.490f,  0.500f,  0.510f,  0.520f,  0.530f,  0.540f,  0.550f,
    0.560f,  0.570f,  0.580f,  0.590f,  0.600f,  0.610f,  0.620f,  0.630f,
    0.640f,  0.650f,  0.660f,  0.670f,  0.680f,  0.690f,  0.700f,  0.710f,
    0.720f,  0.730f,  0.740f,  0.750f,  0.760f,  0.770f,  0.780f,  0.790f,
    0.800f,  0.810f,  0.820f,  0.830f,  0.840f,  0.850f,  0.860f,  0.870f,
    0.880f,  0.890f,  0.900f,  0.910f,  0.920f,  0.930f,  0.940f,  0.950f,
    0.960f,  0.970f,  0.980f,  0.990f,  1.000f,  1.010f,  1.020f,  1.030f,
    1.040f,  1.050f,  1.060f,  1.070f,  1.080f,  1.090f,  1.100f,  1.110f,
    1.120f,  1.130f,  1.140f,  1.150f,  1.160f,  1.170f,  1.180f,  1.190f,
    1.200f,  1.210f,  1.220f,  1.230f,  1.240f,  1.250f,  1.260f,  1.270f,
    1.307f,  1.345f,  1.384f,  1.424f,  1.466f,  1.509f,  1.553f,  1.598f,
    1.644f,  1.692f,  1.741f,  1.792f,  1.844f,  1.898f,  1.953f,  2.010f,
    2.069f,  2.129f,  2.191f,  2.254f,  2.320f,  2.388f,  2.457f,  2.529f,
    2.602f,  2.678f,  2.756f,  2.836f,  2.919f,  3.004f,  3.091f,  3.181f,
    3.274f,  3.369f,  3.467f,  3.568f,  3.672f,  3.779f,  3.889f,  4.002f,
    4.119f,  4.239f,  4.362f,  4.489f,  4.620f,  4.754f,  4.892f,  5.035f,
    5.181f,  5.332f,  5.488f,  5.647f,  5.812f,  5.981f,  6.155f,  6.334f,
    6.519f,  6.708f,  6.904f,  7.105f,  7.311f,  7.524f,  7.743f,  7.969f,
    8.201f,  8.439f,  8.685f,  8.938f,  9.198f,  9.466f,  9.741f,  10.025f,
    10.317f, 10.617f, 10.926f, 11.244f, 11.572f, 11.909f, 12.255f, 12.612f,
    12.979f, 13.357f, 13.746f, 14.146f, 14.558f, 14.982f, 15.418f, 15.866f,
    16.328f, 16.804f, 17.293f, 17.796f, 18.314f, 18.848f, 19.396f, 19.961f,
    20.542f, 21.140f, 21.755f, 22.389f, 23.040f, 23.711f, 24.401f, 25.112f,
    25.843f, 26.595f, 27.369f, 28.166f, 28.986f, 29.830f, 30.698f, 31.592f,
    32.511f, 33.458f, 34.432f, 35.434f, 36.466f, 37.527f, 38.620f, 39.744f,
    40.901f, 42.092f, 43.317f, 44.578f, 45.876f, 47.211f, 48.586f, 50.000f,
};

void CsvExporter::ExportPoint(CeptonPoint const &p, int64_t timestamp) {
  if (p.x == 0 && p.y == 0 && p.z == 0) return;
  float image_x = -(float)p.x / p.y;
  float image_z = -(float)p.z / p.y;
  float distance = sqrt((float)(p.x * p.x + p.y * p.y + p.z * p.z)) * 0.005f;
  float azim = atan(-image_x);
  float elev = atan2(-image_z, sqrt(image_x * image_x + 1));
  int second_return = (p.flags & CEPTON_POINT_SECOND_RETURN) ? 1 : 0;
  int low_snr = (p.flags & CEPTON_POINT_LOW_SNR) ? 1 : 0;
  int valid = (p.flags & CEPTON_POINT_NO_RETURN) ? 0 : 1;
  int saturated = (p.flags & CEPTON_POINT_SATURATED) ? 1 : 0;
  // clang-format off
  //*export_stream
  //  << p.timestamp << ','
  //  << p.image_x << ','
  //  << p.image_z << ','
  //  << p.distance << ','
  //  << x << ','
  //  << y << ','
  //  << z << ','
  //  << azim << ','
  //  << elev << ','
  //  << p.reflectivity << ','
  //  << strongest << ','
  //  << farthest << ','
  //  << (int)p.valid << ','
  //  << (int)p.saturated << ','
  //  << (int)p.segment_id << "\n";
  // clang-format on
  int sz;
  if (split_timestamp) {
    sz = snprintf(buffer, sizeof(buffer),
                  "%u,%u,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,%d,%d\n",
                  (unsigned)(timestamp / 1000000),
                  (unsigned)(timestamp % 1000000), image_x, image_z, distance,
                  p.x * 0.005, p.y * 0.005, p.z * 0.005, azim, elev,
                  reflectivity_LUT[p.reflectivity], second_return, low_snr,
                  valid, saturated, p.channel_id);
  } else {
    sz = snprintf(buffer, sizeof(buffer),
                  "%lld,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,%d,%d\n",
                  (long long)timestamp, image_x, image_z, distance, p.x * 0.005,
                  p.y * 0.005, p.z * 0.005, azim, elev,
                  reflectivity_LUT[p.reflectivity], second_return, low_snr,
                  valid, saturated, p.channel_id);
  }
  export_stream->write(buffer, sz);
}

int CsvExporter::Export(uint8_t const *points, int64_t timestamp, size_t size,
                        size_t stride, bool include_invalid, bool append) {
  if (!append) {
    if (split_timestamp) {
      *export_stream
          << "# "
             "timestamp(sec),timestamp(us),image_x,image_z,distance,x,y,z,"
             "azimuth,"
             "elevation,reflectivity,return_strongest,return_farthest,"
             "valid,saturated,segment_id\n";
    } else {
      *export_stream
          << "# "
             "timestamp,image_x,image_z,distance,x,y,z,azimuth,"
             "elevation,reflectivity,return_strongest,return_farthest,"
             "valid,saturated,segment_id\n";
    }
  }
  for (size_t i = 0; i < size; i++) {
    CeptonPoint const &p =
        *reinterpret_cast<CeptonPoint const *>(points + i * stride);
    timestamp += p.relative_timestamp;
    if ((p.flags & CEPTON_POINT_NO_RETURN) == 0 || include_invalid)
      ExportPoint(p, timestamp);
  }
  return 0;
}
