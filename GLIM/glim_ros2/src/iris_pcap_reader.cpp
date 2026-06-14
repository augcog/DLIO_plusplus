#include <glim_ros/iris_pcap_reader.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <pcap/pcap.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include <sensor_msgs/msg/point_field.hpp>
#include <spdlog/spdlog.h>

namespace glim_ros {

namespace {

constexpr uint32_t kPcapMagicMicroLE = 0xA1B2C3D4;  // native-endian, microsecond
constexpr uint32_t kPcapMagicMicroBE = 0xD4C3B2A1;  // byte-swapped, microsecond
constexpr uint32_t kPcapMagicNanoLE = 0xA1B23C4D;   // native-endian, nanosecond
constexpr uint32_t kPcapMagicNanoBE = 0x4D3CB2A1;   // byte-swapped, nanosecond
constexpr uint32_t kPcapngMagic = 0x0A0D0D0A;       // pcapng SHB block_type

constexpr uint16_t kEtherTypeIPv4 = 0x0800;
constexpr uint16_t kEtherTypeVlan = 0x8100;
constexpr uint16_t kEtherTypeQinQ = 0x88a8;

// Output PointCloud2 layout. MUST be byte-identical to scripts/merge_luminar_pcap.py
// PC2_FIELDS / POINT_STEP=56 — extract_raw_points keys on field name + offset
// + datatype, and config_sensors.json points at "reflectance" + "line_index".
constexpr uint32_t kPointStep = 56;

// Per-point byte offsets inside one row (matches PC2_FIELDS in scripts/merge_luminar_pcap.py).
struct PointOff {
  static constexpr int timestamp = 0;        // UINT8[8]
  static constexpr int x = 8;
  static constexpr int y = 12;
  static constexpr int z = 16;
  static constexpr int reflectance = 20;
  static constexpr int return_index = 24;
  static constexpr int last_return_index = 25;
  static constexpr int sensor_id = 26;
  static constexpr int azimuth = 32;
  static constexpr int elevation = 36;
  static constexpr int depth = 40;
  static constexpr int line_index = 44;
  static constexpr int frame_index = 46;
  static constexpr int detector_site_id = 47;
  static constexpr int scan_checkpoint = 48;
  static constexpr int existence_probability_percent = 49;
  static constexpr int data_qualifier = 50;
  static constexpr int blockage_level = 51;
};

std::vector<sensor_msgs::msg::PointField> make_pc2_fields() {
  using PF = sensor_msgs::msg::PointField;
  std::vector<PF> f;
  auto add = [&](const char* name, int offset, uint8_t datatype, uint32_t count) {
    PF p;
    p.name = name;
    p.offset = offset;
    p.datatype = datatype;
    p.count = count;
    f.push_back(p);
  };
  add("timestamp", PointOff::timestamp, PF::UINT8, 8);
  add("x", PointOff::x, PF::FLOAT32, 1);
  add("y", PointOff::y, PF::FLOAT32, 1);
  add("z", PointOff::z, PF::FLOAT32, 1);
  add("reflectance", PointOff::reflectance, PF::FLOAT32, 1);
  add("return_index", PointOff::return_index, PF::UINT8, 1);
  add("last_return_index", PointOff::last_return_index, PF::UINT8, 1);
  add("sensor_id", PointOff::sensor_id, PF::UINT8, 1);
  add("azimuth", PointOff::azimuth, PF::FLOAT32, 1);
  add("elevation", PointOff::elevation, PF::FLOAT32, 1);
  add("depth", PointOff::depth, PF::FLOAT32, 1);
  add("line_index", PointOff::line_index, PF::UINT16, 1);
  add("frame_index", PointOff::frame_index, PF::UINT8, 1);
  add("detector_site_id", PointOff::detector_site_id, PF::UINT8, 1);
  add("scan_checkpoint", PointOff::scan_checkpoint, PF::UINT8, 1);
  add("existence_probability_percent", PointOff::existence_probability_percent, PF::UINT8, 1);
  add("data_qualifier", PointOff::data_qualifier, PF::UINT8, 1);
  add("blockage_level", PointOff::blockage_level, PF::UINT8, 1);
  return f;
}

// One ray return, all fields stored in their final type so create_pc2() is a
// straight memcpy/pack.
struct RayReturn {
  uint64_t timestamp_ns;
  float x, y, z;
  float reflectance;
  uint8_t return_index;
  uint8_t last_return_index;
  uint8_t sensor_id;
  float azimuth;
  float elevation;
  float depth;
  uint16_t line_index;
  uint8_t frame_index;
  uint8_t detector_site_id;
  uint8_t scan_checkpoint;
  uint8_t existence_probability_percent;
  uint8_t data_qualifier;
  uint8_t blockage_level;
};

struct IrisPacketHeader {
  uint8_t version_major;
  uint8_t version_minor;
  uint8_t version_patch;
  uint8_t packet_sequence;
  uint8_t num_rays;
  uint8_t frame_sequence;
  uint64_t ptp_timestamp;  // seconds (48 bits)
  uint8_t sensor_id;
  uint8_t data_qualifier;
};

// Bit-level reader for the Luminar Iris payload. Mirrors the Python
// read_bits() in scripts/merge_luminar_pcap.py.
class BitReader {
public:
  BitReader(const uint8_t* data, size_t len) : data_(data), bits_(len * 8) {}

  // Read nbits starting at absolute bit offset. nbits <= 32.
  uint64_t read32(size_t bit_ofs, uint32_t nbits) const {
    return read_generic(bit_ofs, nbits);
  }

  // Read up to 56 bits (used for ptp_timestamp 48 bits).
  uint64_t read64(size_t bit_ofs, uint32_t nbits) const {
    return read_generic(bit_ofs, nbits);
  }

  size_t total_bits() const { return bits_; }

private:
  uint64_t read_generic(size_t bit_ofs, uint32_t nbits) const {
    const size_t byte_ofs = bit_ofs / 8;
    const size_t bit_in_byte = bit_ofs % 8;
    const size_t nbytes_needed = (bit_in_byte + nbits + 7) / 8;
    uint64_t chunk = 0;
    for (size_t i = 0; i < nbytes_needed && i < 8; i++) {
      chunk |= static_cast<uint64_t>(data_[byte_ofs + i]) << (8 * i);
    }
    chunk >>= bit_in_byte;
    if (nbits >= 64) return chunk;
    return chunk & ((1ULL << nbits) - 1ULL);
  }

  const uint8_t* data_;
  size_t bits_;
};

float q2_14_to_float(uint16_t x16) {
  int32_t signed_x = (x16 & 0x8000) ? (static_cast<int32_t>(x16) - 0x10000) : static_cast<int32_t>(x16);
  return static_cast<float>(signed_x) / static_cast<float>(1 << 14);
}

float uq12_12_to_float(uint32_t u) { return static_cast<float>(u) / static_cast<float>(1 << 12); }
float uq1_15_to_float(uint32_t u) { return static_cast<float>(u) / static_cast<float>(1 << 15); }

// Parse an Iris UDP payload into a header and a list of returns. Returns
// false if the packet is not a v1.3 data packet or the payload is malformed.
// Mirrors parse_iris_payload() in scripts/merge_luminar_pcap.py.
bool parse_iris_payload(const uint8_t* payload, size_t len, IrisPacketHeader& hdr, std::vector<RayReturn>& returns) {
  if (len < 16) return false;
  BitReader br(payload, len);
  hdr.version_major = static_cast<uint8_t>(br.read32(0, 8));
  hdr.version_minor = static_cast<uint8_t>(br.read32(8, 8));
  hdr.version_patch = static_cast<uint8_t>(br.read32(16, 8));
  hdr.packet_sequence = static_cast<uint8_t>(br.read32(24, 8));
  hdr.num_rays = static_cast<uint8_t>(br.read32(32, 8));
  hdr.frame_sequence = static_cast<uint8_t>(br.read32(40, 8));
  hdr.ptp_timestamp = br.read64(48, 48);
  hdr.sensor_id = static_cast<uint8_t>(br.read32(96, 8));
  hdr.data_qualifier = static_cast<uint8_t>(br.read32(104, 8));
  // reserved 112..127

  if (hdr.version_major != 1 || hdr.version_minor != 3) return false;

  size_t bit = 128;
  const size_t payload_bits = br.total_bits();

  for (uint32_t r = 0; r < hdr.num_rays; r++) {
    if (bit + 128 > payload_bits) return true;  // partial; emit what we have
    const uint16_t azimuth_q = static_cast<uint16_t>(br.read32(bit, 16));
    const uint16_t elev_q = static_cast<uint16_t>(br.read32(bit + 16, 16));
    const uint32_t ts_offset_ns = static_cast<uint32_t>(br.read32(bit + 32, 32));
    const uint8_t scan_checkpoint = static_cast<uint8_t>(br.read32(bit + 64, 8));
    const uint8_t num_returns = static_cast<uint8_t>(br.read32(bit + 76, 4));
    const uint8_t detector_number = static_cast<uint8_t>(br.read32(bit + 80, 1));
    const uint8_t blockage_number = static_cast<uint8_t>(br.read32(bit + 81, 4));
    const uint16_t line_number = static_cast<uint16_t>(br.read32(bit + 87, 9));
    bit += 128;

    const float az = q2_14_to_float(azimuth_q);
    const float el = q2_14_to_float(elev_q);
    const float cos_el = std::cos(el);
    const float sin_el = std::sin(el);
    const float cos_az = std::cos(az);
    const float sin_az = std::sin(az);

    const uint64_t base_ns = hdr.ptp_timestamp * 1'000'000'000ULL + static_cast<uint64_t>(ts_offset_ns);

    for (uint32_t i = 0; i < num_returns; i++) {
      if (bit + 64 > payload_bits) return true;
      const uint8_t existence_prob = static_cast<uint8_t>(br.read32(bit, 8));
      const uint32_t range_q = static_cast<uint32_t>(br.read32(bit + 8, 24));
      const uint32_t reflectance_q = static_cast<uint32_t>(br.read32(bit + 32, 16));
      bit += 64;

      const float range = uq12_12_to_float(range_q);
      const float reflectance = uq1_15_to_float(reflectance_q);
      const float x = range * cos_el * cos_az;
      const float y = range * cos_el * sin_az;
      const float z = range * sin_el;

      RayReturn rr;
      rr.timestamp_ns = base_ns;
      rr.x = x;
      rr.y = y;
      rr.z = z;
      rr.reflectance = reflectance;
      rr.return_index = static_cast<uint8_t>(i);
      rr.last_return_index = static_cast<uint8_t>(num_returns > 0 ? num_returns - 1 : 0);
      rr.sensor_id = hdr.sensor_id;
      rr.azimuth = az;
      rr.elevation = el;
      rr.depth = range;
      rr.line_index = line_number;
      rr.frame_index = hdr.frame_sequence;
      rr.detector_site_id = detector_number;
      rr.scan_checkpoint = scan_checkpoint;
      rr.existence_probability_percent = existence_prob;
      rr.data_qualifier = hdr.data_qualifier;
      rr.blockage_level = blockage_number;
      returns.push_back(rr);
    }
  }
  return true;
}

uint64_t timestamp_from_record(uint32_t ts_sec, uint32_t ts_frac, bool is_nano) {
  const uint64_t sec_ns = static_cast<uint64_t>(ts_sec) * 1'000'000'000ULL;
  if (is_nano) return sec_ns + static_cast<uint64_t>(ts_frac);
  return sec_ns + static_cast<uint64_t>(ts_frac) * 1000ULL;
}

uint16_t bswap16(uint16_t v) { return static_cast<uint16_t>((v >> 8) | (v << 8)); }
uint32_t bswap32(uint32_t v) {
  return ((v >> 24) & 0xFFu) | ((v >> 8) & 0xFF00u) | ((v << 8) & 0xFF0000u) | ((v << 24) & 0xFF000000u);
}

// Read the first record-header timestamp from the raw pcap file. Validates
// the global header magic (rejects pcapng), then reads exactly one packet
// record header (16 B) — true O(1).
//
// We deliberately do NOT linearly pre-scan to find the last packet timestamp:
// for multi-GB / multi-hundred-GB captures, that pass costs many seconds at
// startup. Callers should treat the pcap as unbounded-end and rely on the
// bag's window + the inactivity_sec tail as the upper bound. The IrisPcapReader
// will naturally stop emitting when libpcap reaches EOF on the iteration pass.
uint64_t prescan_pcap_first_epoch_ns(const std::string& path) {
  FILE* fp = std::fopen(path.c_str(), "rb");
  if (!fp) {
    throw std::runtime_error("pcap: failed to open '" + path + "' for raw pre-scan");
  }
  uint8_t global[24];
  if (std::fread(global, 1, sizeof(global), fp) != sizeof(global)) {
    std::fclose(fp);
    throw std::runtime_error("pcap: file shorter than 24 B global header: '" + path + "'");
  }

  uint32_t magic;
  std::memcpy(&magic, global, 4);

  if (magic == kPcapngMagic) {
    std::fclose(fp);
    throw std::runtime_error(
      "pcap: input '" + path + "' is pcapng (block magic 0x0A0D0D0A); convert with `editcap -F pcap input.pcapng output.pcap`");
  }

  bool swap = false;
  bool is_nano = false;
  switch (magic) {
    case kPcapMagicMicroLE: swap = false; is_nano = false; break;
    case kPcapMagicMicroBE: swap = true; is_nano = false; break;
    case kPcapMagicNanoLE: swap = false; is_nano = true; break;
    case kPcapMagicNanoBE: swap = true; is_nano = true; break;
    default: {
      std::fclose(fp);
      char buf[64];
      std::snprintf(buf, sizeof(buf), "0x%08X", magic);
      throw std::runtime_error("pcap: unrecognized magic " + std::string(buf) + " in '" + path + "'");
    }
  }

  uint8_t hdr[16];
  if (std::fread(hdr, 1, sizeof(hdr), fp) != sizeof(hdr)) {
    std::fclose(fp);
    throw std::runtime_error("pcap: file '" + path + "' has no records");
  }
  uint32_t ts_sec, ts_frac;
  std::memcpy(&ts_sec, hdr + 0, 4);
  std::memcpy(&ts_frac, hdr + 4, 4);
  if (swap) {
    ts_sec = bswap32(ts_sec);
    ts_frac = bswap32(ts_frac);
  }
  std::fclose(fp);
  return timestamp_from_record(ts_sec, ts_frac, is_nano);
}

}  // namespace

// ---------------------------------------------------------------------------
// AssemblerImpl: groups rays into scans, finalizes on key change or timeout.
// Mirrors ScanAssemblerPC2 in scripts/merge_luminar_pcap.py.
// ---------------------------------------------------------------------------
struct IrisPcapReader::AssemblerImpl {
  struct Key {
    std::string src_ip;
    uint8_t sensor_id;
    uint8_t frame_seq;
    bool operator==(const Key& other) const {
      return sensor_id == other.sensor_id && frame_seq == other.frame_seq && src_ip == other.src_ip;
    }
  };
  struct KeyHash {
    size_t operator()(const Key& k) const noexcept {
      size_t h = std::hash<std::string>{}(k.src_ip);
      h ^= (size_t)k.sensor_id + 0x9e3779b9 + (h << 6) + (h >> 2);
      h ^= (size_t)k.frame_seq + 0x9e3779b9 + (h << 6) + (h >> 2);
      return h;
    }
  };

  struct State {
    Key key;
    std::string frame_id;
    std::string topic;
    double first_time = 0.0;
    double last_time = 0.0;
    std::vector<RayReturn> points;
  };

  double inactivity_sec;
  std::unordered_map<Key, State, KeyHash> active;
  std::unordered_map<std::string, Key> last_key_by_src;

  static std::vector<sensor_msgs::msg::PointField> kFields;

  AssemblerImpl(double inactivity) : inactivity_sec(inactivity) {}

  // Build a PointCloud2 message from a finalized state. Returns nullopt if
  // the state has no points.
  std::optional<AssembledScan> create_scan(State& st) {
    if (st.points.empty()) return std::nullopt;
    uint64_t first_ts_ns = st.points.front().timestamp_ns;
    for (const auto& p : st.points) {
      if (p.timestamp_ns < first_ts_ns) first_ts_ns = p.timestamp_ns;
    }
    auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    cloud->header.frame_id = st.frame_id;
    cloud->header.stamp.sec = static_cast<int32_t>(first_ts_ns / 1'000'000'000ULL);
    cloud->header.stamp.nanosec = static_cast<uint32_t>(first_ts_ns % 1'000'000'000ULL);
    cloud->fields = kFields;
    cloud->is_bigendian = false;
    cloud->point_step = kPointStep;
    cloud->height = 1;
    cloud->width = static_cast<uint32_t>(st.points.size());
    cloud->row_step = kPointStep * cloud->width;
    cloud->is_dense = false;

    cloud->data.resize(static_cast<size_t>(cloud->row_step));
    uint8_t* buf = cloud->data.data();
    for (size_t i = 0; i < st.points.size(); i++) {
      uint8_t* row = buf + i * kPointStep;
      const RayReturn& p = st.points[i];

      std::memcpy(row + PointOff::timestamp, &p.timestamp_ns, sizeof(uint64_t));
      std::memcpy(row + PointOff::x, &p.x, sizeof(float));
      std::memcpy(row + PointOff::y, &p.y, sizeof(float));
      std::memcpy(row + PointOff::z, &p.z, sizeof(float));
      std::memcpy(row + PointOff::reflectance, &p.reflectance, sizeof(float));
      row[PointOff::return_index] = p.return_index;
      row[PointOff::last_return_index] = p.last_return_index;
      row[PointOff::sensor_id] = p.sensor_id;
      // padding 27..31 left zero
      std::memcpy(row + PointOff::azimuth, &p.azimuth, sizeof(float));
      std::memcpy(row + PointOff::elevation, &p.elevation, sizeof(float));
      std::memcpy(row + PointOff::depth, &p.depth, sizeof(float));
      std::memcpy(row + PointOff::line_index, &p.line_index, sizeof(uint16_t));
      row[PointOff::frame_index] = p.frame_index;
      row[PointOff::detector_site_id] = p.detector_site_id;
      row[PointOff::scan_checkpoint] = p.scan_checkpoint;
      row[PointOff::existence_probability_percent] = p.existence_probability_percent;
      row[PointOff::data_qualifier] = p.data_qualifier;
      row[PointOff::blockage_level] = p.blockage_level;
    }
    AssembledScan out;
    out.t_ns = first_ts_ns;
    out.wall_clock_first_packet_ns = static_cast<uint64_t>(st.first_time * 1e9);
    out.wall_clock_last_packet_ns = static_cast<uint64_t>(st.last_time * 1e9);
    out.topic = st.topic;
    out.frame_id = st.frame_id;
    out.cloud = cloud;
    return out;
  }

  // Move-out and finalize one active key.
  std::optional<AssembledScan> finalize(const Key& k) {
    auto it = active.find(k);
    if (it == active.end()) return std::nullopt;
    State st = std::move(it->second);
    active.erase(it);
    return create_scan(st);
  }

  std::vector<AssembledScan> maybe_timeout(double now_sec) {
    std::vector<AssembledScan> out;
    for (auto it = active.begin(); it != active.end();) {
      if ((now_sec - it->second.last_time) >= inactivity_sec) {
        State st = std::move(it->second);
        it = active.erase(it);
        if (auto s = create_scan(st)) out.push_back(std::move(*s));
      } else {
        ++it;
      }
    }
    return out;
  }

  std::vector<AssembledScan> feed(
    double t_epoch,
    const std::string& src_ip,
    const std::string& topic,
    const std::string& frame_id,
    const IrisPacketHeader& hdr,
    const std::vector<RayReturn>& points) {
    std::vector<AssembledScan> completed;
    Key k{src_ip, hdr.sensor_id, hdr.frame_sequence};
    auto prev_it = last_key_by_src.find(src_ip);
    if (prev_it != last_key_by_src.end() && !(prev_it->second == k)) {
      if (auto s = finalize(prev_it->second)) completed.push_back(std::move(*s));
    }
    last_key_by_src[src_ip] = k;

    auto& st = active[k];
    if (st.points.empty()) {
      st.key = k;
      st.frame_id = frame_id;
      st.topic = topic;
      st.first_time = t_epoch;
    }
    st.last_time = std::max(st.last_time, t_epoch);
    st.points.insert(st.points.end(), points.begin(), points.end());

    auto timed_out = maybe_timeout(t_epoch);
    completed.insert(completed.end(),
                     std::make_move_iterator(timed_out.begin()),
                     std::make_move_iterator(timed_out.end()));
    return completed;
  }

  std::vector<AssembledScan> flush_all() {
    std::vector<AssembledScan> out;
    for (auto& kv : active) {
      if (auto s = create_scan(kv.second)) out.push_back(std::move(*s));
    }
    active.clear();
    last_key_by_src.clear();
    return out;
  }
};

std::vector<sensor_msgs::msg::PointField> IrisPcapReader::AssemblerImpl::kFields = make_pc2_fields();

// ---------------------------------------------------------------------------
// IrisPcapReader public methods
// ---------------------------------------------------------------------------

IrisPcapReader::IrisPcapReader() = default;

IrisPcapReader::~IrisPcapReader() { close(); }

void IrisPcapReader::close() {
  if (handle_) {
    pcap_close(handle_);
    handle_ = nullptr;
  }
}

void IrisPcapReader::set_window(uint64_t lo_ns, uint64_t hi_ns) {
  window_lo_ns_ = lo_ns;
  window_hi_ns_ = hi_ns;
}

void IrisPcapReader::open(const std::string& pcap_path, const IrisPcapConfig& config) {
  close();
  config_ = config;
  lidar_by_ip_.clear();
  warned_unknown_.clear();
  for (const auto& l : config.lidars) {
    lidar_by_ip_[l.ip] = l;
  }

  first_packet_epoch_ns_ = prescan_pcap_first_epoch_ns(pcap_path);
  last_packet_epoch_ns_ = std::numeric_limits<uint64_t>::max();  // unbounded; see prescan_pcap_first_epoch_ns docs

  char errbuf[PCAP_ERRBUF_SIZE] = {0};
  handle_ = pcap_open_offline(pcap_path.c_str(), errbuf);
  if (!handle_) {
    throw std::runtime_error("pcap_open_offline failed for '" + pcap_path + "': " + errbuf);
  }
  link_type_ = pcap_datalink(handle_);
  switch (link_type_) {
    case DLT_EN10MB: link_header_len_ = 14; break;
    case DLT_LINUX_SLL: link_header_len_ = 16; break;
#ifdef DLT_LINUX_SLL2
    case DLT_LINUX_SLL2: link_header_len_ = 20; break;
#endif
    case DLT_RAW: link_header_len_ = 0; break;
    default: {
      const char* name = pcap_datalink_val_to_name(link_type_);
      throw std::runtime_error(
        std::string("pcap: unsupported link type ") + (name ? name : "(unknown)") +
        " in '" + pcap_path + "' (only EN10MB / LINUX_SLL / LINUX_SLL2 / RAW are supported)");
    }
  }

  assembler_ = std::make_unique<AssemblerImpl>(config_.inactivity_sec);
  ready_.clear();
  newest_completed_t_ns_ = 0;
  eof_ = false;
}

void IrisPcapReader::emit_completed(std::vector<AssembledScan>&& scans) {
  for (auto& s : scans) {
    if (s.t_ns > newest_completed_t_ns_) newest_completed_t_ns_ = s.t_ns;
    ready_.push_back(std::move(s));
  }
  std::sort(ready_.begin(), ready_.end(), [](const AssembledScan& a, const AssembledScan& b) {
    return a.t_ns < b.t_ns;
  });
}

bool IrisPcapReader::feed_next_packet() {
  if (!handle_) return false;
  while (true) {
    struct pcap_pkthdr* pkt_hdr = nullptr;
    const u_char* pkt_data = nullptr;
    int rc = pcap_next_ex(handle_, &pkt_hdr, &pkt_data);
    if (rc == -2 || rc == 0) {
      // EOF or no packet ready (shouldn't happen for offline)
      eof_ = true;
      return false;
    }
    if (rc < 0) {
      spdlog::error("pcap_next_ex error: {}", pcap_geterr(handle_));
      eof_ = true;
      return false;
    }

    const uint64_t pkt_ts_ns = static_cast<uint64_t>(pkt_hdr->ts.tv_sec) * 1'000'000'000ULL +
                               static_cast<uint64_t>(pkt_hdr->ts.tv_usec) * 1000ULL;
    if (pkt_ts_ns < window_lo_ns_ || pkt_ts_ns > window_hi_ns_) continue;
    const double t_epoch = static_cast<double>(pkt_ts_ns) / 1e9;

    const uint8_t* p = pkt_data;
    size_t remaining = pkt_hdr->caplen;
    if (remaining < static_cast<size_t>(link_header_len_)) continue;

    uint16_t ethertype = 0;
    if (link_type_ == DLT_EN10MB) {
      // 6 dst + 6 src + 2 ethertype
      ethertype = static_cast<uint16_t>((p[12] << 8) | p[13]);
      p += 14; remaining -= 14;
      // Skip VLAN tags (4 B each)
      while ((ethertype == kEtherTypeVlan || ethertype == kEtherTypeQinQ) && remaining >= 4) {
        ethertype = static_cast<uint16_t>((p[2] << 8) | p[3]);
        p += 4; remaining -= 4;
      }
    } else if (link_type_ == DLT_LINUX_SLL) {
      // SLL: 16 B header; ethertype at offset 14..15 (big-endian)
      ethertype = static_cast<uint16_t>((pkt_data[14] << 8) | pkt_data[15]);
      p += 16; remaining = (pkt_hdr->caplen >= 16) ? (pkt_hdr->caplen - 16) : 0;
#ifdef DLT_LINUX_SLL2
    } else if (link_type_ == DLT_LINUX_SLL2) {
      // SLL2: 20 B header; ethertype at offset 0..1 (big-endian)
      ethertype = static_cast<uint16_t>((pkt_data[0] << 8) | pkt_data[1]);
      p += 20; remaining = (pkt_hdr->caplen >= 20) ? (pkt_hdr->caplen - 20) : 0;
#endif
    } else if (link_type_ == DLT_RAW) {
      ethertype = kEtherTypeIPv4;  // implicit
    }

    if (ethertype != kEtherTypeIPv4) continue;
    if (remaining < 20) continue;

    const uint8_t ihl = p[0] & 0x0F;
    const size_t ip_hdr_len = static_cast<size_t>(ihl) * 4;
    if (ip_hdr_len < 20 || ip_hdr_len > remaining) continue;
    const uint8_t protocol = p[9];
    if (protocol != IPPROTO_UDP) continue;
    char src_ip_buf[INET_ADDRSTRLEN] = {0};
    in_addr src_ip_in;
    std::memcpy(&src_ip_in, p + 12, 4);
    if (!inet_ntop(AF_INET, &src_ip_in, src_ip_buf, sizeof(src_ip_buf))) continue;

    const uint8_t* udp = p + ip_hdr_len;
    if (remaining < ip_hdr_len + 8) continue;
    const uint16_t udp_src = static_cast<uint16_t>((udp[0] << 8) | udp[1]);
    const uint16_t udp_dst = static_cast<uint16_t>((udp[2] << 8) | udp[3]);
    const uint16_t udp_len_total = static_cast<uint16_t>((udp[4] << 8) | udp[5]);
    if (udp_len_total < 8) continue;
    const size_t udp_payload_len = std::min<size_t>(static_cast<size_t>(udp_len_total) - 8,
                                                    remaining - ip_hdr_len - 8);
    const uint8_t* payload = udp + 8;

    if (udp_src != config_.iris_data_src_port) continue;

    auto it = lidar_by_ip_.find(src_ip_buf);
    if (it == lidar_by_ip_.end() || it->second.dst_port != udp_dst) {
      const std::string key = std::string(src_ip_buf) + ":" + std::to_string(udp_dst);
      if (!warned_unknown_.count(key)) {
        warned_unknown_[key] = true;
        spdlog::warn("pcap: unknown traffic from {}:{} -> dst={} (one-shot)", src_ip_buf, udp_src, udp_dst);
      }
      continue;
    }

    IrisPacketHeader hdr{};
    std::vector<RayReturn> rays;
    rays.reserve(64);
    if (!parse_iris_payload(payload, udp_payload_len, hdr, rays)) continue;
    if (rays.empty()) continue;

    auto completed = assembler_->feed(t_epoch, src_ip_buf, it->second.topic, it->second.frame_id, hdr, rays);
    if (!completed.empty()) {
      emit_completed(std::move(completed));
    }
    return true;
  }
}

std::optional<AssembledScan> IrisPcapReader::next() {
  // Pull more packets until either we have a scan that's outside the
  // reorder lookahead, or we hit EOF.
  while (true) {
    if (!ready_.empty()) {
      const uint64_t head_t = ready_.front().t_ns;
      const bool past_lookahead = (newest_completed_t_ns_ > head_t) &&
                                  ((newest_completed_t_ns_ - head_t) >= config_.scan_reorder_lookahead_ns);
      if (eof_ || past_lookahead) {
        AssembledScan out = std::move(ready_.front());
        ready_.pop_front();
        return out;
      }
    }
    if (eof_) return std::nullopt;
    if (!feed_next_packet()) {
      // EOF: don't drain here — caller invokes flush_all() then keeps pulling.
      // Continue loop to release ready_ items past the lookahead under EOF.
      continue;
    }
  }
}

std::vector<AssembledScan> IrisPcapReader::flush_all() {
  std::vector<AssembledScan> out;
  if (!assembler_) return out;
  auto remaining = assembler_->flush_all();
  for (auto& s : remaining) {
    if (s.t_ns > newest_completed_t_ns_) newest_completed_t_ns_ = s.t_ns;
    out.push_back(std::move(s));
  }
  // Also flush whatever is still buffered in ready_ to caller; sorted.
  std::sort(out.begin(), out.end(), [](const AssembledScan& a, const AssembledScan& b) {
    return a.t_ns < b.t_ns;
  });
  // Drain ready_ into out (preserving order).
  std::vector<AssembledScan> merged;
  merged.reserve(ready_.size() + out.size());
  auto a = ready_.begin();
  auto b = out.begin();
  while (a != ready_.end() && b != out.end()) {
    if (a->t_ns <= b->t_ns) merged.push_back(std::move(*a++));
    else merged.push_back(std::move(*b++));
  }
  while (a != ready_.end()) merged.push_back(std::move(*a++));
  while (b != out.end()) merged.push_back(std::move(*b++));
  ready_.clear();
  eof_ = true;
  return merged;
}

}  // namespace glim_ros
