/**
 * Whistle detection for Booster robot microphone (libduerwen).
 * Pipeline: 6ch ALSA -> Duerwen DSP -> AEC 3ch -> FFT whistle detection.
 * Publishes to /whistle_detected ROS topic when whistle is heard.
 */

#include "whistle_detection.h"

extern "C" {
#include <RecvDataCache.h>
#include <WakeupApi.h>
#include <duerwen_alsa.h>
}

#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/String.h>

namespace {

constexpr int kChannels = 6;
constexpr int kFrames = 1024;
constexpr int kSampleRate = 16000;
constexpr const char *kPcmDevice = "hw:1,0";
constexpr const char *kWhistleTopic = "/whistle_detected";

volatile sig_atomic_t g_running = 1;

void sigintHandler(int) { g_running = 0; }

#pragma pack(push, 1)
struct WavHeader {
  char chunk_id[4];
  uint32_t chunk_size;
  char format[4];
  char sub_chunk1_id[4];
  uint32_t sub_chunk1_size;
  uint16_t audio_format;
  uint16_t num_channels;
  uint32_t sample_rate;
  uint32_t byte_rate;
  uint16_t block_align;
  uint16_t bits_per_sample;
  char sub_chunk2_id[4];
  uint32_t sub_chunk2_size;
};
#pragma pack(pop)

void writeWavHeader(FILE *file, int channels, int sample_rate,
                    int bits_per_sample) {
  WavHeader h = {};
  std::memcpy(h.chunk_id, "RIFF", 4);
  h.chunk_size = 0;
  std::memcpy(h.format, "WAVE", 4);
  std::memcpy(h.sub_chunk1_id, "fmt ", 4);
  h.sub_chunk1_size = 16;
  h.audio_format = 1;
  h.num_channels = static_cast<uint16_t>(channels);
  h.sample_rate = static_cast<uint32_t>(sample_rate);
  h.bits_per_sample = static_cast<uint16_t>(bits_per_sample);
  h.byte_rate = sample_rate * channels * bits_per_sample / 8;
  h.block_align = channels * bits_per_sample / 8;
  std::memcpy(h.sub_chunk2_id, "data", 4);
  h.sub_chunk2_size = 0;
  std::fwrite(&h, sizeof(WavHeader), 1, file);
}

void finalizeWavHeader(FILE *file) {
  if (!file) return;
  uint32_t file_size = static_cast<uint32_t>(std::ftell(file));
  uint32_t chunk_size = file_size - 8;
  uint32_t data_size = file_size - sizeof(WavHeader);
  std::fseek(file, 4, SEEK_SET);
  std::fwrite(&chunk_size, 4, 1, file);
  std::fseek(file, 40, SEEK_SET);
  std::fwrite(&data_size, 4, 1, file);
}

void printWhistleBanner(const char *context) {
  std::printf("\n");
  std::printf("============================================================\n");
  std::printf("!!!  WHISTLE DETECTED  !!!  %s\n", context);
  std::printf("============================================================\n");
  std::printf("\n");
  std::fflush(stdout);
}

bool readRecordedWav(const char *path, std::vector<int16_t> *samples,
                     std::string *error) {
  FILE *file = std::fopen(path, "rb");
  if (!file) {
    *error = "cannot open wav file";
    return false;
  }

  WavHeader h = {};
  const size_t header_read = std::fread(&h, 1, sizeof(WavHeader), file);
  if (header_read != sizeof(WavHeader) ||
      std::memcmp(h.chunk_id, "RIFF", 4) != 0 ||
      std::memcmp(h.format, "WAVE", 4) != 0 ||
      std::memcmp(h.sub_chunk1_id, "fmt ", 4) != 0 ||
      std::memcmp(h.sub_chunk2_id, "data", 4) != 0) {
    std::fclose(file);
    *error = "unsupported wav layout; use WAV recorded by this node";
    return false;
  }

  if (h.audio_format != 1 || h.num_channels != 3 ||
      h.sample_rate != kSampleRate || h.bits_per_sample != 16) {
    std::fclose(file);
    *error = "wav must be PCM S16, 3 channels, 16 kHz";
    return false;
  }

  const size_t sample_count = h.sub_chunk2_size / sizeof(int16_t);
  samples->assign(sample_count, 0);
  const size_t samples_read =
      std::fread(samples->data(), sizeof(int16_t), sample_count, file);
  std::fclose(file);

  if (samples_read != sample_count) {
    *error = "wav data is truncated";
    return false;
  }

  return true;
}

int runWavTest(const char *path) {
  std::vector<int16_t> samples;
  std::string error;
  if (!readRecordedWav(path, &samples, &error)) {
    std::fprintf(stderr, "Whistle WAV test failed: %s (%s)\n", error.c_str(),
                 path);
    return 1;
  }

  whistle::WhistleDetector detector;
  bool was_ready = false;
  bool detected = false;
  const int total_frames = static_cast<int>(samples.size() / 3);

  std::printf("Testing whistle detection from WAV: %s\n", path);
  std::printf("Input: %d frames, 3ch PCM S16, 16 kHz\n", total_frames);

  for (int offset = 0; offset + kFrames <= total_frames; offset += kFrames) {
    bool ready = detector.processFrame(samples.data() + offset * 3, kFrames);
    if (ready && !was_ready) {
      std::printf("Whistle detector ready (noise floor calibrated).\n");
      was_ready = true;
    }
    if (detector.whistleDetected()) {
      printWhistleBanner("offline WAV test");
      detected = true;
      break;
    }
  }

  if (!detected) {
    std::printf("\nNo whistle detected in WAV test.\n\n");
    return 2;
  }
  return 0;
}

void printUsage(const char *prog) {
  std::fprintf(stderr, "Usage: %s [-r|--record <output.wav>] [--test-live] [--test-wav <input.wav>]\n", prog);
  std::fprintf(stderr, "  -r, --record   Record 3ch RAW output to a WAV file.\n");
  std::fprintf(stderr, "  --test-live    Open the microphone and print detections without ROS publishing.\n");
  std::fprintf(stderr, "  --test-wav     Run detector on a recorded 3ch 16 kHz S16 WAV and exit.\n");
}

} // namespace

int main(int argc, char **argv) {
  const char *record_path = nullptr;
  const char *test_wav_path = nullptr;
  bool test_live = false;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "-r") == 0 || std::strcmp(argv[i], "--record") == 0) {
      if (i + 1 >= argc) {
        printUsage(argv[0]);
        return 1;
      }
      record_path = argv[++i];
    } else if (std::strcmp(argv[i], "--test-wav") == 0) {
      if (i + 1 >= argc) {
        printUsage(argv[0]);
        return 1;
      }
      test_wav_path = argv[++i];
    } else if (std::strcmp(argv[i], "--test-live") == 0) {
      test_live = true;
    } else if (std::strcmp(argv[i], "-h") == 0 || std::strcmp(argv[i], "--help") == 0) {
      printUsage(argv[0]);
      return 0;
    }
  }

  if (test_wav_path) {
    return runWavTest(test_wav_path);
  }

  ros::Publisher publisher;
  std::unique_ptr<ros::NodeHandle> nh;
  if (!test_live) {
    ros::init(argc, argv, "whistle_detection");
    nh.reset(new ros::NodeHandle());
    publisher = nh->advertise<std_msgs::String>(kWhistleTopic, 10);
  }

  std::signal(SIGINT, sigintHandler);

  std::vector<unsigned char> cache_buf(1024 * 64, 0);
  RecvDataCacheInfo alsa_cache;
  RecvDataCacheInit(&alsa_cache, cache_buf.data(), static_cast<unsigned int>(cache_buf.size()));

  HWWakeup wakeup_handle = nullptr;
  int ret = Duerwen_wakeup_init(&wakeup_handle, 0);
  if (ret != 0) {
    std::fprintf(stderr, "Duerwen_wakeup_init failed: %d\n", ret);
    return 1;
  }

  void *alsa_handle = nullptr;
  ret = duerwen_alsa_init(&alsa_handle, const_cast<char *>(kPcmDevice),
                          kChannels, kSampleRate, SND_PCM_STREAM_CAPTURE);
  if (ret != 0) {
    std::fprintf(stderr, "duerwen_alsa_init failed: %d\n", ret);
    Duerwen_wakeup_unit(wakeup_handle);
    return 1;
  }

  std::vector<int16_t> sources(kFrames * kChannels);
  std::vector<int16_t> mic1(kFrames), mic2(kFrames), mic3(kFrames);
  std::vector<int16_t> ref1(kFrames);
  std::vector<int16_t> aec1(kFrames), aec2(kFrames), aec3(kFrames);
  std::vector<int16_t> raw_interleaved(kFrames * 3);
  std::vector<int16_t> naec(kFrames);

  FILE *record_file = nullptr;
  if (record_path) {
    record_file = std::fopen(record_path, "wb");
    if (!record_file) {
      std::fprintf(stderr, "Cannot open record file: %s\n", record_path);
      duerwen_alsa_unit(alsa_handle);
      Duerwen_wakeup_unit(wakeup_handle);
      return 1;
    }
    writeWavHeader(record_file, 3, kSampleRate, 16);
    ROS_INFO("Recording 3ch RAW to %s", record_path);
  }

  whistle::WhistleDetector detector;
  bool was_ready = false;

  if (test_live) {
    std::printf("Whistle live test started on microphone '%s'.\n", kPcmDevice);
    std::printf("No ROS node/topic will be started in this mode.\n");
    std::printf("Calibrating noise floor for about 10 seconds...\n");
  } else {
    ROS_INFO("Whistle detection started on topic '%s'", kWhistleTopic);
    ROS_INFO("Whistle detection calibrating (need ~10s of ambient noise)...");
  }

  while (g_running && (test_live || ros::ok())) {
    int frames = duerwen_alsa_read(alsa_handle, sources.data());
    if (frames <= 0) {
      continue;
    }
    if (frames != kFrames) {
      continue;
    }

    for (int i = 0; i < kFrames; ++i) {
      mic1[i] = sources[i * kChannels + 0];
      mic2[i] = sources[i * kChannels + 1];
      mic3[i] = sources[i * kChannels + 2];
      ref1[i] = sources[i * kChannels + 4];
    }

    ret = Duerwen_wakeup_three_write_data(
        wakeup_handle, mic1.data(), mic2.data(), mic3.data(), ref1.data(),
        aec1.data(), aec2.data(), aec3.data(), naec.data());

    for (int i = 0; i < kFrames; ++i) {
      raw_interleaved[i * 3 + 0] = aec1[i];
      raw_interleaved[i * 3 + 1] = aec2[i];
      raw_interleaved[i * 3 + 2] = aec3[i];
    }

    if (record_file) {
      std::fwrite(raw_interleaved.data(), sizeof(int16_t), static_cast<size_t>(kFrames * 3), record_file);
    }

    bool ready = detector.processFrame(raw_interleaved.data(), kFrames);
    if (ready && !was_ready) {
      if (test_live) {
        std::printf("Whistle detection ready (noise floor calibrated).\n");
      } else {
        ROS_INFO("Whistle detection ready (noise floor calibrated)");
      }
      was_ready = true;
    }

    if (detector.whistleDetected()) {
      printWhistleBanner(test_live ? "live microphone test" : "live microphone");
      if (!test_live) {
        ROS_INFO("Whistle DETECTED! Publishing to '%s'", kWhistleTopic);
        std_msgs::String msg;
        msg.data = "whistle_detected";
        publisher.publish(msg);
      }
      detector.reset();
      if (test_live) {
        std::printf("Whistle detection recalibrating...\n");
      } else {
        ROS_INFO("Whistle detection recalibrating...");
      }
      was_ready = false;
    }

    if (!test_live) {
      ros::spinOnce();
    }
  }

  if (record_file) {
    finalizeWavHeader(record_file);
    std::fclose(record_file);
  }

  Duerwen_wakeup_unit(wakeup_handle);
  duerwen_alsa_unit(alsa_handle);
  return 0;
}
