// bno085_logger.cpp
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <queue>
#include <string>
#include <sstream>
#include <thread>
#include <vector>
#include <unistd.h>
#include <sys/stat.h>

extern "C" {
#include "sh2_hal.h"
#include "sh2.h"
#include "sh2_SensorValue.h"
}
#include "sh2_hal_linux.h"

using namespace std::chrono;

/* ---------- configuration ---------- */
static const char* LOG_DIR = "logs";
static const char* BASE_FN = "bno085";
static const size_t ROTATE_FILE_MB = 50;
static const size_t BATCH_SIZE = 128;
static const unsigned TARGET_HZ = 500;
static const double SAMPLE_PERIOD_S = 1.0 / double(TARGET_HZ);
static const size_t MAX_QUEUE = 20000;

/* ---------- runtime globals ---------- */
static std::atomic<bool> g_running(true);
void signal_handler(int) { g_running = false; }

/* ---------- Sample definition ---------- */
struct Sample {
    uint64_t host_ts_ns;
    uint64_t sensor_ts;
    uint32_t seq;

    bool has_accel; double accel_x, accel_y, accel_z;
    bool has_gyro; double gyro_x, gyro_y, gyro_z;
    bool has_mag; double mag_x, mag_y, mag_z;
    bool has_linear_accel; double la_x, la_y, la_z;
    bool has_gravity; double grav_x, grav_y, grav_z;
    bool has_quat; double qw, qx, qy, qz;
    bool has_euler; double yaw, pitch, roll;
    bool has_temp; double temp_c;
    int calib_status;

    Sample() { std::memset(this, 0, sizeof(Sample)); seq = 0; calib_status = -1;}
};

/* ---------- RingQueue ---------- */
class RingQueue {
public:
    RingQueue(size_t capacity) : cap(capacity) {}
    bool push(Sample &&s) {
        std::lock_guard<std::mutex> lk(m);
        if (q.size() >= cap) return false;
        q.push(std::move(s));
        return true;
    }
    bool pop(Sample &out) {
        std::lock_guard<std::mutex> lk(m);
        if (q.empty()) return false;
        out = std::move(q.front());
        q.pop();
        return true;
    }
    size_t size() {
        std::lock_guard<std::mutex> lk(m);
        return q.size();
    }
private:
    std::queue<Sample> q;
    size_t cap;
    std::mutex m;
};
static RingQueue g_queue(MAX_QUEUE);

/* ---------- DiskWriter ---------- */
class DiskWriter {
public:
    DiskWriter() : bytes_written(0) { open_new_file(); }
    ~DiskWriter() { if (ofs.is_open()) ofs.close(); }
    void push_sample(const Sample &s) {
        buffer.push_back(s);
        if (buffer.size() >= BATCH_SIZE) flush();
    }
    void flush() {
        if (buffer.empty()) return;
        std::string out;
        out.reserve(buffer.size() * 256);
        for (auto &s : buffer) {
            out += sample_to_json(s);
            out += '\n';
        }
        ofs << out;
        ofs.flush();
        bytes_written += out.size();
        buffer.clear();
        if (bytes_written >= ROTATE_FILE_MB * 1024 * 1024) rotate_file();
    }
    void rotate_file() {
        if (ofs.is_open()) ofs.close();
        open_new_file();
    }
private:
    std::ofstream ofs;
    std::vector<Sample> buffer;
    size_t bytes_written;

    void open_new_file() {
        std::string dir(LOG_DIR);
        mkdir(dir.c_str(), 0755);
        auto t = system_clock::to_time_t(system_clock::now());
        std::tm tm;
        localtime_r(&t, &tm);
        char tsbuf[64];
        strftime(tsbuf, sizeof(tsbuf), "%Y%m%d_%H%M%S", &tm);
        std::ostringstream ss;
        ss << dir << "/" << BASE_FN << "_" << tsbuf << ".ndjson";
        ofs.open(ss.str(), std::ios::out | std::ios::app);
        bytes_written = 0;
    }

    std::string sample_to_json(const Sample &s) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(6);
        ss << "{";
        // required fields
        ss << "\"host_ts_ns\":" << s.host_ts_ns << ",";
        ss << "\"sensor_ts\":" << s.sensor_ts << ",";
        ss << "\"seq\":" << s.seq;

        auto add_comma = [&ss]() { ss << ","; };

        if (s.has_accel) {
            add_comma();
            ss << "\"accel\":{"
               << "\"x\":" << s.accel_x << ","
               << "\"y\":" << s.accel_y << ","
               << "\"z\":" << s.accel_z
               << "}";
        }
        if (s.has_gyro) {
            add_comma();
            ss << "\"gyro\":{"
               << "\"x\":" << s.gyro_x << ","
               << "\"y\":" << s.gyro_y << ","
               << "\"z\":" << s.gyro_z
               << "}";
        }
        if (s.has_mag) {
            add_comma();
            ss << "\"mag\":{"
               << "\"x\":" << s.mag_x << ","
               << "\"y\":" << s.mag_y << ","
               << "\"z\":" << s.mag_z
               << "}";
        }
        if (s.has_linear_accel) {
            add_comma();
            ss << "\"linear_accel\":{"
               << "\"x\":" << s.la_x << ","
               << "\"y\":" << s.la_y << ","
               << "\"z\":" << s.la_z
               << "}";
        }
        if (s.has_gravity) {
            add_comma();
            ss << "\"gravity\":{"
               << "\"x\":" << s.grav_x << ","
               << "\"y\":" << s.grav_y << ","
               << "\"z\":" << s.grav_z
               << "}";
        }
        if (s.has_quat) {
            add_comma();
            ss << "\"quat\":{"
               << "\"w\":" << s.qw << ","
               << "\"x\":" << s.qx << ","
               << "\"y\":" << s.qy << ","
               << "\"z\":" << s.qz
               << "}";
        }
        if (s.has_euler) {
            add_comma();
            ss << "\"euler\":{"
               << "\"yaw\":" << s.yaw << ","
               << "\"pitch\":" << s.pitch << ","
               << "\"roll\":" << s.roll
               << "}";
        }
        if (s.has_temp) {
            add_comma();
            ss << "\"temp_c\":" << s.temp_c;
        }

        add_comma();
        ss << "\"calib_status\":" << s.calib_status;

        ss << "}";
        return ss.str();
    }
};
static DiskWriter g_writer;

/* ---------- SH-2 sensor handling ---------- */

/* We'll register a sensor callback (sh2_SensorCallback_t) that will be invoked
 * by the sh2 library whenever a sensor event arrives. We need a small adapter
 * to decode the sh2_SensorEvent_t -> sh2_SensorValue_t using
 * sh2_decodeSensorEvent(), then convert to our Sample and push to the queue.
 */

/* sensor callback — called by SH2 when sensor data arrives */
static void sensor_callback(void *cookie, sh2_SensorEvent_t *pEvent) {
    (void)cookie;
    if (!pEvent) return;

    sh2_SensorValue_t value;
    int rc = sh2_decodeSensorEvent(&value, pEvent);
    if (rc != SH2_OK) return;

    Sample s;
    s.host_ts_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                      std::chrono::steady_clock::now().time_since_epoch()).count();
    s.sensor_ts = value.timestamp;
    s.seq = value.sequence;
    s.calib_status = -1;

    std::cerr << "CB: sensorId=0x" << std::hex << int(value.sensorId) << std::dec
          << " seq=" << int(value.sequence) 
          << " ts=" << value.timestamp << "\n";

    switch (value.sensorId) {
        case SH2_ACCELEROMETER:
            s.has_accel = true;
            s.accel_x = value.un.accelerometer.x;
            s.accel_y = value.un.accelerometer.y;
            s.accel_z = value.un.accelerometer.z;
            break;

        case SH2_GYROSCOPE_CALIBRATED:
        case SH2_GYROSCOPE_UNCALIBRATED:
        case SH2_RAW_GYROSCOPE:
            s.has_gyro = true;
            /* Gyroscope struct is sh2_Gyroscope_t (rad/s) */
            s.gyro_x = value.un.gyroscope.x;
            s.gyro_y = value.un.gyroscope.y;
            s.gyro_z = value.un.gyroscope.z;
            break;

        case SH2_ROTATION_VECTOR:
        case SH2_GAME_ROTATION_VECTOR:
        case SH2_GEOMAGNETIC_ROTATION_VECTOR:
            s.has_quat = true;
            /* rotationVector: i,j,k,real -> map to x,y,z,w */
            s.qx = value.un.rotationVector.i;
            s.qy = value.un.rotationVector.j;
            s.qz = value.un.rotationVector.k;
            s.qw = value.un.rotationVector.real;
            break;

        case SH2_LINEAR_ACCELERATION:
            s.has_linear_accel = true;
            s.la_x = value.un.linearAcceleration.x;
            s.la_y = value.un.linearAcceleration.y;
            s.la_z = value.un.linearAcceleration.z;
            break;

        case SH2_GRAVITY:
            s.has_gravity = true;
            s.grav_x = value.un.gravity.x;
            s.grav_y = value.un.gravity.y;
            s.grav_z = value.un.gravity.z;
            break;

        case SH2_MAGNETIC_FIELD_CALIBRATED:
        case SH2_MAGNETIC_FIELD_UNCALIBRATED:
            s.has_mag = true;
            s.mag_x = value.un.magneticField.x;
            s.mag_y = value.un.magneticField.y;
            s.mag_z = value.un.magneticField.z;
            break;

        case SH2_TEMPERATURE:
            s.has_temp = true;
            s.temp_c = value.un.temperature.value;
            break;

        default:
            break;
    }

    // push (best-effort) to queue
    bool pushed = g_queue.push(std::move(s));
    if (!pushed) std::cerr << "CB: queue full – dropped sample\n";
}

/* async event callback — handle resets, etc. Keep simple */
static void async_event_cb(void *cookie, sh2_AsyncEvent_t *event) {
    (void)cookie;
    if (!event) return;
    // For now we just print reset events for debugging
    if (event->eventId == SH2_RESET) {
        std::cerr << "SH2 RESET event\n";
    } else if (event->eventId == SH2_SHTP_EVENT) {
        std::cerr << "SH2 SHTP event: " << int(event->sh2SensorConfigResp.sensorId) << "\n";
    }
}

/* sensor service thread — calls sh2_service repeatedly (this will use HAL read/write) */
static void sh2_service_thread() {
    while (g_running) {
        sh2_service();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

/* disk writer thread */
static void disk_thread_func() {
    Sample s;
    while (g_running || g_queue.size() > 0) {
        if (g_queue.pop(s)) {
            g_writer.push_sample(s);
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    g_writer.flush();
}

/* ---------- Main: setup, configure sensors, run for duration ---------- */
int main(int argc, char **argv) {
    int duration_sec = 10;
    if (argc > 1) duration_sec = atoi(argv[1]);

    std::signal(SIGINT, signal_handler);

    // init HAL
    linux_hal_t halWrapper;
    sh2_hal_linux_init(&halWrapper);
    // optionally set bus/address:
    // halWrapper.i2c_bus = "/dev/i2c-1";
    // halWrapper.i2c_addr = 0x4A;

    // open SH2 (pass async event handler)
    if (sh2_open(&halWrapper.hal, async_event_cb, nullptr) != SH2_OK) {
        std::cerr << "sh2_open failed\n";
        return -1;
    }

    // register sensor callback
    if (sh2_setSensorCallback(sensor_callback, nullptr) != SH2_OK) {
        std::cerr << "sh2_setSensorCallback failed\n";
        sh2_close();
        return -1;
    }

    // configure sensors: rotation vector, accel, gyro at 500 Hz (2000 us interval)
    sh2_SensorConfig_t cfg;
    std::memset(&cfg, 0, sizeof(cfg));
    cfg.reportInterval_us = 2000; // 500 Hz
    cfg.changeSensitivityEnabled = false;
    cfg.wakeupEnabled = false;
    cfg.alwaysOnEnabled = false;
    cfg.sniffEnabled = false;

    rc = sh2_setSensorConfig(SH2_ROTATION_VECTOR, &cfg);
    std::cerr << "SH2: setSensorConfig ROTATION_VECTOR -> " << rc << "\n";
    rc = sh2_setSensorConfig(SH2_ACCELEROMETER, &cfg);
    std::cerr << "SH2: setSensorConfig ACCEL -> " << rc << "\n";
    rc = sh2_setSensorConfig(SH2_GYROSCOPE_CALIBRATED, &cfg);
    std::cerr << "SH2: setSensorConfig GYRO -> " << rc << "\n";


    // start service & disk threads
    std::thread svc_thread(sh2_service_thread);
    std::thread disk_thread(disk_thread_func);

    // run for duration
    std::this_thread::sleep_for(std::chrono::seconds(duration_sec));
    g_running = false;

    // join & cleanup
    svc_thread.join();
    disk_thread.join();

    sh2_close();

    std::cout << "Logging complete\n";
    return 0;
}
