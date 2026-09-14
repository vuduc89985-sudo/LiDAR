// lidar_web_gateway_v3.cpp
// Backend headless: WebSocket + LiDAR SDK
// v3: (1) Zone da hinh: rect/polygon/circle/sector/freehand + sua dinh
//     (2) IP dong: nhap IP tren web, them/bot LiDAR, luu lidar_config.cfg
//     (3) Latency: RTT + E2E + frame-to-display
//     (4) Da bo sung day du thong tin Z / do cao (dz) vao CSV va CSV Player
//     (5) Hop nhat uu diem Code 1: Zone.sourceId doc lap voi Zone.id + tuong thich cau hinh cu
//     (6) CSV Player tu dong nhan dien header, tranh nham format 11 cot

#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif
#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0601
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef ASIO_STANDALONE
#define ASIO_STANDALONE
#endif
#ifndef WEBSOCKETPP_CPP11_STL
#define WEBSOCKETPP_CPP11_STL
#endif
// >>> QUAN TRONG: ep websocketpp dung std C++11, KHONG di tim boost <<<
#ifndef _WEBSOCKETPP_CPP11_STL_
#define _WEBSOCKETPP_CPP11_STL_
#endif

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#include <Windows.h>
#undef min
#undef max
#endif
#include <iphlpapi.h>
#include <icmpapi.h>
#pragma comment(lib, "iphlpapi.lib")
#include <stdio.h>
#include <stdarg.h>
#include <thread>
#include <vector>
#include <memory>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <cmath>
#include <mutex>
#include <atomic>
#include <deque>
#include <string>
#include <sstream>
#include <map>
#include <set>
#include <unordered_map>
#include <chrono>
#include <climits>
#include <algorithm>
#include <array>
#include <cstring>
#include "ladar_api.h"
#pragma comment(lib, "ladarsdk_gd.lib")

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <nlohmann/json.hpp>
using json = nlohmann::json;
using WsServer = websocketpp::server<websocketpp::config::asio>;
using ConnHdl = websocketpp::connection_hdl;

static void QueueWebJson(const json& j);
static void BroadcastText(const std::string& s);
static void BroadcastExtr(int id);
static void BroadcastLidarConfig();

// ================= 1. CAU HINH =================
static const int MAX_LIDARS = 16;  // >>> v3: ho tro toi da 16 LiDAR <<<
static const char* LIDAR_CFG_FILE = "lidar_config.cfg";  // >>> v3: file cau hinh IP <<<
const size_t MAX_HISTORY_FRAMES = 500;
std::atomic<bool> gIsPaused(false);
std::atomic<bool> gNewDataAvailable(false);
int gSelectedLidar = 0;
std::atomic<int> gActiveLidarCount(0);  // >>> v3: so LiDAR hien tai <<<

const float PI_F = 3.14159265358979f;
inline float Deg2Rad(float d) { return d * PI_F / 180.0f; }
inline float Rad2Deg(float r) { return r * 180.0f / PI_F; }
const float ANGLE_START = -45.0f;
const float ANGLE_INC = 0.0625f;
inline int GetPointIndexFromAngle(float angleDeg)
{
    return (int)roundf((angleDeg - ANGLE_START) / ANGLE_INC);
}

#include "extrinsic3d.h"
struct Ptf { float x, y; };
// >>> v3.1: ten khong dau cach (de luu vao lidar_config.cfg) <<<
static std::string SanitizeName(const std::string& s)
{
    std::string r = s;
    for (auto& c : r) if (c == ' ' || c == '\t' || c == '\r' || c == '\n') c = '_';
    if (r.empty()) r = "LiDAR";
    return r;
}

// ================= 2. LOG =================
static FILE* gLogFile = NULL;
static std::mutex gLogMutex;
static HANDLE gConOut = NULL;

void LogPrintf(const char* fmt, ...)
{
    char buf[1024]; va_list ap;
    va_start(ap, fmt); vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
    std::lock_guard<std::mutex> lk(gLogMutex);
    if (gConOut) { DWORD w = 0; WriteFile(gConOut, buf, (DWORD)strlen(buf), &w, NULL); }
    if (gLogFile) { fputs(buf, gLogFile); fflush(gLogFile); }
}

// ================= 6. PARAM LOC =================
static const int PERSISTENCE_FRAMES = 3;
static const float MIN_TARGET_W = 0.15f, MIN_TARGET_H = 0.15f;
static const float MAX_TARGET_W = 3.0f, MAX_TARGET_H = 3.0f;
static const int MIN_CLUSTER_POINTS = 8;
static const float ZONE_BUFFER = 0.30f;

// ================= 7. OCCLUSION =================
static int gLowPointFrames = 0;
static bool gOcclusionAlarm = false;
static float gAvgPointCount = 0.0f;
static int gAvgCount = 0;
static const int OCCLUSION_MIN_POINTS = 200;
static const int OCCLUSION_PERSIST_FRAMES = 25;

// ================= 8. MOTION =================
struct DynTarget { int id; float x, y, vx, vy, w, h, z1, z2; int n, age; };
static std::vector<DynTarget> gTargets;
static std::unordered_map<long long, int> gBgSeen;
static int gBgFrames = 0;
static int gNextTargetId = 1;
static bool gPrevAlarm = false;

void ResetMotionModel()
{
    gBgSeen.clear(); gBgFrames = 0; gTargets.clear(); gPrevAlarm = false;
}

// ================= 9. CLUSTER =================
static bool gClusterMode = false;
static DWORD64 gClusterLast = 0;
struct Box { float x1, y1, x2, y2; int n; };
static std::vector<Box> gBoxes;
static float gNearest = 0.0f;
static int gNObj = 0;

// ================= 10. PICK =================
static Ptf gPickRef[2], gPickTarget[2];
static int gPickNRef = 0, gPickNTarget = 0;

// ================= 11/12/13. ICP / RMSE / CFG =================
static bool gIcpRunning = false;
static std::mutex gRmseMutex;
static bool gRmseActive = false;
static float gRmseTrue = 0;
static int gRmseCount = 0;
static double gRmseSum = 0, gRmseSumSq = 0;
static const int gRmseN = 100;
static const char* EXTR_CFG = "extrinsics.cfg";

// ================= 14. POINT CLOUD =================
struct PointCloudData
{
    std::vector<float> x_coords, y_coords, z_coords, local_x, local_y, local_z;
    std::vector<float> intensities, angles_deg, distances;
    int frame_id = 0;
    float range_min = 0, range_max = 0, angle_min = 0, angle_max = 0;
    float angle_increment = 0, scan_time = 0, time_increment = 0;
    int32_t wave_count = 0;
    uint32_t ts_secs = 0, ts_usecs = 0;
    bool has_param = false;
    float origin_x = 0, origin_y = 0, origin_z = 0;
    float origin_roll = 0, origin_pitch = 0, origin_yaw = 0;
    int source_id = 0;
};
using FramePtr = std::shared_ptr<PointCloudData>;

// ================= 15. DO TRE =================
static long long NowUs()
{
    using namespace std::chrono;
    return duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
}

struct LatencyStats
{
    std::atomic<long long> last_us{ 0 }, min_us{ LLONG_MAX }, max_us{ 0 }, sum_us{ 0 };
    std::atomic<int> count{ 0 };
    void Record(long long us)
    {
        if (us < 0) us = 0;
        last_us = us; sum_us += us; count++;
        long long m = min_us.load();
        while (us < m && !min_us.compare_exchange_weak(m, us));
        m = max_us.load();
        while (us > m && !max_us.compare_exchange_weak(m, us));
    }
    void Print(const char* tag)
    {
        if (!count) return;
        LogPrintf("[%s] lan_cuoi=%.2f ms | nho=%.2f | lon=%.2f | TB=%.2f | n=%d\n",
            tag, last_us.load() / 1000.0, min_us.load() / 1000.0,
            max_us.load() / 1000.0, (sum_us.load() / 1000.0) / count.load(), count.load());
        json j; j["type"] = "latency"; j["tag"] = tag;
        j["last"] = last_us.load() / 1000.0; j["min"] = min_us.load() / 1000.0;
        j["max"] = max_us.load() / 1000.0;
        j["avg"] = (sum_us.load() / 1000.0) / count.load(); j["n"] = count.load();
        QueueWebJson(j);
    }
};
static LatencyStats gRttStats, gE2eStats, gPackStats;   // v3.3: gPackStats = "ping" goi 4320 tia
static std::atomic<long long> gMinDeltaUs{ LLONG_MAX };
static std::atomic<int>       gMinResetCnt{ 0 };
static std::atomic<long long   > gCmdSentUs{ 0 };
static std::atomic<uint16_t>  gCmdExpect{ 0 };
static std::atomic<long long> gConnectUs{ 0 };
static std::atomic<bool>      gGotFirstFrame{ false };
static std::atomic<long long> gBaseOffsetUs{ 0 };

void MeasureCommandLatency(ILadar* ladar, bool sleep)
{
    if (!ladar) return;
    gCmdSentUs = NowUs();
    if (sleep) { gCmdExpect = 7140; ladar->Standby(1); LogPrintf("[DO TRE] Gui lenh NGU...\n"); }
    else { gCmdExpect = 110;  ladar->Standby(0); LogPrintf("[DO TRE] Gui lenh THUC...\n"); }
}

void CheckCommandResponse(uint16_t cmdWord)
{
    long long t0 = gCmdSentUs.load();
    if (t0 && cmdWord == gCmdExpect.load())
    {
        gRttStats.Record(NowUs() - t0); gCmdSentUs = 0; gRttStats.Print("RTT");
    }
}

void CheckDataLatency(const ladar_data_t& data)
{
    if (!gGotFirstFrame.exchange(true))
    {
        long long t0 = gConnectUs.load();
        if (t0) LogPrintf("[DO TRE] Ket noi -> frame dau: %.2f ms\n", (NowUs() - t0) / 1000.0);
    }
    if (data.flags & dsTimeStamp)
    {
        long long lidarUs = (long long)data.ts_secs * 1000000LL + data.ts_usecs;
        long long delta = NowUs() - lidarUs;
        if (gBaseOffsetUs.load() == 0) { gBaseOffsetUs = delta; }
        gE2eStats.Record(delta - gBaseOffsetUs.load());

        // >>> v3.4: GOI4320 TU BU LECH DONG HO (khong can SetTime) <<<
        long long m = gMinDeltaUs.load();
        if (delta < m) gMinDeltaUs.store(delta);
        if (++gMinResetCnt >= 250) { gMinResetCnt = 0; gMinDeltaUs = delta; }  // choi lai dinh ky de theo troi dong ho
        long long baseUs = (long long)((data.scan_time > 0 ? data.scan_time : 0.04f) * 1000000LL) + 3000;
        long long shown = delta - gMinDeltaUs.load() + baseUs;
        gPackStats.Record(shown);
    }
}

int GetNowString(char* sNow, int size)
{
    SYSTEMTIME now; GetLocalTime(&now);
    sprintf_s(sNow, size, "%04d-%02d-%02d %02d:%02d:%02d.%03d",
        now.wYear, now.wMonth, now.wDay, now.wHour, now.wMinute, now.wSecond, now.wMilliseconds);
    return (int)strlen(sNow);
}

// ================= v3: ZONE DA HINH =================
// >>> v3: Zone ho tro nhieu loai hinh <<<
enum ZoneType { ZT_RECT = 0, ZT_POLYGON = 1, ZT_CIRCLE = 2, ZT_SECTOR = 3, ZT_FREEHAND = 4 };

struct Zone
{
    bool local = false;
    int id = 0;
    int sourceId = 0;  // LiDAR whose local coordinate frame owns this Zone
    int type = ZT_RECT;  // ZoneType
    // Rectangle (giu nguyen tuong thich cu)
    float x1 = 0, y1 = 0, x2 = 0, y2 = 0;
    // Polygon / Freehand: danh sach dinh
    std::vector<Ptf> vertices;
    // Circle: tam + ban kinh
    float cx = 0, cy = 0, radius = 0;
    // Sector: tam + ban kinh + goc bat dau/ket thuc (radian)
    float sx = 0, sy = 0, sRadius = 0, sAngleStart = 0, sAngleEnd = 0;

    // >>> v3: Kiem tra diem co nam trong zone khong <<<
    bool ContainsPoint(float px, float py) const
    {
        switch (type)
        {
        case ZT_RECT:
            return px >= x1 && px <= x2 && py >= y1 && py <= y2;

        case ZT_POLYGON:
        case ZT_FREEHAND:
        {
            if (vertices.size() < 3) return false;
            // Ray casting algorithm
            bool inside = false;
            int n = (int)vertices.size();
            for (int i = 0, j = n - 1; i < n; j = i++)
            {
                float xi = vertices[i].x, yi = vertices[i].y;
                float xj = vertices[j].x, yj = vertices[j].y;
                if (((yi > py) != (yj > py)) &&
                    (px < (xj - xi) * (py - yi) / (yj - yi) + xi))
                    inside = !inside;
            }
            return inside;
        }

        case ZT_CIRCLE:
        {
            float dx = px - cx, dy = py - cy;
            return (dx * dx + dy * dy) <= (radius * radius);
        }

        case ZT_SECTOR:
        {
            float dx = px - sx, dy = py - sy;
            float dist = sqrtf(dx * dx + dy * dy);
            if (dist > sRadius) return false;
            float angle = atan2f(dy, dx);
            // Chuan hoa goc vao [0, 2*PI)
            while (angle < 0) angle += 2.0f * PI_F;
            while (angle >= 2.0f * PI_F) angle -= 2.0f * PI_F;
            float aStart = sAngleStart, aEnd = sAngleEnd;
            while (aStart < 0) aStart += 2.0f * PI_F;
            while (aEnd < 0) aEnd += 2.0f * PI_F;
            if (aStart <= aEnd)
                return angle >= aStart && angle <= aEnd;
            else
                return angle >= aStart || angle <= aEnd;
        }

        default:
            return false;
        }
    }
};

static std::vector<Zone> gZones;
static std::mutex gZoneMutex;

// ================= 16. IFrameSource =================
class IFrameSource
{
public:
    virtual ~IFrameSource() {}
    virtual bool Start() = 0;
    virtual void Stop() = 0;
    virtual bool IsRunning() const = 0;
    virtual FramePtr GetFrameAtOffset(int offset) = 0;
    virtual void SetExtr(const Extrinsic3D& e) = 0;
    virtual Extrinsic3D GetExtr() = 0;
    virtual std::string GetName() const = 0;
    virtual ILadar* GetLadar() { return nullptr; }
    virtual void SetEnableCsvRecord(bool e) { (void)e; }
    virtual void SetPaused(bool p) { (void)p; }
    virtual void SetName(const std::string& n) { (void)n; }   // >>> v3.1: doi ten <<<
    // >>> v3: ho tro reconnect voi IP moi <<<
    virtual bool Reconnect(const std::string& newIp, uint16_t newPort) { return false; }
    virtual std::string GetIp() const { return ""; }
    virtual uint16_t GetPort() const { return 0; }
};

// ================= 17. CSV PLAYER =================
class CsvLidarPlayer : public IFrameSource
{
    std::string csvPath, name;
    std::string csvHeader;
    std::ifstream csvFile;
    std::thread playerThread;
    std::atomic<bool> isRunning{ false }, isPaused{ false };
    FramePtr building = std::make_shared<PointCloudData>();
    FramePtr latest;
    std::deque<FramePtr> history;
    std::mutex dataMutex, extrMutex;
    Extrinsic3D extr;
    int sourceId = 0;
public:
    CsvLidarPlayer(int id, const std::string& n, const std::string& p, const Extrinsic3D& e = Extrinsic3D())
        : name(n), csvPath(p), extr(e), sourceId(id) {}
    ~CsvLidarPlayer() { Stop(); }
    std::string GetName() const override { return name; }
    void SetName(const std::string& n) override { name = SanitizeName(n); }
    void SetExtr(const Extrinsic3D& e) override { std::lock_guard<std::mutex> lk(extrMutex); extr = e; }
    Extrinsic3D GetExtr() override { std::lock_guard<std::mutex> lk(extrMutex); return extr; }
    void SetPaused(bool p) override { isPaused = p; }
    bool Start() override
    {
        if (isRunning.load()) return true;
        csvFile.open(csvPath);
        if (!csvFile.is_open()) { LogPrintf("[ERROR] Cannot open CSV: %s\n", csvPath.c_str()); return false; }
        std::getline(csvFile, csvHeader);
        isRunning = true;
        playerThread = std::thread(&CsvLidarPlayer::PlayerLoop, this);
        LogPrintf("[OK] CSV Started: %s\n", csvPath.c_str());
        return true;
    }
    void Stop() override
    {
        if (!isRunning.load()) return;
        isRunning = false;
        if (playerThread.joinable()) playerThread.join();
        if (csvFile.is_open()) csvFile.close();
    }
    bool IsRunning() const override { return isRunning.load(); }
    FramePtr GetFrameAtOffset(int o) override
    {
        std::lock_guard<std::mutex> lock(dataMutex);
        if (o <= 0 || history.empty()) return latest;
        int idx = (int)history.size() - 1 - o;
        if (idx < 0) idx = 0;
        return history[idx];
    }
private:
    void PlayerLoop()
    {
        int lastFrameId = -1, frameSeq = 0, replayed = 0;
        auto startTime = std::chrono::high_resolution_clock::now();
        Extrinsic3D curExtr; Transform3D transform(curExtr);
        while (isRunning.load())
        {
            if (isPaused.load()) { std::this_thread::sleep_for(std::chrono::milliseconds(50)); continue; }
            std::string line;
            if (!std::getline(csvFile, line))
            {
                csvFile.clear(); csvFile.seekg(0);
                std::getline(csvFile, line);
                startTime = std::chrono::high_resolution_clock::now();
                replayed = 0; lastFrameId = -1; continue;
            }
            std::stringstream ss(line); std::string token;
            std::vector<std::string> tok;
            while (std::getline(ss, token, ',')) tok.push_back(token);

            // >>> CAP NHAT:
            //     Ho tro 4 format CSV:
            //       10 cot = cu, khong Source_ID, khong Local_Z
            //       11 cot = cu, co Source_ID, khong Local_Z
            //       11 cot = moi, khong Source_ID, co Local_Z
            //       12 cot = moi, co Source_ID, co Local_Z
            //
            //     Vi format 11 cot bi mo ho, uu tien nhan dien theo header
            //     (Start() da doc header) va fallback theo so cot.
            bool hasSrc = false;
            bool hasLocalZ = false;

            try
            {
                // Header CSV duoc luu rieng de phan biet 11-cot old/new.
                // Neu header khong co (file cu/khac), dung fallback an toan.
                // Chuoi header duoc cache trong member csvHeader.
                if (!csvHeader.empty()) {
                    hasSrc = (csvHeader.find("Source_ID") != std::string::npos);
                    hasLocalZ = (csvHeader.find("Local_Z") != std::string::npos);
                }
                else {
                    if (tok.size() == 12) {
                        hasSrc = true; hasLocalZ = true;
                    }
                    else if (tok.size() == 11) {
                        // Ambiguous: assume new no-source format only if Local_Z
                        // can be parsed consistently; otherwise use old source format.
                        hasSrc = false; hasLocalZ = true;
                    }
                    else {
                        hasSrc = (tok.size() >= 11);
                        hasLocalZ = false;
                    }
                }

                int base = hasSrc ? 1 : 0;
                int required = base + (hasLocalZ ? 11 : 10);
                if ((int)tok.size() < required) continue;

                int frameId = std::stoi(tok[base + 0]);
                uint32_t tsS = (uint32_t)std::stoul(tok[base + 1]);
                uint32_t tsU = (uint32_t)std::stoul(tok[base + 2]);
                float angleDeg = std::stof(tok[base + 4]);
                float distance = std::stof(tok[base + 5]);
                float x = std::stof(tok[base + 6]);
                float y = std::stof(tok[base + 7]);

                float lz = 0.0f;
                float inten = 0.0f;
                float scanTime = 0.0f;

                if (hasLocalZ) {
                    lz = std::stof(tok[base + 8]);
                    inten = std::stof(tok[base + 9]);
                    scanTime = std::stof(tok[base + 10]);
                }
                else {
                    inten = std::stof(tok[base + 8]);
                    scanTime = std::stof(tok[base + 9]);
                }

                if (frameId != lastFrameId)
                {
                    if (lastFrameId >= 0)
                    {
                        auto expT = startTime + std::chrono::milliseconds(replayed * 40);
                        auto nowT = std::chrono::high_resolution_clock::now();
                        if (expT > nowT) std::this_thread::sleep_until(expT);
                    }
                    lastFrameId = frameId; replayed++;
                    std::lock_guard<std::mutex> lock(dataMutex);
                    int prev = (int)building->x_coords.size();
                    if (prev > 0)
                    {
                        building->frame_id = frameSeq++;
                        history.push_back(building);
                        if (history.size() > MAX_HISTORY_FRAMES) history.pop_front();
                        latest = building;
                    }
                    FramePtr f = std::make_shared<PointCloudData>();
                    f->source_id = sourceId; f->ts_secs = tsS; f->ts_usecs = tsU;
                    curExtr = GetExtr(); transform = Transform3D(curExtr);
                    f->origin_x = curExtr.x; f->origin_y = curExtr.y; f->origin_yaw = curExtr.yaw;
                    f->origin_z = curExtr.z; f->origin_roll = curExtr.roll; f->origin_pitch = curExtr.pitch;
                    f->range_max = 80.0f;
                    f->angle_min = Deg2Rad(-45.0f); f->angle_max = Deg2Rad(225.0f);
                    f->angle_increment = Deg2Rad(0.0625f);
                    f->scan_time = scanTime;
                    f->time_increment = prev ? scanTime / (float)prev : 0;
                    f->wave_count = 1; f->has_param = true;
                    building = f;
                    gNewDataAvailable = true;
                }
                std::lock_guard<std::mutex> lock(dataMutex);
                building->local_x.push_back(x);
                building->local_y.push_back(y);
                building->local_z.push_back(lz); // <<< THEM: Luu local_z tu CSV

                const auto world = transform.Apply(x, y);
                building->x_coords.push_back(world.x);
                building->y_coords.push_back(world.y);
                building->z_coords.push_back(world.z);

                building->intensities.push_back(inten);
                building->angles_deg.push_back(angleDeg);
                building->distances.push_back(distance);
            }
            catch (...) { continue; }
        }
    }
};

// ================= 18. REAL LIDAR =================
class RealLidarSource;
static void PrintLidarInfo(int sourceId, const char* name, ILadar* ladar_, const ladar_data_t& data, int curId);
static std::map<ILadar*, RealLidarSource*> gLidarMap;
static std::mutex gLidarMapMutex;
std::ofstream gCsvFile;
std::mutex gCsvMutex;
std::atomic<bool> gIsRecording(false);

class RealLidarSource : public IFrameSource
{
    std::string name, ip;
    uint16_t port;
    int sourceId;
    ILadar* ladar = nullptr;
    ILadarPoller* poller = nullptr;
    std::thread pollerThread;
    std::thread syncThread;
    std::atomic<bool> isRunning{ false };
    FramePtr latest;
    std::deque<FramePtr> history;
    std::mutex dataMutex, extrMutex;
    Extrinsic3D extr;
    std::atomic<int> frameCount{ 0 };
public:
    bool enableCsvRecord = true;

    RealLidarSource(int id, const std::string& n, const std::string& ip_, uint16_t p, const Extrinsic3D& e = Extrinsic3D())
        : name(n), ip(ip_), port(p), sourceId(id), extr(e) {}
    ~RealLidarSource() { Stop(); }

    std::string GetName() const override { return name; }
    void SetName(const std::string& n) override { name = SanitizeName(n); }
    std::string GetIp() const override { return ip; }
    uint16_t GetPort() const override { return port; }
    void SetExtr(const Extrinsic3D& e) override { std::lock_guard<std::mutex> lk(extrMutex); extr = e; }
    Extrinsic3D GetExtr() override { std::lock_guard<std::mutex> lk(extrMutex); return extr; }
    ILadar* GetLadar() override { return ladar; }
    void SetEnableCsvRecord(bool e) override { enableCsvRecord = e; }
    bool IsRunning() const override { return isRunning.load(); }

    FramePtr GetFrameAtOffset(int o) override
    {
        std::lock_guard<std::mutex> lock(dataMutex);
        if (o <= 0 || history.empty()) return latest;
        int idx = (int)history.size() - 1 - o;
        if (idx < 0) idx = 0;
        return history[idx];
    }

    // >>> v3: Reconnect voi IP/port moi <<<
    bool Reconnect(const std::string& newIp, uint16_t newPort) override
    {
        LogPrintf("[IP] %s: Reconnect %s:%d -> %s:%d\n", name.c_str(), ip.c_str(), port, newIp.c_str(), newPort);
        Stop();
        ip = newIp;
        port = newPort;
        frameCount = 0;
        gGotFirstFrame = false;
        gConnectUs = NowUs();
        return Start();
    }

    static void OnFrameCallback(ILadar* ladar_, const uint8_t* frame, int frameLength)
    {
        RealLidarSource* self = nullptr;
        {
            std::lock_guard<std::mutex> lock(gLidarMapMutex);
            auto it = gLidarMap.find(ladar_);
            if (it == gLidarMap.end()) return;
            self = it->second;
        }
        uint16_t cmdWord = (uint16_t)(frame[2] | (frame[3] << 8));
        CheckCommandResponse(cmdWord);
        if (gIsPaused.load()) return;
        if (cmdWord != 110) return;
        ladar_data_t data;
        if (ladar_->GetData(data, frame, frameLength) <= 0) return;
        if (self->sourceId == 0) CheckDataLatency(data);
        int curId = self->frameCount.load();
        PrintLidarInfo(self->sourceId, self->name.c_str(), ladar_, data, curId);
        if (self->sourceId == 0 && gRmseActive)
        {
            int idx = GetPointIndexFromAngle(0.0f);
            if (idx >= 0 && idx < (int)data.ranges.size())
            {
                float d = data.ranges[idx];
                if (d > 0 && d < data.range_max)
                {
                    std::lock_guard<std::mutex> lk(gRmseMutex);
                    double err = (double)d - (double)gRmseTrue;
                    gRmseSum += err; gRmseSumSq += err * err; gRmseCount++;
                    if (gRmseCount >= gRmseN)
                    {
                        double mean = gRmseSum / gRmseCount;
                        double rmse = sqrt(gRmseSumSq / gRmseCount);
                        double var = gRmseSumSq / gRmseCount - mean * mean;
                        if (var < 0) var = 0;
                        LogPrintf("[RMSE] N=%d | TB=%.3f | RMSE=%.3f | std=%.3f | true=%.3f\n",
                            gRmseCount, mean, rmse, sqrt(var), gRmseTrue);
                        json r; r["type"] = "rmse"; r["mean"] = mean; r["rmse"] = rmse;
                        r["std"] = sqrt(var); r["true"] = gRmseTrue;
                        QueueWebJson(r);
                        gRmseActive = false;
                    }
                }
            }
        }
        if (self->sourceId == 0 && curId % 50 == 0) { gE2eStats.Print("E2E"); gPackStats.Print("GOI4320"); }
        FramePtr f = std::make_shared<PointCloudData>();
        Extrinsic3D e = self->GetExtr();
        Transform3D transform(e);
        f->source_id = self->sourceId; f->frame_id = curId;
        f->range_min = data.range_min; f->range_max = data.range_max;
        f->angle_min = data.angle_min; f->angle_max = data.angle_max;
        f->angle_increment = data.angle_increment;
        f->scan_time = data.scan_time; f->time_increment = data.time_increment;
        f->wave_count = data.wave_count;
        f->ts_secs = data.ts_secs; f->ts_usecs = data.ts_usecs;
        f->origin_x = e.x; f->origin_y = e.y; f->origin_yaw = e.yaw;
        f->origin_z = e.z; f->origin_roll = e.roll; f->origin_pitch = e.pitch;
        f->has_param = true;
        float angle = data.angle_min;
        for (size_t i = 0; i < data.ranges.size(); i++)
        {
            float d = data.ranges[i];
            if (d > 0.0f && d < data.range_max)
            {
                float lx = d * cosf(angle), ly = d * sinf(angle);
                f->local_x.push_back(lx); f->local_y.push_back(ly);
                f->local_z.push_back(0); // TR70 2D: diem do nam tren mat phang local Z=0; e.z la do cao cam bien
                const auto world = transform.Apply(lx, ly);
                f->x_coords.push_back(world.x); f->y_coords.push_back(world.y); f->z_coords.push_back(world.z);
                f->angles_deg.push_back(angle * 180.0f / PI_F);
                f->distances.push_back(d);
                f->intensities.push_back(i < data.intensities.size() ? data.intensities[i] : 0.0f);
            }
            angle += data.angle_increment;
        }
        {
            std::lock_guard<std::mutex> lock(self->dataMutex);
            self->history.push_back(f);
            if (self->history.size() > MAX_HISTORY_FRAMES) self->history.pop_front();
            self->latest = f;
        }
        if (gIsRecording.load() && self->enableCsvRecord)
        {
            std::lock_guard<std::mutex> lk(gCsvMutex);
            if (gCsvFile.is_open())
            {
                float a2 = data.angle_min;
                for (size_t i = 0; i < data.ranges.size(); i++)
                {
                    if (data.ranges[i] > 0 && data.ranges[i] < data.range_max)
                    {
                        float lx = data.ranges[i] * cosf(a2), ly = data.ranges[i] * sinf(a2);
                        float inten = i < data.intensities.size() ? data.intensities[i] : 0.0f;
                        gCsvFile << (self->sourceId + 1) << "," << curId << ","
                            << data.ts_secs << "," << data.ts_usecs << "," << i << ","
                            << std::fixed << std::setprecision(4) << (a2 * 180.0f / PI_F) << ","
                            << data.ranges[i] << "," << lx << "," << ly << ","
                            << std::setprecision(4) << e.z << ","   // <<< THEM: Local_Z (do cao dz)
                            << std::setprecision(2) << inten << ","
                            << std::setprecision(4) << data.scan_time << "\n";
                    }
                    a2 += data.angle_increment;
                }
            }
        }
        self->frameCount++;
        gNewDataAvailable = true;
    }

    static void OnStatusCallback(ILadar* ladar_, uint32_t, uint32_t status)
    {
        RealLidarSource* self = nullptr;
        {
            std::lock_guard<std::mutex> lock(gLidarMapMutex);
            auto it = gLidarMap.find(ladar_);
            if (it == gLidarMap.end()) return;
            self = it->second;
        }
        char sNow[64]; GetNowString(sNow, sizeof(sNow));
        const char* ss[] = { "closed", "connecting...", "connected", "connect error" };
        LogPrintf("[%s] %s: %s\n", sNow, self->name.c_str(), status <= 3 ? ss[status] : "unknown");
        if (status == csConnected)
        {
            int ok = 0;
            for (int i = 0; i < 5 && !(ok = (ladar_->SetTime() > 0)); i++) Sleep(300);
            LogPrintf("[DO TRE] SetTime %s\n", ok ? "OK" : "FAIL");
            if (ok) gBaseOffsetUs = 0;   // cho E2E hoc lai moc sau khi dong bo
        }
    }

    bool Start() override
    {
        if (isRunning.load()) return true;
        poller = CreateLadarPoller();
        if (!poller) return false;
        ladar = poller->AddLadar(ip.c_str(), port, OnFrameCallback, OnStatusCallback);
        if (!ladar) { DeleteLadarPoller(poller); poller = nullptr; return false; }
        { std::lock_guard<std::mutex> lock(gLidarMapMutex); gLidarMap[ladar] = this; }
        isRunning = true;
        pollerThread = std::thread([this]() { poller->Run(); });
        syncThread = std::thread([this]() {
            int tick = 0;
            while (isRunning.load()) {
                Sleep(1000); if (++tick < 60) continue;
                tick = 0;
                if (ladar && ladar->IsConnected()) {
                    int ok = ladar->SetTime() > 0;
                    LogPrintf("[DO TRE] %s: SetTime dinh ky %s\n", name.c_str(), ok ? "OK" : "FAIL");
                    if (ok && sourceId == 0) gBaseOffsetUs = 0;  // E2E hoc lai moc
                }
            }
            });
        LogPrintf("[OK] LiDAR %s:%d (%s)\n", ip.c_str(), port, name.c_str());
        return true;
    }

    void Stop() override
    {
        if (!isRunning.load()) return;
        isRunning = false;
        if (syncThread.joinable()) syncThread.join();
        if (ladar) { std::lock_guard<std::mutex> lock(gLidarMapMutex); gLidarMap.erase(ladar); }
        if (poller)
        {
            poller->Stop();
            if (pollerThread.joinable()) pollerThread.join();
            DeleteLadarPoller(poller); poller = nullptr;
        }
        ladar = nullptr;
    }
};

// >>> PrintLidarInfo (giu nguyen) <<<
static void PrintLidarInfo(int sourceId, const char* name, ILadar* ladar_, const ladar_data_t& data, int curId)
{
    int period = (sourceId == 0) ? 50 : 200;
    if (curId % period != 0) return;
    char sNow[64]; GetNowString(sNow, sizeof(sNow));
    LogPrintf("\n");
    LogPrintf("+--------------------------------------------------------------+\n");
    LogPrintf("| [%s] LiDAR %d - %s | Frame #%d\n", sNow, sourceId + 1, name, curId);
    LogPrintf("+--------------------------------------------------------------+\n");
    LogPrintf("| [SCAN]\n");
    LogPrintf("|   Timestamp    : %u.%06u s\n", data.ts_secs, data.ts_usecs);
    LogPrintf("|   Scan time    : %.4f s\n", data.scan_time);
    LogPrintf("|   Time incr    : %.6f s\n", data.time_increment);
    LogPrintf("|   Wave count   : %d\n", data.wave_count);
    LogPrintf("|   Points/frame : %zu\n", data.ranges.size());
    LogPrintf("| [GEOMETRY]\n");
    LogPrintf("|   Angle range  : %7.2f -> %7.2f deg (incr %.4f deg)\n",
        Rad2Deg(data.angle_min), Rad2Deg(data.angle_max), Rad2Deg(data.angle_increment));
    LogPrintf("|   Range        : %5.2f m -> %6.2f m\n", data.range_min, data.range_max);
    LogPrintf("|   Flags        : 0x%02X (%s%s%s%s%s)\n", data.flags,
        (data.flags & dsSyncInfo) ? "SyncInfo " : "",
        (data.flags & dsTimeStamp) ? "TimeStamp " : "",
        (data.flags & dsLogin) ? "Login " : "",
        (data.flags & dsParam) ? "Param " : "",
        (data.flags & dsDataFmt) ? "DataFmt " : "");
    ladar_param_t param;
    ladar_->GetParam(param);
    LogPrintf("| [ladar_param_t]\n");
    LogPrintf("|   flags        : 0x%08X\n", param.flags);
    if (param.flags & dsParam)
    {
        LogPrintf("|   radius       : %.1f m\n", param.radius);
        LogPrintf("|   ticks        : %.4f s (chu ky quet)\n", param.ticks);
    }
    if (param.flags & dsDataFmt)
    {
        LogPrintf("|   angle_start  : %.2f deg\n", Rad2Deg(param.angle_start));
        LogPrintf("|   angle_stop   : %.2f deg\n", Rad2Deg(param.angle_stop));
        LogPrintf("|   angle_step   : %.4f deg\n", Rad2Deg(param.angle_step));
        LogPrintf("|   wave_count   : %d\n", param.wave_count);
        LogPrintf("|   wave_types   : [%u, %u]\n", param.wave_types[0], param.wave_types[1]);
        LogPrintf("|   intensity    : %s\n", param.intensity ? "YES" : "NO");
    }
    LogPrintf("| [SAMPLE 7 GOC]\n");
    LogPrintf("|   %-7s | %-8s | %-10s | %-9s | %-5s\n",
        "Goc", "Pt_Idx", "Distance", "Intensity", "HopLe");
    LogPrintf("|   --------+----------+------------+-----------+------\n");
    const float TEST_ANGLES[7] = { -45.0f, 0.0f, 45.0f, 90.0f, 135.0f, 180.0f, 225.0f };
    for (int i = 0; i < 7; i++)
    {
        float a = TEST_ANGLES[i];
        int idx = GetPointIndexFromAngle(a);
        if (idx >= 0 && idx < (int)data.ranges.size())
        {
            float dist = data.ranges[idx];
            float inten = (!data.intensities.empty() && idx < (int)data.intensities.size())
                ? data.intensities[idx] : 0.0f;
            bool valid = (dist > 0.0f && dist < data.range_max);
            LogPrintf("|   %6.2f | %8d | %9.4f m | %9.2f | %-5s\n",
                a, idx, dist, inten, valid ? "CO" : "KHONG");
        }
        else
            LogPrintf("|   %6.2f | %-8s | %-10s | %-9s | %-5s\n", a, "N/A", "N/A", "N/A", "OUT");
    }
    LogPrintf("+--------------------------------------------------------------+\n\n");
}

// ================= 19. SOURCES =================
std::vector<std::shared_ptr<IFrameSource>> gSources;
static std::recursive_mutex gSourcesMutex;

// >>> v3: Cau hinh LiDAR dong <<<
struct LidarConfigEntry
{
    std::string ip;
    uint16_t port = 1112;
    std::string name;
};

static void SaveLidarConfig()
{
    FILE* f = fopen(LIDAR_CFG_FILE, "w");
    if (!f) return;
    for (size_t i = 0; i < gSources.size(); i++)
    {
        auto src = std::dynamic_pointer_cast<RealLidarSource>(gSources[i]);
        if (src)
            fprintf(f, "%s %d %s\n", src->GetIp().c_str(), src->GetPort(), src->GetName().c_str());
        else
            fprintf(f, "csv 0 %s\n", gSources[i]->GetName().c_str());
    }
    fclose(f);
    LogPrintf("[CFG] Da luu %s (%d LiDAR)\n", LIDAR_CFG_FILE, (int)gSources.size());
    QueueWebJson({ {"type","log"},{"text","[CFG] Da luu lidar_config.cfg"} });
}

static bool LoadLidarConfig(std::vector<LidarConfigEntry>& entries)
{
    FILE* f = fopen(LIDAR_CFG_FILE, "r");
    if (!f) return false;
    char ipBuf[64], nameBuf[64];
    int port;
    while (fscanf(f, "%63s %d %63s", ipBuf, &port, nameBuf) == 3)
    {
        LidarConfigEntry e;
        e.ip = ipBuf; e.port = (uint16_t)port; e.name = nameBuf;
        entries.push_back(e);
    }
    fclose(f);
    return !entries.empty();
}

static void BroadcastLidarConfig()
{
    json arr = json::array();
    for (size_t i = 0; i < gSources.size(); i++)
    {
        auto src = std::dynamic_pointer_cast<RealLidarSource>(gSources[i]);
        json item;
        item["id"] = (int)i;
        item["name"] = gSources[i]->GetName();
        if (src)
        {
            item["ip"] = src->GetIp();
            item["port"] = src->GetPort();
            item["connected"] = src->GetLadar() && src->GetLadar()->IsConnected();
        }
        else
        {
            item["ip"] = "csv";
            item["port"] = 0;
            item["connected"] = gSources[i]->IsRunning();
        }
        Extrinsic3D e = gSources[i]->GetExtr();
        item["dx"] = e.x; item["dy"] = e.y; item["thetaDeg"] = Rad2Deg(e.yaw);
        item["x"] = e.x; item["y"] = e.y; item["z"] = e.z; item["roll"] = Rad2Deg(e.roll); item["pitch"] = Rad2Deg(e.pitch); item["yaw"] = Rad2Deg(e.yaw);
        arr.push_back(item);
    }
    json out; out["type"] = "lidar_config"; out["lidars"] = arr; out["count"] = (int)gSources.size();
    QueueWebJson(out);
}

// ================= WEB CORE =================
static WsServer g_server;
static std::set<ConnHdl, std::owner_less<ConnHdl>> g_conns;
static std::mutex g_connMutex;
static std::mutex gWebJsonMutex;
static std::vector<std::string> gWebJsonQueue;

static void appendU8(std::vector<uint8_t>& b, uint8_t v) { b.push_back(v); }
static void appendU16(std::vector<uint8_t>& b, uint16_t v)
{
    b.push_back(v & 0xFF); b.push_back((v >> 8) & 0xFF);
}
static void appendU32(std::vector<uint8_t>& b, uint32_t v)
{
    b.push_back(v & 0xFF); b.push_back((v >> 8) & 0xFF); b.push_back((v >> 16) & 0xFF); b.push_back((v >> 24) & 0xFF);
}
static void appendF32(std::vector<uint8_t>& b, float v)
{
    static_assert(sizeof(float) == 4, "Protocol requires IEEE-754 float32");
    uint32_t bits; memcpy(&bits, &v, 4); appendU32(b, bits);
}

static bool HasClients() { std::lock_guard<std::mutex> lk(g_connMutex); return !g_conns.empty(); }
static void BroadcastText(const std::string& s)
{
    std::lock_guard<std::mutex> lk(g_connMutex);
    for (auto h : g_conns)
    {
        websocketpp::lib::error_code ec; g_server.send(h, s, websocketpp::frame::opcode::value::text, ec);
    }
}
static void BroadcastBinary(const std::vector<uint8_t>& p)
{
    if (p.empty()) return;
    std::lock_guard<std::mutex> lk(g_connMutex);
    for (auto h : g_conns)
    {
        websocketpp::lib::error_code ec; g_server.send(h, p.data(), p.size(), websocketpp::frame::opcode::value::binary, ec);
    }
}
static void QueueWebJson(const json& j)
{
    std::lock_guard<std::mutex> lk(gWebJsonMutex); gWebJsonQueue.push_back(j.dump());
}
static void DrainWebJson()
{
    std::vector<std::string> out;
    { std::lock_guard<std::mutex> lk(gWebJsonMutex); out.swap(gWebJsonQueue); }
    for (auto& s : out) BroadcastText(s);
}
static void BroadcastExtr(int id)
{
    if (id < 0 || id >= (int)gSources.size()) return;
    Extrinsic3D e = gSources[id]->GetExtr();
    json j; j["type"] = "extr"; j["id"] = id;
    j["dx"] = e.x; j["dy"] = e.y; j["thetaDeg"] = Rad2Deg(e.yaw);
    j["x"] = e.x; j["y"] = e.y; j["z"] = e.z; j["roll"] = Rad2Deg(e.roll); j["pitch"] = Rad2Deg(e.pitch); j["yaw"] = Rad2Deg(e.yaw);
    QueueWebJson(j);
}

// ================= 20. EXTRINSIC HELPERS =================
IFrameSource* GetSelectedSource()
{
    if (gSelectedLidar >= 0 && gSelectedLidar < (int)gSources.size())
        return gSources[gSelectedLidar].get();
    return nullptr;
}

void SaveExtrConfig()
{
    FILE* f = fopen(EXTR_CFG, "w");
    if (!f) return;
    for (int i = 0; i < (int)gSources.size(); i++)
    {
        Extrinsic3D e = gSources[i]->GetExtr();
        fprintf(f, "%.4f %.4f %.4f %.4f %.4f %.4f\n", e.x, e.y, e.z, Rad2Deg(e.roll), Rad2Deg(e.pitch), Rad2Deg(e.yaw));
    }
    fclose(f);
    LogPrintf("[CFG] Da luu %s (%d dong)\n", EXTR_CFG, (int)gSources.size());
    QueueWebJson({ {"type","log"},{"text","[CFG] Da luu extrinsics.cfg"} });
}

bool LoadExtrConfig(std::vector<Extrinsic3D>& extrs)
{
    std::ifstream f(EXTR_CFG);
    if (!f) return false;
    auto loaded = extrs;
    std::string line; size_t i = 0;
    while (i < loaded.size() && std::getline(f, line)) {
        if (line.find_first_not_of(" \t\r") == std::string::npos || line[line.find_first_not_of(" \t\r")] == '#') continue;
        if (!ParseExtrinsic(line, loaded[i++])) return false;
    }
    if (i != loaded.size()) return false;
    extrs = loaded; return true;
}

void AdjustSelectedExtr(float ddx, float ddy, float dthRad)
{
    IFrameSource* src = GetSelectedSource();
    if (!src) return;
    Extrinsic3D e = src->GetExtr();
    e.x += ddx; e.y += ddy; e.yaw += dthRad;
    src->SetExtr(e);
    ResetMotionModel();
    LogPrintf("[EXTRINSIC %s] dx=%.2f dy=%.2f th=%.2f\n", src->GetName().c_str(), e.x, e.y, Rad2Deg(e.yaw));
    BroadcastExtr(gSelectedLidar);
}

void PrintSelectedExtr()
{
    IFrameSource* src = GetSelectedSource();
    if (!src) return;
    Extrinsic3D e = src->GetExtr();
    char b[240];
    sprintf_s(b, "[EXTRINSIC %s] { x=%.3f m, y=%.3f m, z=%.3f m, roll=%.2f deg, pitch=%.2f deg, yaw=%.2f deg }",
        src->GetName().c_str(), e.x, e.y, e.z, Rad2Deg(e.roll), Rad2Deg(e.pitch), Rad2Deg(e.yaw));
    LogPrintf("%s\n", b);
    QueueWebJson({ {"type","log"},{"text",b} });
}

void ApplyPickCalib()
{
    if (gSelectedLidar == 0) { LogPrintf("[PICK] Chon L2..L4.\n"); return; }
    Ptf v1a = gPickRef[0], v1b = gPickRef[1], v2a = gPickTarget[0], v2b = gPickTarget[1];
    float a1 = atan2f(v1b.y - v1a.y, v1b.x - v1a.x);
    float a2 = atan2f(v2b.y - v2a.y, v2b.x - v2a.x);
    float th = a1 - a2, c = cosf(th), s = sinf(th);
    float tx = v1a.x - (c * v2a.x - s * v2a.y);
    float ty = v1a.y - (s * v2a.x + c * v2a.y);
    IFrameSource* src = GetSelectedSource();
    if (!src) return;
    Extrinsic3D e = src->GetExtr(); e.x = tx; e.y = ty; e.yaw = th;
    src->SetExtr(e);
    ResetMotionModel();
    LogPrintf("[CALIB PICK] %s dx=%.3f dy=%.3f th=%.2f\n", src->GetName().c_str(), tx, ty, Rad2Deg(th));
    BroadcastExtr(gSelectedLidar);
    gPickNRef = 0; gPickNTarget = 0;
}

void RunIcpRefine()
{
    if (gIcpRunning) return;
    if (gSelectedLidar == 0) { LogPrintf("[ICP] Chon L2..L4.\n"); return; }
    if (gSources.empty() || gSelectedLidar >= (int)gSources.size()) return;
    FramePtr f1, f2;
    if (gSources[0] && gSources[0]->IsRunning()) f1 = gSources[0]->GetFrameAtOffset(0);
    if (gSources[gSelectedLidar] && gSources[gSelectedLidar]->IsRunning()) f2 = gSources[gSelectedLidar]->GetFrameAtOffset(0);
    if (!f1 || !f2 || f1->local_x.empty() || f2->local_x.empty()) { LogPrintf("[ICP] Khong du du lieu.\n"); return; }
    gIcpRunning = true;
    std::vector<Ptf> T, S;
    for (size_t i = 0; i < f1->local_x.size(); i += 4) T.push_back({ f1->local_x[i], f1->local_y[i] });
    for (size_t i = 0; i < f2->local_x.size(); i += 4) S.push_back({ f2->local_x[i], f2->local_y[i] });
    Extrinsic3D e = gSources[gSelectedLidar]->GetExtr();
    auto rms = [&](float c, float s, float tx, float ty) {
        double sum = 0; int n = 0;
        for (size_t i = 0; i < S.size(); i += 2)
        {
            float gx = c * S[i].x - s * S[i].y + tx, gy = s * S[i].x + c * S[i].y + ty;
            float bd = 1e9f;
            for (auto& t : T) { float d = (gx - t.x) * (gx - t.x) + (gy - t.y) * (gy - t.y); if (d < bd) bd = d; }
            if (bd < 4.0f) { sum += bd; n++; }
        }
        return n ? sqrt(sum / n) : 999.0f;
        };
    float c = cosf(e.yaw), s = sinf(e.yaw);
    float r0 = (float)rms(c, s, e.x, e.y);
    for (int iter = 0; iter < 5; iter++)
    {
        c = cosf(e.yaw); s = sinf(e.yaw);
        double cgx = 0, cgy = 0, cmx = 0, cmy = 0; int n = 0;
        for (size_t i = 0; i < S.size(); i += 2)
        {
            float gx = c * S[i].x - s * S[i].y + e.x, gy = s * S[i].x + c * S[i].y + e.y;
            float bd = 1e9f; Ptf bm = { 0,0 };
            for (auto& t : T) { float d = (gx - t.x) * (gx - t.x) + (gy - t.y) * (gy - t.y); if (d < bd) { bd = d; bm = t; } }
            if (bd < 2.25f) { cgx += gx; cgy += gy; cmx += bm.x; cmy += bm.y; n++; }
        }
        if (n < 20) break;
        cgx /= n; cgy /= n; cmx /= n; cmy /= n;
        double dot = 0, cr = 0; int k = 0;
        for (size_t i = 0; i < S.size(); i += 2)
        {
            float gx = c * S[i].x - s * S[i].y + e.x, gy = s * S[i].x + c * S[i].y + e.y;
            float bd = 1e9f; Ptf bm = { 0,0 };
            for (auto& t : T) { float d = (gx - t.x) * (gx - t.x) + (gy - t.y) * (gy - t.y); if (d < bd) { bd = d; bm = t; } }
            if (bd < 2.25f)
            {
                double px = gx - cgx, py = gy - cgy, qx = bm.x - cmx, qy = bm.y - cmy;
                dot += px * qx + py * qy; cr += px * qy - py * qx; k++;
            }
        }
        if (!k) break;
        float dth = atan2f((float)cr, (float)dot);
        float dc = cosf(dth), ds = sinf(dth);
        float dtx = (float)(cmx - (dc * cgx - ds * cgy));
        float dty = (float)(cmy - (ds * cgx + dc * cgy));
        float nc = dc * c - ds * s, ns = dc * s + ds * c;
        float ntx = dc * e.x - ds * e.y + dtx;
        float nty = ds * e.x + dc * e.y + dty;
        e.yaw = atan2f(ns, nc); e.x = ntx; e.y = nty;
    }
    float r1 = (float)rms(cosf(e.yaw), sinf(e.yaw), e.x, e.y);
    gSources[gSelectedLidar]->SetExtr(e);
    ResetMotionModel();
    LogPrintf("[ICP] %s RMS %.3f -> %.3f | dx=%.3f dy=%.3f th=%.2f\n",
        gSources[gSelectedLidar]->GetName().c_str(), r0, r1, e.x, e.y, Rad2Deg(e.yaw));
    json r; r["type"] = "icp"; r["id"] = gSelectedLidar;
    r["dx"] = e.x; r["dy"] = e.y; r["thetaDeg"] = Rad2Deg(e.yaw);
    r["rmsBefore"] = r0; r["rmsAfter"] = r1;
    QueueWebJson(r);
    BroadcastExtr(gSelectedLidar);
    gIcpRunning = false;
}

// ================= 23. RECORDING =================
void ToggleRecording()
{
    if (!gIsRecording.load())
    {
        char fn[100]; time_t now = time(NULL); struct tm t;
        localtime_s(&t, &now);
        strftime(fn, sizeof(fn), "lidar_full_data_%Y%m%d_%H%M%S.csv", &t);
        std::lock_guard<std::mutex> lk(gCsvMutex);
        gCsvFile.open(fn);
        if (gCsvFile.is_open())
        {
            // >>> CAP NHAT: Them Local_Z vao header
            gCsvFile << "Source_ID,Frame_ID,Timestamp_sec,Timestamp_usec,Point_Index,Angle_Degree,Distance_m,Local_X,Local_Y,Local_Z,Intensity,Scan_Time\n";
            gIsRecording = true;
            LogPrintf("[INFO] Ghi CSV: %s\n", fn);
        }
    }
    else
    {
        { std::lock_guard<std::mutex> lk(gCsvMutex); if (gCsvFile.is_open()) gCsvFile.close(); }
        gIsRecording = false;
        LogPrintf("[INFO] Dung ghi CSV.\n");
    }
}

// ================= 26. PROCESSING (v3: dung vector thay array) =================
static void CheckOcclusion(const std::vector<FramePtr>& frames)
{
    if (gIsPaused.load()) return;
    int total = 0;
    for (size_t i = 0; i < frames.size(); i++) if (frames[i]) total += (int)frames[i]->x_coords.size();
    if (total == 0) return;
    gAvgPointCount = (gAvgCount == 0) ? (float)total : 0.98f * gAvgPointCount + 0.02f * total;
    gAvgCount++;
    if (gAvgCount < 50) return;
    float th = std::max((float)OCCLUSION_MIN_POINTS, 0.30f * gAvgPointCount);
    if ((float)total < th) gLowPointFrames++; else gLowPointFrames = 0;
    bool should = (gLowPointFrames >= OCCLUSION_PERSIST_FRAMES);
    if (should && !gOcclusionAlarm)
    {
        LogPrintf("[OCCLUSION] CANH BAO: diem giam con %d (TB %.0f)\n", total, gAvgPointCount);
        gOcclusionAlarm = true;
    }
    else if (!should && gOcclusionAlarm)
    {
        LogPrintf("[OCCLUSION] Het canh bao.\n");
        gOcclusionAlarm = false;
    }
}

// >>> v3: UpdateMotionDetection dung Zone::ContainsPoint <<<
static void UpdateMotionDetection(const std::vector<FramePtr>& frames, DWORD64 nowMs)
{
    static DWORD64 lastMs = 0;
    float dt = (lastMs && nowMs > lastMs) ? (nowMs - lastMs) / 1000.0f : 0.05f;
    if (dt < 0.01f) dt = 0.01f; if (dt > 1.0f) dt = 1.0f;
    lastMs = nowMs;
    if (gZones.empty()) { if (!gTargets.empty() || gBgFrames) ResetMotionModel(); return; }
    std::vector<Ptf> pts;
    std::vector<float> ptsZ;
    for (size_t i = 0; i < frames.size(); i++)
    {
        if (!frames[i]) continue;
        const auto& xs = frames[i]->x_coords, ys = frames[i]->y_coords, zs = frames[i]->z_coords;
        const auto& lxs = frames[i]->local_x, lys = frames[i]->local_y;
        for (size_t k = 0; k < xs.size(); k++)
        {
            bool in = false;
            for (const auto& z : gZones)
            {
                if (!z.local && z.ContainsPoint(xs[k], ys[k])) { in = true; break; }
                if (z.local && z.sourceId == (int)i && z.ContainsPoint(lxs[k], lys[k])) { in = true; break; }
            }
            if (in) {
                pts.push_back(Ptf{ xs[k], ys[k] });
                ptsZ.push_back(k < zs.size() ? zs[k] : 0.0f);
            }
        }
    }
    const float CS = 0.3f;
    auto key = [](int ix, int iy) { return ((long long)(ix + 2048) << 32) | (uint32_t)(iy + 2048); };
    gBgFrames++;
    std::unordered_map<long long, char> occ;
    for (auto& p : pts) occ[key((int)floorf(p.x / CS), (int)floorf(p.y / CS))] = 1;
    for (auto& kv : occ) gBgSeen[kv.first]++;
    std::vector<Ptf> dyn;
    std::vector<float> dynZ;
    for (size_t pi = 0; pi < pts.size(); pi++)
    {
        auto& p = pts[pi];
        long long k = key((int)floorf(p.x / CS), (int)floorf(p.y / CS));
        int seen = gBgSeen.count(k) ? gBgSeen[k] : 0;
        bool isStatic = (gBgFrames > 20) && (seen * 100 >= 70 * gBgFrames);
        if (!isStatic) { dyn.push_back(p); dynZ.push_back(ptsZ[pi]); }
    }
    std::unordered_map<long long, std::vector<int>> grid;
    for (int i = 0; i < (int)dyn.size(); i++) grid[key((int)floorf(dyn[i].x / CS), (int)floorf(dyn[i].y / CS))].push_back(i);
    std::vector<char> vis(dyn.size(), 0);
    struct RawT { float x, y, w, h; int n; float z1, z2; };
    std::vector<RawT> raw;
    for (auto& kv : grid)
        for (int seed : kv.second)
        {
            if (vis[seed]) continue;
            std::vector<int> st{ seed }; vis[seed] = 1;
            float x1 = 1e9f, y1 = 1e9f, x2 = -1e9f, y2 = -1e9f, sx = 0, sy = 0; int cnt = 0;
            float zLo = 1e9f, zHi = -1e9f;
            while (!st.empty())
            {
                int cur = st.back(); st.pop_back();
                float px = dyn[cur].x, py = dyn[cur].y, pz = dynZ[cur];
                x1 = std::min(x1, px); y1 = std::min(y1, py);
                x2 = std::max(x2, px); y2 = std::max(y2, py);
                zLo = std::min(zLo, pz); zHi = std::max(zHi, pz);
                sx += px; sy += py; cnt++;
                int ix = (int)floorf(px / CS), iy = (int)floorf(py / CS);
                for (int dx = -1; dx <= 1; dx++) for (int dy = -1; dy <= 1; dy++)
                {
                    auto it = grid.find(key(ix + dx, iy + dy));
                    if (it == grid.end()) continue;
                    for (int j : it->second) if (!vis[j]) { vis[j] = 1; st.push_back(j); }
                }
            }
            float w = x2 - x1, h = y2 - y1;
            if (cnt >= MIN_CLUSTER_POINTS && w >= MIN_TARGET_W && h >= MIN_TARGET_H &&
                w <= MAX_TARGET_W && h <= MAX_TARGET_H)
                raw.push_back({ sx / cnt, sy / cnt, w, h, cnt, zLo, zHi });
        }
    std::vector<DynTarget> next;
    std::vector<char> used(gTargets.size(), 0);
    for (auto& r : raw)
    {
        int best = -1; float bd = 1.2f;
        for (size_t i = 0; i < gTargets.size(); i++)
            if (!used[i])
            {
                float d = sqrtf((gTargets[i].x - r.x) * (gTargets[i].x - r.x) + (gTargets[i].y - r.y) * (gTargets[i].y - r.y));
                if (d < bd) { bd = d; best = (int)i; }
            }
        DynTarget t;
        if (best >= 0)
        {
            used[best] = 1; t = gTargets[best];
            t.age = gTargets[best].age + 1;
            t.vx = 0.6f * t.vx + 0.4f * ((r.x - t.x) / dt);
            t.vy = 0.6f * t.vy + 0.4f * ((r.y - t.y) / dt);
            t.x = r.x; t.y = r.y; t.w = r.w; t.h = r.h; t.n = r.n; t.z1 = r.z1; t.z2 = r.z2;
        }
        else
        {
            t.id = gNextTargetId++; t.x = r.x; t.y = r.y; t.vx = 0; t.vy = 0;
            t.w = r.w; t.h = r.h; t.n = r.n; t.age = 1; t.z1 = r.z1; t.z2 = r.z2;
        }
        next.push_back(t);
    }
    gTargets = next;
    int confirmed = 0;
    for (auto& tg : gTargets) if (tg.age >= PERSISTENCE_FRAMES) confirmed++;
    bool alarm = (confirmed > 0);
    if (alarm && !gPrevAlarm) LogPrintf("[CANH BAO] %d muc tieu DI CHUYEN trong vung!\n", confirmed);
    if (!alarm && gPrevAlarm) LogPrintf("[AN TOAN] Vung an toan.\n");
    gPrevAlarm = alarm;
}

void ComputeClusters(const std::vector<FramePtr>& frames)
{
    std::vector<Ptf> pts;
    for (size_t i = 0; i < frames.size(); i++)
        if (frames[i])
            for (size_t j = 0; j < frames[i]->x_coords.size(); j += 2)
                pts.push_back({ frames[i]->x_coords[j], frames[i]->y_coords[j] });
    const float CS = 0.25f;
    auto key = [](int ix, int iy) { return ((long long)(ix + 2048) << 32) | (uint32_t)(iy + 2048); };
    std::unordered_map<long long, std::vector<int>> grid;
    for (int i = 0; i < (int)pts.size(); i++) grid[key((int)floorf(pts[i].x / CS), (int)floorf(pts[i].y / CS))].push_back(i);
    std::vector<char> vis(pts.size(), 0);
    gBoxes.clear(); gNearest = 1e9f;
    for (auto& kv : grid)
        for (int seed : kv.second)
        {
            if (vis[seed]) continue;
            std::vector<int> st{ seed }; vis[seed] = 1;
            float x1 = 1e9f, y1 = 1e9f, x2 = -1e9f, y2 = -1e9f; int cnt = 0;
            while (!st.empty())
            {
                int cur = st.back(); st.pop_back();
                float px = pts[cur].x, py = pts[cur].y;
                x1 = std::min(x1, px); y1 = std::min(y1, py);
                x2 = std::max(x2, px); y2 = std::max(y2, py); cnt++;
                int ix = (int)floorf(px / CS), iy = (int)floorf(py / CS);
                for (int dx = -1; dx <= 1; dx++) for (int dy = -1; dy <= 1; dy++)
                {
                    auto it = grid.find(key(ix + dx, iy + dy));
                    if (it == grid.end()) continue;
                    for (int j : it->second) if (!vis[j]) { vis[j] = 1; st.push_back(j); }
                }
            }
            if (cnt >= 6)
            {
                gBoxes.push_back({ x1, y1, x2, y2, cnt });
                float d = sqrtf(((x1 + x2) / 2) * ((x1 + x2) / 2) + ((y1 + y2) / 2) * ((y1 + y2) / 2));
                if (d < gNearest) gNearest = d;
            }
        }
    gNObj = (int)gBoxes.size();
}

// ================= SERIALIZE + EVENTS + TICK =================
static std::shared_ptr<std::vector<uint8_t>> SerializeFrameV3(const PointCloudData& f)
{
    size_t n = f.local_x.size();
    auto buf = std::make_shared<std::vector<uint8_t>>();
    buf->reserve(76 + n * 16);
    appendU8(*buf, 3); appendU8(*buf, (uint8_t)f.source_id); appendU16(*buf, 0);
    appendU32(*buf, (uint32_t)f.frame_id);
    appendU32(*buf, f.ts_secs); appendU32(*buf, f.ts_usecs);
    appendF32(*buf, f.origin_x); appendF32(*buf, f.origin_y); appendF32(*buf, f.origin_z);
    appendF32(*buf, f.origin_roll); appendF32(*buf, f.origin_pitch); appendF32(*buf, f.origin_yaw);
    appendF32(*buf, f.angle_min); appendF32(*buf, f.angle_max); appendF32(*buf, f.angle_increment);
    appendF32(*buf, f.range_min); appendF32(*buf, f.range_max);
    appendF32(*buf, f.scan_time); appendF32(*buf, f.time_increment);
    appendU32(*buf, (uint32_t)f.wave_count);
    appendU32(*buf, (uint32_t)n);
    for (size_t i = 0; i < n; i++)
    {
        appendF32(*buf, f.x_coords[i]); appendF32(*buf, f.y_coords[i]); appendF32(*buf, f.z_coords[i]);
        appendF32(*buf, i < f.intensities.size() ? f.intensities[i] : 0.0f);
    }
    return buf;
}

// >>> v3: GetFrames tra ve vector thay vi array <<<
static std::vector<FramePtr> GetFrames(int offset)
{
    std::vector<FramePtr> fr(gSources.size());
    for (size_t i = 0; i < gSources.size(); i++)
        if (gSources[i] && gSources[i]->IsRunning())
        {
            auto raw = gSources[i]->GetFrameAtOffset(offset);
            if (!raw) continue;
            auto e = gSources[i]->GetExtr();
            if (raw->origin_x == e.x && raw->origin_y == e.y && raw->origin_z == e.z && raw->origin_roll == e.roll && raw->origin_pitch == e.pitch && raw->origin_yaw == e.yaw && raw->source_id == (int)i) { fr[i] = raw; continue; }
            auto f = std::make_shared<PointCloudData>(*raw); Transform3D transform(e);
            f->source_id = (int)i;
            f->origin_x = e.x; f->origin_y = e.y; f->origin_z = e.z;
            f->origin_roll = e.roll; f->origin_pitch = e.pitch; f->origin_yaw = e.yaw;
            for (size_t k = 0; k < f->local_x.size(); ++k) {
                auto p = transform.Apply(f->local_x[k], f->local_y[k]);
                f->x_coords[k] = p.x; f->y_coords[k] = p.y; f->z_coords[k] = p.z;
            }
            fr[i] = f;
        }
    return fr;
}

static json BuildEventsJson()
{
    json j;
    j["type"] = "events";
    j["recording"] = gIsRecording.load();
    j["occlusion"] = gOcclusionAlarm;
    j["clusterMode"] = gClusterMode;
    j["sel"] = gSelectedLidar;
    j["paused"] = gIsPaused.load();
    json tgs = json::array(); int confirmed = 0;
    for (auto& t : gTargets)
    {
        if (t.age >= PERSISTENCE_FRAMES) confirmed++;
        tgs.push_back({ {"id",t.id},{"x",t.x},{"y",t.y},{"vx",t.vx},{"vy",t.vy},
    {"w",t.w},{"h",t.h},{"n",t.n},{"age",t.age},{"z1",t.z1},{"z2",t.z2} });
    }
    j["targets"] = tgs; j["confirmed"] = confirmed;
    json bxs = json::array();
    for (auto& b : gBoxes) bxs.push_back({ {"x1",b.x1},{"y1",b.y1},{"x2",b.x2},{"y2",b.y2},{"n",b.n} });
    j["boxes"] = bxs; j["nearest"] = gNearest; j["nObj"] = gNObj;
    return j;
}

static std::atomic<bool> gRun{ true };

static void TickLoop()
{
    // >>> v3: lastSent dung vector dong <<<
    std::vector<FramePtr> lastSent;
    DWORD64 lastEvent = 0;
    while (gRun.load())
    {
        DWORD64 nowMs = GetTickCount64();
        {
            std::lock_guard<std::recursive_mutex> sourcesLock(gSourcesMutex);
            auto frames = GetFrames(0);
            {
                std::lock_guard<std::mutex> lk(gZoneMutex);
                CheckOcclusion(frames);
                UpdateMotionDetection(frames, nowMs);
                if (gClusterMode && nowMs - gClusterLast > 500) { gClusterLast = nowMs; ComputeClusters(frames); }
            }
            if (HasClients())
            {
                lastSent.resize(frames.size());
                for (size_t i = 0; i < frames.size() && i < lastSent.size(); i++)
                    if (frames[i] && (!lastSent[i] || frames[i]->frame_id != lastSent[i]->frame_id || frames[i]->source_id != lastSent[i]->source_id || frames[i]->origin_x != lastSent[i]->origin_x || frames[i]->origin_y != lastSent[i]->origin_y || frames[i]->origin_z != lastSent[i]->origin_z || frames[i]->origin_roll != lastSent[i]->origin_roll || frames[i]->origin_pitch != lastSent[i]->origin_pitch || frames[i]->origin_yaw != lastSent[i]->origin_yaw))
                    {
                        lastSent[i] = frames[i];
                        BroadcastBinary(*SerializeFrameV3(*frames[i]));
                    }
                if (nowMs - lastEvent > 100) { lastEvent = nowMs; BroadcastText(BuildEventsJson().dump()); }
            }
        }
        DrainWebJson();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}
// >>> v3.2: PING - thoi gian phan hoi cua 1 goi tin mang (ICMP echo) <<<
static void DoPing(int id)
{
    if (id < 0 || id >= (int)gSources.size()) return;
    std::string ip = gSources[id]->GetIp();
    in_addr bin{};
    IPAddr addr = (InetPtonA(AF_INET, ip.c_str(), &bin) == 1)
        ? (IPAddr)bin.S_un.S_addr : INADDR_NONE;
    int ms = -1;
    HANDLE h = IcmpCreateFile();
    if (h != INVALID_HANDLE_VALUE && addr != INADDR_NONE)
    {
        char data[32]; memset(data, 'L', sizeof(data));
        DWORD bufSz = sizeof(ICMP_ECHO_REPLY) + sizeof(data) + 8;
        std::vector<char> buf(bufSz);
        for (int i = 0; i < 4; i++)
        {
            PICMP_ECHO_REPLY rep = (PICMP_ECHO_REPLY)buf.data();
            DWORD r = IcmpSendEcho2(h, NULL, NULL, NULL, addr, data, (WORD)sizeof(data),
                NULL, rep, bufSz, 1000);
            if (r > 0) { ms = (int)rep->RoundTripTime; break; }
        }
        IcmpCloseHandle(h);
    }
    LogPrintf("[PING] %s: %d ms\n", ip.c_str(), ms);
    json j; j["type"] = "ping"; j["id"] = id; j["ip"] = ip; j["ms"] = ms;
    QueueWebJson(j);
}
// ================= HANDLE MESSAGE (v3: them lenh moi) =================
static void HandleMessage(ConnHdl hdl, WsServer::message_ptr msg)
{
    std::lock_guard<std::recursive_mutex> sourcesLock(gSourcesMutex);
    (void)hdl;
    try
    {
        auto j = json::parse(msg->get_payload());
        std::string cmd = j.value("cmd", "");

        if (cmd == "select") { gSelectedLidar = j.at("id").get<int>(); BroadcastExtr(gSelectedLidar); }
        else if (cmd == "extr_apply")
        {
            int id = j.at("id").get<int>();
            if (id >= 0 && id < (int)gSources.size())
            {
                Extrinsic3D e = gSources[id]->GetExtr();
                e.x = j.value("x", j.value("dx", e.x)); e.y = j.value("y", j.value("dy", e.y));
                e.z = j.value("z", e.z);
                e.roll = Deg2Rad(j.value("roll", Rad2Deg(e.roll)));
                e.pitch = Deg2Rad(j.value("pitch", Rad2Deg(e.pitch)));
                e.yaw = Deg2Rad(j.value("yaw", j.value("thetaDeg", Rad2Deg(e.yaw))));
                if (!std::isfinite(e.x) || !std::isfinite(e.y) || !std::isfinite(e.z) || !std::isfinite(e.roll) || !std::isfinite(e.pitch) || !std::isfinite(e.yaw)) return;
                gSources[id]->SetExtr(e);
                ResetMotionModel();
                BroadcastExtr(id);
            }
        }
        else if (cmd == "extr_adjust")
            AdjustSelectedExtr((float)j.value("ddx", 0.0), (float)j.value("ddy", 0.0), Deg2Rad((float)j.value("dth", 0.0)));
        else if (cmd == "extr_save") SaveExtrConfig();
        else if (cmd == "extr_print") PrintSelectedExtr();
        else if (cmd == "record") ToggleRecording();
        else if (cmd == "pause")
        {
            bool p = j.value("paused", false);
            gIsPaused = p;
            for (auto& src : gSources) src->SetPaused(p);
            LogPrintf("[INFO] %s\n", p ? "PAUSED - backend NGUNG nhan/xu ly du lieu"
                : "RESUMED - backend chay lai");
        }
        else if (cmd == "cluster") gClusterMode = !gClusterMode;
        else if (cmd == "rmse_start")
        {
            float v = (float)j.value("true", 0.0);
            if (v <= 0) QueueWebJson({ {"type","log"},{"text","[RMSE] Nhap true > 0"} });
            else
            {
                std::lock_guard<std::mutex> lk(gRmseMutex);
                gRmseTrue = v; gRmseSum = 0; gRmseSumSq = 0; gRmseCount = 0; gRmseActive = true;
            }
        }
        else if (cmd == "icp") RunIcpRefine();
        else if (cmd == "latency")
        {
            if (!gSources.empty() && gSources[0]->IsRunning())
                MeasureCommandLatency(gSources[0]->GetLadar(), j.value("sleep", false));
        }
        else if (cmd == "pick_apply")
        {
            auto rf = j.at("ref"), tg = j.at("target");
            for (int k = 0; k < 2; k++)
            {
                gPickRef[k] = { (float)rf[k]["x"].get<double>(), (float)rf[k]["y"].get<double>() };
                gPickTarget[k] = { (float)tg[k]["x"].get<double>(), (float)tg[k]["y"].get<double>() };
            }
            gPickNRef = 2; gPickNTarget = 2;
            ApplyPickCalib();
        }
        // >>> v3: zones_set ho tro nhieu loai hinh <<<
        else if (cmd == "zones_set")
        {
            std::lock_guard<std::mutex> lk(gZoneMutex);
            gZones.clear();
            for (auto& z : j.at("zones"))
            {
                Zone zn;
                zn.local = (z.value("scope", "fusion") == "local");
                zn.id = z.value("id", 0);

                // Code 1 compatibility:
                // - New local Zones explicitly carry sourceId.
                // - Old local Zones used id as the LiDAR index.
                // - Fusion/world Zones do not belong to a specific LiDAR.
                zn.sourceId = (zn.local && z.contains("sourceId") && !z["sourceId"].is_null())
                    ? z["sourceId"].get<int>()
                    : (zn.local ? zn.id : 0);

                zn.type = z.value("type", 0);  // >>> v3: type <<<

                if (zn.type == ZT_RECT)
                {
                    // Tuong thich cu: x1,y1,x2,y2
                    zn.x1 = (float)z.at("x1").get<double>();
                    zn.y1 = (float)z.at("y1").get<double>();
                    zn.x2 = (float)z.at("x2").get<double>();
                    zn.y2 = (float)z.at("y2").get<double>();
                }
                else if (zn.type == ZT_POLYGON || zn.type == ZT_FREEHAND)
                {
                    // >>> v3: Polygon/Freehand nhan mang vertices <<<
                    if (z.contains("vertices"))
                        for (auto& v : z["vertices"])
                            zn.vertices.push_back({ (float)v[0].get<double>(), (float)v[1].get<double>() });
                }
                else if (zn.type == ZT_CIRCLE)
                {
                    // >>> v3: Circle nhan cx, cy, radius <<<
                    zn.cx = (float)z.at("cx").get<double>();
                    zn.cy = (float)z.at("cy").get<double>();
                    zn.radius = (float)z.at("radius").get<double>();
                }
                else if (zn.type == ZT_SECTOR)
                {
                    // >>> v3: Sector nhan sx, sy, sRadius, sAngleStart, sAngleEnd <<<
                    zn.sx = (float)z.at("sx").get<double>();
                    zn.sy = (float)z.at("sy").get<double>();
                    zn.sRadius = (float)z.at("sRadius").get<double>();
                    zn.sAngleStart = Deg2Rad((float)z.at("sAngleStart").get<double>());
                    zn.sAngleEnd = Deg2Rad((float)z.at("sAngleEnd").get<double>());
                }
                gZones.push_back(zn);
            }
            ResetMotionModel();
            LogPrintf("[ZONE] Nhan %d vung canh bao (da hinh)\n", (int)gZones.size());
        }
        else if (cmd == "zone_clear")
        {
            std::lock_guard<std::mutex> lk(gZoneMutex);
            gZones.clear();
            ResetMotionModel();
        }
        // >>> v3: Cap nhat dinh zone (vertex editing) <<<
        else if (cmd == "zone_update")
        {
            std::lock_guard<std::mutex> lk(gZoneMutex);
            int zid = j.at("zoneId").get<int>();
            for (size_t i = 0; i < gZones.size(); i++)
            {
                if (gZones[i].id == zid)
                {
                    if (j.contains("vertices"))
                    {
                        gZones[i].vertices.clear();
                        for (auto& v : j["vertices"])
                            gZones[i].vertices.push_back({ (float)v[0].get<double>(), (float)v[1].get<double>() });
                    }
                    if (j.contains("x1")) gZones[i].x1 = (float)j["x1"].get<double>();
                    if (j.contains("y1")) gZones[i].y1 = (float)j["y1"].get<double>();
                    if (j.contains("x2")) gZones[i].x2 = (float)j["x2"].get<double>();
                    if (j.contains("y2")) gZones[i].y2 = (float)j["y2"].get<double>();
                    if (j.contains("cx")) gZones[i].cx = (float)j["cx"].get<double>();
                    if (j.contains("cy")) gZones[i].cy = (float)j["cy"].get<double>();
                    if (j.contains("radius")) gZones[i].radius = (float)j["radius"].get<double>();
                    if (j.contains("sRadius")) gZones[i].sRadius = (float)j["sRadius"].get<double>();
                    if (j.contains("sAngleStart")) gZones[i].sAngleStart = Deg2Rad((float)j["sAngleStart"].get<double>());
                    if (j.contains("sAngleEnd")) gZones[i].sAngleEnd = Deg2Rad((float)j["sAngleEnd"].get<double>());

                    // Keep local Zone ownership independently from Zone ID.
                    if (j.contains("sourceId") && !j["sourceId"].is_null())
                        gZones[i].sourceId = j["sourceId"].get<int>();

                    break;
                }
            }
            ResetMotionModel();
            LogPrintf("[ZONE] Cap nhat zone %d\n", zid);
        }
        // >>> v3: Cau hinh IP LiDAR dong <<<
        else if (cmd == "lidar_set_ip")
        {
            int id = j.at("id").get<int>();
            std::string newIp = j.at("ip").get<std::string>();
            uint16_t newPort = (uint16_t)j.value("port", 1112);
            if (id >= 0 && id < (int)gSources.size())
            {
                auto src = std::dynamic_pointer_cast<RealLidarSource>(gSources[id]);
                if (src)
                {
                    LogPrintf("[IP] Thay doi LiDAR %d: %s:%d -> %s:%d\n", id + 1,
                        src->GetIp().c_str(), src->GetPort(), newIp.c_str(), newPort);
                    src->Reconnect(newIp, newPort);
                    SaveLidarConfig();
                    BroadcastLidarConfig();
                }
            }
        }
        // >>> v3: Them LiDAR moi <<<
        else if (cmd == "lidar_add")
        {
            if ((int)gSources.size() >= MAX_LIDARS)
            {
                QueueWebJson({ {"type","log"},{"text","[ERROR] Toi da 16 LiDAR"} });
            }
            else
            {
                std::string newIp = j.value("ip", "192.168.201.15");
                uint16_t newPort = (uint16_t)j.value("port", 1112);
                int newId = (int)gSources.size();
                std::string nm = SanitizeName(j.value("name", ""));
                if (nm.empty() || j.value("name", "") == "") { char buf[32]; sprintf_s(buf, "LiDAR_%d", newId + 1); nm = buf; }
                Extrinsic3D defExtr;
                auto src = std::make_shared<RealLidarSource>(newId, nm, newIp, newPort, defExtr);
                src->SetEnableCsvRecord(true);
                gSources.push_back(src);
                src->Start();
                gActiveLidarCount = (int)gSources.size();
                SaveLidarConfig();
                SaveExtrConfig();
                BroadcastLidarConfig();
                LogPrintf("[IP] Them LiDAR %d: %s:%d\n", newId + 1, newIp.c_str(), newPort);
            }
        }
        // >>> v3: Xoa LiDAR <<<
        else if (cmd == "lidar_remove")
        {
            int id = j.at("id").get<int>();
            if (id >= 0 && id < (int)gSources.size() && gSources.size() > 1)
            {
                gSources[id]->Stop();
                gSources.erase(gSources.begin() + id);
                // Cap nhat source_id cho cac source con lai
                for (size_t i = 0; i < gSources.size(); i++)
                {
                    // Khong co SetSourceId, nen ta chi log
                }
                if (gSelectedLidar >= (int)gSources.size())
                    gSelectedLidar = (int)gSources.size() - 1;
                gActiveLidarCount = (int)gSources.size();
                SaveLidarConfig();
                SaveExtrConfig();
                BroadcastLidarConfig();
                LogPrintf("[IP] Xoa LiDAR %d. Con lai: %d\n", id + 1, (int)gSources.size());
            }
        }
        // >>> v3: Lay cau hinh LiDAR <<<
        else if (cmd == "lidar_get_config")
        {
            BroadcastLidarConfig();
        }
        // >>> v3: Reconnect 1 LiDAR <<<
        else if (cmd == "lidar_reconnect")
        {
            int id = j.at("id").get<int>();
            if (id >= 0 && id < (int)gSources.size())
            {
                auto src = std::dynamic_pointer_cast<RealLidarSource>(gSources[id]);
                if (src)
                {
                    src->Reconnect(src->GetIp(), src->GetPort());
                    LogPrintf("[IP] Reconnect LiDAR %d\n", id + 1);
                }
            }
        }
        // >>> v3.1: DOI TEN LiDAR (khong reconnect) <<<
        else if (cmd == "lidar_rename")
        {
            int id = j.at("id").get<int>();
            std::string nm = j.value("name", "");
            if (id >= 0 && id < (int)gSources.size() && !nm.empty())
            {
                gSources[id]->SetName(nm);
                LogPrintf("[IP] Doi ten LiDAR %d -> %s\n", id + 1, gSources[id]->GetName().c_str());
                SaveLidarConfig();        // luu vao lidar_config.cfg
                BroadcastLidarConfig();   // cap nhat web
            }
        }
        else if (cmd == "ping") DoPing(j.value("id", 0));
        else if (cmd == "get_state")
        {
            json arr = json::array();
            for (size_t i = 0; i < gSources.size(); i++)
            {
                Extrinsic3D e = gSources[i]->GetExtr();
                arr.push_back({ {"id",(int)i},{"name",gSources[i]->GetName()},
                    {"dx",e.x},{"dy",e.y},{"thetaDeg",Rad2Deg(e.yaw)},
                    {"x",e.x},{"y",e.y},{"z",e.z},{"roll",Rad2Deg(e.roll)},{"pitch",Rad2Deg(e.pitch)},{"yaw",Rad2Deg(e.yaw)} });
            }
            json out; out["type"] = "state"; out["lidars"] = arr;
            BroadcastText(out.dump());
        }
    }
    catch (...) {}
}

// ================= MAIN (v3: doc lidar_config.cfg) =================
int main(int argc, char** argv)
{
    {
        HANDLE hOrig = GetStdHandle(STD_OUTPUT_HANDLE);
        HANDLE hDup = NULL;
        if (hOrig && hOrig != INVALID_HANDLE_VALUE)
        {
            DuplicateHandle(GetCurrentProcess(), hOrig,
                GetCurrentProcess(), &hDup, 0, FALSE, DUPLICATE_SAME_ACCESS);
        }
        gConOut = hDup ? hDup : hOrig;
    }
    freopen("lidar_spam_stdout.log", "w", stdout);
    freopen("lidar_spam_stderr.log", "w", stderr);
    gLogFile = fopen("lidar_web_log.txt", "a");
    LogPrintf("\n=== LiDAR WEB GATEWAY v3 ===\n");

    // >>> v3: Doc cau hinh IP tu lidar_config.cfg, neu khong co thi dung mac dinh <<<
    std::vector<LidarConfigEntry> cfgEntries;
    if (!LoadLidarConfig(cfgEntries))
    {
        // Mac dinh 4 LiDAR
        const char* DEF_IPS[4] = { "192.168.201.15","192.168.201.16","192.168.201.17","192.168.201.18" };
        for (int i = 0; i < 4; i++)
        {
            LidarConfigEntry e;
            e.ip = DEF_IPS[i]; e.port = 1112;
            char nm[32]; sprintf_s(nm, "TR70_Real%d", i + 1);
            e.name = nm;
            cfgEntries.push_back(e);
        }
        LogPrintf("[CFG] Khong tim thay %s, dung mac dinh 4 LiDAR\n", LIDAR_CFG_FILE);
    }
    else
        LogPrintf("[CFG] Nap %s: %d LiDAR\n", LIDAR_CFG_FILE, (int)cfgEntries.size());

    int nLidars = (int)cfgEntries.size();
    if (nLidars > MAX_LIDARS) nLidars = MAX_LIDARS;

    std::vector<Extrinsic3D> extrs(nLidars);
    // Extrinsics mac dinh
    if (nLidars >= 4)
    {
        extrs[0] = { 0,0,0 }; extrs[1] = { 5,0,0 };
        extrs[2] = { 0,5,0,0,0,Deg2Rad(90) }; extrs[3] = { 5,5,0,0,0,Deg2Rad(180) };
    }
    if (LoadExtrConfig(extrs)) LogPrintf("[CFG] Nap %s\n", EXTR_CFG);

    if (argc > 1)
    {
        LogPrintf("[INIT] CSV: %s\n", argv[1]);
        gSources.push_back(std::make_shared<CsvLidarPlayer>(0, "CSV_Player", argv[1], extrs.empty() ? Extrinsic3D() : extrs[0]));
    }
    else
    {
        for (int i = 0; i < nLidars; i++)
        {
            auto src = std::make_shared<RealLidarSource>(i, cfgEntries[i].name,
                cfgEntries[i].ip, cfgEntries[i].port, extrs[i]);
            src->SetEnableCsvRecord(true);
            gSources.push_back(src);
        }
    }

    gActiveLidarCount = (int)gSources.size();
    gConnectUs = NowUs();
    for (auto& s : gSources) s->Start();

    try
    {
        g_server.init_asio();
        g_server.set_reuse_addr(true);
        g_server.set_open_handler([](ConnHdl h) {
            std::lock_guard<std::mutex> lk(g_connMutex); g_conns.insert(h);
            LogPrintf("[WEB] client connect\n");
            // >>> v3: Gui cau hinh LiDAR khi client ket noi <<<
            BroadcastLidarConfig();
            });
        g_server.set_close_handler([](ConnHdl h) {
            std::lock_guard<std::mutex> lk(g_connMutex); g_conns.erase(h);
            LogPrintf("[WEB] client disconnect\n");
            });
        g_server.set_message_handler([](ConnHdl h, WsServer::message_ptr m) { HandleMessage(h, m); });
        g_server.listen(9001);
        g_server.start_accept();
        LogPrintf("WebSocket listening ws://0.0.0.0:9001\n");
    }
    catch (std::exception& e) { LogPrintf("[ERROR] WS: %s\n", e.what()); return 1; }

    std::thread wsTh([]() { try { g_server.run(); } catch (...) {} });
    std::thread tick(TickLoop);
    LogPrintf("Press q + Enter to quit.\n");
    char c;
    while (std::cin.get(c)) if (c == 'q' || c == 'Q') break;
    gRun = false;
    if (tick.joinable()) tick.join();
    g_server.stop_listening(); g_server.stop();
    if (wsTh.joinable()) wsTh.join();
    if (gIsRecording.load()) ToggleRecording();
    for (auto& s : gSources) s->Stop();
    if (gLogFile) fclose(gLogFile);
    return 0;
}
