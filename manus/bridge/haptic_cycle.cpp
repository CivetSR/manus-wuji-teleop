// Cycle haptic feedback on index -> middle -> ring -> pinky (1s each, repeat).
// Build: make haptic_cycle.out

#include "ManusSDK.h"
#include "ManusSDKTypes.h"

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

namespace {

constexpr float kFullPower = 1.0f;
constexpr int kFingerCount = 5;
// Thumb, Index, Middle, Ring, Pinky — cycle skips thumb (index 0).
constexpr int kCycleFingers[] = {1, 2, 3, 4};
constexpr int kCycleLen = 4;

volatile std::sig_atomic_t g_running = 1;

Landscape* g_landscape = nullptr;
std::mutex g_mutex;

void OnSignal(int) { g_running = 0; }

void OnLandscape(const Landscape* landscape) {
    if (landscape == nullptr) {
        return;
    }
    std::lock_guard<std::mutex> lock(g_mutex);
    if (g_landscape != nullptr) {
        delete g_landscape;
    }
    g_landscape = new Landscape(*landscape);
}

bool InitSdk() {
    if (CoreSdk_InitializeIntegrated() != SDKReturnCode_Success) {
        std::fprintf(stderr, "[haptic] CoreSdk_InitializeIntegrated failed\n");
        return false;
    }

    if (CoreSdk_RegisterCallbackForLandscapeStream(OnLandscape) != SDKReturnCode_Success) {
        std::fprintf(stderr, "[haptic] failed to register landscape callback\n");
        return false;
    }

    CoordinateSystemVUH vuh{};
    CoordinateSystemVUH_Init(&vuh);
    vuh.handedness = Side_Right;
    vuh.up = AxisPolarity_PositiveZ;
    vuh.view = AxisView_XFromViewer;
    vuh.unitScale = 1.0f;

    if (CoreSdk_InitializeCoordinateSystemWithVUH(vuh, true) != SDKReturnCode_Success) {
        std::fprintf(stderr, "[haptic] failed to initialize coordinate system\n");
        return false;
    }

    ManusHost host{};
    ManusHost_Init(&host);
    if (CoreSdk_ConnectToHost(host) != SDKReturnCode_Success) {
        std::fprintf(stderr, "[haptic] CoreSdk_ConnectToHost failed\n");
        return false;
    }

    return true;
}

struct GloveTarget {
    uint32_t id;
    Side side;
};

std::vector<GloveTarget> GetHapticGloves(const char* handFilter) {
    std::vector<GloveTarget> gloves;
    std::lock_guard<std::mutex> lock(g_mutex);
    if (g_landscape == nullptr) {
        return gloves;
    }

    for (uint32_t i = 0; i < g_landscape->gloveDevices.gloveCount; ++i) {
        const auto& g = g_landscape->gloveDevices.gloves[i];
        if (!g.isHaptics || g.pairedState != DevicePairedState_Paired) {
            continue;
        }
        if (handFilter != nullptr) {
            if (std::strcmp(handFilter, "left") == 0 && g.side != Side_Left) {
                continue;
            }
            if (std::strcmp(handFilter, "right") == 0 && g.side != Side_Right) {
                continue;
            }
        }
        gloves.push_back({g.id, g.side});
    }
    return gloves;
}

void SendVibration(uint32_t gloveId, const float* powers) {
    CoreSdk_VibrateFingersForGlove(gloveId, powers);
}

void StopAll(const std::vector<GloveTarget>& gloves) {
    float zero[kFingerCount] = {0, 0, 0, 0, 0};
    for (const auto& glove : gloves) {
        SendVibration(glove.id, zero);
    }
}

const char* FingerName(int idx) {
    switch (idx) {
    case 0:
        return "拇指";
    case 1:
        return "食指";
    case 2:
        return "中指";
    case 3:
        return "无名指";
    case 4:
        return "小指";
    default:
        return "?";
    }
}

const char* SideName(Side side) {
    if (side == Side_Left) {
        return "左手";
    }
    if (side == Side_Right) {
        return "右手";
    }
    return "手";
}

void PrintUsage(const char* prog) {
    std::fprintf(stderr,
                 "Usage: %s [--hand left|right|all] [--power 0.0-1.0] [--duration-ms N]\n"
                 "Default: all haptic gloves, power=1.0, 1000ms per finger.\n"
                 "Sequence: index -> middle -> ring -> pinky, repeat. Ctrl+C to stop.\n",
                 prog);
}

}  // namespace

int main(int argc, char** argv) {
    const char* handFilter = nullptr;
    float power = kFullPower;
    int durationMs = 1000;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--hand") == 0 && i + 1 < argc) {
            handFilter = argv[++i];
            if (std::strcmp(handFilter, "all") == 0) {
                handFilter = nullptr;
            }
        } else if (std::strcmp(argv[i], "--power") == 0 && i + 1 < argc) {
            power = std::atof(argv[++i]);
            if (power < 0.0f) {
                power = 0.0f;
            }
            if (power > 1.0f) {
                power = 1.0f;
            }
        } else if (std::strcmp(argv[i], "--duration-ms") == 0 && i + 1 < argc) {
            durationMs = std::atoi(argv[++i]);
            if (durationMs < 50) {
                durationMs = 50;
            }
        } else if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
            PrintUsage(argv[0]);
            return 0;
        } else {
            PrintUsage(argv[0]);
            return 1;
        }
    }

    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    std::printf("Manus haptic cycle demo\n");
    if (!InitSdk()) {
        return 1;
    }

    std::printf("[haptic] waiting for paired haptic glove(s)...\n");
    std::vector<GloveTarget> gloves;
    for (int i = 0; i < 30 && g_running; ++i) {
        gloves = GetHapticGloves(handFilter);
        if (!gloves.empty()) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    if (gloves.empty()) {
        std::fprintf(stderr,
                     "[haptic] no paired haptic glove found. Check glove is Metagloves Pro Haptic.\n");
        CoreSdk_ShutDown();
        return 1;
    }

    for (const auto& glove : gloves) {
        std::printf("[haptic] target glove 0x%X (%s)\n", glove.id, SideName(glove.side));
    }

    int step = 0;
    while (g_running) {
        gloves = GetHapticGloves(handFilter);
        if (gloves.empty()) {
            std::fprintf(stderr, "[haptic] glove disconnected, waiting...\n");
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        const int fingerIdx = kCycleFingers[step % kCycleLen];
        float powers[kFingerCount] = {0, 0, 0, 0, 0};
        powers[fingerIdx] = power;

        for (const auto& glove : gloves) {
            SendVibration(glove.id, powers);
            std::printf("[haptic] %s %s 振动 %.0f%% (%dms)\n", SideName(glove.side),
                        FingerName(fingerIdx), power * 100.0f, durationMs);
        }

        const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(durationMs);
        while (g_running && std::chrono::steady_clock::now() < end) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        StopAll(gloves);
        step = (step + 1) % kCycleLen;
    }

    gloves = GetHapticGloves(handFilter);
    StopAll(gloves);
    std::printf("\n[haptic] stopped, motors off.\n");
    CoreSdk_ShutDown();
    return 0;
}
