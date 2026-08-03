// Pair Manus Metagloves with the connected dongle (Integrated SDK mode).
// Build: make pair_gloves.out

#include "ManusSDK.h"
#include "ManusSDKTypes.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

namespace {

Landscape* g_landscape = nullptr;
std::mutex g_mutex;

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
        std::fprintf(stderr, "[pair] CoreSdk_InitializeIntegrated failed\n");
        return false;
    }

    if (CoreSdk_RegisterCallbackForLandscapeStream(OnLandscape) != SDKReturnCode_Success) {
        std::fprintf(stderr, "[pair] failed to register landscape callback\n");
        return false;
    }

    CoordinateSystemVUH vuh{};
    CoordinateSystemVUH_Init(&vuh);
    vuh.handedness = Side_Right;
    vuh.up = AxisPolarity_PositiveZ;
    vuh.view = AxisView_XFromViewer;
    vuh.unitScale = 1.0f;

    if (CoreSdk_InitializeCoordinateSystemWithVUH(vuh, true) != SDKReturnCode_Success) {
        std::fprintf(stderr, "[pair] failed to initialize coordinate system\n");
        return false;
    }

    ManusHost host{};
    ManusHost_Init(&host);
    if (CoreSdk_ConnectToHost(host) != SDKReturnCode_Success) {
        std::fprintf(stderr, "[pair] CoreSdk_ConnectToHost failed\n");
        return false;
    }

    return true;
}

void PrintDevices() {
    std::lock_guard<std::mutex> lock(g_mutex);
    if (g_landscape == nullptr) {
        std::printf("[pair] waiting for device list...\n");
        return;
    }

    std::printf("[pair] dongles: %u, gloves: %u\n", g_landscape->gloveDevices.dongleCount,
                g_landscape->gloveDevices.gloveCount);

    for (uint32_t i = 0; i < g_landscape->gloveDevices.dongleCount; ++i) {
        const auto& d = g_landscape->gloveDevices.dongles[i];
        std::printf("  dongle id=0x%X family=%u fw=%u.%u.%u\n", d.id, d.familyType,
                    d.firmwareVersion.major, d.firmwareVersion.minor, d.firmwareVersion.patch);
    }

    for (uint32_t i = 0; i < g_landscape->gloveDevices.gloveCount; ++i) {
        const auto& g = g_landscape->gloveDevices.gloves[i];
        const char* side = g.side == Side_Left ? "left" : g.side == Side_Right ? "right" : "?";
        const char* state = "?";
        switch (g.pairedState) {
        case DevicePairedState_Paired:
            state = "paired";
            break;
        case DevicePairedState_Unpaired:
            state = "unpaired";
            break;
        case DevicePairedState_Pairing:
            state = "pairing";
            break;
        default:
            break;
        }
        std::printf("  glove id=0x%X side=%s state=%s fw=%u.%u.%u\n", g.id, side, state,
                    g.firmwareVersion.major, g.firmwareVersion.minor, g.firmwareVersion.patch);
    }
}

bool PairUnpairedGloves() {
    std::vector<uint32_t> toPair;
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        if (g_landscape == nullptr) {
            return false;
        }
        for (uint32_t i = 0; i < g_landscape->gloveDevices.gloveCount; ++i) {
            const auto& g = g_landscape->gloveDevices.gloves[i];
            if (g.pairedState == DevicePairedState_Unpaired ||
                g.pairedState == DevicePairedState_Pairing) {
                toPair.push_back(g.id);
            }
        }
    }

    if (toPair.empty()) {
        return false;
    }

    bool anyPaired = false;
    for (uint32_t gloveId : toPair) {
        bool paired = false;
        const SDKReturnCode rc = CoreSdk_PairGlove(gloveId, &paired);
        std::printf("[pair] PairGlove(0x%X) -> rc=%d paired=%s\n", gloveId, static_cast<int>(rc),
                    paired ? "yes" : "no");
        anyPaired = anyPaired || paired;
    }
    return anyPaired;
}

}  // namespace

int main() {
    std::printf("Manus glove pairing tool\n");
    std::printf("Put gloves in pairing mode first (switch past ON to Wi-Fi icon, white LED blinks).\n\n");

    if (!InitSdk()) {
        return 1;
    }

    std::printf("[pair] SDK ready, scanning for gloves (up to 60s)...\n");

    for (int i = 0; i < 60; ++i) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        PrintDevices();

        if (PairUnpairedGloves()) {
            std::printf("\n[pair] Pair command sent. Watch glove LED — solid blue = success.\n");
            std::this_thread::sleep_for(std::chrono::seconds(3));
            PrintDevices();
            CoreSdk_ShutDown();
            return 0;
        }
    }

    std::fprintf(stderr,
                 "\n[pair] No unpaired glove detected.\n"
                 "Checklist:\n"
                 "  1. Dongle plugged in (lsusb should show Manus VR Sensor Dongle)\n"
                 "  2. Glove switch pushed RIGHT past ON to Wi-Fi symbol\n"
                 "  3. Keep glove within 1m of dongle, no metal between them\n"
                 "  4. Only pair one glove at a time for first setup\n"
                 "  5. Glove/dongle firmware and series must match\n");
    CoreSdk_ShutDown();
    return 1;
}
