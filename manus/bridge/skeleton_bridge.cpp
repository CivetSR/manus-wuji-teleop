// Manus SDK skeleton bridge — streams raw hand skeleton over UDP as JSON.
// Requires Manus SDK (integrated mode) from https://docs.manus-meta.com/
//
// Build: make MANUS_SDK=/path/to/ManusSDK

#include "ManusSDK.h"
#include "ManusSDKTypes.h"

#include <arpa/inet.h>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

constexpr uint16_t kDefaultPort = 9876;
constexpr const char* kDefaultHost = "127.0.0.1";

struct ClientRawSkeleton {
    RawSkeletonInfo info{};
    std::vector<SkeletonNode> nodes;
};

struct ClientRawSkeletonCollection {
    std::vector<ClientRawSkeleton> skeletons;
};

class SkeletonBridge {
public:
    static SkeletonBridge* Instance() { return s_instance; }

    bool Initialize(int port, const char* host) {
        m_port = port;
        m_host = host;

        if (CoreSdk_InitializeIntegrated() != SDKReturnCode_Success) {
            std::fprintf(stderr, "[bridge] CoreSdk_InitializeIntegrated failed\n");
            return false;
        }

        if (CoreSdk_RegisterCallbackForRawSkeletonStream(OnRawSkeletonStream) !=
            SDKReturnCode_Success) {
            std::fprintf(stderr, "[bridge] failed to register raw skeleton callback\n");
            return false;
        }

        CoordinateSystemVUH vuh{};
        CoordinateSystemVUH_Init(&vuh);
        vuh.handedness = Side_Right;
        vuh.up = AxisPolarity_PositiveZ;
        vuh.view = AxisView_XFromViewer;
        vuh.unitScale = 1.0f;

        if (CoreSdk_InitializeCoordinateSystemWithVUH(vuh, true) != SDKReturnCode_Success) {
            std::fprintf(stderr, "[bridge] failed to initialize coordinate system\n");
            return false;
        }

        s_instance = this;
        return true;
    }

    bool Connect() {
        ManusHost host{};
        ManusHost_Init(&host);

        const SDKReturnCode connectResult = CoreSdk_ConnectToHost(host);
        if (connectResult != SDKReturnCode_Success) {
            std::fprintf(stderr, "[bridge] CoreSdk_ConnectToHost failed (%d)\n",
                         static_cast<int>(connectResult));
            return false;
        }

        if (CoreSdk_SetRawSkeletonHandMotion(HandMotion_Auto) != SDKReturnCode_Success) {
            std::fprintf(stderr, "[bridge] warning: failed to set HandMotion_Auto\n");
        }

        std::printf("[bridge] connected to Manus host\n");
        return true;
    }

    void Run() {
        std::printf("[bridge] Manus integrated mode running, streaming to %s:%d\n", m_host.c_str(),
                    m_port);
        std::printf("[bridge] waiting for glove data (pair gloves if LED blinks white)...\n");

        while (m_running) {
            ClientRawSkeletonCollection* frame = nullptr;
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                frame = m_pending;
                m_pending = nullptr;
            }

            if (frame != nullptr) {
                PublishFrame(*frame);
                delete frame;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }
    }

    void Stop() { m_running = false; }

private:
    static void OnRawSkeletonStream(const SkeletonStreamInfo* info) {
        if (s_instance == nullptr || info == nullptr) {
            return;
        }

        auto* collection = new ClientRawSkeletonCollection();
        collection->skeletons.resize(info->skeletonsCount);

        for (uint32_t i = 0; i < info->skeletonsCount; ++i) {
            auto& skeleton = collection->skeletons[i];
            CoreSdk_GetRawSkeletonInfo(i, &skeleton.info);
            skeleton.nodes.resize(skeleton.info.nodesCount);
            skeleton.info.publishTime = info->publishTime;
            CoreSdk_GetRawSkeletonData(i, skeleton.nodes.data(), skeleton.info.nodesCount);
        }

        std::lock_guard<std::mutex> lock(s_instance->m_mutex);
        if (s_instance->m_pending != nullptr) {
            delete s_instance->m_pending;
        }
        s_instance->m_pending = collection;
    }

    static const char* SideToString(Side side) {
        switch (side) {
        case Side_Left:
            return "left";
        case Side_Right:
            return "right";
        default:
            return "unknown";
        }
    }

    static const char* ChainTypeToString(ChainType type) {
        switch (type) {
        case ChainType_Hand:
            return "hand";
        case ChainType_FingerThumb:
            return "thumb";
        case ChainType_FingerIndex:
            return "index";
        case ChainType_FingerMiddle:
            return "middle";
        case ChainType_FingerRing:
            return "ring";
        case ChainType_FingerPinky:
            return "pinky";
        default:
            return "other";
        }
    }

    void PublishFrame(const ClientRawSkeletonCollection& frame) {
        std::string json = BuildJson(frame);
        SendUdp(json);
        ++m_frameCounter;
    }

    std::string BuildJson(const ClientRawSkeletonCollection& frame) {
        std::string json;
        json.reserve(8192);
        json += "{\"frame\":";
        json += std::to_string(m_frameCounter);
        json += ",\"skeletons\":[";

        for (size_t s = 0; s < frame.skeletons.size(); ++s) {
            const auto& skeleton = frame.skeletons[s];
            const uint32_t gloveId = skeleton.info.gloveId;

            uint32_t nodeCount = 0;
            if (CoreSdk_GetRawSkeletonNodeCount(gloveId, nodeCount) != SDKReturnCode_Success ||
                nodeCount == 0) {
                continue;
            }

            std::vector<NodeInfo> hierarchy(nodeCount);
            if (CoreSdk_GetRawSkeletonNodeInfoArray(gloveId, hierarchy.data(), nodeCount) !=
                SDKReturnCode_Success) {
                continue;
            }

            if (s > 0) {
                json += ',';
            }

            json += "{\"glove_id\":";
            json += std::to_string(gloveId);
            json += ",\"nodes\":[";

            for (uint32_t n = 0; n < nodeCount; ++n) {
                const NodeInfo& info = hierarchy[n];
                const SkeletonNode& node = skeleton.nodes[n];
                const ManusVec3& p = node.transform.position;

                if (n > 0) {
                    json += ',';
                }

                json += "{\"id\":";
                json += std::to_string(info.nodeId);
                json += ",\"parent_id\":";
                json += std::to_string(info.parentId);
                json += ",\"side\":\"";
                json += SideToString(info.side);
                json += "\",\"chain\":\"";
                json += ChainTypeToString(info.chainType);
                json += "\",\"x\":";
                json += std::to_string(p.x);
                json += ",\"y\":";
                json += std::to_string(p.y);
                json += ",\"z\":";
                json += std::to_string(p.z);
                json += '}';
            }

            json += "]}";
        }

        json += "]}";
        return json;
    }

    void SendUdp(const std::string& payload) {
        if (m_socket < 0) {
            m_socket = socket(AF_INET, SOCK_DGRAM, 0);
            if (m_socket < 0) {
                return;
            }
        }

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(m_port);
        inet_pton(AF_INET, m_host.c_str(), &addr.sin_addr);

        sendto(m_socket, payload.data(), payload.size(), 0, reinterpret_cast<sockaddr*>(&addr),
               sizeof(addr));
    }

    static SkeletonBridge* s_instance;

    std::atomic<bool> m_running{true};
    std::mutex m_mutex;
    ClientRawSkeletonCollection* m_pending = nullptr;
    uint64_t m_frameCounter = 0;
    int m_port = kDefaultPort;
    std::string m_host = kDefaultHost;
    int m_socket = -1;
};

SkeletonBridge* SkeletonBridge::s_instance = nullptr;

}  // namespace

int main(int argc, char** argv) {
    int port = kDefaultPort;
    const char* host = kDefaultHost;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
            port = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--host") == 0 && i + 1 < argc) {
            host = argv[++i];
        }
    }

    SkeletonBridge bridge;
    if (!bridge.Initialize(port, host)) {
        return 1;
    }

    while (!bridge.Connect()) {
        std::fprintf(stderr,
                     "[bridge] not connected yet — ensure dongle is plugged in and gloves are "
                     "paired (solid blue LED)\n");
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    bridge.Run();
    CoreSdk_ShutDown();
    return 0;
}
