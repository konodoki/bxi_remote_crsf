#include <stdint.h>
#include "CSerialPort/SerialPort.h"
#include "CSerialPort/SerialPortInfo.h"
#include <functional>
#include <string>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <vector>
using namespace itas109;
namespace fs = std::filesystem;

namespace CRSF
{
static constexpr uint8_t SYNC_BYTE = 0xC8;
static constexpr size_t MAX_FRAME_SIZE = 64;
static constexpr uint8_t FRAMETYPE_RC_CHANNELS_PACKED = 0x16;
static constexpr uint8_t FRAMETYPE_LINK_STATISTICS = 0x14;
static constexpr uint8_t FRAMETYPE_GPS = 0x02;
static constexpr uint8_t FRAMETYPE_BATTERY_SENSOR = 0x08;
static constexpr uint8_t FRAMETYPE_HEARTBEAT = 0x0B;

static constexpr int LS_UPLINK_RSSI_1 = 0;
static constexpr int LS_UPLINK_LQ = 2;
static constexpr int LS_PAYLOAD_SIZE = 10;
}

class CRSFParser : public CSerialPortListener,
                   public CSerialPortHotPlugListener {
public:
    CRSFParser(std::string port, int baudrate,
               std::function<void(uint16_t[])> cb)
        : channel_cb_(cb)
        , port_(port)
        , baudrate_(baudrate)
    {
        buildCRCTable();
        port_ = get_real_serial_port(port_);

        auto ports = CSerialPortInfo::availablePortInfos();
        printf("AvailableFriendlyPorts:\n");
        bool has_port = false;
        for (size_t i = 0; i < ports.size(); ++i) {
            printf("%zu - %s %s %s\n", i + 1, ports[i].portName,
                   ports[i].description, ports[i].hardwareId);
            if (port_ == ports[i].portName)
                has_port = true;
        }
        if (!has_port)
            printf("没有找到串口: %s\n", port_.c_str());
        fflush(stdout);

        sp_.connectReadEvent(this);
        sp_.connectHotPlugEvent(this);
        sp_.init(port_.c_str(), baudrate_, itas109::ParityNone,
                 itas109::DataBits8, itas109::StopOne, itas109::FlowNone, 4096);
        if (has_port)
            tryOpen();
    }

private:
    // ----------------------------------------------------------
    // 串口事件：收到数据 → 追加到 buffer → 解析
    // ----------------------------------------------------------
    void onReadEvent(const char * /*portName*/,
                     unsigned int /*readBufferLen*/) override
    {
        uint8_t tmp[4096];
        int n = sp_.readData(tmp, sizeof(tmp));
        if (n <= 0)
            return;

        // 追加到滑动缓冲区
        buffer_.insert(buffer_.end(), tmp, tmp + n);
        processBuffer();
    }

    // ----------------------------------------------------------
    // 核心解析：从缓冲区里不断提取完整的 CRSF 帧
    //
    // CRSF 帧格式：
    //   [0]      SYNC  = 0xC8
    //   [1]      LENGTH（后续字节数，含 TYPE + PAYLOAD + CRC）
    //   [2]      TYPE
    //   [3..N-1] PAYLOAD（LENGTH-2 字节）
    //   [N]      CRC（对 TYPE+PAYLOAD 做 CRC-8/DVB-S2）
    //
    // 最小完整帧 = 1(SYNC) + 1(LEN) + LENGTH 字节
    // ----------------------------------------------------------
    void processBuffer()
    {
        while (!buffer_.empty()) {
            // 1. 找 SYNC 字节
            if (buffer_[0] != CRSF::SYNC_BYTE) {
                buffer_.erase(buffer_.begin()); // 丢弃错位字节，继续找头
                continue;
            }

            // 2. 等够 2 个字节才能读到 LENGTH
            if (buffer_.size() < 2)
                break;

            uint8_t length = buffer_[1];

            // 合法性检查：LENGTH 表示 TYPE+PAYLOAD+CRC，至少 3 字节
            if (length < 3 || length > CRSF::MAX_FRAME_SIZE) {
                buffer_.erase(buffer_.begin()); // 非法 length，丢掉这个 SYNC
                continue;
            }

            // 3. 完整帧 = SYNC(1) + LENGTH(1) + length 字节，等数据凑够再处理
            size_t frame_total = 2 + length;
            if (buffer_.size() < frame_total)
                break;

            // 4. 取出各字段
            uint8_t frame_type = buffer_[2];
            size_t payload_len = length - 2; // 去掉 TYPE 和 CRC
            const uint8_t *payload = buffer_.data() + 3; // payload 起始
            uint8_t crc_received = buffer_[2 + length - 1]; // 帧最后一字节

            // 5. 校验 CRC（覆盖 TYPE + PAYLOAD）
            uint8_t crc_calc =
                crc8Bulk(payload - 1, payload_len + 1, 0); // 从 TYPE 开始
            if (crc_calc != crc_received) {
                if (debug_)
                    printf("CRC 校验失败: 期望 0x%02X, 实际 0x%02X\n", crc_calc,
                           crc_received);
                buffer_.erase(buffer_.begin()); // CRC 错就丢掉这个 SYNC，重新找
                continue;
            }

            // 6. CRC 通过，处理帧
            processFrame(frame_type, payload, payload_len);

            // 7. 从缓冲区移除这一整帧
            buffer_.erase(buffer_.begin(), buffer_.begin() + frame_total);
        }
    }

    // ----------------------------------------------------------
    // 帧分发
    // ----------------------------------------------------------
    void processFrame(uint8_t type, const uint8_t *payload, size_t payload_len)
    {
        switch (type) {
        case CRSF::FRAMETYPE_RC_CHANNELS_PACKED:
            processRCChannels(payload, payload_len);
            break;
        case CRSF::FRAMETYPE_LINK_STATISTICS:
            processLinkStatistics(payload, payload_len);
            break;
        case CRSF::FRAMETYPE_GPS:
            if (debug_)
                printf("GPS 帧\n");
            break;
        case CRSF::FRAMETYPE_BATTERY_SENSOR:
            if (debug_)
                printf("电池帧\n");
            break;
        case CRSF::FRAMETYPE_HEARTBEAT:
            printf("心跳\n");
            break;
        default:
            if (debug_)
                printf("未知帧类型: 0x%02X\n", type);
            break;
        }
    }

    void processRCChannels(const uint8_t *payload, size_t payload_len)
    {
        if (payload_len < 22)
            return; // RC 通道固定 22 字节

        const uint8_t *p = payload;
        uint32_t bits = 0;
        uint8_t bit_cnt = 0;
        for (int ch = 0; ch < 16; ch++) {
            while (bit_cnt < 11) {
                bits |= static_cast<uint32_t>(*p++) << bit_cnt;
                bit_cnt += 8;
            }
            channels_[ch] = bits & 0x7FFu;
            bits >>= 11;
            bit_cnt -= 11;
        }
        channel_cb_(channels_);
    }

    void processLinkStatistics(const uint8_t *payload, size_t payload_len)
    {
        if (payload_len < static_cast<size_t>(CRSF::LS_PAYLOAD_SIZE)) {
            if (debug_)
                printf("Link Statistics 帧长度不足: %zu\n", payload_len);
            return;
        }
        uint8_t rssi_dbm = payload[CRSF::LS_UPLINK_RSSI_1];
        uint8_t link_quality = payload[CRSF::LS_UPLINK_LQ];

        rssi_dbm_ = rssi_dbm;
        link_quality_ = link_quality;

        bool new_failsafe = (link_quality_ < 20 || rssi_dbm_ > 110);
        if (new_failsafe != failsafe_) {
            failsafe_ = new_failsafe;
            printf(failsafe_ ? "⚠️  失控保护激活! LQ: %d%%, RSSI: -%d dBm\n" :
                               "✅  链路恢复. LQ: %d%%, RSSI: -%d dBm\n",
                   link_quality_, rssi_dbm_);
        }
    }

    // ----------------------------------------------------------
    // 热插拔 & 开启
    // ----------------------------------------------------------
    void onHotPlugEvent(const char *portName, int isAdd) override
    {
        printf("热插拔: %s %s\n", portName, isAdd ? "插入" : "拔出");
        if (port_ != portName)
            return;
        if (isAdd)
            tryOpen();
        else {
            sp_.close();
            buffer_.clear(); // 断开时清空缓冲区，防止脏数据残留
            printf("串口已关闭: %s\n", port_.c_str());
        }
    }

    void tryOpen()
    {
        sp_.open();
        printf("打开 %s %s\n", port_.c_str(), sp_.isOpen() ? "成功" : "失败");
        if (!sp_.isOpen()) {
            printf("Code: %d, Message: %s\n", sp_.getLastError(),
                   sp_.getLastErrorMsg());
            exit(1);
        }
    }

    std::string get_real_serial_port(const std::string &port_path)
    {
        try {
            fs::path path(port_path);
            if (!fs::exists(path))
                return port_path;
            if (fs::is_symlink(path)) {
                std::cout << "真实路径:" << fs::canonical(path).string()
                          << std::endl;
                return fs::canonical(path).string();
            }
        } catch (const fs::filesystem_error &e) {
            std::cerr << "FS Error: " << e.what() << std::endl;
        }
        return port_path;
    }

    // ----------------------------------------------------------
    // CRC-8/DVB-S2 (poly 0xD5)
    // ----------------------------------------------------------
    uint8_t crc_table_[256];

    void buildCRCTable()
    {
        for (int i = 0; i < 256; i++) {
            uint8_t crc = static_cast<uint8_t>(i);
            for (int j = 0; j < 8; j++)
                crc = (crc & 0x80) ? (crc << 1) ^ 0xD5u : (crc << 1);
            crc_table_[i] = crc;
        }
    }

    inline uint8_t crc8Bulk(const uint8_t *data, size_t len, uint8_t crc) const
    {
        while (len--)
            crc = crc_table_[crc ^ *data++];
        return crc;
    }

    // ----------------------------------------------------------
    // 成员变量
    // ----------------------------------------------------------
    CSerialPort sp_;
    std::function<void(uint16_t[])> channel_cb_;
    std::vector<uint8_t> buffer_; // 滑动接收缓冲区

    uint16_t channels_[16]{};
    uint8_t link_quality_ = 100;
    uint8_t rssi_dbm_ = 0;
    bool failsafe_ = false;

    std::string port_;
    int baudrate_;
    bool debug_ = false;
};
