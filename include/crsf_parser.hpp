#include <stdint.h>
#include "CSerialPort/SerialPort.h"
#include "CSerialPort/SerialPortInfo.h"
#include <functional>
#include <string>
#include <cstring>
#include <filesystem>
#include <iostream>
using namespace itas109;
namespace fs = std::filesystem;
// ============================================================
// CRSF 协议常量（独立出来，方便复用）
// ============================================================
namespace CRSF
{
static constexpr uint8_t SYNC_BYTE = 0xC8;
static constexpr size_t MAX_FRAME_SIZE = 64;
static constexpr uint8_t FRAMETYPE_RC_CHANNELS_PACKED = 0x16;
static constexpr uint8_t FRAMETYPE_LINK_STATISTICS = 0x14;
static constexpr uint8_t FRAMETYPE_GPS = 0x02;
static constexpr uint8_t FRAMETYPE_BATTERY_SENSOR = 0x08;

// 标准 Link Statistics 帧字段偏移（10字节 payload）
static constexpr int LS_UPLINK_RSSI_1 = 0;
static constexpr int LS_UPLINK_RSSI_2 = 1;
static constexpr int LS_UPLINK_LQ = 2;
static constexpr int LS_UPLINK_SNR = 3;
static constexpr int LS_ACTIVE_ANT = 4;
static constexpr int LS_RF_MODE = 5;
static constexpr int LS_TX_POWER = 6;
static constexpr int LS_DOWNLINK_RSSI = 7;
static constexpr int LS_DOWNLINK_LQ = 8;
static constexpr int LS_DOWNLINK_SNR = 9;
static constexpr int LS_PAYLOAD_SIZE = 10;
}

// ============================================================
// CRSFParser
// ============================================================
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
    /**
     * 获取串口的真实路径
     * @param port_path 传入的路径（如 "/dev/ttyCRSF"）
     * @return 如果是软链接则返回真实路径（如 "/dev/ttyUSB0"），否则返回原路径
     */
    std::string get_real_serial_port(const std::string &port_path)
    {
        try {
            fs::path path(port_path);

            // 1. 检查文件是否存在
            if (!fs::exists(path)) {
                return port_path;
            }

            // 2. 检查是否是符号链接（软链接）
            if (fs::is_symlink(path)) {
                // fs::canonical 会解析路径中所有的符号链接并返回绝对路径
                std::cout << "真实路径:" << fs::canonical(path).string()
                          << std::endl;
                return fs::canonical(path).string();
            }
        } catch (const fs::filesystem_error &e) {
            // 如果发生权限错误或其他系统错误，返回原始路径
            // 这样可以防止程序因为获取不到真实路径而直接崩溃
            std::cerr << "FS Error: " << e.what() << std::endl;
        }

        return port_path;
    }
    // ----------------------------------------------------------
    // 串口事件（在 CSerialPort 内部线程中被调用）
    // ----------------------------------------------------------
    void onReadEvent(const char * /*portName*/,
                     unsigned int /*readBufferLen*/) override
    {
        uint8_t buf[4096];
        int n = sp_.readData(buf, sizeof(buf));
        if (n > 0)
            parseBuffer(buf, n);
    }

    void onHotPlugEvent(const char *portName, int isAdd) override
    {
        printf("热插拔: %s %s\n", portName, isAdd ? "插入" : "拔出");
        if (port_ != portName)
            return;
        if (isAdd)
            tryOpen();
        else {
            sp_.close();
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

    // ----------------------------------------------------------
    // 状态机解析（顺序与帧格式一致：SYNC→LENGTH→TYPE→PAYLOAD→CRC）
    // ----------------------------------------------------------
    enum ParserState : uint8_t {
        STATE_SYNC,
        STATE_LENGTH,
        STATE_TYPE,
        STATE_PAYLOAD,
        STATE_CRC
    };

    void parseBuffer(const uint8_t *buf, int len)
    {
        int i = 0;
        while (i < len) {
            switch (parser_state_) {
            case STATE_SYNC:
                while (i < len && buf[i] != CRSF::SYNC_BYTE)
                    i++;
                if (i < len) {
                    parser_state_ = STATE_LENGTH;
                    crc_accum_ = 0;
                    data_index_ = 0;
                    i++;
                }
                break;

            case STATE_LENGTH:
                frame_length_ = buf[i++];
                // 合法帧：payload+type+crc 至少3字节，最多MAX_FRAME_SIZE
                if (frame_length_ > 2 && frame_length_ <= CRSF::MAX_FRAME_SIZE)
                    parser_state_ = STATE_TYPE;
                else
                    parser_state_ = STATE_SYNC;
                break;

            case STATE_TYPE:
                frame_type_ = buf[i++];
                crc_accum_ = crc_table_[frame_type_];
                parser_state_ = STATE_PAYLOAD;
                break;

            case STATE_PAYLOAD: {
                // payload 长度 = frame_length_ - 2（TYPE 和 CRC 各1字节）
                size_t payload_total = frame_length_ - 2;
                size_t remaining = payload_total - data_index_;
                size_t available = static_cast<size_t>(len - i);
                size_t chunk = std::min(remaining, available);

                memcpy(frame_buffer_ + data_index_, buf + i, chunk);
                crc_accum_ = crc8Bulk(buf + i, chunk, crc_accum_);
                data_index_ += chunk;
                i += static_cast<int>(chunk);

                if (data_index_ >= payload_total)
                    parser_state_ = STATE_CRC;
                break;
            }

            case STATE_CRC:
                if (crc_accum_ == buf[i])
                    processFrame();
                else if (debug_)
                    printf("CRC 校验失败: 期望 0x%02X, 实际 0x%02X\n",
                           crc_accum_, buf[i]);
                i++;
                parser_state_ = STATE_SYNC;
                break;
            }
        }
    }

    // ----------------------------------------------------------
    // 帧处理
    // ----------------------------------------------------------
    void processFrame()
    {
        switch (frame_type_) {
        case CRSF::FRAMETYPE_RC_CHANNELS_PACKED:
            processRCChannels();
            break;
        case CRSF::FRAMETYPE_LINK_STATISTICS:
            processLinkStatistics();
            break;
        case CRSF::FRAMETYPE_GPS:
            if (debug_)
                printf("GPS 帧\n");
            break;
        case CRSF::FRAMETYPE_BATTERY_SENSOR:
            if (debug_)
                printf("电池帧\n");
            break;
        default:
            if (debug_)
                printf("未知帧类型: 0x%02X\n", frame_type_);
            break;
        }
    }

    void processRCChannels()
    {
        // CRSF RC 通道：16 × 11bit，紧密排列，共 22 字节
        const uint8_t *p = frame_buffer_;
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

    void processLinkStatistics()
    {
        // 修正：按照标准 CRSF Link Statistics 帧格式解析
        if (data_index_ < static_cast<size_t>(CRSF::LS_PAYLOAD_SIZE)) {
            if (debug_)
                printf("Link Statistics 帧长度不足: %zu\n", data_index_);
            return;
        }
        uint8_t uplink_rssi_1 = frame_buffer_[CRSF::LS_UPLINK_RSSI_1]; // -dBm
        link_quality_ = frame_buffer_[CRSF::LS_UPLINK_LQ]; // 0~100%
        rssi_dbm_ = uplink_rssi_1;

        // 失控保护：LQ < 20% 或 RSSI > 110dBm（即信号强度低于 -110dBm）
        bool new_failsafe = (link_quality_ < 20 || rssi_dbm_ > 110);
        if (new_failsafe != failsafe_) {
            failsafe_ = new_failsafe;
            printf(failsafe_ ? "⚠️  失控保护激活! LQ: %d%%, RSSI: -%d dBm\n" :
                               "✅  链路恢复. LQ: %d%%, RSSI: -%d dBm\n",
                   link_quality_, rssi_dbm_);
        }
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

    // 解析器状态
    ParserState parser_state_ = STATE_SYNC;
    uint8_t frame_type_ = 0;
    uint8_t frame_length_ = 0;
    uint8_t frame_buffer_[CRSF::MAX_FRAME_SIZE]{};
    size_t data_index_ = 0;
    uint8_t crc_accum_ = 0;

    // 通道 & 链路
    uint16_t channels_[16]{};
    uint8_t link_quality_ = 100;
    uint8_t rssi_dbm_ = 0;
    bool failsafe_ = false;

    std::string port_;
    int baudrate_;
    bool debug_ = false;
};
