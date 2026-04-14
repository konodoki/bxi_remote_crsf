#include <cstdint>
#include <cstdio>
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
        , port_raw_(port)
        , baudrate_(baudrate)
    {
        buildCRCTable();
        port_ = get_real_serial_port(port_raw_);

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
    /**
     * 构建 CRSF 电池电传包
     * @param voltage: 电压，单位 0.1V (例如 126 代表 12.6V)
     * @param current: 电流，单位 0.1A (例如 50 代表 5.0A)
     * @param fuel: 消耗容量，单位 mAh
     * @param remaining: 剩余百分比 (0-100)
     */
    void send_battery(uint16_t voltage, uint16_t current, uint32_t fuel,
                      uint8_t remaining)
    {
        if (!sp_.isOpen())
            return;
        uint8_t buffer_[256] = { 0 };
        uint8_t len =
            packBatterySensor(buffer_, voltage, current, fuel, remaining);
        sp_.writeData(buffer_, len);
    }
    /**
     * @param latitude: 纬度，单位 1e-7 (例如 30.1234567 -> 301234567)
     * @param longitude: 经度，单位 1e-7 (例如 120.1234567 -> 1201234567)
     * @param groundSpeed: 地速，单位 0.1km/h (例如 50 代表 5.0km/h)
     * @param heading: 航向，单位 0.01度 (例如 35000 代表 350.00度)
     * @param altitude: 海拔，单位 米 (需加上1000m偏移，例如填 1200 代表实际海拔
     * 200m)
     * @param satellites: 卫星数量 (0-255)
     */
    void send_gps(int32_t latitude, int32_t longitude, uint16_t groundSpeed,
                  uint16_t heading, uint16_t altitude, uint8_t satellites)
    {
        if (!sp_.isOpen())
            return;
        uint8_t buffer_[256] = { 0 };
        uint8_t len = packGPSSensor(buffer_, latitude, longitude, groundSpeed,
                                    heading, altitude, satellites);
        sp_.writeData(buffer_, len);
    }
    void send_text(std::string text)
    {
        if (!sp_.isOpen())
            return;
        uint8_t buffer_[256] = { 0 };
        uint8_t len = packTextSensor(buffer_, (uint8_t *)text.c_str(),
                                     strlen(text.c_str()));
        sp_.writeData(buffer_, len);
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
        if (port_ != portName && port_ != port_raw_)
            return;
        if (isAdd) {
            port_ = get_real_serial_port(port_raw_);
            if (port_ != portName)
                return;
            tryOpen();
        } else {
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
    /**
     * 构建 CRSF 电池电传包
     * @param buffer: 用于存储生成的包（建议至少 12 字节）
     * @param voltage: 电压，单位 0.1V (例如 126 代表 12.6V)
     * @param current: 电流，单位 0.1A (例如 50 代表 5.0A)
     * @param fuel: 消耗容量，单位 mAh
     * @param remaining: 剩余百分比 (0-100)
     * @return: 写入 buffer 的总字节数
     */
    uint8_t packBatterySensor(uint8_t *buffer, uint16_t voltage,
                              uint16_t current, uint32_t fuel,
                              uint8_t remaining)
    {
        // 1. 设置头部
        buffer[0] = 0xC8; // 目的地址: CRSF_ADDRESS_FLIGHT_CONTROLLER
        buffer[1] = 0x0A; // 长度: Type(1) + Data(8) + CRC(1) = 10

        // 2. 填充 Payload (从 buffer[2] 开始计算 CRC)
        buffer[2] =
            CRSF::FRAMETYPE_BATTERY_SENSOR; // Payload Type:
                                            // CRSF_FRAMETYPE_BATTERY_SENSOR

        // 电压 (大端序)
        buffer[3] = (uint8_t)(voltage >> 8);
        buffer[4] = (uint8_t)(voltage & 0xFF);

        // 电流 (大端序)
        buffer[5] = (uint8_t)(current >> 8);
        buffer[6] = (uint8_t)(current & 0xFF);

        // 容量 (3字节, 大端序)
        buffer[7] = (uint8_t)((fuel >> 16) & 0xFF);
        buffer[8] = (uint8_t)((fuel >> 8) & 0xFF);
        buffer[9] = (uint8_t)(fuel & 0xFF);

        // 剩余电量
        buffer[10] = remaining;

        // 3. 计算 CRC
        // 根据 CRSF 标准，校验和计算范围是 [Type] 到 [Data_Last]
        // 初始值通常为 0
        buffer[11] = crc8Bulk(&buffer[2], 9, 0);

        return 12; // 总长度: Addr(1) + Len(1) + Type(1) + Data(8) + CRC(1)
    }
    /**
     * 构建 CRSF GPS 电传包
     * @param buffer: 用于存储生成的包（建议至少 18 字节）
     * @param latitude: 纬度，单位 1e-7 (例如 30.1234567 -> 301234567)
     * @param longitude: 经度，单位 1e-7 (例如 120.1234567 -> 1201234567)
     * @param groundSpeed: 地速，单位 0.1km/h (例如 50 代表 5.0km/h)
     * @param heading: 航向，单位 0.01度 (例如 35000 代表 350.00度)
     * @param altitude: 海拔，单位 米 (需加上1000m偏移，例如填 1200 代表实际海拔
     * 200m)
     * @param satellites: 卫星数量 (0-255)
     * @return: 写入 buffer 的总字节数
     */
    uint8_t packGPSSensor(uint8_t *buffer, int32_t latitude, int32_t longitude,
                          uint16_t groundSpeed, uint16_t heading,
                          uint16_t altitude, uint8_t satellites)
    {
        // 1. 设置头部
        buffer[0] = 0xC8; // 目的地址 (C8 兼容性最好)
        buffer[1] = 0x11; // 长度: Type(1) + Data(15) + CRC(1) = 17 (0x11)

        // 2. 填充 Payload (从 buffer[2] 开始计算 CRC)
        buffer[2] = CRSF::FRAMETYPE_GPS; // 0x02

        // 纬度 Latitude (4字节, Big-Endian)
        buffer[3] = (uint8_t)(latitude >> 24);
        buffer[4] = (uint8_t)(latitude >> 16);
        buffer[5] = (uint8_t)(latitude >> 8);
        buffer[6] = (uint8_t)(latitude & 0xFF);

        // 经度 Longitude (4字节, Big-Endian)
        buffer[7] = (uint8_t)(longitude >> 24);
        buffer[8] = (uint8_t)(longitude >> 16);
        buffer[9] = (uint8_t)(longitude >> 8);
        buffer[10] = (uint8_t)(longitude & 0xFF);

        // 地速 Ground Speed (2字节, Big-Endian)
        buffer[11] = (uint8_t)(groundSpeed >> 8);
        buffer[12] = (uint8_t)(groundSpeed & 0xFF);

        // 航向 Heading (2字节, Big-Endian)
        buffer[13] = (uint8_t)(heading >> 8);
        buffer[14] = (uint8_t)(heading & 0xFF);

        // 海拔 Altitude (2字节, Big-Endian)
        // 注意：CRSF 协议规定海拔需要加上 1000m 的偏移量
        buffer[15] = (uint8_t)(altitude >> 8);
        buffer[16] = (uint8_t)(altitude & 0xFF);

        // 卫星数 Satellites (1字节)
        buffer[17] = satellites;

        // 3. 计算 CRC
        // 校验范围: buffer[2] 到 buffer[17]，共 16 字节 (Type + 15字节数据)
        buffer[18] = crc8Bulk(&buffer[2], 16, 0);

        return 19; // 总长度: Addr(1) + Len(1) + Type(1) + Data(15) + CRC(1) =
                   // 19
    }
    /**
     * 构建 CRSF 文本电传包 (Type 0x21)
     * @param buffer: 用于存储生成的包
     * @param motorIdx: 电机编号 (1-31)
     * @param temp: 温度值
     * @return: 写入 buffer 的总字节数
     */
    uint8_t packTextSensor(uint8_t *buffer, uint8_t *textBuf, uint8_t strLen)
    {
        // 1. 设置头部
        buffer[0] = 0xC8;
        // 长度 = Type(1) + String(strLen) + NullTerminator(1) + CRC(1)
        buffer[1] = strLen + 3;

        // 2. 填充 Payload
        buffer[2] = 0x21; // Type: CRSF_FRAMETYPE_TEXT_SELECTION / General Text

        // 拷贝字符串到 buffer
        memcpy(&buffer[3], textBuf, strLen);

        // 字符串结束符
        buffer[3 + strLen] = 0x00;

        // 3. 计算 CRC
        // 校验范围从 buffer[2] (Type) 到结束符为止
        buffer[3 + strLen + 1] = crc8Bulk(&buffer[2], strLen + 2, 0);

        return strLen + 5; // 总长度: Addr + Len + Type + String + Null + CRC
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
    std::string port_raw_;
    std::string port_;
    int baudrate_;
    bool debug_ = false;
};
