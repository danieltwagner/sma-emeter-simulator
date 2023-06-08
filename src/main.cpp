#ifdef _WIN32
#include <Winsock2.h>
#include <Ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include <chrono>
#include <iostream>
#include <thread>

#include <AddressConversion.hpp>
#include <Logger.hpp>
#include <SpeedwireSocketFactory.hpp>
#include <SpeedwireHeader.hpp>
#include <SpeedwireEmeterProtocol.hpp>
#include <ObisData.hpp>
using namespace libspeedwire;
using namespace std::chrono;

// since firmware version 2.03.4.R a frequency measurement has been added to emeter packets
// and the udp packet size is 608 bytes
#define INCLUDE_FREQUENCY_MEASUREMENT (1)
#if INCLUDE_FREQUENCY_MEASUREMENT
  #define UDP_PACKET_SIZE 608
#else
  #define UDP_PACKET_SIZE 600
#endif


static void* insert(SpeedwireEmeterProtocol& emeter_packet, void* const obis, const ObisData& obis_data, const double value);
static void* insert(SpeedwireEmeterProtocol& emeter_packet, void* const obis, const ObisData& obis_data, const std::string& value);

class LogListener : public ILogListener {
public:
    virtual ~LogListener() {}

    virtual void log_msg(const std::string& msg, const LogLevel& level) {
        fprintf(stdout, "%s", msg.c_str());
    }

    virtual void log_msg_w(const std::wstring& msg, const LogLevel& level) {
        fprintf(stdout, "%ls", msg.c_str());
    }
};

static Logger logger("main");

void populatePacket(const uint8_t *udp_packet, double feed_in_watts) {
    SpeedwireHeader speedwire_packet(udp_packet, sizeof(udp_packet));
    uint16_t udp_payload_length = (uint16_t)(UDP_PACKET_SIZE - SpeedwireHeader::getPayloadOffset(SpeedwireHeader::sma_emeter_protocol_id) - 2);  // -2 for whatever reason
    speedwire_packet.setDefaultHeader(1, udp_payload_length, SpeedwireHeader::sma_emeter_protocol_id);

    SpeedwireEmeterProtocol emeter_packet(speedwire_packet);
    emeter_packet.setSusyID(0x174);
    emeter_packet.setSerialNumber(1901567274);

    uint64_t epoch_ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    emeter_packet.setTime(epoch_ms);

    void* obis = emeter_packet.getFirstObisElement();

    // compute totals
    double power_factor = 0.99;
    double app_power = feed_in_watts/power_factor;

    // Write totals into packet in the same order that Sunny Home Manager does.
    // We assume that the charger doesn't care that totals don't increase...
    obis = insert(emeter_packet, obis, ObisData::PositiveActivePowerTotal,           0.00); // Power drawn from grid
    obis = insert(emeter_packet, obis, ObisData::PositiveActiveEnergyTotal,   12345678.90);
    obis = insert(emeter_packet, obis, ObisData::NegativeActivePowerTotal,  feed_in_watts); // Grid feed-in
    obis = insert(emeter_packet, obis, ObisData::NegativeActiveEnergyTotal,   12345678.90);
    obis = insert(emeter_packet, obis, ObisData::PositiveReactivePowerTotal,         0.00); // Reactive power grid feed-in
    obis = insert(emeter_packet, obis, ObisData::PositiveReactiveEnergyTotal, 12345678.90);
    obis = insert(emeter_packet, obis, ObisData::NegativeReactivePowerTotal,         0.00); // Reactive power drawn from grid
    obis = insert(emeter_packet, obis, ObisData::NegativeReactiveEnergyTotal, 12345678.90);
    obis = insert(emeter_packet, obis, ObisData::PositiveApparentPowerTotal,         0.00); // Apparent power drawn from grid
    obis = insert(emeter_packet, obis, ObisData::PositiveApparentEnergyTotal, 12345678.90);
    obis = insert(emeter_packet, obis, ObisData::NegativeApparentPowerTotal,    app_power); // Apparent power fed into grid
    obis = insert(emeter_packet, obis, ObisData::NegativeApparentEnergyTotal, 12345678.90);
    obis = insert(emeter_packet, obis, ObisData::PowerFactorTotal,           power_factor);
#if INCLUDE_FREQUENCY_MEASUREMENT
    obis = insert(emeter_packet, obis, ObisData::Frequency,                         50.16);
#endif

    // assume exactly equal distribution across phases

    // line 1
    obis = insert(emeter_packet, obis, ObisData::PositiveActivePowerL1,            0.00);
    obis = insert(emeter_packet, obis, ObisData::PositiveActiveEnergyL1,    12345678.90);
    obis = insert(emeter_packet, obis, ObisData::NegativeActivePowerL1, feed_in_watts/3);
    obis = insert(emeter_packet, obis, ObisData::NegativeActiveEnergyL1,    12345678.90);
    obis = insert(emeter_packet, obis, ObisData::PositiveReactivePowerL1,          0.00);
    obis = insert(emeter_packet, obis, ObisData::PositiveReactiveEnergyL1,  12345678.90);
    obis = insert(emeter_packet, obis, ObisData::NegativeReactivePowerL1,          0.00);
    obis = insert(emeter_packet, obis, ObisData::NegativeReactiveEnergyL1,  12345678.90);
    obis = insert(emeter_packet, obis, ObisData::PositiveApparentPowerL1,          0.00);
    obis = insert(emeter_packet, obis, ObisData::PositiveApparentEnergyL1,  12345678.90);
    obis = insert(emeter_packet, obis, ObisData::NegativeApparentPowerL1,   app_power/3);
    obis = insert(emeter_packet, obis, ObisData::NegativeApparentEnergyL1,  12345678.90);
    obis = insert(emeter_packet, obis, ObisData::CurrentL1,                        0.18);
    obis = insert(emeter_packet, obis, ObisData::VoltageL1,                      231.97);
    obis = insert(emeter_packet, obis, ObisData::PowerFactorL1,            power_factor);

    // line 2
    obis = insert(emeter_packet, obis, ObisData::PositiveActivePowerL2,            0.00);
    obis = insert(emeter_packet, obis, ObisData::PositiveActiveEnergyL2,    12345678.90);
    obis = insert(emeter_packet, obis, ObisData::NegativeActivePowerL2, feed_in_watts/3);
    obis = insert(emeter_packet, obis, ObisData::NegativeActiveEnergyL2,    12345678.90);
    obis = insert(emeter_packet, obis, ObisData::PositiveReactivePowerL2,          0.00);
    obis = insert(emeter_packet, obis, ObisData::PositiveReactiveEnergyL2,  12345678.90);
    obis = insert(emeter_packet, obis, ObisData::NegativeReactivePowerL2,          0.00);
    obis = insert(emeter_packet, obis, ObisData::NegativeReactiveEnergyL2,  12345678.90);
    obis = insert(emeter_packet, obis, ObisData::PositiveApparentPowerL2,          0.00);
    obis = insert(emeter_packet, obis, ObisData::PositiveApparentEnergyL2,  12345678.90);
    obis = insert(emeter_packet, obis, ObisData::NegativeApparentPowerL2,   app_power/3);
    obis = insert(emeter_packet, obis, ObisData::NegativeApparentEnergyL2,  12345678.90);
    obis = insert(emeter_packet, obis, ObisData::CurrentL2,                        1.12);
    obis = insert(emeter_packet, obis, ObisData::VoltageL2,                      230.66);
    obis = insert(emeter_packet, obis, ObisData::PowerFactorL2,            power_factor);

    // line 3
    obis = insert(emeter_packet, obis, ObisData::PositiveActivePowerL3,            0.00);
    obis = insert(emeter_packet, obis, ObisData::PositiveActiveEnergyL3,    12345678.90);
    obis = insert(emeter_packet, obis, ObisData::NegativeActivePowerL3, feed_in_watts/3);
    obis = insert(emeter_packet, obis, ObisData::NegativeActiveEnergyL3,    12345678.90);
    obis = insert(emeter_packet, obis, ObisData::PositiveReactivePowerL3,          0.00);
    obis = insert(emeter_packet, obis, ObisData::PositiveReactiveEnergyL3,  12345678.90);
    obis = insert(emeter_packet, obis, ObisData::NegativeReactivePowerL3,         40.66);
    obis = insert(emeter_packet, obis, ObisData::NegativeReactiveEnergyL3,  12345678.90);
    obis = insert(emeter_packet, obis, ObisData::PositiveApparentPowerL3,          0.00);
    obis = insert(emeter_packet, obis, ObisData::PositiveApparentEnergyL3,  12345678.90);
    obis = insert(emeter_packet, obis, ObisData::NegativeApparentPowerL3,   app_power/3);
    obis = insert(emeter_packet, obis, ObisData::NegativeApparentEnergyL3,  12345678.90);
    obis = insert(emeter_packet, obis, ObisData::CurrentL3,                        0.23);
    obis = insert(emeter_packet, obis, ObisData::VoltageL3,                      230.09);
    obis = insert(emeter_packet, obis, ObisData::PowerFactorL3,            power_factor);

    // software version
    obis = insert(emeter_packet, obis, ObisData::SoftwareVersion, "2.11.5.R");
    obis = insert(emeter_packet, obis, ObisData::EndOfData,       "");

    // check if the packet is fully assembled
    if (((uint8_t*)obis - udp_packet) != UDP_PACKET_SIZE) {
        logger.print(LogLevel::LOG_ERROR, "invalid udp packet size %lu\n", (unsigned long)((uint8_t*)obis - udp_packet));
    }
}

int main(int argc, char** argv) {

    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " ip_addr" << std::endl;
        return 1;
    }
    char *hostname = argv[1];

    // configure logger and logging levels
    ILogListener* log_listener = new LogListener();
    LogLevel log_level = LogLevel::LOG_ERROR | LogLevel::LOG_WARNING;
    log_level = log_level | LogLevel::LOG_INFO_0;
    log_level = log_level | LogLevel::LOG_INFO_1;
    log_level = log_level | LogLevel::LOG_INFO_2;
    log_level = log_level | LogLevel::LOG_INFO_3;
    Logger::setLogListener(log_listener, log_level);

    int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in destination;
    destination.sin_family = AF_INET;
    destination.sin_port = htons(9522);
    destination.sin_addr.s_addr = inet_addr(hostname);

    uint8_t udp_packet[UDP_PACKET_SIZE];
    double wattsMax = 11000;
    double wattsMin = 1400;
    double feedInWatts = 1500;
    double increment = 10;

    while (true) {

        logger.print(LogLevel::LOG_INFO_0, "Sending packet feeding in %5.0f Watts...", feedInWatts);
        populatePacket(udp_packet, feedInWatts);

#if 0
        // for debugging purposes
        SpeedwireHeader protocol(udp_packet, sizeof(udp_packet));
        bool valid = protocol.checkHeader();
        if (valid) {
            uint32_t group      = protocol.getGroup();
            uint16_t length     = protocol.getLength();
            uint16_t protocolID = protocol.getProtocolID();
            int      offset     = protocol.getPayloadOffset();

            if (protocolID == SpeedwireHeader::sma_emeter_protocol_id) {
                SpeedwireEmeterProtocol emeter(protocol);
                uint16_t susyid = emeter.getSusyID();
                uint32_t serial = emeter.getSerialNumber();
                uint32_t timer  = emeter.getTime();

                // extract obis data from the emeter packet and print each obis element
                void* obis = emeter.getFirstObisElement();
                while (obis != NULL) {
                    //emeter.printObisElement(obis, stderr);
                    logger.print(LogLevel::LOG_INFO_2, "%s %s %s", SpeedwireEmeterProtocol::toHeaderString(obis).c_str(), SpeedwireEmeterProtocol::toValueString(obis, true).c_str(), SpeedwireEmeterProtocol::toValueString(obis, false).c_str());
                    obis = emeter.getNextObisElement(obis);
                }
            }
        }
#endif

        ::sendto(sock, udp_packet, UDP_PACKET_SIZE, 0, reinterpret_cast<sockaddr*>(&destination), sizeof(destination));

        feedInWatts += increment;
        if (feedInWatts >= wattsMax || feedInWatts <= wattsMin) {
            increment *= -1;
        }

        std::this_thread::sleep_for(1000ms);
    }

    ::close(sock);

    return 0;
}


// insert obis data into the given emeter packet
void* insert(SpeedwireEmeterProtocol& emeter_packet, void* const obis, const ObisData& obis_data, const double value) {
    // create a new obis data instance from the given obis data template instance
    ObisData temp(obis_data);
    // set its measurement value
    temp.measurementValues.addMeasurement(value, 0);
    // convert it into the obis byte representation
    std::array<uint8_t, 12> byte_array = temp.toByteArray();
    // insert it into the given emeter packet
    return emeter_packet.setObisElement(obis, byte_array.data());
}

// insert obis data into the given emeter packet
void* insert(SpeedwireEmeterProtocol& emeter_packet, void* const obis, const ObisData& obis_data, const std::string& value) {
    // create a new obis data instance from the given obis data template instance
    ObisData temp(obis_data);
    // set its measurement value
    temp.measurementValues.value_string = value;
    // convert it into the obis byte representation
    std::array<uint8_t, 12> byte_array = temp.toByteArray();
    // insert it into the given emeter packet
    return emeter_packet.setObisElement(obis, byte_array.data());
}