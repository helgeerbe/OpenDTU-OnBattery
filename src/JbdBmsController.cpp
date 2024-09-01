#include <Arduino.h>
#include <numeric>
#include "Configuration.h"
#include "HardwareSerial.h"
#include "PinMapping.h"
#include "MessageOutput.h"
#include "JbdBmsDataPoints.h"
#include "JbdBmsController.h"
#include "JbdBmsSerialMessage.h"
#include "SerialPortManager.h"
#include <frozen/map.h>

namespace JbdBms {

bool Controller::init(bool verboseLogging)
{
    _verboseLogging = verboseLogging;

    std::string ifcType = "transceiver";
    if (Interface::Transceiver != getInterface()) { ifcType = "TTL-UART"; }
    MessageOutput.printf("[JBD BMS] Initialize %s interface...\r\n", ifcType.c_str());

    const PinMapping_t& pin = PinMapping.get();
    MessageOutput.printf("[JBD BMS] rx = %d, rxen = %d, tx = %d, txen = %d\r\n",
            pin.battery_rx, pin.battery_rxen, pin.battery_tx, pin.battery_txen);

    if (pin.battery_rx < 0 || pin.battery_tx < 0) {
        MessageOutput.println("[JBD BMS] Invalid RX/TX pin config");
        return false;
    }

#ifdef JBDBMS_DUMMY_SERIAL
    _upSerial = std::make_unique<DummySerial>();
#else
    auto oHwSerialPort = SerialPortManager.allocatePort(_serialPortOwner);
    if (!oHwSerialPort) { return false; }

    _upSerial = std::make_unique<HardwareSerial>(*oHwSerialPort);
#endif

    _upSerial->end(); // make sure the UART will be re-initialized
    _upSerial->begin(9600, SERIAL_8N1, pin.battery_rx, pin.battery_tx);
    _upSerial->flush();

    if (Interface::Transceiver != getInterface()) { return true; }

    _rxEnablePin = pin.battery_rxen;
    _txEnablePin = pin.battery_txen;

    if (_rxEnablePin < 0 || _txEnablePin < 0) {
        MessageOutput.println("[JBD BMS] Invalid transceiver pin config");
        return false;
    }

    pinMode(_rxEnablePin, OUTPUT);
    pinMode(_txEnablePin, OUTPUT);

    return true;
}

void Controller::deinit()
{
    _upSerial->end();

    if (_rxEnablePin > 0) { pinMode(_rxEnablePin, INPUT); }
    if (_txEnablePin > 0) { pinMode(_txEnablePin, INPUT); }

    SerialPortManager.freePort(_serialPortOwner);
}

Controller::Interface Controller::getInterface() const
{
    CONFIG_T& config = Configuration.get();
    if (0x00 == config.Battery.JkBmsInterface) { return Interface::Uart; }
    if (0x01 == config.Battery.JkBmsInterface) { return Interface::Transceiver; }
    return Interface::Invalid;
}

frozen::string const& Controller::getStatusText(Controller::Status status)
{
    static constexpr frozen::string missing = "programmer error: missing status text";

    static constexpr frozen::map<Status, frozen::string, 6> texts = {
        { Status::Timeout, "timeout wating for response from BMS" },
        { Status::WaitingForPollInterval, "waiting for poll interval to elapse" },
        { Status::HwSerialNotAvailableForWrite, "UART is not available for writing" },
        { Status::BusyReading, "busy waiting for or reading a message from the BMS" },
        { Status::RequestSent, "request for data sent" },
        { Status::FrameCompleted, "a whole frame was received" }
    };

    auto iter = texts.find(status);
    if (iter == texts.end()) { return missing; }

    return iter->second;
}

void Controller::announceStatus(Controller::Status status)
{
    if (_lastStatus == status && millis() < _lastStatusPrinted + 10 * 1000) { return; }

    MessageOutput.printf("[%11.3f] JBD BMS: %s\r\n",
        static_cast<double>(millis())/1000, getStatusText(status).data());

    _lastStatus = status;
    _lastStatusPrinted = millis();
}

void Controller::sendRequest(uint8_t pollInterval)
{
    if (ReadState::Idle != _readState) {
        return announceStatus(Status::BusyReading);
    }

    if ((millis() - _lastRequest) < pollInterval * 1000) {
        return announceStatus(Status::WaitingForPollInterval);
    }

    if (!_upSerial->availableForWrite()) {
        return announceStatus(Status::HwSerialNotAvailableForWrite);
    }

    SerialCommand::Command cmd;
    switch (SerialCommand::getLastCommand()) {
        case SerialCommand::Command::Init: // read only once
            cmd = SerialCommand::Command::ReadHardwareVersionNumber;
            break;
        case SerialCommand::Command::ReadBasicInformation:
            cmd = SerialCommand::Command::ReadCellVoltages;
            break;
        case SerialCommand::Command::ReadCellVoltages:
            cmd = SerialCommand::Command::ReadBasicInformation;
            break;
        default:
            cmd = SerialCommand::Command::ReadBasicInformation;
            break;
    }

    SerialCommand readCmd(SerialCommand::Status::Read, cmd);

    if (Interface::Transceiver == getInterface()) {
        digitalWrite(_rxEnablePin, HIGH); // disable reception (of our own data)
        digitalWrite(_txEnablePin, HIGH); // enable transmission
    }

    //readCmd.printMessage(); // TODO: Only for testing. Comment out
    _upSerial->write(readCmd.data(), readCmd.size());

    if (Interface::Transceiver == getInterface()) {
        _upSerial->flush();
        digitalWrite(_rxEnablePin, LOW); // enable reception
        digitalWrite(_txEnablePin, LOW); // disable transmission (free the bus)
    }

    _lastRequest = millis();

    setReadState(ReadState::WaitingForFrameStart);
    return announceStatus(Status::RequestSent);
}

void Controller::loop()
{
    CONFIG_T& config = Configuration.get();
    uint8_t pollInterval = config.Battery.JkBmsPollingInterval;

    while (_upSerial->available()) {
        rxData(_upSerial->read());
    }

    sendRequest(pollInterval);

    if (millis() > _lastRequest + 2 * pollInterval * 1000 + 250) {
        reset();
        return announceStatus(Status::Timeout);
    }
}

void Controller::rxData(uint8_t inbyte)
{
    _buffer.push_back(inbyte);

    /* Print inbyte
    if (inbyte == 0xDD) {MessageOutput.printf("[%11.3f] JBD BMS: Byte received: %02x", static_cast<double>(millis())/1000, inbyte);}
    MessageOutput.printf(" %02x", inbyte);
    if (inbyte == 0x77) {MessageOutput.println();}
    */

    switch(_readState) {
        case ReadState::Idle: // unsolicited message from BMS
        case ReadState::WaitingForFrameStart:
            if (inbyte == SerialMessage::startMarker) {
                //MessageOutput.printf("[JBD BMS] Byte received: startMarker received:  %02x.", inbyte);
                //MessageOutput.println();
                return setReadState(ReadState::FrameStartReceived);
            }
            break;
        case ReadState::FrameStartReceived:
            //MessageOutput.printf("[JBD BMS] Byte received: FrameStartReceived:  %02x.", inbyte);
            //MessageOutput.println();
            return setReadState(ReadState::StateReceived);
            break;
        case ReadState::StateReceived:
            //MessageOutput.printf("[JBD BMS] Byte received: StateReceived:  %02x.", inbyte);
            //MessageOutput.println();
            return setReadState(ReadState::CommandCodeReceived);
            break;
        case ReadState::CommandCodeReceived:
            _dataLength = inbyte;
            if (_dataLength == 0) {
                //MessageOutput.printf("[JBD BMS] Byte received: CommandCodeReceived with dataLength == 0:  %02x.", inbyte);
                //MessageOutput.println();
                return setReadState(ReadState::DataContentReceived);
            }
            else {
                //MessageOutput.printf("[JBD BMS] Byte received: CommandCodeReceived with dataLength > 0:  %02x.", inbyte);
                //MessageOutput.println();
                return setReadState(ReadState::ReadingDataContent);
            }
            break;
        case ReadState::ReadingDataContent:
            //MessageOutput.printf("[JBD BMS] Byte received: ReadingDataContent:%02x, remaining datalength %i.", inbyte, _dataLength);
            //MessageOutput.println();
            _dataLength--;
            if (_dataLength == 0) {
                return setReadState(ReadState::DataContentReceived);
            }
            return setReadState(ReadState::ReadingDataContent);
            break;
        case ReadState::DataContentReceived:
            //MessageOutput.printf("[JBD BMS] Byte received: DataContentReceived. First Checksum byte:  %02x.", inbyte);
            //MessageOutput.println();
            return setReadState(ReadState::ReadingCheckSum);
            break;
        case ReadState::ReadingCheckSum:
            //MessageOutput.printf("[JBD BMS] Byte received: DataContentReceived. Second Checksum byte:  %02x.", inbyte);
            //MessageOutput.println();
            return setReadState(ReadState::CheckSumReceived);
            break;
        case ReadState::CheckSumReceived:
            //MessageOutput.printf("[JBD BMS] Byte received: CheckSumReceived. Next byte:  %02x\n.", inbyte);
            //MessageOutput.println();
            if (inbyte == SerialMessage::endMarker) {
                //MessageOutput.printf("[JBD BMS] Byte received: endMarker received.");
                //MessageOutput.println();

                /*
                // Test with Dummy Messages
                if (_buffer[1] == 0x03) {

                    // test AlarmBitMask with FFFF
                    _buffer[20] = uint8_t (0xFF);
                    _buffer[21] = uint8_t (0xFF);
                    // _buffer = {0xdd, 0x03, 0x00, 0x26, 0x0a, 0x4a, 0x00, 0x00, 0x20, 0xc5, 0x4e, 0x20, 0x00, 0x00, 0x2f, 0x9d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x25, 0x2a, 0x03, 0x08, 0x03, 0x0b, 0x93, 0x0b, 0x88, 0x0b, 0x88, 0x00, 0x00, 0x00, 0x4e, 0x20, 0x20, 0xc5, 0x00, 0x00, 0xf9, 0xf3, 0x77};
                }
                uint16_t checksumUpdate = (~std::accumulate(_buffer.cbegin()+2, _buffer.cend()-3, 0) + 0x01);
                *(_buffer.end()-3) = uint8_t (checksumUpdate >> 8);
                *(_buffer.end()-2) = uint8_t (checksumUpdate);
                */

                return frameComplete();
            }
            else {
                MessageOutput.printf("[JBD BMS] Invalid Frame: end marker not found.");
                MessageOutput.println();
                return reset();
            }
            break;
    }
    reset();
}

void Controller::reset()
{
    _buffer.clear();
    return setReadState(ReadState::Idle);
}

void Controller::frameComplete()
{
    announceStatus(Status::FrameCompleted);

    if (_verboseLogging) {
        double ts = static_cast<double>(millis())/1000;
        MessageOutput.printf("[%11.3f] JBD BMS: raw data (%d Bytes):",
            ts, _buffer.size());
        for (size_t ctr = 0; ctr < _buffer.size(); ++ctr) {
            if (ctr % 16 == 0) {
                MessageOutput.printf("\r\n[%11.3f] JBD BMS:", ts);
            }
            MessageOutput.printf(" %02x", _buffer[ctr]);
        }
        MessageOutput.println();
    }

    auto pResponse = std::make_unique<SerialResponse>(std::move(_buffer));
    if (pResponse->isValid()) {
        processDataPoints(pResponse->getDataPoints());
    } // if invalid, error message has been produced by SerialResponse c'tor

    reset();
}

void Controller::processDataPoints(DataPointContainer const& dataPoints)
{
    _stats->updateFrom(dataPoints);

    if (!_verboseLogging) { return; }

    auto iter = dataPoints.cbegin();
    while ( iter != dataPoints.cend() ) {
        MessageOutput.printf("[%11.3f] JBD BMS: %s: %s%s\r\n",
            static_cast<double>(iter->second.getTimestamp())/1000,
            iter->second.getLabelText().c_str(),
            iter->second.getValueText().c_str(),
            iter->second.getUnitText().c_str());
        ++iter;
    }
}

} /* namespace JbdBms */
