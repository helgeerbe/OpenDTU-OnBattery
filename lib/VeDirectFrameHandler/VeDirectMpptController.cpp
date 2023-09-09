#include <Arduino.h>
#include "VeDirectMpptController.h"

#ifndef VICTRON_PIN_TX
#define VICTRON_PIN_TX 26      // HardwareSerial TX Pin
#endif

#ifndef VICTRON_PIN_RX
#define VICTRON_PIN_RX 25      // HardwareSerial RX Pin
#endif

VeDirectMpptController VeDirectMppt;

VeDirectMpptController::VeDirectMpptController() 
{
}

void VeDirectMpptController::init(int8_t rx, int8_t tx, Print* msgOut, bool verboseLogging)
{
    _vedirectSerial = new HardwareSerial(1);
	_vedirectSerial->begin(19200, SERIAL_8N1, rx, tx);
    _vedirectSerial->flush();
	VeDirectFrameHandler::init(msgOut, verboseLogging);
	_msgOut->println("Finished init MPPTCOntroller");
}

bool VeDirectMpptController::isDataValid() {
	return VeDirectFrameHandler::isDataValid(veFrame);
}



void VeDirectMpptController::textRxEvent(char * name, char * value) {
    //_msgOut->printf("Received Text Event %s: Value: %s\r\n", name, value );
	VeDirectFrameHandler::textRxEvent(name, value, _tmpFrame);
	if (strcmp(name, "LOAD") == 0) {
		if (strcmp(value, "ON") == 0)
			_tmpFrame.LOAD = true;
		else	
			_tmpFrame.LOAD = false;
	}
	else if (strcmp(name, "CS") == 0) {
		_tmpFrame.CS = atoi(value);
	}
	else if (strcmp(name, "ERR") == 0) {
		_tmpFrame.ERR = atoi(value);
	}
	else if (strcmp(name, "OR") == 0) {
		_tmpFrame.OR = strtol(value, nullptr, 0);
	}
	else if (strcmp(name, "MPPT") == 0) {
		_tmpFrame.MPPT = atoi(value);
	}
	else if (strcmp(name, "HSDS") == 0) {
		_tmpFrame.HSDS = atoi(value);
	}
	else if (strcmp(name, "VPV") == 0) {
		_tmpFrame.VPV = round(atof(value) / 10.0) / 100.0;
	}
	else if (strcmp(name, "PPV") == 0) {
		_tmpFrame.PPV = atoi(value);
	}
	else if (strcmp(name, "H19") == 0) {
		_tmpFrame.H19 = atof(value) / 100.0;
	}
	else if (strcmp(name, "H20") == 0) {
		_tmpFrame.H20 = atof(value) / 100.0;
	}
	else if (strcmp(name, "H21") == 0) {
		_tmpFrame.H21 = atoi(value);
	}
	else if (strcmp(name, "H22") == 0) {
		_tmpFrame.H22 = atof(value) / 100.0;
	}
	else if (strcmp(name, "H23") == 0) {
		_tmpFrame.H23 = atoi(value);
	}
}

/*
 *  frameEndEvent
 *  This function is called at the end of the received frame.  If the checksum is valid, the temp buffer is read line by line.
 *  If the name exists in the public buffer, the new value is copied to the public buffer.	If not, a new name/value entry
 *  is created in the public buffer.
 */
void VeDirectMpptController::frameEndEvent(bool valid) {
	if ( valid ) {
		_tmpFrame.P = _tmpFrame.V * _tmpFrame.I;

		_tmpFrame.IPV = 0;
		if ( _tmpFrame.VPV > 0) {
			_tmpFrame.IPV = _tmpFrame.PPV / _tmpFrame.VPV;
		}

		_tmpFrame.E = 0;
		if ( _tmpFrame.PPV > 0) {
			_efficiency.addNumber(static_cast<double>(_tmpFrame.P * 100) / _tmpFrame.PPV);
			_tmpFrame.E = _efficiency.getAverage();
		}

		veFrame = _tmpFrame;
		_lastUpdate = millis();
	}
	_tmpFrame = {};
}


/*
 * getCsAsString
 * This function returns the state of operations (CS) as readable text.
 */
String VeDirectMpptController::getCsAsString(uint8_t cs)
{
	String strCS ="";

	switch(cs) {
		case 0:
			strCS =  "OFF";
			break;
		case 2:
			strCS =  "Fault";
			break;
		case 3:
			strCS =  "Bulk";
			break;
		case 4:
			strCS =  "Absorbtion";
			break;
		case 5:
			strCS =  "Float";
			break;
		case 7:
			strCS =  "Equalize (manual)";
			break;
		case 245:
			strCS =  "Starting-up";
			break;
		case 247:
			strCS =  "Auto equalize / Recondition";
			break;
		case 252:
			strCS =  "External Control";
			break;
		default:
			strCS = cs;
	}
	return strCS;
}
/*
 * getMpptAsString
 * This function returns the state of MPPT (MPPT) as readable text.
 */
String VeDirectMpptController::getMpptAsString(uint8_t mppt)
{
	String strMPPT ="";

	switch(mppt) {
		case 0:
			strMPPT =  "OFF";
			break;
		case 1:
			strMPPT =  "Voltage or current limited";
			break;
		case 2:
			strMPPT =  "MPP Tracker active";
			break;
		default:
			strMPPT = mppt;
	}
	return strMPPT;
}