#include <dccomms/CommsDevice.h>
#include <dccomms/CommsException.h>

namespace dccomms {

CommsDevice::CommsDevice() {}

unsigned long CommsDevice::GetTimeout() { return _timeout; }

void CommsDevice::SetTimeout(unsigned long ms) { _timeout = ms; }

bool CommsDevice::BusyTransmitting() { return false; }

bool CommsDevice::Open() {
  throw CommsException("Operator not implemented",
                       COMMS_EXCEPTION_NOTIMPLEMENTED);
}

void CommsDevice::Close() {
  throw CommsException("Operator not implemented",
                       COMMS_EXCEPTION_NOTIMPLEMENTED);
}
}
