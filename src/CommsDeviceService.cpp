#include <chrono>
#include <dccomms/CommsDeviceService.h>
#include <dccomms/CommsException.h>
#include <dccomms/Utils.h>
#include <errno.h>
#include <fcntl.h> /* Defines O_* constants */
#include <iostream>
#include <mqueue.h>
#include <sys/stat.h> /* Defines mode constants */

namespace dccomms {

#define TX_MQ 0
#define RX_MQ 1
#define RTS_MQ 2
#define CTS_MQ 3

#define MSG_OVERHEAD_SIZE 1

static void ThrowPhyLayerException(std::string msg) {
  throw CommsException("PHYLAYER EXCEPTION: " + msg,
                       COMMS_EXCEPTION_PHYLAYER_ERROR);
}

std::string GetMQErrorMsg(int e) {
  switch (e) {
  case EACCES:
    return "The queue exists, but the caller does not have permission to open "
           "it in the specified mode / Name Contained more than one slash";
  case EEXIST:
    return "Both O_CREAT and O_EXCL were specified in oflag, but a queue with "
           "this name already exists";
  case EINVAL:
    return "O_CREAT  was  specified in oflag, and attr was not NULL, but attr->mq_maxmsg or attr->mq_msqsize was invalid.  Both of these\
            fields must be greater than zero.  In a process that is  unprivileged  (does  not  have  the  CAP_SYS_RESOURCE  capability),\
            attr->mq_maxmsg must be less than or equal to the msg_max limit, and attr->mq_msgsize must be less than or equal to the msg‐\
            size_max limit.  In addition, even in a privileged process, attr->mq_maxmsg cannot exceed the HARD_MAX limit.  (See mq_over‐\
                                                                                                                            view(7) for details of these limits.)";
  case EMFILE:
    return "The process already has the maximum number of files and message "
           "queues open.";
  case ENAMETOOLONG:
    return "name was too long.";
  case ENFILE:
    return "The system limit on the total number of open files and message "
           "queues has been reached.";
  case ENOENT:
    return "The O_CREAT flag was not specified in oflag, and no queue with "
           "this name exists.";
  case ENOMEM:
    return "Insufficient memory.";
  case ENOSPC:
    return "Insufficient space for the creation of a new message queue.  This probably occurred because the queues_max limit was encoun‐\
                tered; see mq_overview(7).";
  default:
    return "Unknown Error";
  }
}
CommsDeviceService::CommsDeviceService(PacketBuilderPtr pb, int _type,
                                       int maxframesize)
    : service(this), rxmsg(pb), txmsg(pb), replymsg(pb) {
  comattr.mq_maxmsg = 10;
  comattr.mq_msgsize = maxframesize + MSG_OVERHEAD_SIZE;
  comperm = 0777;
  qprefix = "";
  type = _type;
  maxQueueSize = 40000;
  rxQueueSize = 0;
  SetLogName("CommsDeviceService");
  service.SetWork(&CommsDeviceService::Work);
  SetLogLevel(cpplogging::off);
  _started = false;
  _timeout = 0;
}

void CommsDeviceService::SetCommsDeviceId(std::string m) {
  _namespace = m;
  SetQueuePrefix(_namespace);
}

void CommsDeviceService::SetMaxQueueSize(uint32_t size) { maxQueueSize = size; }

uint32_t CommsDeviceService::GetMaxQueueSize() { return maxQueueSize; }

void CommsDeviceService::Init(int _type, struct mq_attr attr, int perm) {
  type = _type;
  txmqname = "/" + qprefix;
  rxmqname = "/" + qprefix;
  switch (type) {
  case IPHY_TYPE_DLINK:
    txmqname += "_tx_dlnk_phy";
    rxmqname += "_rx_dlnk_phy";
    _SetPhyLayerState(PhyState::BUSY);
    break;
  case IPHY_TYPE_PHY:
    rxmqname += "_tx_dlnk_phy";
    txmqname += "_rx_dlnk_phy";
    _SetPhyLayerState(PhyState::BUSY);
    break;
  default:
    ThrowPhyLayerException("Tipo de interfaz con la capa fisica incorrecto");
  }

  txattr = attr;
  rxattr = attr;

  int openops = O_CREAT;

  mode_t omask;
  omask = umask(
      0); // http://stackoverflow.com/questions/22780277/mq-open-eacces-permission-denied
  txmqid = mq_open(txmqname.c_str(), openops | O_WRONLY, perm, &txattr);
  std::string emsg;
  if (txmqid == -1) {
    emsg = GetMQErrorMsg(errno);
    ThrowPhyLayerException(
        std::string("Error(") + std::to_string(errno) +
        std::string(
            "): Error al abrir/crear la cola para envio de mensajes: ") +
        emsg);
  }
  rxmqid = mq_open(rxmqname.c_str(), openops | O_RDONLY, perm, &rxattr);
  if (rxmqid == -1) {
    emsg = GetMQErrorMsg(errno);
    ThrowPhyLayerException(
        std::string("Error(") + std::to_string(errno) +
        std::string(
            "): Error al abrir/crear la cola para recepcion de mensajes") +
        emsg);
  }
  umask(omask);

  SetNonblockFlag(false, TX_MQ);
  SetNonblockFlag(false, RX_MQ);

  ClearInputQueue();

#ifdef DEBUG
  std::cerr << "TXMQ:" << std::endl;
  ShowMQAttr(std::cerr, TX_MQ);
  std::cerr << "RXMQ:" << std::endl;
  ShowMQAttr(std::cerr, RX_MQ);
#endif

  maxmsgsize = GetMaxMsgSize(RX_MQ);
  rxmsg.Init(maxmsgsize);
  txmsg.Init(maxmsgsize);
  replymsg.Init(maxmsgsize);
}

CommsDeviceService::~CommsDeviceService() {
  service.Stop();
  mq_close(rxmqid);
  mq_close(txmqid);
}

void CommsDeviceService::UpdateMQAttr() {
  if (mq_getattr(txmqid, &txattr) == -1)
    ThrowPhyLayerException(
        std::string("Error(") + std::to_string(errno) +
        std::string("): Error interno: no ha sido posible obtener los "
                    "atributos de la cola de mensajes tx"));
  if (mq_getattr(rxmqid, &rxattr) == -1)
    ThrowPhyLayerException(
        std::string("Error(") + std::to_string(errno) +
        std::string("): Error interno: no ha sido posible obtener los "
                    "atributos de la cola de mensajes rx"));
}

struct mq_attr *CommsDeviceService::GetMQAttr(int mq) {
  UpdateMQAttr();

  struct mq_attr *attr;

  switch (mq) {
  case TX_MQ:
    attr = &txattr;
    break;
  case RX_MQ:
    attr = &rxattr;
    break;
  default:
    ThrowPhyLayerException("Error interno: la cola de mensajes no existe");
  }
  return attr;
}

mqd_t CommsDeviceService::GetMQId(int mq) {
  switch (mq) {
  case TX_MQ:
    return txmqid;
  case RX_MQ:
    return rxmqid;
  default:
    ThrowPhyLayerException("Error interno: la cola de mensajes no existe");
  }
  return 0; // nunca llegara aqui
}

void CommsDeviceService::ShowMQAttr(std::ostream &o, int mq) {
  struct mq_attr *attr = GetMQAttr(mq);

  o << " - Maximum # of messages on queue:\t" << attr->mq_maxmsg << std::endl;
  o << " - Maximum message size:\t" << attr->mq_msgsize << std::endl;
  o << " - # of messages currently on queue:\t" << attr->mq_curmsgs
    << std::endl;
  o << " - O_NONBLOCK:\t"
    << (attr->mq_flags & O_NONBLOCK ? "activado" : "desactivado") << std::endl;
}

long CommsDeviceService::GetMaxMsgOnQueue(int mq) {
  struct mq_attr *attr = GetMQAttr(mq);
  return attr->mq_maxmsg;
}

long CommsDeviceService::GetMaxMsgSize(int mq) {
  struct mq_attr *attr = GetMQAttr(mq);
  return attr->mq_msgsize;
}

long CommsDeviceService::GetNumMsgOnQueue(int mq) {
  struct mq_attr *attr = GetMQAttr(mq);
  return attr->mq_curmsgs;
}

bool CommsDeviceService::GetNonblockFlag(int mq) {
  struct mq_attr *attr = GetMQAttr(mq);
  return attr->mq_flags & O_NONBLOCK;
}

void CommsDeviceService::SetNonblockFlag(bool v, int mq) {
  struct mq_attr *attr = GetMQAttr(mq);
  mqd_t id = GetMQId(mq);
  if (v)
    attr->mq_flags |= O_NONBLOCK;
  else
    attr->mq_flags &= ~O_NONBLOCK;

  if (mq_setattr(id, attr, NULL) == -1) {
    ThrowPhyLayerException(
        std::string("Error(") + std::to_string(errno) +
        std::string("): Error interno: no ha sido posible establecer los "
                    "atributos de la cola de mensajes"));
  }
}

void CommsDeviceService::ClearInputQueue() {
  bool curflag = GetNonblockFlag(RX_MQ);
  SetNonblockFlag(true, RX_MQ);

  char _auxb[4000];
  int n = 0;
  while (n >= 0) {
    n = mq_receive(rxmqid, _auxb, 4000, NULL);
  }
  SetNonblockFlag(curflag, RX_MQ);
}

void CommsDeviceService::WritePacket(const PacketPtr &dlf) {
  txmsg.BuildPacketMsg(dlf);
  if (type == IPHY_TYPE_DLINK) {
    // the frame is directed from de dlink layer to the phy layer, so we set the
    // phy layer state to BUSY
    Log->debug("Seteando manualmente el estado de 'OCUPADO'");
    _SetPhyLayerState(BUSY);
  }
  SendMsg(txmsg);
}

void CommsDeviceService::SendMsg(const ServiceMessage &msg) {
  if (mq_send(txmqid, (char *)msg.GetBuffer(), msg.GetSize(), 0) == -1) {
    ThrowPhyLayerException(
        std::string("Error(") + std::to_string(errno) +
        std::string("): Error interno: no se ha podido enviar el mensaje"));
  }
}

bool CommsDeviceService::ReceiveMsg(ServiceMessage &msg) {
  struct timespec tm;
  clock_gettime(CLOCK_REALTIME, &tm);
  tm.tv_sec += 2;
  if (mq_timedreceive(rxmqid, (char *)msg.GetBuffer(), msg.GetMaxSize(), NULL,
                      &tm) == -1) {
    if (errno != ETIMEDOUT)
      ThrowPhyLayerException(
          std::string("Error(") + std::to_string(errno) +
          std::string(
              "): Error interno: fallo al intentar recibir algun mensaje"));
    return false;
  }
  return true;
}

void CommsDeviceService::WaitForFramesFromRxFifo() {
  std::unique_lock<std::mutex> lock(rxfifo_mutex);
  while (rxfifo.empty()) {
    rxfifo_cond.wait(lock);
  }
}

bool CommsDeviceService::WaitForFramesFromRxFifo(unsigned int timeout) {
  std::unique_lock<std::mutex> lock(rxfifo_mutex);
  while (rxfifo.empty()) {
    auto status =
        rxfifo_cond.wait_for(lock, std::chrono::milliseconds(timeout));
    if (status == std::cv_status::timeout) {
      return false;
    }
  }
  return true;
}

void CommsDeviceService::WaitForDeviceReadyToTransmit() {
  std::unique_lock<std::mutex> lock(phyState_mutex);
  while (phyState == PhyState::BUSY) {
    phyState_cond.wait(lock);
  }
}

PacketPtr CommsDeviceService::GetNextPacket() {
  std::unique_lock<std::mutex> lock(rxfifo_mutex);
  while (rxfifo.empty()) {
    if (_timeout <= 0)
      rxfifo_cond.wait(lock);
    else {
      auto status = rxfifo_cond.wait_for(lock, std::chrono::milliseconds(_timeout));
      if (status == std::cv_status::timeout) {
        throw CommsException("Timeout waiting for the next packet",
                             COMMS_EXCEPTION_TIMEOUT);
      }
    }
  }
  PacketPtr dlf = rxfifo.front();
  auto size = dlf->GetPacketSize();
  rxfifo.pop();
  rxQueueSize -= size;
  // unique_lock destructor unlocks automatically rxfifo_mutex
  return dlf;
}

void CommsDeviceService::PushNewFrame(PacketPtr dlf) {
  rxfifo_mutex.lock();
  auto size = dlf->GetPacketSize();
  if (size + rxQueueSize <= maxQueueSize) {
    rxQueueSize += size;
    rxfifo.push(dlf);
  } else
    Log->warn("Rx queue full. Packet dropped");

  rxfifo_cond.notify_one();
  rxfifo_mutex.unlock();
}

void CommsDeviceService::ReadPacket(const PacketPtr &dlf) {
  PacketPtr npkt = GetNextPacket();
  dlf->CopyFromRawBuffer(npkt->GetBuffer());
}

void CommsDeviceService::_SetPhyLayerState(const PhyState &state) {
  phyState_mutex.lock();

  phyState = state;
  if (state == PhyState::READY) {
    phyState_cond.notify_one();
  }

  phyState_mutex.unlock();
}

CommsDeviceService::PhyState CommsDeviceService::_GetPhyLayerState() {
  PhyState state;
  phyState_mutex.lock();

  state = phyState;

  phyState_mutex.unlock();

  return state;
}

void CommsDeviceService::SendPhyLayerState(const PhyState &state) {
  replymsg.BuildCmdStateMsg(state);
  SendMsg(replymsg);
  switch (state) {
  case PhyState::BUSY:
    Log->debug("Enviado estado OCUPADO");
    break;
  case PhyState::READY:
    Log->debug("Enviado estado LISTO");
    break;
  default:
    Log->critical("ERROR GRAVE: ENVIADO ESTADO IMPOSIBLE!!");
  }
}

bool CommsDeviceService::BusyTransmitting() {
  if (type != IPHY_TYPE_DLINK)
    ThrowPhyLayerException("Method call not allowed");
  return _GetPhyLayerState() == PhyState::BUSY;
}

void CommsDeviceService::ReqPhyLayerState() {
  if (type != IPHY_TYPE_DLINK)
    ThrowPhyLayerException("Method call not allowed");
  txmsg.BuildReqStateMsg();
  SendMsg(txmsg);
}

void CommsDeviceService::SetPhyLayerState(const PhyState &state) {
  if (type != IPHY_TYPE_PHY)
    ThrowPhyLayerException("Method call not allowed");
  _SetPhyLayerState(state);
  SendPhyLayerState();
}

unsigned int CommsDeviceService::GetRxFifoSize() {
  unsigned int size;
  rxfifo_mutex.lock();
  size = rxQueueSize;
  rxfifo_mutex.unlock();
  return size;
}

void CommsDeviceService::Start() {

  Init(type, comattr, comperm);
  service.Start();
  if (type == IPHY_TYPE_PHY) {
    // Enviamos el estado actual a la capa de arriba
    SendPhyLayerState();
  } else {
    ReqPhyLayerState();
  }
  _started = true;
}

void CommsDeviceService::Stop() {
  service.Stop();
  _started = false;
}

CommsDeviceService::ServiceMessage::ServiceMessage(PacketBuilderPtr pb) {
  _pktBuilder = pb;
  buffer = NULL;
}

CommsDeviceService::ServiceMessage::~ServiceMessage() { free(buffer); }

void CommsDeviceService::ServiceMessage::Init(int maxs) {
  maxPayloadSize = maxs - MSG_OVERHEAD_SIZE;
  maxSize = maxs;
  buffer = malloc(maxs);
  type = (uint8_t *)buffer;
  *type = (uint8_t)NOTBUILT;
  payload = type + 1;
  size = 0;
}

void CommsDeviceService::ServiceMessage::BuildPacketMsg(const PacketPtr &dlf) {
  int frsize = dlf->GetPacketSize();
  if (frsize <= maxPayloadSize) {

    memcpy(payload, dlf->GetBuffer(), frsize);
    *type = (uint8_t)MsgType::FRAME;
    size = frsize + MSG_OVERHEAD_SIZE;
  } else {
    *type = (uint8_t)MsgType::NOTBUILT;
    ThrowPhyLayerException(
        "Error interno: la trama no cabe en el formato de mensaje de la cola");
  }
}

void CommsDeviceService::ServiceMessage::BuildReqStateMsg() {
  *type = (uint8_t)MsgType::REQ_STATE;
  size = MSG_OVERHEAD_SIZE;
}

void CommsDeviceService::ServiceMessage::BuildCmdStateMsg(
    const PhyState &state) {
  *type = (uint8_t)MsgType::CMD_STATE;
  *payload = (uint8_t)state;
  size = MSG_OVERHEAD_SIZE + 1;
}

PacketPtr CommsDeviceService::ServiceMessage::GetPacket() const {
  return _pktBuilder->CreateFromBuffer(payload);
}

void CommsDeviceService::SaveFrameFromMsg(const ServiceMessage &msg) {
  PushNewFrame(msg.GetPacket());
}

void CommsDeviceService::SavePhyStateFromMsg(const ServiceMessage &msg) {
  _SetPhyLayerState(msg.GetPhyState());
}

void CommsDeviceService::SendPhyLayerState() {
  SendPhyLayerState(_GetPhyLayerState());
}

void CommsDeviceService::Work() {
  Log->debug("Esperando mensaje...");
  if (ReceiveMsg(rxmsg)) {
    switch (rxmsg.GetMsgType()) {
    case ServiceMessage::FRAME:
      if (type == IPHY_TYPE_DLINK)
        Log->debug("Received frame from the physical layer");
      else
        Log->debug("Received frame from the D-Link layer");

      SaveFrameFromMsg(rxmsg);
      break;
    case ServiceMessage::CMD_STATE:
      Log->debug("Recibido mensaje de estado de la capa fisica");
      SavePhyStateFromMsg(rxmsg);
      break;
    case ServiceMessage::REQ_STATE:
      Log->debug("Recibida peticion de estado de la capa fisica");
      SendPhyLayerState();
      break;
    default:
      break;
    }
  }
}

int CommsDeviceService::Available() {
  //TODO: return the payload bytes in the rx fifo instead
  return GetRxFifoSize();
}

bool CommsDeviceService::IsOpen() { return _started; }

int CommsDeviceService::Read(void *, uint32_t, unsigned long msTimeout) {
  throw CommsException("int CommsDeviceService::Read() Not implemented",
                       COMMS_EXCEPTION_NOTIMPLEMENTED);
}
int CommsDeviceService::Write(const void *, uint32_t, uint32_t msTimeout) {
  throw CommsException("int CommsDeviceService::Write() Not implemented",
                       COMMS_EXCEPTION_NOTIMPLEMENTED);
}

void CommsDeviceService::FlushInput() {
  throw CommsException("void CommsDeviceService::FlushInput() Not implemented",
                       COMMS_EXCEPTION_NOTIMPLEMENTED);
}
void CommsDeviceService::FlushOutput() {
  throw CommsException("void CommsDeviceService::FlushOutput() Not implemented",
                       COMMS_EXCEPTION_NOTIMPLEMENTED);
}
void CommsDeviceService::FlushIO() {
  throw CommsException("void CommsDeviceService::FlushIO() Not implemented",
                       COMMS_EXCEPTION_NOTIMPLEMENTED);
}
} /* namespace radiotransmission */
