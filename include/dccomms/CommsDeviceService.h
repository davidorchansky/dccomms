/*
 * SdrRadioPhyLayer.h
 *
 *  Created on: 16 set. 2016
 *      Author: diego
 */

#ifndef DCCOMMS_COMMSDEVICESERVICE_H_
#define DCCOMMS_COMMSDEVICESERVICE_H_

#include <chrono>
#include <condition_variable>
#include <dccomms/CommsDeviceService.h>
#include <dccomms/CommsException.h>
#include <dccomms/ICommsLink.h>
#include <dccomms/Packet.h>
#include <dccomms/Utils.h>
#include <errno.h>
#include <fcntl.h> /* Defines O_* constants */
#include <iostream>
#include <memory>
#include <mqueue.h>
#include <mutex>
#include <queue>
#include <sys/stat.h> /* Defines mode constants */
#include <thread>

namespace dccomms {

#define IPHY_TYPE_DLINK 0
#define IPHY_TYPE_PHY 1

template <typename PacketType> class CommsDeviceService;

template <typename PacketType>
using CommsDeviceServicePtr = std::shared_ptr<CommsDeviceService<PacketType>>;

template <typename PacketType> class CommsDeviceService : public ICommsLink {
public:
  enum PhyState { BUSY = 0, READY };

  static CommsDeviceServicePtr<PacketType>
  BuildCommsDeviceService(int iphytype = IPHY_TYPE_DLINK,
                          int maxframesize = 7000) {
    return CommsDeviceServicePtr<PacketType>(
        new CommsDeviceService(iphytype, maxframesize));
  }

  CommsDeviceService(int iphytype = IPHY_TYPE_DLINK, int maxframesize = 7000);
  virtual ~CommsDeviceService();

  virtual ICommsLink &operator<<(PacketPtr);
  virtual ICommsLink &operator>>(PacketPtr);

  virtual void Start();
  virtual void Stop();

  virtual unsigned int GetRxFifoSize();

  // Methods only for type IPHY_TYPE_DLINK
  virtual bool BusyTransmitting();

  // Methods only for type IPHY_TYPE_PHY
  virtual void SetPhyLayerState(const PhyState &);
  // SetPhyLayerState cambia el estado de la capa fisica y se lo hace saber a la
  // capa de arriba

  // Two instances of CommsDeviceService for the same purpose in the same
  // machine (for debug reasons) must have different namespaces
  // This method must be called before Start
  void SetCommsDeviceId(std::string nspace);

  void WaitForFramesFromRxFifo();
  bool WaitForFramesFromRxFifo(unsigned int timeout);
  void WaitForDeviceReadyToTransmit();
  int type;

private:
  std::string _namespace;
  // El siguiente metodo es para Debug en la misma maquina y ha de llamarse
  // antes que Start().
  // Especifica el prefijo de las colas de mensajes (sin contar el '/'
  void SetQueuePrefix(std::string _qprefix) { qprefix = _qprefix; }
  // Pide a la capa fisica su estado (solo para IPHY_TYPE_PHY)
  void ReqPhyLayerState();
  PhyState _GetPhyLayerState();

  void _SetPhyLayerState(const PhyState &state);

  PacketPtr GetNextPacket();
  void PushNewFrame(PacketPtr);

  void UpdateMQAttr();
  void ShowMQAttr(std::ostream &, int);

  long GetMaxMsgOnQueue(int);
  long GetMaxMsgSize(int);
  long GetNumMsgOnQueue(int);
  bool GetNonblockFlag(int);

  void ClearInputQueue();
  void SetNonblockFlag(bool, int);

  void SendPhyLayerState();
  void SendPhyLayerState(const PhyState &);

  struct mq_attr *GetMQAttr(int);
  mqd_t GetMQId(int);
  void Init(int type, struct mq_attr attr, int perm);

  class ServiceMessage {
  public:
    enum MsgType { FRAME = 0, REQ_STATE, CMD_STATE, NOTBUILT };

    ServiceMessage();
    ~ServiceMessage();

    void Init(int maxmsgsize);

    MsgType GetMsgType() const { return (MsgType)*type; }
    PhyState GetPhyState() const { return (PhyState)*payload; }
    PacketPtr GetPacket() const;
    void *GetBuffer() const { return buffer; }
    unsigned int GetSize() const { return size; }
    unsigned int GetMaxSize() const { return maxSize; }
    unsigned int GetMaxPayloadSize() const { return maxPayloadSize; }

    void BuildPacketMsg(const PacketPtr &);
    void BuildReqStateMsg();
    void BuildCmdStateMsg(const PhyState &state);

  private:
    void *buffer;
    uint8_t *payload;
    uint8_t *type;
    int size;
    int maxSize, maxPayloadSize;
  };

  class ServiceThread {
    // En C++11 una clase anidada puede acceder a los metodos de la "enclosing"
    // class
  public:
    ServiceThread(CommsDeviceService *);
    ~ServiceThread();
    bool IsRunning();
    void Start();
    void Stop();
    void Work();

  private:
    std::thread thread;
    bool mcontinue;
    bool terminated;
    bool started;
    CommsDeviceService *physervice;
  };

  void ReceiveMsg(ServiceMessage &);
  void SendMsg(const ServiceMessage &);

  void SavePhyStateFromMsg(const ServiceMessage &);
  void SaveFrameFromMsg(const ServiceMessage &);

  std::queue<PacketPtr> rxfifo;

  std::mutex rxfifo_mutex;
  std::mutex phyState_mutex;
  std::condition_variable rxfifo_cond, phyState_cond;

  std::string txmqname, rxmqname;
  mqd_t txmqid, rxmqid;
  struct mq_attr txattr, rxattr;

  struct mq_attr comattr;
  int comperm;
  std::string qprefix;

  unsigned int maxmsgsize;

  PhyState phyState;

  // rxmsg y replymsg se tratan en un thread diferente a txmsg.
  // Concretamente, rxmsg y replymsg se tratan en un ServiceThread, y txmsg en
  // el main thread
  ServiceMessage rxmsg, txmsg, replymsg;
  ServiceThread service;
};

//////////// IMPLEMENTATION ////////////////

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

template <class PacketType>
CommsDeviceService<PacketType>::CommsDeviceService(int _type, int maxframesize)
    : service(this) {
  comattr.mq_maxmsg = 10;
  comattr.mq_msgsize = maxframesize + MSG_OVERHEAD_SIZE;
  comperm = 0777;
  qprefix = "";
  type = _type;
  SetLogName("CommsDeviceService");
}

template <class PacketType>
void CommsDeviceService<PacketType>::SetCommsDeviceId(std::string m) {
  _namespace = m;
  SetQueuePrefix(_namespace);
}

template <class PacketType>
void CommsDeviceService<PacketType>::Init(int _type, struct mq_attr attr,
                                          int perm) {
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

template <class PacketType>
CommsDeviceService<PacketType>::~CommsDeviceService() {
  // TODO Auto-generated destructor stub
  mq_close(rxmqid);
  mq_close(txmqid);
  mq_unlink(txmqname.c_str());
  mq_unlink(rxmqname.c_str());
}

template <class PacketType>
void CommsDeviceService<PacketType>::UpdateMQAttr() {
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

template <class PacketType>
struct mq_attr *CommsDeviceService<PacketType>::GetMQAttr(int mq) {
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

template <class PacketType>
mqd_t CommsDeviceService<PacketType>::GetMQId(int mq) {
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

template <class PacketType>
void CommsDeviceService<PacketType>::ShowMQAttr(std::ostream &o, int mq) {
  struct mq_attr *attr = GetMQAttr(mq);

  o << " - Maximum # of messages on queue:\t" << attr->mq_maxmsg << std::endl;
  o << " - Maximum message size:\t" << attr->mq_msgsize << std::endl;
  o << " - # of messages currently on queue:\t" << attr->mq_curmsgs
    << std::endl;
  o << " - O_NONBLOCK:\t"
    << (attr->mq_flags & O_NONBLOCK ? "activado" : "desactivado") << std::endl;
}
template <class PacketType>
long CommsDeviceService<PacketType>::GetMaxMsgOnQueue(int mq) {
  struct mq_attr *attr = GetMQAttr(mq);
  return attr->mq_maxmsg;
}
template <class PacketType>
long CommsDeviceService<PacketType>::GetMaxMsgSize(int mq) {
  struct mq_attr *attr = GetMQAttr(mq);
  return attr->mq_msgsize;
}
template <class PacketType>
long CommsDeviceService<PacketType>::GetNumMsgOnQueue(int mq) {
  struct mq_attr *attr = GetMQAttr(mq);
  return attr->mq_curmsgs;
}
template <class PacketType>
bool CommsDeviceService<PacketType>::GetNonblockFlag(int mq) {
  struct mq_attr *attr = GetMQAttr(mq);
  return attr->mq_flags & O_NONBLOCK;
}
template <class PacketType>
void CommsDeviceService<PacketType>::SetNonblockFlag(bool v, int mq) {
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
template <class PacketType>
void CommsDeviceService<PacketType>::ClearInputQueue() {
  bool curflag = GetNonblockFlag(RX_MQ);
  SetNonblockFlag(true, RX_MQ);

  char _auxb[4000];
  int n = 0;
  while (n >= 0) {
    n = mq_receive(rxmqid, _auxb, 4000, NULL);
  }
  SetNonblockFlag(curflag, RX_MQ);
}
template <class PacketType>
ICommsLink &CommsDeviceService<PacketType>::operator<<(PacketPtr dlf) {
  txmsg.BuildPacketMsg(dlf);
  if (type == IPHY_TYPE_DLINK) {
    // the frame is directed from de dlink layer to the phy layer, so we set the
    // phy layer state to BUSY
    Log->debug("Seteando manualmente el estado de 'OCUPADO'");
    _SetPhyLayerState(BUSY);
  }
  SendMsg(txmsg);

  return *this; // nunca llegara aqui
}
template <class PacketType>
void CommsDeviceService<PacketType>::SendMsg(const ServiceMessage &msg) {
  if (mq_send(txmqid, (char *)msg.GetBuffer(), msg.GetSize(), 0) == -1) {
    ThrowPhyLayerException(
        std::string("Error(") + std::to_string(errno) +
        std::string("): Error interno: no se ha podido enviar el mensaje"));
  }
}
template <class PacketType>
void CommsDeviceService<PacketType>::ReceiveMsg(ServiceMessage &msg) {
  if (mq_receive(rxmqid, (char *)msg.GetBuffer(), msg.GetMaxSize(), NULL) ==
      -1) {
    ThrowPhyLayerException(
        std::string("Error(") + std::to_string(errno) +
        std::string(
            "): Error interno: fallo al intentar recibir algun mensaje"));
  }
}
template <class PacketType>
void CommsDeviceService<PacketType>::WaitForFramesFromRxFifo() {
  std::unique_lock<std::mutex> lock(rxfifo_mutex);
  while (rxfifo.empty()) {
    rxfifo_cond.wait(lock);
  }
}
template <class PacketType>
bool CommsDeviceService<PacketType>::WaitForFramesFromRxFifo(
    unsigned int timeout) {
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
template <class PacketType>
void CommsDeviceService<PacketType>::WaitForDeviceReadyToTransmit() {
  std::unique_lock<std::mutex> lock(phyState_mutex);
  while (phyState == PhyState::BUSY) {
    phyState_cond.wait(lock);
  }
}
template <class PacketType>
PacketPtr CommsDeviceService<PacketType>::GetNextPacket() {
  std::unique_lock<std::mutex> lock(rxfifo_mutex);
  while (rxfifo.empty()) {
    rxfifo_cond.wait(lock);
  }
  PacketPtr dlf = rxfifo.front();
  rxfifo.pop();

  // unique_lock destryctor unlocks automatically rxfifo_mutex

  return dlf;
}
template <class PacketType>
void CommsDeviceService<PacketType>::PushNewFrame(PacketPtr dlf) {
  rxfifo_mutex.lock();

  rxfifo.push(dlf);

  rxfifo_cond.notify_one();
  rxfifo_mutex.unlock();
}
template <class PacketType>
ICommsLink &CommsDeviceService<PacketType>::operator>>(PacketPtr dlf) {
  dlf = GetNextPacket();
  return *this;
}
template <class PacketType>
void CommsDeviceService<PacketType>::_SetPhyLayerState(const PhyState &state) {
  phyState_mutex.lock();

  phyState = state;
  if (state == PhyState::READY) {
    phyState_cond.notify_one();
  }

  phyState_mutex.unlock();
}
template <class PacketType>
typename CommsDeviceService<PacketType>::PhyState
CommsDeviceService<PacketType>::_GetPhyLayerState() {
  PhyState state;
  phyState_mutex.lock();

  state = phyState;

  phyState_mutex.unlock();

  return state;
}

template <class PacketType>
void CommsDeviceService<PacketType>::SendPhyLayerState(const PhyState &state) {
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

template <class PacketType>
bool CommsDeviceService<PacketType>::BusyTransmitting() {
  if (type != IPHY_TYPE_DLINK)
    ThrowPhyLayerException("Method call not allowed");
  return _GetPhyLayerState() == PhyState::BUSY;
}

template <class PacketType>
void CommsDeviceService<PacketType>::ReqPhyLayerState() {
  if (type != IPHY_TYPE_DLINK)
    ThrowPhyLayerException("Method call not allowed");
  txmsg.BuildReqStateMsg();
  SendMsg(txmsg);
}

template <class PacketType>
void CommsDeviceService<PacketType>::SetPhyLayerState(const PhyState &state) {
  if (type != IPHY_TYPE_PHY)
    ThrowPhyLayerException("Method call not allowed");
  _SetPhyLayerState(state);
  SendPhyLayerState();
}

template <class PacketType>
unsigned int CommsDeviceService<PacketType>::GetRxFifoSize() {
  unsigned int size;
  rxfifo_mutex.lock();
  size = rxfifo.size();
  rxfifo_mutex.unlock();
  return size;
}

template <class PacketType> void CommsDeviceService<PacketType>::Start() {

  Init(type, comattr, comperm);
  service.Start();
  if (type == IPHY_TYPE_PHY) {
    // Enviamos el estado actual a la capa de arriba
    SendPhyLayerState();
  } else {
    ReqPhyLayerState();
  }
}

template <class PacketType> void CommsDeviceService<PacketType>::Stop() {
  service.Stop();
}

template <class PacketType>
CommsDeviceService<PacketType>::ServiceMessage::ServiceMessage() {
  buffer = NULL;
}

template <class PacketType>
CommsDeviceService<PacketType>::ServiceMessage::~ServiceMessage() {
  free(buffer);
}

template <class PacketType>
void CommsDeviceService<PacketType>::ServiceMessage::Init(int maxs) {
  maxPayloadSize = maxs - MSG_OVERHEAD_SIZE;
  maxSize = maxs;
  buffer = malloc(maxs);
  type = (uint8_t *)buffer;
  *type = (uint8_t)NOTBUILT;
  payload = type + 1;
  size = 0;
}

template <class PacketType>
void CommsDeviceService<PacketType>::ServiceMessage::BuildPacketMsg(
    const PacketPtr &dlf) {
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

template <class PacketType>
void CommsDeviceService<PacketType>::ServiceMessage::BuildReqStateMsg() {
  *type = (uint8_t)MsgType::REQ_STATE;
  size = MSG_OVERHEAD_SIZE;
}

template <class PacketType>
void CommsDeviceService<PacketType>::ServiceMessage::BuildCmdStateMsg(
    const PhyState &state) {
  *type = (uint8_t)MsgType::CMD_STATE;
  *payload = (uint8_t)state;
  size = MSG_OVERHEAD_SIZE + 1;
}

template <class PacketType>
PacketPtr CommsDeviceService<PacketType>::ServiceMessage::GetPacket() const {
  return PacketType::CreatePacketFromBuffer(payload);
}

template <class PacketType>
CommsDeviceService<PacketType>::ServiceThread::ServiceThread(
    CommsDeviceService *parent) {
  mcontinue = true;
  terminated = false;
  started = false;
  physervice = parent;
}

template <class PacketType>
CommsDeviceService<PacketType>::ServiceThread::~ServiceThread() {
  this->Stop();
}

template <class PacketType>
void CommsDeviceService<PacketType>::ServiceThread::Start() {
  thread = std::thread(&ServiceThread::Work, this);
  started = true;
}

template <class PacketType>
void CommsDeviceService<PacketType>::ServiceThread::Stop() {
  mcontinue = false;
}

template <class PacketType>
bool CommsDeviceService<PacketType>::ServiceThread::IsRunning() {
  return started && !terminated;
}

template <class PacketType>
void CommsDeviceService<PacketType>::SaveFrameFromMsg(
    const ServiceMessage &msg) {
  PushNewFrame(msg.GetPacket());
}

template <class PacketType>
void CommsDeviceService<PacketType>::SavePhyStateFromMsg(
    const ServiceMessage &msg) {
  _SetPhyLayerState(msg.GetPhyState());
}

template <class PacketType>
void CommsDeviceService<PacketType>::SendPhyLayerState() {
  SendPhyLayerState(_GetPhyLayerState());
}

template <class PacketType>
void CommsDeviceService<PacketType>::ServiceThread::Work() {
  while (mcontinue) {
    physervice->Log->debug("Esperando mensaje...");
    physervice->ReceiveMsg(physervice->rxmsg);
    switch (physervice->rxmsg.GetMsgType()) {
    case ServiceMessage::FRAME:
      if (physervice->type == IPHY_TYPE_DLINK)
        physervice->Log->debug("Received frame from the physical layer");
      else
        physervice->Log->debug("Received frame from the D-Link layer");

      physervice->SaveFrameFromMsg(physervice->rxmsg);
      break;
    case ServiceMessage::CMD_STATE:
      physervice->Log->debug("Recibido mensaje de estado de la capa fisica");
      physervice->SavePhyStateFromMsg(physervice->rxmsg);
      break;
    case ServiceMessage::REQ_STATE:
      physervice->Log->debug("Recibida peticion de estado de la capa fisica");
      physervice->SendPhyLayerState();
      break;
    default:
      break;
    }
    // std::this_thread::sleep_for(std::chrono::seconds(2));
  }
  physervice->Log->debug("Terminando...");
  terminated = true;
}

} /* namespace radiotransmission */

#endif /* DCCOMMS_COMMSDEVICESERVICE_H_ */
