/*
 * DataLinkFrame.h
 *
 *  Created on: Feb 12, 2015
 *      Author: diego
 */

#ifndef DATALINKFRAME_H_
#define DATALINKFRAME_H_


#include <dccomms/DataLinkFrame.h>
#include <ostream>
#include <cstring>
#include <boost/shared_ptr.hpp>
#include <dccomms/CommsException.h>
#include <dccomms/Packet.h>

namespace dccomms {

#define DLNK_PREAMBLE_SIZE 10
#define DLNK_DIR_SIZE   1
#define DLNK_DSIZE_SIZE 2
#define DLNK_MAX_PAYLOAD_SIZE 8000



  class DataLinkFrame;

  typedef boost::shared_ptr<DataLinkFrame> DataLinkFramePtr;

  class DataLinkFrame: Packet{
    friend class IStreamCommsDevice;
  public:
    enum fcsType { crc16, crc32, nofcs };

    static DataLinkFramePtr BuildDataLinkFrame(fcsType fcst);
    static DataLinkFramePtr BuildDataLinkFrame(
        uint8_t, //destination dir
        uint8_t, //source dir
        uint16_t, //data size
        uint8_t *, //data
        fcsType //fcstype
        );
    static DataLinkFramePtr Copy(DataLinkFramePtr src);

    virtual ~DataLinkFrame();

    uint8_t GetDesDir(){return *_ddir;}
    uint8_t GetSrcDir(){return *_sdir;}
    int GetFrameSize() const {return _frameSize;}
    uint8_t * GetFrameBuffer() const {return _buffer;}
    fcsType GetFcsType() const {return _fcstype;}

    void SetDesDir(uint8_t _ddir);
    void SetSrcDir(uint8_t _sdir);

    void UpdateFrame(
        uint8_t, //destination dir
        uint8_t, //source dir
        uint16_t, //data size
        uint8_t * //data
        );

    void PayloadUpdated(unsigned int datasize);
    inline uint8_t * GetPayloadBuffer()
    {
      return _payload;
    }

    inline uint32_t GetPayloadSize()
    {
      return _payloadSize;
    }

    void GetInfoFromBuffer(void *);
    void GetInfoFromBufferWithPreamble(void *o);

    uint8_t* GetFrameBits(void *dst);

    void printFrame(std::ostream &);
    bool checkFrame();

    static bool IsBigEndian();
    static const unsigned char * manchesterPre;
  private:
    DataLinkFrame(fcsType fcst);
    DataLinkFrame(
        uint8_t, //destination dir
        uint8_t, //source dir
        uint16_t, //data size
        uint8_t *, //data
        fcsType //fcstype
        );

    void Init(DataLinkFrame::fcsType fcst);

    uint8_t * _pre,* _ddir,*_sdir,* _fcs;
    uint16_t * _dsize;

    uint8_t *_buffer = NULL;
    uint16_t _overheadSize = 0;

    fcsType _fcstype;
    uint32_t _fcsSize = 0;


    int _frameSize = 0;

    uint16_t _payloadSize = 0;
    uint8_t* _payload = NULL;


    uint8_t _totalInfoSize = 0;

    void _calculateCRC();
    void _SetPayloadSizeInBuffer(unsigned int datasize);
    bool _BigEndian;
    bool _dataIn = false;

  };

} /* namespace radiotransmission */

#endif /* DATALINKFRAME_H_ */
