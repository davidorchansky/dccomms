#include <ControladorMotores.h>

#define FCS_SIZE 2 
#define INFO_SIZE 4
#define PREAMBLE_SIZE 7

#define AREYOU_SIZE 18
#define IMFROM_SIZE 11

#define BUFFER_SIZE 500


#define TH_FW 0
#define TH_RV 1
#define TH_SP 2
#define TH_ADD_1 0x52
#define TH_ADD_2 0x54
#define TH_ADD_3 0x56
#define TH_ADD_4 0x58
#define TH_ADD_5 0x5A
#define TH_ADD_6 0x5C

#include <VirtualWire.h>

#define MAX_RADIOUNIT_SIZE 20//VW_MAX_PAYLOAD
#define NMOTORS 6

char buf[BUFFER_SIZE];

bool BigEndian;

char preamble[] = "juanito";
unsigned char * MaxPrePos; //preamble + PREAMBLE_SIZE

char areyou[] = "Hello, are you RX?"; //18
unsigned char * MaxAreYouPos; //areyou + PREAMBLE_SIZE

char imfrom[] = "Yes, I'm RX"; //11

unsigned char *serialPreamblePos;
unsigned char *serialAreYouPos;

ControladorMotores controlador;

#define MAX_RADIOUNIT_SIZE 20

const int transmit_pin = 8;
const int receive_pin = 9;
const int transmit_en_pin = 3;

char caracterActual = -1;

class Radio
{
  public:
    Radio(){}
    ~Radio(){};
    int Available()
    {
       return vw_have_message();
    }
    

    void ReadBytes(void * b, unsigned int t) //Blocking call
    {
        int bytes = bytesInBuffer;
        if(bytesInBuffer < t)
        {
          memcpy(b, buff, bytesInBuffer);
        }
        else
        {
          memcpy(b, buff, t);
          bytesInBuffer -= t;
          memcpy(buff, buff+t, bytesInBuffer);
          return;
        }
    
        uint8_t * ptr = ((uint8_t*) b) + bytesInBuffer;
        int bytesLeft = t - bytesInBuffer;
        bytesInBuffer = 0;
        
;
        while(bytesLeft > 0)
        {
            int len = MAX_RADIOUNIT_SIZE < bytesLeft ? MAX_RADIOUNIT_SIZE : bytesLeft;
            int bytesRead = ReadUnit(ptr, len);
            ptr += bytesRead;
            bytesLeft -= bytesRead;
        }
    }
    private:
      uint8_t buff[MAX_RADIOUNIT_SIZE];
      int bytesInBuffer = 0;
      int ReadUnit(void * b, uint8_t left) //Blocking call
      {
        uint8_t maxb;
        maxb = MAX_RADIOUNIT_SIZE;
        vw_wait_rx();
        if(vw_get_message(buff, &maxb))
        {
          if(maxb > left)
          {
            memcpy(b, buff, left);
            bytesInBuffer = maxb-left;
            memcpy(buff, buff+left, bytesInBuffer);
            return left;
          }
          memcpy(b, buff, maxb);
          return maxb;
        }
        return 0;
      }
    
    
};
Radio radio;
bool IsBigEndian()
{
	uint32_t word = 0x1;
	uint8_t * byte = (uint8_t *)&word;
	return *byte != 0x1;
}

void readNBytes(char* buff, int tam, Stream * stream)
{
  int bytes = 0;
  while (bytes < tam)
  {
    if (stream->available() > 0)
      buff[bytes++] = stream->read();
  }
}


uint16_t radioFrameReceived(char* buffer, unsigned char* pre, unsigned char* maxPrePos, unsigned char **preamblePos)
{
  
  if (*preamblePos == maxPrePos)
  {
    //Serial.println("Esperando datos!");
    *preamblePos = pre;
    radio.ReadBytes(buffer, INFO_SIZE);
    uint16_t dsize;
    uint16_t *fdsize = (uint16_t *)(buffer+2);
    if(BigEndian)
    {
      dsize = *fdsize;
    }
    else
    {
      dsize = ((*fdsize) << 8) | ((*fdsize) >> 8);
    }
    
   //Serial.print("Size0: "); Serial.println((int)*((uint8_t*)fdsize));
   //Serial.print("Size1: "); Serial.println((int)*(((uint8_t*)fdsize)+1));
    if(dsize <= BUFFER_SIZE-FCS_SIZE)
    {
      
      radio.ReadBytes(buffer+INFO_SIZE, dsize + FCS_SIZE);
      return dsize;
    }
    
    return 0;
  }
  char car;
  radio.ReadBytes(&car,1);
  //Serial.print("leido: "); Serial.println(car);
  //Serial.print("Se esperaba: "); Serial.println((char)**preamblePos);
  if (car == **preamblePos)
  {
    //Serial.println("Coincide!");

    (*preamblePos)++;
  }
  else
  {
   
    *preamblePos = pre;
  }

  return 0;
}

boolean commandReceived(Stream * s, char* buffer, int buffSize, unsigned char* pre, unsigned char* maxPrePos, unsigned char **preamblePos)
{
  if (*preamblePos == maxPrePos)
  {
    //s->println("Comando reconocido!");
    //analogWrite(analogOutPin, 255);
    readNBytes(buffer, buffSize, s);
    *preamblePos = pre;
    //analogWrite(analogOutPin, 0);
    return true;
  }

  char car = caracterActual;

  if (car == **preamblePos)
  {
    //s->print("Coincide! "); s->println(iteracion);
    (*preamblePos)++;
  }
  else
  {
    //s->print("No coincide..."); s->println(iteracion);
    *preamblePos = pre;
  }


  return false;
}
void setup() {
  //Serial.begin(9600);
  Serial.begin(115200);
  //radioStream->begin(57600);

  BigEndian = IsBigEndian();
  
  MaxPrePos = (unsigned char *) preamble + PREAMBLE_SIZE;
  MaxAreYouPos = (unsigned char*) areyou + AREYOU_SIZE;

  serialPreamblePos = (unsigned char*)preamble;
  serialAreYouPos = (unsigned char*)areyou;
  
    vw_set_tx_pin(transmit_pin);
    vw_set_rx_pin(receive_pin);
    vw_set_ptt_pin(transmit_en_pin);
    vw_set_ptt_inverted(true); // Required for DR3100
    vw_setup(5000);	 // Bits per sec

    vw_rx_start();       // Start the receiver PLL running
}


uint16_t
_crc_xmodem_update (uint16_t crc, uint8_t data)
{
      int i;

      crc = crc ^ ((uint16_t)data << 8);
      for (i=0; i<8; i++)
      {
          if (crc & 0x8000)
              crc = (crc << 1) ^ 0x1021;
          else
              crc <<= 1;
      }

      return crc;
}

uint16_t
crc16(const void *buf, size_t size)
{
	size_t i;
	uint16_t crc = 0;
	uint8_t c;
	uint8_t * b = (uint8_t *) buf;
	for (i=0; i<size; i++)
	{
		c = b[i];
		crc = _crc_xmodem_update (crc, c);
	}
        return crc;
}

bool frameIsOk(void *obuff, int lenpld)
{
    return crc16(obuff, INFO_SIZE + lenpld + FCS_SIZE)==0;
}

int paylsize;
uint8_t * thrusterCommand[50];

struct Command
{
  int cmd;
  int arg;  
};


boolean execMotorCommand(int8_t arg, int motor)
{
  
  if(arg <= 100)
  {
    if(arg >= 0){
   Serial.print("Orden: motor(0x"); Serial.print(motor, HEX);Serial.print(") speed(");Serial.print((int)arg); Serial.println(")");
 controlador.forward(motor, arg); return true;}
    else if( arg >= -100){
   Serial.print("Orden: motor(0x"); Serial.print(motor, HEX);Serial.print(") speed(");Serial.print((int)arg); Serial.println(")"); controlador.reverse(motor, -1*arg); return true;}
    
  }
  
  return false;
}

boolean execMotorCommands(void *obuff, int len)
{
  if(len != NMOTORS) return false;
  
  int8_t * ptr = (int8_t*) obuff;
  return execMotorCommand(*ptr, TH_ADD_1) &&
  execMotorCommand(*(ptr+1),TH_ADD_2) &&
  execMotorCommand(*(ptr+2),TH_ADD_3) &&
  execMotorCommand(*(ptr+3),TH_ADD_4) &&
  execMotorCommand(*(ptr+4),TH_ADD_5) &&
  execMotorCommand(*(ptr+5),TH_ADD_6);
  
        
}

void loop()
{

  if (radio.Available())
  {

    if (paylsize=radioFrameReceived(buf, (unsigned char*)preamble, MaxPrePos, &serialPreamblePos))
    { 
        if(frameIsOk(buf, paylsize))
        {
          //Serial.write(buf + INFO_SIZE, paylsize);
          
          if(!execMotorCommands(buf + INFO_SIZE, paylsize))
            Serial.println("Mensaje de motores no valido");
          else
            {
              //Serial.println();
              Serial.print((int)buf[INFO_SIZE]); Serial.print(" "); Serial.print((int)buf[INFO_SIZE+1]); Serial.print(" ... "); Serial.println((int)buf[INFO_SIZE+5]);
            }
        }
        else
        {
          Serial.println("Trama recibida con errores");
        }
        
    }
   
  }
}
