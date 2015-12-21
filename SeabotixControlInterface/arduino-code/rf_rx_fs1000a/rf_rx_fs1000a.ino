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
  boolean motor1, motor2, motor3, motor4, motor5, motor6;
  int arg;  
};


char * getInt(char *s, int *v, int *sign)
{
        *v = -1;
        if (*s == '+') {
                *sign = 1;
                ++s;
        } else if (*s == '-') {
                *sign = -1;
                ++s;
        } else {
                *sign = 0;
        }
        if (*s >= '0' && *s <= '9') {
                int x = *s++ - '0';
                while (*s >= '0' && *s <= '9') {
                        x = x * 10 + (*s++ - '0');
                }
                *v = x;
        }
        return s;
}

char * getMotorsFromCommand(char * obuff, char *final, struct Command * cmd)
{
   char * ptr = obuff;
   
   cmd->motor1 = false;
   cmd->motor2 = false;
   cmd->motor3 = false;
   cmd->motor4 = false;
   cmd->motor5 = false;
   cmd->motor6 = false;
   
   while(ptr != final)
   {
     int v, s;
     ptr = getInt(ptr, &v, &s);
     if(s != 0) return NULL;
     switch(v)
     {
        case 1:
          cmd->motor1 = true;
          break;
        case 2:
          cmd->motor2 = true;
          break;
        case 3:
          cmd->motor3 = true;
          break;
        case 4:
          cmd->motor4 = true;
          break;
        case 5:
          cmd->motor5 = true;
          break;
        case 6:
          cmd->motor6 = true;
          break;
        default:
          return NULL;
     }
     if(*ptr != ',')
       return ptr;
     ptr++;
   }
}

void printMotorCommand(struct Command cmd)
{
  switch(cmd.cmd)
  {
    case TH_FW:
      Serial.print("Marcha hacia adelante a ");Serial.print(cmd.arg);Serial.println("%");
      break;
    case TH_RV:
      Serial.print("Marcha hacia atras a ");Serial.print(cmd.arg);Serial.println("%");
      break;
    case TH_SP:
      Serial.println("Orden de paro");
      break;
    default:
      Serial.println("Comando desconocido");
      break;
  }
  Serial.print("Motor 1: ");
  cmd.motor1 ? Serial.println("Si") : Serial.println("No");
  
    Serial.print("Motor 2: ");
  cmd.motor2 ? Serial.println("Si") : Serial.println("No");
  
    Serial.print("Motor 3: ");
  cmd.motor3 ? Serial.println("Si") : Serial.println("No");
  
    Serial.print("Motor 4: ");
  cmd.motor4 ? Serial.println("Si") : Serial.println("No");
  
    Serial.print("Motor 5: ");
  cmd.motor5 ? Serial.println("Si") : Serial.println("No");
  
    Serial.print("Motor 6: ");
  cmd.motor6 ? Serial.println("Si") : Serial.println("No");
}

boolean getMotorCommand(void *obuff, int len, struct Command * cmd)
{
   uint8_t * ptr = (uint8_t *) obuff;
   uint8_t * final = ptr + len;
   if(*ptr == 'f')
   {
       ptr++;
       if(*ptr++ == 'w')
       {
         if(*ptr++ == ' ')
         {
           ptr = (uint8_t*) getMotorsFromCommand((char *)ptr, (char *)final, cmd);
           if(ptr == NULL) return false;
           if(*ptr++ != ' ') return false;
           int v, s;
           ptr = (uint8_t*) getInt((char *)ptr, &v, &s);
           if(s != 0) return false;
           if(v < 0 || v > 100) return false;
           cmd->cmd = TH_FW;
           cmd->arg = v;
           if(*ptr++ != '\n') return false;
           return true;
         }
         else
           return false;
       }
       else
         return false;
   }
   else if(*ptr == 'r')
   {
     ptr++;
     if(*ptr++ == 'v')
     {
         if(*ptr++ == ' ')
         {
           ptr = (uint8_t*) getMotorsFromCommand((char *)ptr, (char *)final, cmd);
           if(ptr == NULL) return false;
           if(*ptr++ != ' ') return false;
           int v, s;
           ptr = (uint8_t*) getInt((char *)ptr, &v, &s);
           if(s != 0) return false;
           if(v < 0 || v > 100) return false;
           cmd->cmd = TH_RV;
           cmd->arg = v;
           if(*ptr++ != '\n') return false;
           return true;
         }     
     }
     else return false;
   }
   else if (*ptr == 's')
   {
     ptr++;
     if(*ptr++ == 'p')
     {
           if(*ptr++ == ' ')
         {
           ptr = (uint8_t*) getMotorsFromCommand((char *)ptr, (char *)final, cmd);
           if(ptr == NULL) return false;
           if(*ptr++ != '\n') return false;
           cmd->cmd = TH_SP;
           return true;     
         }
     }
     else return false;
   }
   else return false;

}

void execMotorCommand(struct Command cmd)
{
  switch(cmd.cmd)
  {
    case TH_FW:
      if(cmd.motor1) controlador.forward(TH_ADD_1,cmd.arg);
      if(cmd.motor2) controlador.forward(TH_ADD_2,cmd.arg);
      if(cmd.motor3) controlador.forward(TH_ADD_3,cmd.arg);
      if(cmd.motor4) controlador.forward(TH_ADD_4,cmd.arg);
      if(cmd.motor5) controlador.forward(TH_ADD_5,cmd.arg);
      if(cmd.motor6) controlador.forward(TH_ADD_6,cmd.arg);
      break;
    case TH_RV:
      if(cmd.motor1) controlador.reverse(TH_ADD_1,cmd.arg);
      if(cmd.motor2) controlador.reverse(TH_ADD_2,cmd.arg);
      if(cmd.motor3) controlador.reverse(TH_ADD_3,cmd.arg);
      if(cmd.motor4) controlador.reverse(TH_ADD_4,cmd.arg);
      if(cmd.motor5) controlador.reverse(TH_ADD_5,cmd.arg);
      if(cmd.motor6) controlador.reverse(TH_ADD_6,cmd.arg);
      break;
    case TH_SP:
      if(cmd.motor1) controlador.stop(TH_ADD_1);
      if(cmd.motor2) controlador.stop(TH_ADD_2);
      if(cmd.motor3) controlador.stop(TH_ADD_3);
      if(cmd.motor4) controlador.stop(TH_ADD_4);
      if(cmd.motor5) controlador.stop(TH_ADD_5);
      if(cmd.motor6) controlador.stop(TH_ADD_6);
      break;
  }
}
void loop()
{

  if (radio.Available())
  {

    if (paylsize=radioFrameReceived(buf, (unsigned char*)preamble, MaxPrePos, &serialPreamblePos))
    { 
        if(frameIsOk(buf, paylsize))
        {
          Serial.write(buf + INFO_SIZE, paylsize);
          struct Command cmd;
          if(getMotorCommand(buf + INFO_SIZE, paylsize, &cmd))
          {
            printMotorCommand(cmd);
            execMotorCommand(cmd);
          }
          else
            Serial.println("Comando no valido");
        }
        else
        {
          Serial.println("Trama recibida con errores");
        }
        
    }
   
  }
}
