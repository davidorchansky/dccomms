#include <SoftwareSerial.h>

#define PAYLOAD_SIZE 1500
#define PREAMBLE_SIZE 6

#define AREYOU_SIZE 5
#define IMFROM_SIZE 7


char areyou[] = "Hello"; //size 5
unsigned char * MaxAreYouPos; //areyou + PREAMBLE_SIZE

char imfrom[] = "Vinroma"; //size 7



unsigned char *serialPreamblePos;
unsigned char *serialAreYouPos;
const int analogOutPin = A0;

char caracterActual = -1;

//DEBUG
unsigned int iteracion = 0;
void readNBytes(char* buff, int tam, Stream * stream)
{
	int bytes = 0;
	while (bytes < tam)
	{
		//Serial.println("Obteniendo payload...");
		if (stream->available()>0)
			buff[bytes++] = stream->read();
	}
}



boolean commandReceived(Stream * s, char* buffer, int buffSize, unsigned char* pre, unsigned char* maxPrePos, unsigned char **preamblePos)
{
	if (*preamblePos == maxPrePos)
	{
		//s->println("Comando reconocido!");
		analogWrite(analogOutPin, 255);
		readNBytes(buffer, buffSize, s);
		*preamblePos = pre;
		analogWrite(analogOutPin, 0);
		return true;
	}

	char car = caracterActual;

	if (car == '$') //Per a sincronitzar
		s->print('$');
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


int pin = 13;
char buf[2100];
void setup()
{
	//Serial1.begin(9600);
	Serial.begin(115200);
	//Serial1.begin(115200);
  /* add setup code here */
	pinMode(pin, OUTPUT);
	digitalWrite(pin, LOW);

	analogWrite(A0, 0);
	MaxAreYouPos = (unsigned char*)areyou + AREYOU_SIZE;
	serialAreYouPos = (unsigned char*)areyou;
      for(int i = 0; i < 1500; i++)
      {
        buf[i] = 'A';
      }
}

bool canRoute = true;
bool initi = false;
void loop()
{
	if (Serial.available()>0)
	{
		caracterActual = Serial.peek();

		if (commandReceived(&Serial, NULL, 0, (unsigned char*)areyou, MaxAreYouPos, &serialAreYouPos))
		{
			analogWrite(A0, 255);
			Serial.write((const uint8_t*)imfrom, IMFROM_SIZE);
			Serial.flush();
			canRoute = true;
			analogWrite(A0, 0);
		}

		Serial.read();
	}
	if (canRoute)
	{
		if (!initi) 
			delay(10);

		initi = true;
      

		digitalWrite(pin, HIGH);
		Serial.print("juanito");
                Serial.write((uint8_t) 0x1);
                Serial.write((uint8_t) 0x2);
                Serial.write((uint8_t) 0x08);Serial.write((uint8_t) 0x34);
                Serial.write(buf, 2100);
                Serial.write((uint8_t)0x47);
                Serial.write((uint8_t)0xb1);
		

		Serial.print("juanito");
                Serial.write((uint8_t) 0x1);
                Serial.write((uint8_t) 0x2);
                Serial.write((uint8_t) 0x00);Serial.write((uint8_t) 0x09);
               
                Serial.print("cagontota");
                //uint8_t pay[9] = {0x63,0x61,0x67,0x6f,0x6e,0x74,0x6f,0x74,0x61};
                //Serial.write(pay,9);
                Serial.write((uint8_t)0x18);
                Serial.write((uint8_t)0x78);
	        Serial.flush();

		digitalWrite(pin, LOW);
delay(200);
		
		
	}
}





//Serial.print("juanito");
//Serial.print("1");
//Serial.print("2");
//Serial.write((unsigned short)0x0900);
//Serial.print("123456789");
//Serial.write((unsigned short)0x47b1);
