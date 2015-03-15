
/*
  OBD datalogger

 */

#include <SoftwareSerial.h>


#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <I2Cdev.h>
#include <MPU9150.h>

// Globals
#define SD_CS_PIN 10 // SD breakout
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F\r"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F\r"
#define OBD_RECV_BUF_SIZE 128
#define OBD_TIMEOUT_SHORT 2000 /* ms */
#define OBD_TIMEOUT_LONG 7000 /* ms */
#define OBD_SERIAL_BAUDRATE 38400
#define FILE_NAME_FORMAT "/DAT%05d.CSV"
#define VERBOSE 0



// Hardware Components
SoftwareSerial SerialInfo(A2, A3); /* for BLE Shield on UNO/leonardo*/
#define OBDUART Serial
MPU6050 accelgyro;

// File
static File sdfile;

//Global Vars
uint32_t dataTime;
uint32_t dataSize;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
int32_t lat,lon;
long gdate,gtime;
int16_t alt,sp,hd,sat;
unsigned long time;


// Low level stuff
void writeOBDUART(const char* s)
{
	OBDUART.write(s);
}

void writeOBDUART(char c)
{
	OBDUART.write(c);
}

byte receiveOBDUART()
{
  char* buffer;
  return receiveOBDUART(buffer,OBD_TIMEOUT_SHORT);
}

byte receiveOBDUART(char* buffer)
{
 return receiveOBDUART(buffer,OBD_TIMEOUT_SHORT);
}

byte receiveOBDUART(char* buffer, int timeout)
{
	unsigned char n = 0;
	for (unsigned long startTime = millis();;) {
	    if (OBDUART.available()) {
	        char c = OBDUART.read();
	        if (n > 2 && c == '>') {
	            // prompt char received
	            break;
	        } else if (!buffer) {
                n++;
	        } else if (n < OBD_RECV_BUF_SIZE - 1) {
                if (c == '.' && n > 2 && buffer[n - 1] == '.' && buffer[n - 2] == '.') {
                    n = 0;
                    timeout = OBD_TIMEOUT_LONG;
                } else {
                    buffer[n++] = c;
                }
	        }
	    } else {
	        if (millis() - startTime > timeout) {
	            // timeout
	            return 0;
	        }
	    }
	}
    if (buffer) buffer[n] = 0;
	return n;
}


// High level stuff

void initConsole()
{
  // Open serial communications and wait for port to open:
  SerialInfo.begin(38400);

}

boolean initSD()
{
  SerialInfo.print("Initializing SD card...");
  pinMode(SD_CS_PIN, OUTPUT);

  // see if the card is present and can be initialized:
  if (!SD.begin(SD_CS_PIN)) {
    SerialInfo.println("Card failed, or not present");
    // don't do anything more:
    return false;
  }
  SerialInfo.println("card initialized.");
  return true;

}

boolean initGPS()
{
	OBDUART.begin(OBD_SERIAL_BAUDRATE);
        delay(100);
  	flushOBDUART();
  
  	writeOBDUART("ATZ");
  	flushOBDUART();
        delay(100);

  	writeOBDUART("ATE0");
  	flushOBDUART();
        delay(100);

  	writeOBDUART("ATL1");
  	flushOBDUART();
        delay(100);
  
        char buf[OBD_RECV_BUF_SIZE];
        // setting GPS baudrate
        writeOBDUART("ATBR2 38400\r");
        if (receiveOBDUART(buf) && strstr(buf, "OK")) {
            
            writeOBDUART("ATSGC ");
            writeOBDUART(PMTK_SET_NMEA_UPDATE_10HZ);
            receiveOBDUART();
            
            return true;
        } else {
            return false;
        }
  

}

void flushOBDUART()
{
	writeOBDUART('\r');
	delay(100);
	while (OBDUART.available()) OBDUART.read();
}


void initNEMS()
{
        Wire.begin();
        accelgyro.initialize();

}

void(* resetFunc) (void) = 0; //declare reset function @ address 0

void logGPSData()
{
        // issue the command to get parsed data
        OBDUART.write("ATGPS\r");
        char buf[16] = {0};
        byte n = 0;
        for (;;) {
            if (OBDUART.available()) {
                char c = OBDUART.read();
                if (c == '>') {
                    // prompt char received
                    break;
                } else {
                    if (c == ',' || c <= 0x0D) {
                        buf[n] = 0;
                        char *p = strchr(buf, '=');
                        if (!p) continue;
                        switch (*(p - 1)) {
                        case 'D':
                            gdate=atol(p + 1);
                            break;
                        case 'T':
                            gtime=atol(p + 1);
                            break;
                        case 'A':
                            alt=atoi(p + 1);
                            break;
                        case 'V':
                            sp=atoi(p + 1);
                            break;
                        case 'C':
                            hd=atoi(p + 1);
                            break;
                        case 'S':
                            //logData(PID_GPS_SAT_COUNT, atoi(p + 1));
                            break;
                        default:
                            if (!memcmp(p - 3, "LAT", 3)) {
                                lat=(int32_t)((float)atof(p + 1) * 100000);
                            } else if (!memcmp(p - 3, "LON", 3)) {
                                lon=(int32_t)((float)atof(p + 1) * 100000);
                            }
                        }
                    } else if (n < sizeof(buf) - 1) {
                        buf[n++] = c;
                    }

                }
            } else if (millis() - dataTime > 100) {
                // timeout
                break;
            }
        }
        //SerialInfo.println(buf);
}
void logMEMSData()
{


    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
#if VERBOSE
    SerialInfo.print("A:");
    SerialInfo.print(ax);
    SerialInfo.print('/');
    SerialInfo.print(ay);
    SerialInfo.print('/');
    SerialInfo.print(az);
    SerialInfo.print(" G:");
    SerialInfo.print(gx);
    SerialInfo.print('/');
    SerialInfo.print(gy);
    SerialInfo.print('/');
    SerialInfo.print(gz);
    SerialInfo.print(" M:");
    SerialInfo.print(mx);
    SerialInfo.print('/');
    SerialInfo.print(my);
    SerialInfo.print('/');
    SerialInfo.println(mz);
#endif

}

uint16_t openFile(uint16_t logFlags = 0, uint32_t dateTime = 0)
{
    uint16_t fileIndex;
    char filename[24] = "/BASELINE";

    if (SD.exists(filename)) {
        for (fileIndex = 1; fileIndex; fileIndex++) {
            sprintf(filename + 9, FILE_NAME_FORMAT, fileIndex);
            if (!SD.exists(filename)) {
                break;
            }
        }
        if (fileIndex == 0)
            return 0;
    } else {
        SD.mkdir(filename);
        fileIndex = 1;
        sprintf(filename + 9, FILE_NAME_FORMAT, 1);
    }

    sdfile = SD.open(filename, FILE_WRITE);
    if (!sdfile) {
        return 0;
    }
    sdfile.println("ms,date,time,lat,lon,alt,sp,hd,sat,ax,ay,az,gx,gy,gz,mx,my,mz");
    dataSize = 1;
    sdfile.flush();
    return fileIndex;
}

void closeFile()
{
    sdfile.close();
}
void flushFile()
{
    sdfile.flush();
}

void writeData()
{
  
        sdfile.print(millis());
        sdfile.print(",");
        sdfile.print(gdate);
        sdfile.print(",");
        sdfile.print(gtime);
        sdfile.print(",");
        sdfile.print(lat);
        sdfile.print(",");
        sdfile.print(lon);
        sdfile.print(",");
        sdfile.print(alt);
        sdfile.print(",");
        sdfile.print(sp);
        sdfile.print(",");
        sdfile.print(hd);
        sdfile.print(",");
        sdfile.print(sat);
        sdfile.print(",");
        sdfile.print(ax);
        sdfile.print(",");
        sdfile.print(ay);
        sdfile.print(",");
        sdfile.print(az);
        sdfile.print(",");
        sdfile.print(gx);
        sdfile.print(",");
        sdfile.print(gy);
        sdfile.print(",");
        sdfile.print(gz);
        sdfile.print(",");
        sdfile.print(mx);
        sdfile.print(",");
        sdfile.print(my);
        sdfile.print(",");
        sdfile.print(mz);
        sdfile.print("\n");
        sdfile.flush();
        dataSize++;
        
        if(dataSize>18001)
        {
            closeFile();
            uint16_t index = openFile();
            if (index) {
                SerialInfo.print("File ID: ");
                SerialInfo.println(index);
            }
          
        }

  
}


void setup()
{
  
  initConsole();
  
  if(!initSD())
  {
       SerialInfo.println("Failed to initialize SD card. Resetting in 3 seconds.");
       delay(3000);
       resetFunc();  //call reset
 
  }
  
  
  if(!initGPS())
  {
       SerialInfo.println("Failed to initialize GPS. Resetting in 3 seconds.");
       delay(3000);
       resetFunc();  //call reset
 
  }

  initNEMS();
  
  uint16_t index = openFile();
  if (index) {
      SerialInfo.print("File ID: ");
      SerialInfo.println(index);
  } else {
      SerialInfo.println("File error");
      delay(3000);
      resetFunc();  //call reset
  }

}

void loop()
{
  time = millis();
  int16_t delta;
  logMEMSData();
  logGPSData();
  writeData();
  delta=millis()-time;
  if(delta>0&&delta<99) delay(100-delta);
}









