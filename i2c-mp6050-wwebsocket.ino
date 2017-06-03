
#include <Arduino.h>

/* ESP286 & WebSocket library*/
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <WebSocketsServer.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <Hash.h>
#include "Wire.h"

#define USE_SERIAL Serial

ESP8266WiFiMulti WiFiMulti;

ESP8266WebServer server = ESP8266WebServer(80);
WebSocketsServer webSocket = WebSocketsServer(81);

String qw, qx, qy, qz, Qtr, sendgyro;

    String err = "error";
  
String readgyro();
/****************************************************/
   /* I2C setup*/
/****************************************************/

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;
//#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_READABLE_QUATERNION
//#define LED_PIN 13 
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================


void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t lenght) {

  switch (type) {
    case WStype_DISCONNECTED:
      USE_SERIAL.printf("[%u] Disconnected!\n", num);
      break;
    case WStype_CONNECTED: {
        IPAddress ip = webSocket.remoteIP(num);
        USE_SERIAL.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);

        // send message to client
        webSocket.sendTXT(num, "Connected");
      }
      break;
    case WStype_TEXT:
      //USE_SERIAL.printf("[%u] get Text: %s\n", num, payload);

      //if (payload[0] == '#') {
        // we get RGB data

        // decode rgb data
        //uint32_t rgb = (uint32_t) strtol((const char *) &payload[1], NULL, 16);

        //analogWrite(LED_RED,    ((rgb >> 16) & 0xFF));
        //analogWrite(LED_GREEN,  ((rgb >> 8) & 0xFF));
        //analogWrite(LED_BLUE,   ((rgb >> 0) & 0xFF));

        //String sendgyro = readgyro();
        //Serial.print("Sending ... " + sendgyro);
        //webSocket.sendTXT(num, "Changed!!");
        webSocket.sendTXT(num, sendgyro);
      //}

      break;
  }

}

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    //#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin(0,2);  //sda,scl
        Wire.setClock(400000);
        //TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    //#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    //    Fastwire::setup(400, true);
    //#endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    //while (!Serial); // wait for Leonardo enumeration, others continue immediately
    mpu.initialize();
    devStatus = mpu.dmpInitialize();

    
    /* ESP286 Setup*/

        USE_SERIAL.println();
        USE_SERIAL.println();
        USE_SERIAL.println();
    
        for(uint8_t t = 4; t > 0; t--) {
            USE_SERIAL.printf("[SETUP] BOOT WAIT %d...\n", t);
            USE_SERIAL.flush();
            delay(1000);
        }
        //WiFiMulti.addAP("WHY So SERIOUS !??", "cuzimbatman");
        //WiFiMulti.addAP("WHY So SERIOUS !??", "cuzimbatman");
        WiFiMulti.addAP("TOTOLINK N100RE", "nrlkinra");
    
    
        while(WiFiMulti.run() != WL_CONNECTED) {
            delay(100);
        }
    
        // start webSocket server
        webSocket.begin();
        webSocket.onEvent(webSocketEvent);
    
        if(MDNS.begin("esp8266")) {
            USE_SERIAL.println("MDNS responder started");
        }
    
        // handle index
        server.on("/", []() {
          server.send(200, "text/html", "<html><head><script>var connection = new WebSocket('ws://'+location.hostname+':81/', ['arduino']);connection.onopen = function () { connection.send('Connect ' + new Date()); }; connection.onerror = function (error) { console.log('WebSocket Error ', error); }; connection.onmessage = function (e) { console.log('Server: ', e.data); }; function reqData() { myReq = setInterval(dataTx, 1000); }; function dataTx() { connection.send('#'); console.log('yo : '); } </script> </head> <body> <p>Check console.</p><br/><br/> <button onclick='reqData()'>Start it</button> <button onclick='clearInterval(myReq)'>Stop it</button> </body> </html> " );
        });
              
        server.begin();
    
        // Add service to MDNS
        MDNS.addService("http", "tcp", 80);
        MDNS.addService("ws", "tcp", 81);
        
    /*************************************************************************/

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        
        USE_SERIAL.print(devStatus);
        USE_SERIAL.println(F(")"));
    }

    // configure LED for output
    //pinMode(LED_PIN, OUTPUT);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

    webSocket.loop();
    server.handleClient();
    
    sendgyro = readgyro();

    //USE_SERIAL.println("\n main looppppppppppp : ");
}


String readgyro(){

    // if programming failed, don't try to do anything
    if (!dmpReady) return err;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
 
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        USE_SERIAL.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
       
        //#ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            //OUTPUT_READABLE_WORLDACCEL
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
         
            Qtr = String(ypr[0] * 180/M_PI) + ',' + String(ypr[1] * 180/M_PI)+ ',' + String(ypr[2] * 180/M_PI) + ',' + String(aaWorld.x) + ',' + String(aaWorld.y) + ',' + String(aaWorld.z) ;
           
            USE_SERIAL.println(Qtr);
    }
    
  return Qtr;
}

