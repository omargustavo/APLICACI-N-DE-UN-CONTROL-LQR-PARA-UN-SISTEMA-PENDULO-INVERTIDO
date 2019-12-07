#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
float VarSensor = 1.5e-5;//0.3371;// 1e-5;//varianza del sensor a determinar  
float VarProcess = 1e-8;/* Varianza del proceso pequeña en caso de que no varie dejarlo bajo,
para respuesta mas rapidas al cambio en el valor del sensor se incrementa esta variaicon mayores al cambio de */
float P = 1.0;
float Pc = 0.0;
float G = 0.0;
float Xp = 0.0;
float Zp = 0.0;
float Xe = 0.0;
//*****************************************HC-SR04**************************************
int Trigger=10;   //Pin digital 2 para el Trigger del sensor
int Echo=11;  //Pin digital 3 para el echo del sensor
double t; //timep que demora en llegar el eco
double d; //distancia en centimetros
//*****************************************LQR ******************************************
double K1=0.0, K2=0.0, K3=0.0,K4=0.0, Ki_angulo=0.0, Ki_x=0.0;
unsigned long lastTime = 0,   SampleTime = 0;                    // Variables de tiempo discreto. 
double        angulo        = 0.0; //MPU
double        x             = 0.0;  //HC-SR04
double        r_angulo              = 0.0;   //referencia  
double        r_x                   = 0.0;   //referencia
double        r_dangulo              = 0.0;   //referencia  
double        r_dx                   = 0.0;   //referencia
double        error                 =  0.0;    
double        error1                = 0.0;  
double        ultima_entrada_angulo = 0.0; 
double        ultima_entrada_x      = 0.0; 
  // angulo 
double        I_angulo     = 0.0; 
double        d_angulo     = 0.0;  
 // x    
double        I_x    = 0.0;
double        d_x     = 0.0;                                                       
double        outMin   = 0.0, outMax     = 0.0;                  
                                   
//***********************************MPU**********************************************
MPU6050 mpu;
#define INTERRUPT_PIN 2
#define LED_PIN 13
bool blinkState = false;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer 
Quaternion q;           // [w, x, y, z]
VectorInt16 aa;         // [x, y, z]
VectorInt16 aaReal;     // [x, y, z]
VectorInt16 aaWorld;    // [x, y, z]
VectorFloat gravity;    // [x, y, z]
float ypr[3];           // [yaw, pitch, roll] 
volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}
//**************************fin mpu****************************** 
double theta;
byte          pwm = 0;
const byte    MA = 7;                  
const byte    MB = 8;                  
const byte    PWM = 6; 
const byte    MA1 = 4;                  
const byte    MB1 = 5;                  
const byte    PWM1 = 9; 
int  activar_control = 12;
int activar; 
void setup() {
    pinMode( activar_control,INPUT); 
    pinMode(MA, OUTPUT);               
    pinMode(MB, OUTPUT);                
    pinMode(PWM, OUTPUT); 
    pinMode(MA1, OUTPUT);               
    pinMode(MB1, OUTPUT);                
    pinMode(PWM1, OUTPUT);    
    pinMode(Trigger, OUTPUT); 
    pinMode(Echo, INPUT);  
    Serial.begin(9600);
  TCCR0B = TCCR0B & B11111000 | 1;   
  outMax =  255.0;                      
  outMin = -outMax;                    

 //**************************MPU****************************
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); 
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    devStatus = mpu.dmpInitialize();
    // Valores de calibracion
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1688);
    if (devStatus == 0) {
       // Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        // Activar interrupcion
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        
          SampleTime = 5;                      //  tiempo de muestreo en milisegundos.
          K1 =52;
          K2 =80;
          K3 =-50;                
          K4 =-30;
          Ki_angulo =0.35;                          
          Ki_x =0.03;  
       // Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
    }
}
 
 
void loop() {
     
    if (!dmpReady) return;
    // Ejecutar mientras no hay interrupcion
    while (!mpuInterrupt && fifoCount < packetSize) {
      
        activar = digitalRead(activar_control);
   if(activar == 1){
    
        
    double Out = LQR();
  //   Serial.println(Out);
    r_angulo =0;
    r_x=0 ;
 
    if (error == 0.0 && error1 == 0.0 )                     
  {
    digitalWrite(MA, LOW);            
    digitalWrite(MB,LOW);
    digitalWrite(MA1,LOW);            
    digitalWrite(MB1,LOW);          
  }
  else                                  
  {
    pwm = abs(Out);                    

    if (Out > 0.0)                      
    {
      digitalWrite(MB, LOW);   
      digitalWrite(MA,HIGH);  
      analogWrite(PWM, pwm);          
      digitalWrite(MB1, LOW);   
      digitalWrite(MA1,HIGH);  
      analogWrite(PWM1, pwm);           
    }
    else                                
    {
       digitalWrite(MB, HIGH);   
       digitalWrite(MA,LOW);  
       analogWrite(PWM, pwm);           
       digitalWrite(MB1, HIGH);   
       digitalWrite(MA1,LOW);  
       analogWrite(PWM1, pwm);           
    }
  }  
 }
 else {
    digitalWrite(MA, LOW);            
    digitalWrite(MB,LOW);
    digitalWrite(MA1,LOW);            
    digitalWrite(MB1,LOW);
 } 

}
    
 //******************************MPU*********************
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
     //   Serial.println(F("FIFO overflow!"));
    } 
    else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
   mpu.dmpGetQuaternion(&q, fifoBuffer);
   mpu.dmpGetGravity(&gravity,&q);
   mpu.dmpGetYawPitchRoll(ypr,&q, &gravity);
    theta= (ypr[1] * 180/M_PI + 180);
    //Serial.print(millis()/100000.0);
    // Serial.print(',');
    //
  // Serial.println(theta-190.0);
   //  Serial.print(','); 
    }
 //******************************HC-SR04*********************
  digitalWrite(Trigger, LOW);//Inicializamos el pin con 0
  digitalWrite(Trigger, HIGH);
  delayMicroseconds(5);//Enviamos un pulso de 5us
  digitalWrite(Trigger, LOW);
  t = pulseIn(Echo, HIGH);//obtenemos el ancho del pulso
  d = t/59.0;
  //FILTRO DE KALMAN
  Pc = P + VarProcess;
  G = Pc/(Pc + VarSensor);    // kalman gain
  P = (1-G)*Pc;
  Xp = Xe;
  Zp = Xp;
  Xe = G*(d-Zp)+Xp;   // the kalman estimate of the sensor voltage
  //Serial.println(d);    
}

double LQR(void)
{
   unsigned long now = millis();                  
   unsigned long timeChange = (now - lastTime);  

   if(timeChange >= SampleTime)                   
   {
      angulo= theta-191.10;                   
      x  = d-25.0;  
      
     error  = (r_angulo - angulo);          
     
     d_angulo = ( angulo - ultima_entrada_angulo);

      
     error1  = (r_x- x); 
               
     d_x = (x  - ultima_entrada_x);          
  
      I_angulo += (error*Ki_angulo);
      I_x += (error1*Ki_x);
    
    // LEY DE CONTROL 
     double Output = (r_angulo-angulo)*K1 +I_angulo-I_x -d_angulo*K2 +(r_x-x)*K3-d_x*K4;  
    // double Output = (r_angulo   - angulo)*K1 + I_angulo - d_angulo*K2-(r_x- x)*K3-d_x*K4-I_x;    
     if (Output > outMax) Output = outMax; else if (Output < outMin) Output = outMin; 
     
     ultima_entrada_angulo = angulo;                          
     ultima_entrada_x= x;  
     lastTime  = now;  
 
                             
     return Output;   
     
   }
}
