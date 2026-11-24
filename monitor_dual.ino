/*
  Monitor dual, modificacion de la v4 proporcionada. ESP32 Dev Module.
  TFT ST7789 240x280: CS 5, DC 2, RST 4, SCLK 18, MOSI 23.
  MAX30102: SDA 21, SCL 22. MLX90614: SDA 32, SCL 33. Buzzer activo: 25.
  Alimentacion y tierra como en la v4. Sin boton adicional.
  Bibliotecas: Adafruit GFX Library, Adafruit ST7735 and ST7789 Library,
  Adafruit BusIO y SparkFun MAX3010x Pulse and Proximity Sensor Library.
  Monitor serie 115200. Enviar una letra: H humano, C canino, A capturar
  aire, T capturar tejido, M texto, P Plotter. Se aceptan CR y LF.
  Canino: C, retirar tejido, A, esperar indicacion, apoyar tejido quieto,
  T, esperar. Repetir A/T si cambia el sitio o la luz. Calibracion en RAM.
  SpO2 canina experimental sin porcentaje: sin calibracion veterinaria.
  Temperatura superficial del objeto. Uso educativo.
*/
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <algorithm>
#include <math.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"

constexpr int TFT_CS=5,TFT_DC=2,TFT_RST=4,SDA_MAX=21,SCL_MAX=22,SDA_MLX=32,SCL_MLX=33,BUZZER_PIN=25;
constexpr bool BUZZER_ACTIVO=true, MODO_PLOTTER=true;
constexpr int BIP_MS=40, DIV_SALIDA=4, LED_BRILLO=60,PROMEDIO=4,MODO_LED=2,TASA=400,ANCHO_PULSO=411,RANGO_ADC=4096;
constexpr int IR_DEDO_PUESTO=55000,IR_DEDO_QUITADO=45000,AMP_MIN_IR=300;
constexpr int REFRACTARIO_MS=300,INTERVALO_MIN_MS=333,INTERVALO_MAX_MS=1500,CADUCIDAD_MS=3000,N_INTERVALOS=6;
// 250..1500 ms = 40..240 BPM: ventana de busqueda de ingenieria.
constexpr int CAN_MIN_MS=250,CAN_MAX_MS=1500;
constexpr int BUF_LEN=100,DECIM=4,PASO_SPO2=25,SPO2_MIN=70,SPO2_MAX=100,SPO2_MAX_SALTO=6;
constexpr float EMA_SPO2=0.25f;
constexpr int PERIODO_TEMP_MS=500,GX=10,GY=170,GW=220,GH=82,HX=205,HY=76,HR_BASE=8,HR_EXTRA=4;
constexpr uint16_t FONDO=ST77XX_BLACK,REJILLA=0x18E3,MAGENTA_A=0xF81F,NARANJA=0xFD20;
Adafruit_ST7789 tft(TFT_CS,TFT_DC,TFT_RST);
MAX30105 particleSensor;
bool canino=false,plotter=MODO_PLOTTER,maxOk=false,dedo=false,filtroListo=false;
bool marcaLatido=false,buzzerEncendido=false,bpmValido=false,tempValida=false,hayPunto=false;
uint32_t ultimoIntentoMax=0,ultimaUI=0,ultimaAnim=0,ultimaTemp=0,ultimoTexto=0,ultimoLatido=0,instanteLatido=0,buzzerHasta=0,irUltimo=0,ultimaMuestra=0;
float suaveA=0,minA=0,maxA=0,onda=0,bpm=-1,spo2Filtrado=-1,tempC=0;
int mlxEstado=-1,contSalida=0,nIntervalos=0,idxIntervalo=0,nBuf=0,contSpo2=0,nDec=0,rechazosSpo2=0;
uint32_t intervalos[N_INTERVALOS],irBuf[BUF_LEN],redBuf[BUF_LEN],acumIr=0,acumRed=0;
int xGrafica=0,yAnterior=GY+GH/2,bpmPintado=-999,spo2Pintado=-999,temp10Pintada=-999,radioPintado=-1;
uint16_t colorPintado=0;
char estadoPintado[48]="";
enum Cal { PEDIR_AIRE, AIRE, PEDIR_TEJIDO, TEJIDO, LISTO, REPETIR };
Cal etapa=PEDIR_AIRE;
constexpr int N_CAL=400; // 4 segundos por captura
uint32_t captura[N_CAL];
int nCal=0;
float aire95=0,ruido=0,entradaIR=0,salidaIR=0,pisoOnda=0;
float baseCan=0,anteriorCan=0,valleCan=0,picoCan=0;
bool buscandoPico=true;
uint32_t ultimoBueno=0;

void reiniciarBPM(){
  nIntervalos=idxIntervalo=0; ultimoLatido=ultimoBueno=0; bpm=-1; bpmValido=false;
}
void reiniciarMedicion(){
  reiniciarBPM(); spo2Filtrado=-1; nBuf=contSpo2=nDec=rechazosSpo2=0; acumIr=acumRed=0;
  instanteLatido=0; marcaLatido=false; filtroListo=false;
  digitalWrite(BUZZER_PIN,LOW); buzzerEncendido=false;
}
void bip(){
  if(!BUZZER_ACTIVO)return;
  digitalWrite(BUZZER_PIN,HIGH); buzzerEncendido=true; buzzerHasta=millis()+BIP_MS;
}
void gestionarBuzzer(){
  if(buzzerEncendido && int32_t(millis()-buzzerHasta)>=0){
    digitalWrite(BUZZER_PIN,LOW); buzzerEncendido=false;
  }
}
void registrarLatido(uint32_t ahora){
  uint32_t iv=ahora-ultimoLatido;
  if(canino ? (ultimoLatido && iv<CAN_MIN_MS) : iv<=REFRACTARIO_MS)return;
  bool aceptado=false;
  if(ultimoLatido){
    if(iv>=uint32_t(canino?CAN_MIN_MS:INTERVALO_MIN_MS) && iv<=uint32_t(canino?CAN_MAX_MS:INTERVALO_MAX_MS)){
      intervalos[idxIntervalo]=iv; idxIntervalo=(idxIntervalo+1)%N_INTERVALOS;
      if(nIntervalos<N_INTERVALOS)nIntervalos++;
      if(nIntervalos>=3){
        uint32_t c[N_INTERVALOS]; memcpy(c,intervalos,nIntervalos*sizeof(*c));
        std::sort(c,c+nIntervalos);
        float med=0.5f*(c[nIntervalos/2]+c[(nIntervalos-1)/2]);
        float dev=0;
        for(int i=0;i<nIntervalos;i++)dev+=fabsf(float(c[i])-med);
        if(dev/nIntervalos/med<=0.20f){
          float b=60000.0f/med;
          bpm=bpmValido?0.4f*b+0.6f*bpm:b;
          bpmValido=true; ultimoBueno=ahora; aceptado=true;
        }else{ bpmValido=false; bpm=-1; }
      }
    }else if(iv>uint32_t(canino?CAN_MAX_MS:INTERVALO_MAX_MS)){
      nIntervalos=idxIntervalo=0;
      if(canino)bpmValido=false;
    }
  }
  ultimoLatido=ahora;
  if(!canino || aceptado){ instanteLatido=ahora; bip(); marcaLatido=true; }
}
void calcularSpo2(){
  int32_t spo2=0,hr=0; int8_t okSpo2=0,okHr=0;
  maxim_heart_rate_and_oxygen_saturation(irBuf,BUF_LEN,redBuf,&spo2,&okSpo2,&hr,&okHr);
  bool rango=spo2>=SPO2_MIN && spo2<=SPO2_MAX;
  bool estable=spo2Filtrado<0 || abs(int(spo2)-int(lroundf(spo2Filtrado)))<=SPO2_MAX_SALTO;
  if(okSpo2 && rango && estable){
    spo2Filtrado=spo2Filtrado<0?float(spo2):EMA_SPO2*spo2+(1-EMA_SPO2)*spo2Filtrado;
    rechazosSpo2=0;
  }else if(okSpo2 && rango && ++rechazosSpo2>=5){spo2Filtrado=-1; rechazosSpo2=0;}
}
void calibrar(uint32_t ir){
  if(etapa!=AIRE && etapa!=TEJIDO)return;
  captura[nCal++]=ir;
  if(nCal<N_CAL)return;
  std::sort(captura,captura+N_CAL);
  float bajo=captura[N_CAL/20],alto=captura[N_CAL*19/20];
  if(etapa==AIRE){
    aire95=alto; ruido=fmaxf(1,alto-bajo); etapa=PEDIR_TEJIDO;
  }else if(bajo<=aire95+ruido || alto-bajo<=ruido || alto>=262143){
    etapa=REPETIR;
  }else{
    // Histeresis dentro del espacio medido entre aire y tejido.
    float margen=bajo-aire95;
    salidaIR=aire95+margen/3; entradaIR=bajo-margen/3;
    pisoOnda=fmaxf(ruido,(alto-bajo)*0.20f); etapa=LISTO;
  }
  nCal=0;
}
void actualizarDedo(uint32_t ir){
  bool nuevo=dedo;
  if(canino && etapa!=LISTO)nuevo=false;
  else if(dedo && ir<(canino?salidaIR:IR_DEDO_QUITADO))nuevo=false;
  else if(!dedo && ir>(canino?entradaIR:IR_DEDO_PUESTO))nuevo=true;
  if(canino && ir>=262143)nuevo=false;
  if(nuevo!=dedo){dedo=nuevo; reiniciarMedicion();}
}
// Detector canino independiente: elimina continua y exige una excursion
// medida respecto al ruido del montaje. Una cresta por ciclo de histeresis.
bool pulsoCanino(){
  baseCan+=0.018f*(suaveA-baseCan);
  float ac=baseCan-suaveA;
  bool latido=false;
  if(buscandoPico){
    valleCan=fminf(valleCan,ac);
    if(ac-valleCan>=pisoOnda){buscandoPico=false; picoCan=ac;}
  }else{
    picoCan=fmaxf(picoCan,ac);
    if(ac<picoCan-0.5f*pisoOnda && ac<anteriorCan){
      latido=true; buscandoPico=true; valleCan=ac;
    }
  }
  anteriorCan=ac; return latido;
}
void procesarMuestra(uint32_t red,uint32_t ir){
  uint32_t ahora=millis(); irUltimo=ir; ultimaMuestra=ahora;
  if(canino)calibrar(ir);
  actualizarDedo(ir);
  float piso=canino?fmaxf(1,pisoOnda):AMP_MIN_IR;
  if(!filtroListo){
    suaveA=baseCan=ir; minA=suaveA-piso/2; maxA=suaveA+piso/2;
    buscandoPico=true; valleCan=picoCan=anteriorCan=0; filtroListo=true;
  }
  suaveA+=0.2f*(float(ir)-suaveA);
  minA=fminf(suaveA,minA+0.01f*(suaveA-minA));
  maxA=fmaxf(suaveA,maxA+0.01f*(suaveA-maxA));
  float norm=0.5f-(suaveA-(minA+maxA)*0.5f)/fmaxf(maxA-minA,piso);
  onda=dedo?100*constrain(norm,0.0f,1.0f):0;
  if(dedo && (canino?pulsoCanino():checkForBeat((int32_t)ir)))registrarLatido(ahora);
  if(dedo && ultimoLatido && ahora-ultimoLatido>CADUCIDAD_MS)reiniciarBPM();
  if(canino && bpmValido && ahora-ultimoBueno>CADUCIDAD_MS)bpmValido=false;
  if(dedo && !canino){
    acumRed+=red; acumIr+=ir;
    if(++nDec>=DECIM){
      memmove(irBuf,irBuf+1,(BUF_LEN-1)*sizeof(*irBuf));
      memmove(redBuf,redBuf+1,(BUF_LEN-1)*sizeof(*redBuf));
      irBuf[BUF_LEN-1]=acumIr/DECIM; redBuf[BUF_LEN-1]=acumRed/DECIM;
      acumIr=acumRed=0; nDec=0;
      if(nBuf<BUF_LEN)nBuf++;
      if(nBuf>=BUF_LEN && ++contSpo2>=PASO_SPO2){contSpo2=0; calcularSpo2();}
    }
  }
  if(++contSalida>=DIV_SALIDA){
    contSalida=0; graficar(onda/100); if(plotter)salidaSerial();
  }
}
bool iniciarMax(){
  if(!particleSensor.begin(Wire,I2C_SPEED_STANDARD))return false;
  particleSensor.setup(LED_BRILLO,PROMEDIO,MODO_LED,TASA,ANCHO_PULSO,RANGO_ADC);
  ultimaMuestra=millis(); return true;
}
void leerSensor(){
  particleSensor.check();
  while(particleSensor.available()){
    uint32_t red=particleSensor.getFIFORed(),ir=particleSensor.getFIFOIR();
    particleSensor.nextSample(); procesarMuestra(red,ir);
  }
  if(millis()-ultimaMuestra>1000){
    maxOk=false; dedo=false; reiniciarMedicion(); nCal=0;
    if(canino)etapa=PEDIR_AIRE;
  }
}
void escanearBus(TwoWire &bus,const char *nombre){
  Serial.println(nombre);
  for(uint8_t a=1;a<127;a++){
    bus.beginTransmission(a);
    if(bus.endTransmission()==0){Serial.print("  0x");Serial.println(a,HEX);}
  }
}
void escanearI2C(){
  escanearBus(Wire,"MAX30102 esperado 0x57"); escanearBus(Wire1,"MLX90614 esperado 0x5A");
}
int mlxLeer(float &tc){
  Wire1.beginTransmission(0x5A); Wire1.write((uint8_t)0x07);
  if(Wire1.endTransmission(false)!=0 || Wire1.requestFrom((uint8_t)0x5A,(uint8_t)3)!=3)return 1;
  uint8_t lo=Wire1.read(),hi=Wire1.read(); Wire1.read();
  uint16_t raw=(uint16_t(hi)<<8)|lo;
  if(raw&0x8000)return 2;
  tc=raw*0.02f-273.15f; return 0;
}
void leerTemperatura(){
  float t=0; mlxEstado=mlxLeer(t);
  if(!mlxEstado && (t<10 || t>60))mlxEstado=3;
  if(mlxEstado)tempValida=false;
  else{tempC=tempValida?0.3f*t+0.7f*tempC:t; tempValida=true;}
}
void dibujarColumna(int x){
  tft.drawFastVLine(GX+x,GY,GH,x%25==0?REJILLA:FONDO);
  for(int y=22;y<GH;y+=22)tft.drawPixel(GX+x,GY+y,REJILLA);
}
void limpiarGrafica(){
  tft.fillRect(GX,GY,GW,GH,FONDO);
  for(int x=0;x<GW;x+=25)tft.drawFastVLine(GX+x,GY,GH,REJILLA);
  for(int y=22;y<GH;y+=22)tft.drawFastHLine(GX,GY+y,GW,REJILLA);
  xGrafica=0; hayPunto=false;
}
void graficar(float normalizada){
  int y=GY+GH-3-int(normalizada*(GH-6));
  for(int i=0;i<3;i++)dibujarColumna((xGrafica+i)%GW);
  uint16_t color=dedo?MAGENTA_A:REJILLA;
  if(hayPunto && xGrafica)tft.drawLine(GX+xGrafica-1,yAnterior,GX+xGrafica,y,color);
  else tft.drawPixel(GX+xGrafica,y,color);
  yAnterior=y; hayPunto=true; xGrafica=(xGrafica+1)%GW;
}
void dibujarCorazon(int r,uint16_t color){
  tft.fillRect(HX-26,HY-26,52,54,FONDO);
  int cy=HY-r/2;
  tft.fillCircle(HX-r,cy,r,color); tft.fillCircle(HX+r,cy,r,color);
  tft.fillTriangle(HX-2*r,cy,HX+2*r,cy,HX,HY+r+r/2+2,color);
  tft.fillRect(HX-r,cy,2*r,r/2+1,color);
  if(color==ST77XX_RED)tft.fillCircle(HX-r-r/3,cy-r/3,r/4,0xFBEF);
}
void animarCorazon(){
  uint32_t t=millis()-instanteLatido; float s=0;
  if(dedo && instanteLatido){
    if(t<80)s=t/80.0f;
    else if(t<200)s=1-(t-80)/120.0f;
    else if(t<250)s=0.5f*(t-200)/50.0f;
    else if(t<380)s=0.5f*(1-(t-250)/130.0f);
  }
  int r=HR_BASE+int(lroundf(HR_EXTRA*s));
  uint16_t color=dedo?ST77XX_RED:0x4000;
  if(r!=radioPintado || color!=colorPintado){
    dibujarCorazon(r,color); radioPintado=r; colorPintado=color;
  }
}
void dibujarSpo2(int v){
  tft.fillRect(10,106,222,18,FONDO); tft.setCursor(10,108);
  tft.setTextColor(ST77XX_GREEN); tft.setTextSize(canino?1:2);
  if(canino)tft.print("SpO2 experimental | sin validar");
  else{ tft.print("SpO2: "); if(v<0)tft.print("--"); else{tft.print(v);tft.print("%");} }
}
void dibujarTemp(int t10){
  tft.fillRect(10,126,220,25,FONDO); tft.setCursor(10,129); tft.setTextSize(2);
  tft.setTextColor(NARANJA); tft.print("T. superf: ");
  if(t10==-2)tft.print("ERR");
  else if(t10<0)tft.print("--");
  else{tft.print(t10/10); tft.print("."); tft.print(t10%10); tft.write(248); tft.print("C");}
}
const char *estadoActual(){
  if(!maxOk)return "MAX30102 no responde";
  if(canino){
    switch(etapa){
      case PEDIR_AIRE:return "Sensor libre. Envia A";
      case AIRE:return "Midiendo aire... 4 s";
      case PEDIR_TEJIDO:return "Apoya tejido quieto. Envia T";
      case TEJIDO:return "Midiendo tejido... 4 s";
      case REPETIR:return "Sin contraste. Retira y envia A";
      default:break;
    }
    if(!dedo)return "Sin contacto";
    return bpmValido?"Pulso detectado":"Buscando pulso regular";
  }
  if(!dedo)return "Coloca el dedo sobre el sensor";
  return bpmValido?"Leyendo. Manten el dedo quieto":"Midiendo... manten el dedo quieto";
}
void dibujarBase(){
  tft.fillScreen(FONDO); tft.setTextSize(2); tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(12,8); tft.print(canino?"MONITOR | CANINO":"MONITOR | HUMANO");
  tft.drawFastHLine(10,32,220,REJILLA); tft.setCursor(12,42);
  tft.setTextColor(ST77XX_CYAN); tft.print("BPM");
  tft.drawRect(GX-1,GY-1,GW+2,GH+2,ST77XX_WHITE);
  tft.setTextSize(1); tft.setTextColor(0x8410); tft.setCursor(10,264);
  tft.print("MAX30102 + MLX90614 | Educativo");
  limpiarGrafica(); estadoPintado[0]=0; radioPintado=-1;
  bpmPintado=spo2Pintado=temp10Pintada=-999;
}
void actualizarUI(){
  int b=bpmValido?int(lroundf(bpm)):-1;
  if(b!=bpmPintado){
    char txt[12]="-- "; if(b>=0)snprintf(txt,sizeof(txt),"%-3d",b);
    tft.setTextSize(5); tft.setTextColor(ST77XX_CYAN,FONDO);
    tft.setCursor(12,62); tft.print(txt); bpmPintado=b;
  }
  int s=dedo && spo2Filtrado>=0?int(lroundf(spo2Filtrado)):-1;
  if(s!=spo2Pintado){dibujarSpo2(s);spo2Pintado=s;}
  int t=tempValida?int(lroundf(tempC*10)):((mlxEstado==1 || mlxEstado==2)?-2:-1);
  if(t!=temp10Pintada){dibujarTemp(t);temp10Pintada=t;}
  const char *estado=estadoActual();
  if(strcmp(estadoPintado,estado)){
    tft.fillRect(10,154,220,14,FONDO);tft.setCursor(10,157);tft.setTextSize(1);
    tft.setTextColor(!maxOk?ST77XX_RED:(!bpmValido?ST77XX_YELLOW:ST77XX_WHITE));
    tft.print(estado);strcpy(estadoPintado,estado);
  }
}
void salidaSerial(){
  int b=bpmValido?int(lroundf(bpm)):0;
  int s=dedo && spo2Filtrado>=0?int(lroundf(spo2Filtrado)):0;
  float t=tempValida?tempC:0;
  if(plotter){
    char linea[128];
    // Cero = sin dato en Plotter. En canino se omite la serie de saturacion.
    if(canino)snprintf(linea,sizeof(linea),"Pulso:%.1f,BPM:%d,Temp:%.1f,Latido:%d,Techo:130\n",onda,b,t,marcaLatido?100:0);
    else snprintf(linea,sizeof(linea),"Pulso:%.1f,BPM:%d,SpO2:%d,Temp:%.1f,Latido:%d,Techo:130\n",onda,b,s,t,marcaLatido?100:0);
    if(Serial.availableForWrite()>=(int)strlen(linea)){Serial.print(linea);marcaLatido=false;}
  }else{
    Serial.printf("%s IR:%lu Contacto:%d BPM:%d ",canino?"CANINO":"HUMANO",(unsigned long)irUltimo,dedo,b);
    if(canino)Serial.print("SpO2 experimental sin validar");
    else Serial.printf("SpO2:%d",s);
    Serial.printf(" Tsuperf:%.1f MLX:%d | %s\n",t,mlxEstado,estadoActual());
  }
}
void comandos(){
  while(Serial.available()){
    int c=toupper(Serial.read());
    if(c=='M')plotter=false;
    if(c=='P')plotter=true;
    if(c=='H' || c=='C'){
      canino=(c=='C'); dedo=false; etapa=PEDIR_AIRE; nCal=0;
      reiniciarMedicion(); tempValida=false; mlxEstado=-1; onda=0;
      dibujarBase();
    }
    if(canino && c=='A'){
      etapa=AIRE; nCal=0; dedo=false; reiniciarMedicion();
    }
    if(canino && c=='T' && etapa==PEDIR_TEJIDO){etapa=TEJIDO;nCal=0;}
    if(c=='I' && !plotter)escanearI2C();
  }
}
void setup(){
  Serial.begin(115200); pinMode(BUZZER_PIN,OUTPUT); digitalWrite(BUZZER_PIN,LOW);
  SPI.begin(18,-1,23,TFT_CS); tft.init(240,280); tft.setRotation(0);
  tft.setTextWrap(false); tft.cp437(true); dibujarBase(); actualizarUI(); animarCorazon();
  Wire.begin(SDA_MAX,SCL_MAX); Wire.setClock(100000);
  Wire1.begin(SDA_MLX,SCL_MLX); Wire1.setClock(100000);
  if(!plotter)escanearI2C(); maxOk=iniciarMax();
}
void loop(){
  comandos(); uint32_t ahora=millis();
  if(maxOk)leerSensor();
  else if(ahora-ultimoIntentoMax>=2000){ultimoIntentoMax=ahora;maxOk=iniciarMax();}
  gestionarBuzzer();
  if(ahora-ultimaTemp>=PERIODO_TEMP_MS){ultimaTemp=ahora;leerTemperatura();}
  if(ahora-ultimaAnim>=20){ultimaAnim=ahora;animarCorazon();}
  if(ahora-ultimaUI>=50){ultimaUI=ahora;actualizarUI();}
  if(!plotter && ahora-ultimoTexto>=500){ultimoTexto=ahora;salidaSerial();}
  delay(1);
}
