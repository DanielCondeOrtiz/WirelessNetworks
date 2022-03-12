

#include "NodoAnclaAppC.h"

configuration NodoAnclaAppC {
}
implementation {
  components MainC;
  components LedsC;
  components  NodoAnclaC as App;
  components new TimerMilliC() as Timer_Localizacion;
  components new TimerMilliC() as Timer_Cambio_Canal;
  components ActiveMessageC;
  components new AMSenderC(AM_CANAL);
  components new AMReceiverC(AM_CANAL);

  //Rssi
  components CC2420ActiveMessageC;
  
  //Cambiar canal
  components CC2420ControlC; 
  
  App.Boot -> MainC;
  App.Leds -> LedsC;
  App.Timer_Localizacion -> Timer_Localizacion;
    App.Timer_Cambio_Canal -> Timer_Cambio_Canal;
  App.Packet -> AMSenderC;
  App.AMPacket -> AMSenderC;
  App.AMControl -> ActiveMessageC;
  App.AMSend -> AMSenderC;
  App.Receive -> AMReceiverC;

  //Rssi
  App -> CC2420ActiveMessageC.CC2420Packet;
  
  //Cambiar canal
  App -> CC2420ControlC.CC2420Config;
}
