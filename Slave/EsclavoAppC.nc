
#include "EsclavoAppC.h"

configuration EsclavoAppC {
}
implementation {
 
  components MainC;
  components LedsC;
  components EsclavoC as App;
  components new TimerMilliC() as Timer_Ubicacion;
  components ActiveMessageC;
  components new AMSenderC(AM_CANAL);
  components new AMReceiverC(AM_CANAL);

  //Rssi
  components CC2420ActiveMessageC;

  App.Boot -> MainC;
  App.Leds -> LedsC;
  App.Timer_Ubicacion -> Timer_Ubicacion;
  App.Packet -> AMSenderC;
  App.AMPacket -> AMSenderC;
  App.AMControl -> ActiveMessageC;
  App.AMSend -> AMSenderC;
  App.Receive -> AMReceiverC;
  
  //Rssi
  App -> CC2420ActiveMessageC.CC2420Packet;
  
}
