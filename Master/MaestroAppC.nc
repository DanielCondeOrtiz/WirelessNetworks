
#include "MaestroAppC.h"

configuration MaestroAppC {
}
implementation {

  components MainC;
  components LedsC;
  components MaestroC as App;
  components new TimerMilliC() as Timer_Localizacion;
  components new TimerMilliC() as Timer_Ubicacion;
  components new TimerMilliC() as Timer_Cambio_Canal;
  components new TimerMilliC() as Timer_Recoleccion_Datos;
  components ActiveMessageC;
  components new AMSenderC(AM_CANAL);
  components new AMReceiverC(AM_CANAL);
  components HplMsp430GeneralIOC;

  //Rssi
  components CC2420ActiveMessageC;

  //Cambiar canal
  components CC2420ControlC;

  App.Boot -> MainC;
  App.Leds -> LedsC;
  App.Timer_Localizacion -> Timer_Localizacion;
  App.Timer_Ubicacion -> Timer_Ubicacion;
  App.Timer_Cambio_Canal -> Timer_Cambio_Canal;
  App.Timer_Recoleccion_Datos -> Timer_Recoleccion_Datos;
  App.Packet -> AMSenderC;
  App.AMPacket -> AMSenderC;
  App.AMControl -> ActiveMessageC;
  App.AMSend -> AMSenderC;
  App.Receive -> AMReceiverC;

  //Rssi
  App -> CC2420ActiveMessageC.CC2420Packet;

  //Cambiar canal
  App -> CC2420ControlC.CC2420Config;

  //Com Arduino
  App.Pin2 -> HplMsp430GeneralIOC.Port26;
  App.Pin1 -> HplMsp430GeneralIOC.Port23;
  App.enablePin ->HplMsp430GeneralIOC.ADC3;

}
