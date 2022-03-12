
#include "NodoAnclaAppC.h"

module NodoAnclaC {
  uses interface Boot;
  uses interface Leds;
  uses interface Timer<TMilli> as Timer_Localizacion;
  uses interface Timer<TMilli> as Timer_Cambio_Canal;
  uses interface Packet;
  uses interface AMPacket;
  uses interface AMSend;
  uses interface Receive;
  uses interface SplitControl as AMControl;
  uses interface CC2420Packet;
  uses interface CC2420Config;
}

implementation {

//-----------Funciones----------//
  int16_t getRssi(message_t *msg);

  void receive_Cambio_Canal_Msg(message_t* msg, void* payload);

//-----------Variabless----------//
  message_t pkt;
  bool busy = FALSE;

  uint16_t turno = 0;

  uint8_t rssi; // Se extrae en 8 bits sin signo
  int16_t rssi2; // Se calcula en 16 bits con signo: la potencia recibida estará entre -10 y -90 dBm

  uint16_t orden_TDMA_ancla [NUM_NODO_ANCLA] = {0,0,0,0}; //Se sustituira al recibir el mensaje el nodo robot-maestro

  uint16_t ID_ack_cambio_canal[NUM_NODO_ANCLA]={0,0,0,0};

  uint8_t canal_deseado;

  bool send_cambio_canal=FALSE;

  void setLeds(uint16_t val) {
    if (val & 0x01)
      call Leds.led0On();
    else 
      call Leds.led0Off();
    if (val & 0x02)
      call Leds.led1On();
    else
      call Leds.led1Off();
    if (val & 0x04)
      call Leds.led2On();
    else
      call Leds.led2Off();
  }




event void Boot.booted() {

    call AMControl.start();
    setLeds(4);
}

event void AMControl.startDone(error_t err) {
    if (err == SUCCESS) {
      //call Timer_Localizacion.startPeriodic(TIMER_PERIOD_MILLI_ANCLA);
    }
    else {
      call AMControl.start();
    }
  }

event void AMControl.stopDone(error_t err) {
  }

event void CC2420Config.syncDone( error_t error ) {

  //Meter variable seleccionar pata cambiar entre AM_CANAL_MOVIL AM_CANAL
  if (error==SUCCESS)
  {
    
  }
  else{
    
    call CC2420Config.setChannel(canal_deseado);//Esto no trampea 
    
  }
}

event void Timer_Localizacion.fired() {

    if(orden_TDMA_ancla[turno] == TOS_NODE_ID && !busy){

      Ancla_Localizacion_Msg* men_send = (Ancla_Localizacion_Msg*)(call Packet.getPayload(&pkt, sizeof(Ancla_Localizacion_Msg)));
      setLeds(0x02);

      if (men_send == NULL) {
        return;
      }

      men_send->nodeid = TOS_NODE_ID;
      men_send->rssi = rssi2;

      if (call AMSend.send(AM_BROADCAST_ADDR,&pkt, sizeof(Ancla_Localizacion_Msg)) == SUCCESS) {
        busy = TRUE;
      }
    }

    if (turno>=NUM_NODO_ANCLA){
      call Timer_Localizacion.stop();
    }

    turno = turno + 1;
}

event void Timer_Cambio_Canal.fired() {
  Cambio_Canal_Msg* men_send = (Cambio_Canal_Msg*)(call Packet.getPayload(&pkt, sizeof(Cambio_Canal_Msg)));
  int i=0;
  bool cambio_canal=TRUE;

  if(orden_TDMA_ancla[turno] == TOS_NODE_ID && !busy){
    setLeds(7);
      for(i=0;i<NUM_NODO_ANCLA;i++)
        {
          if(ID_ack_cambio_canal[i]==0){
            cambio_canal=FALSE;
          }

        }
      if(cambio_canal==TRUE && send_cambio_canal==TRUE){
          setLeds(0);

          for(i=0;i<NUM_NODO_ANCLA;i++)
            {
              ID_ack_cambio_canal[i]=0;
            }
          men_send->nodeid=TOS_NODE_ID;
          men_send->canal=canal_deseado;
          printf("Enviado mensaje cambio de canal\n");
          printfflush();
            if (call AMSend.send(AM_BROADCAST_ADDR, 
            &pkt, sizeof(Cambio_Canal_Msg)) == SUCCESS) {
            busy = TRUE;
            }
          call CC2420Config.setChannel(canal_deseado);
          call CC2420Config.sync();
           send_cambio_canal=FALSE;
          //Deberia ser el ultimo mensaje

          call Timer_Cambio_Canal.stop();

          }else {
          men_send->nodeid=TOS_NODE_ID;
          men_send->canal=canal_deseado;
          printf("Enviado mensaje cambio de canal\n");
          printfflush();
          if (call AMSend.send(AM_BROADCAST_ADDR, 
            &pkt, sizeof(Cambio_Canal_Msg)) == SUCCESS) {
            busy = TRUE;
            }
          send_cambio_canal=TRUE;
      } 
  }
  turno = turno + 1;
  
  if(turno>=NUM_NODO_ANCLA ){
    turno=0;
  }
}


event void AMSend.sendDone(message_t* msg, error_t err) {
  if (&pkt == msg) {
    busy = FALSE;
  }


}

event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len){
   int i=0;
   setLeds(0x01);
    if (len == sizeof(TMDA_Localizacion_Msg)) {
      TMDA_Localizacion_Msg* men_revc = (TMDA_Localizacion_Msg*) payload;
      getRssi(msg);
        for(i=0;i<NUM_NODO_ANCLA;i++){
          orden_TDMA_ancla[i]=men_revc->orden_TDMA[i];
        }

      if(turno >= NUM_NODO_ANCLA){
        turno=0;
      }
      //Activamos el temporizador solo cuando lo usamos
       call Timer_Localizacion.startPeriodic(TIMER_PERIOD_MILLI_ANCLA);
     }
      if (len == sizeof(Cambio_Canal_Msg)) {
        receive_Cambio_Canal_Msg(msg, payload);
         
     }
   return msg;
}
// Lectura de rssi
int16_t getRssi(message_t *msg){
    rssi=call CC2420Packet.getRssi(msg);
    if(rssi>=128){
      rssi2=rssi-45-256;
    }
    else{
      rssi2=rssi-45;
    }

    return rssi2;
}

void receive_Cambio_Canal_Msg(message_t* msg, void* payload){
  int i=0;
  Cambio_Canal_Msg* men_revc = (Cambio_Canal_Msg*) payload;

  if ( call AMPacket.source(msg)==ID_MAESTRO){
   canal_deseado=men_revc->canal;
      call Timer_Cambio_Canal.startPeriodic(TIMER_SLOT_CHANGE_CHANNEL);
    }

  if ( call Timer_Cambio_Canal.isRunning()){
    
    for(i=0;i<NUM_NODO_ANCLA;i++){
        if(men_revc->nodeid==ID_ack_cambio_canal[i]){
         break;
        }else if(ID_ack_cambio_canal[i]==0){
          printf(" nodeid canal %u\n",men_revc->nodeid);
           printfflush();
          ID_ack_cambio_canal[i]=men_revc->nodeid;
         break;
        }   
    }
  }
}

}


//Tengo hecho hasta que el robot de empieza a mover 
//Deberia poder cambiar de canal al recibir un tipo de mensaje