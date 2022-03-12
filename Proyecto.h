

#ifndef PROYECTO_h
#define PROYECTO_h

#include "printf.h"
#include <Timer.h>
#include <math.h>
#include <UserButton.h>
enum {

	ID_MAESTRO=11,
	//Canal radio selecionado para la comunicacion
	AM_CANAL = 26,
	AM_CANAL_MOVIL=26,
	// Muestras necesarias para localizacion
	MUESTRAS=5,
  NUM_REINTETOS=10,
  //Tiempo entre solicitud de datos
  TIMER_PERIOD_MILLI_DATA = 1000,

  //Ajustar en funcion del numero de nodos anclas TIMER_PERIOD_MILLI_CHANGE_CHANNEL=TIMER_SLOT_CHANGE_CHANNEL*(NUM_NODO_ANCLA+2)
  
  TIMER_PERIOD_MILLI_CHANGE_CHANNEL = 500,
  TIMER_SLOT_CHANGE_CHANNEL=50,

   /* El tiempo del temporidor es 5 veces el de los esclavo para permitir tener 
	 ===================================================================================
	 | Slot0(Ancla1)| Slot1(Ancla2) | Slot2(Ancla3) | Slot3(Ancla4) |  Slot4(Calculos) | 
	 ===================================================================================
	 La formula para calcularlo seria TIMER_PERIOD_MILLI_ANCLA*(NUM_NODO_ANCLA+1)
 */
  	//Ajsutar tiempo por si fueran muy corto para los calculos necesarios probazd *10
  TIMER_PERIOD_MILLI_LOCALIZACION = 500,
  /*
  	Es el tiempo que se usara en los nodos ancla para enviar un mensaje
	devuelta tras soliciar el robot-maestro el envio de rssi para calculo de la posicion	
  */
  TIMER_PERIOD_MILLI_ANCLA=100,

  /* 
   La formula para calcularlo seria TIMER_PERIOD_MILLI_UBICACION_MAESTRO=TIMER_PERIOD_MILLI_UBICACION_ESCLAVO*(NUM_NODO_ESCLAVO+1)
  */
  
  //Ajsutar tiempo por si fueran muy corto para los calculos necesarios probar *10
  TIMER_PERIOD_MILLI_UBICACION_MAESTRO = 75,
  TIMER_PERIOD_MILLI_UBICACION_ESCLAVO=25 ,
  
  NUM_NODO_ANCLA=4,
  NUM_NODO_ESCLAVO=2,
  NUM_COORDENADAS=2,
  
  ID_ANCLA_1=1,
  ID_ANCLA_2=2,
  ID_ANCLA_3=3,
  ID_ANCLA_4=4,

  X_1 = 0, Y_1 = 0,
  X_2 = 3, Y_2 = 0,
  X_3 = 0, Y_3 = 5,
  X_4 = 3, Y_4 = 5,
  /*
  3--------------------4
  |           R        |		+ nodo esclvo
  |  [+]           [+] |
  |                    |		[] rango
  |                    |
  |     [+]       [+]  |		R nodo maestro-robot
  1--------------------2
  */

 	//Se usara para no tener que estar justamente encima del nodo
  RANGO=10
};



typedef nx_struct Ancla_Localizacion_Msg {
  nx_uint16_t nodeid;
  //Solo sera util en el robot los demas no deberan hacer la media
  nx_int16_t rssi;

} Ancla_Localizacion_Msg;


typedef nx_struct TMDA_Localizacion_Msg {
  nx_uint16_t nodeid;
  nx_uint16_t orden_TDMA [NUM_NODO_ANCLA];
} TMDA_Localizacion_Msg;

typedef nx_struct TMDA_Ubicacion_Msg {
  nx_uint16_t nodeid;
  nx_uint16_t orden_TDMA [NUM_NODO_ESCLAVO];
  nx_uint8_t relleno; //Nos permitira diferenciar los mensajes si NUM_NODO_ESCLAVO==NUM_NODO_ANCLA 


} TMDA_Ubicacion_Msg;

typedef nx_struct TMDA_Ubicacion_Esclavo_Msg {
  nx_uint16_t nodeid;
  nx_float x;
  nx_float y;
} TMDA_Ubicacion_Esclavo_Msg;

typedef nx_struct Cambio_Canal_Msg {
  nx_uint16_t nodeid;
  nx_uint8_t canal;
} Cambio_Canal_Msg;

typedef nx_struct Datos {
   nx_uint16_t datos;
} Datos;

#endif
