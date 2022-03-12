
#include "EsclavoAppC.h"

module EsclavoC {
	uses interface Boot;
	uses interface Leds;
	uses interface Timer<TMilli> as Timer_Ubicacion;
	uses interface Packet;
	uses interface AMPacket;
	uses interface AMSend;
	uses interface Receive;
	uses interface SplitControl as AMControl;
	uses interface CC2420Packet;
}
implementation {

//-----------Funciones----------//
	//Procesa la recepcion del tipo de mensaje determinado
	void receive_Datos(message_t* msg, void* payload);

	void receive_TMDA_Ubicacion(message_t* msg, void* payload);

	//Funcion para imprimir datos de tipo entero
	void printfFloat(float floatToBePrinted);

	//Calculamo la posicion respecto a los odos ancla
	void calculatePosition();

	// Lectura de rssi
	int16_t getRssi(message_t *msg);

	//Procesa la recepcion de este tipo de mensaje
	void receiveAncla_Localizacion(message_t* msg, void* payload);

	//Calculamos la mediana
	float median(int lenght, float x[]);

	
//-----------Variabless----------//
 	message_t pkt;
	bool busy = FALSE;

	uint16_t turno = 0;

	uint8_t rssi; // Se extrae en 8 bits sin signo
	int16_t rssi2; // Se calcula en 16 bits con signo: la potencia recibida estará entre -10 y -90 dBm

	uint16_t orden_TDMA_ancla [NUM_NODO_ANCLA] = {ID_ANCLA_1,ID_ANCLA_2,ID_ANCLA_3,ID_ANCLA_4};

	float POS_NODOS_X [NUM_NODO_ANCLA] = {X_1,X_2,X_3,X_4};
	float POS_NODOS_Y [NUM_NODO_ANCLA] = {Y_1,Y_2,Y_3,Y_4};

	float distancia_nodos [NUM_NODO_ANCLA] = {0,0,0,0};
	float distancia_nodos_muestas [NUM_NODO_ANCLA][MUESTRAS];
	float posicion[NUM_COORDENADAS] = {0,0};

	uint8_t num_muestras=MUESTRAS-1;

	uint16_t orden_TDMA_esclavos [NUM_NODO_ESCLAVO] = {0};



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
}

event void Timer_Ubicacion.fired() {
  	if(orden_TDMA_esclavos[turno] == TOS_NODE_ID && !busy){

		TMDA_Ubicacion_Esclavo_Msg* men_send = (TMDA_Ubicacion_Esclavo_Msg*)(call Packet.getPayload(&pkt, sizeof(TMDA_Ubicacion_Esclavo_Msg)));
	
		setLeds(0x07);

		men_send->nodeid = TOS_NODE_ID;
		men_send->x = posicion[0];
		men_send->y = posicion[1];

		if(call AMSend.send(ID_MAESTRO, 
		    &pkt, sizeof(TMDA_Ubicacion_Esclavo_Msg)) == SUCCESS){

		    busy = TRUE;
		}
	}

	if(turno>=NUM_NODO_ESCLAVO){

	call Timer_Ubicacion.stop();
	}

  turno = turno + 1;
}

event void AMControl.startDone(error_t err) {
	if(err == SUCCESS) {
		
		}else {
	
		call AMControl.start();
	}
}

event void AMControl.stopDone(error_t err) {
}

event void AMSend.sendDone(message_t* msg, error_t err) {
	if (&pkt == msg) {

		busy = FALSE;
	}
}

event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len){

	setLeds(7);

	if (len == sizeof(Ancla_Localizacion_Msg)) {

	receiveAncla_Localizacion(msg,payload);
	}
	if (len == sizeof(TMDA_Ubicacion_Msg)){

	receive_TMDA_Ubicacion(msg,payload);
 	}

	if(len == sizeof(Datos)){
 	
 	setLeds(0);
 	
 	receive_Datos(msg,payload);
	}

	return msg;
}

void receiveAncla_Localizacion(message_t* msg, void* payload){

	int i=0;
	Ancla_Localizacion_Msg* men_revc = (Ancla_Localizacion_Msg*) payload;
	int16_t rssiMedido = ((float) getRssi(msg) ) ;
	float exponente = (rssiMedido + 1.678)/(-10.302);
	float distancia = powf(10, exponente);

    //Guardamos la distancia en el vector de distancias
	for(i=0; i< NUM_NODO_ANCLA; i=i+1){
		


		if(men_revc->nodeid==orden_TDMA_ancla[i]){ 

			//Deberiamos tener al menos 10 muestrass y hacer la media
			distancia_nodos_muestas[i][num_muestras]=distancia;
		}
	}

	setLeds(0x01);
	//Final del TDMA de localizacion
	if(orden_TDMA_ancla[NUM_NODO_ANCLA-1]==men_revc->nodeid){
printf("tdma");
printfflush();
		setLeds(0x04);
		
		if(num_muestras<=0){
printf("1");
printfflush();
			for(i=0; i< NUM_NODO_ANCLA; i=i+1){
				//Calculo la media de 10 calculo de distancia calculcula con el rssi
				distancia_nodos[i]=median(MUESTRAS,distancia_nodos_muestas[i]);
			}
											
	printf("1");
	printfFloat( distancia_nodos[0]);

	printfflush();								
	printf("2");
	printfFloat( distancia_nodos[1]);
	printfflush();								
	printf("3");
	printfFloat( distancia_nodos[2]);
	printfflush();								
	printf("4");
	printfFloat( distancia_nodos[3]);
	printfflush();

			//Una vez tenemos todas las distancias, triangulamos
			if(distancia_nodos[0] != 0 && distancia_nodos[1] != 0 
				&& distancia_nodos[2] != 0  && distancia_nodos[3] != 0){
		 	
		 		 calculatePosition();
			}
			}else{
			num_muestras--;
		}	
	}
}

void receive_Datos(message_t* msg, void* payload){

	//Ver que datos a enviar
	Datos* men_send = (Datos*)(call Packet.getPayload(&pkt, sizeof(Datos)));
  	men_send->datos=TOS_NODE_ID*100;
  	if (call AMSend.send(ID_MAESTRO, 
  		&pkt, sizeof(Datos)) == SUCCESS) {
  		busy = TRUE;
 	}
}

void receive_TMDA_Ubicacion(message_t* msg, void* payload){
	
	int i=0;
	TMDA_Ubicacion_Msg* men_revc = (TMDA_Ubicacion_Msg*) payload;
		
	for(i=0;i<NUM_NODO_ANCLA;i++){

	  orden_TDMA_esclavos[i]=men_revc->orden_TDMA[i];
	}
	
	turno=0;
    //Activamos el temporizador solo cuando lo usamos
	 call Timer_Ubicacion.startPeriodic(TIMER_PERIOD_MILLI_UBICACION_ESCLAVO);
}

void printfFloat(float floatToBePrinted) {
	uint32_t fi, f0, f1, f2, f3;
	char c;
	float f = floatToBePrinted;

	if (f<0){
      c = '-';    // Añade signo negativo
      f = -f;     // Invertir signo al flotante
  } else {
      c = ' ';    // Añade signo "Positivo"
  }

    // Obtener parte entera
  fi = (uint32_t) f;

    // Parte decimal (4 decimales)
    f  = f - ((float) fi);          // Restar parte entera
    f0 = f*10;    f0 %= 10;
    f1 = f*100;   f1 %= 10;
    f2 = f*1000;  f2 %= 10;
    f3 = f*10000; f3 %= 10;
    printf("%c%ld.%d%d%d%d", c, fi, (uint8_t) f0, (uint8_t) f1, (uint8_t) f2, (uint8_t) f3);
    printfflush();
}

void calculatePosition(){

	int i; 
	//Se puede modificar la p
	float p=2;

	float sumwij =0;
	float sumMultiX = 0;
	float sumMultiY = 0;

	for(i=0; i<NUM_NODO_ANCLA;i=i+1){
		float wij = 1/(powf(distancia_nodos[i],p));
		distancia_nodos[i]=0;
		sumwij = sumwij + wij;

		sumMultiX = sumMultiX + (wij*POS_NODOS_X[i]);
		sumMultiY = sumMultiY + (wij*POS_NODOS_Y[i]);
	}

	//Calculo la posicion x
	posicion[0] = sumMultiX/sumwij;
	//Calculo la posicion y
	posicion[1] = sumMultiY/sumwij; 

	printf("Posicion: (");
	printfFloat( posicion[0]);
	printf(",");
	printfFloat( posicion[1]);
	printf(")\n");
	printfflush();


	num_muestras=MUESTRAS-1;
}


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


float median(int lenght, float x[]) {

    float temp;
    int i, j;

    for(i=0; i<lenght-1; i++) {
        for(j=i+1; j<lenght; j++) {
            if(x[j] < x[i]) {
                temp = x[i];
                x[i] = x[j];
                x[j] = temp;
            }
        }
    }

    if(lenght%2==0) {

        return((x[lenght/2] + x[lenght/2 - 1]) / 2.0);
    	} else {

        return x[lenght/2];
    }
}

}

/*Solo queda ver los datos a comunicarse*/