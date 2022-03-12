
#include "MaestroAppC.h"

module MaestroC {
	uses interface Boot;
	uses interface Leds;
	uses interface Timer<TMilli> as Timer_Localizacion;

	//Se usara para saber la posicion de todos los nodos
	uses interface Timer<TMilli> as Timer_Ubicacion;

	uses interface Timer<TMilli> as Timer_Cambio_Canal;

	uses interface Timer<TMilli> as Timer_Recoleccion_Datos;
	uses interface Packet;
	uses interface AMPacket;
	uses interface AMSend;
	uses interface Receive;
	uses interface SplitControl as AMControl;
	uses interface CC2420Packet;
	uses interface CC2420Config;

		uses interface HplMsp430GeneralIO as Pin1;
		uses interface HplMsp430GeneralIO as Pin2;
		uses interface HplMsp430GeneralIO as enablePin;
}
implementation {

//-----------Funciones----------//

	//Procesa la recepcion del tipo de mensaje determinado
	void receiveAncla_Localizacion(message_t* msg, void* payload);

	void receiveUbicacion_Esclavo(message_t* msg,void* payload);

	void receive_Cambio_Canal_Msg(message_t* msg, void* payload);

	void receive_Datos(message_t* msg, void* payload);

	//Funcion para imprimir datos de tipo entero
	void printfFloat(float floatToBePrinted);

	//Obtenemos el rssi del mensaje recibido.
	int16_t getRssi(message_t *msg);

	//Calculamo la posicion respecto a los odos ancla
	void calculatePosition();

	//Calcula cual es el nodo mas cercano mediante el modulo
	void calculateNodeNearest();

	//Calculamos la direccion a movernos
	void calculateDirection();

	//Calculamos la mediana
	float median(int lenght, float x[]);

	//Para mover el robot
	void moverRobot(int mover);

//-----------Variabless----------//
	message_t pkt;
 	bool busy = FALSE;
 	bool otro_nodo=FALSE;
  	uint8_t rssi; // Se extrae en 8 bits sin signo
  	int16_t rssi2; // Se calcula en 16 bits con signo: la potencia recibida estará entre -10 y -90 dBm

  	uint16_t orden_TDMA_ancla [NUM_NODO_ANCLA] = {ID_ANCLA_1,ID_ANCLA_2,ID_ANCLA_3,ID_ANCLA_4};

	float POS_NODOS_X [NUM_NODO_ANCLA] = {X_1,X_2,X_3,X_4};
	float POS_NODOS_Y [NUM_NODO_ANCLA] = {Y_1,Y_2,Y_3,Y_4};

	float distancia_nodos [NUM_NODO_ANCLA] = {0,0,0,0};
	float distancia_nodos_muestas [NUM_NODO_ANCLA][MUESTRAS];
	float posicion[NUM_COORDENADAS] = {0,0};

	uint8_t num_muestras=MUESTRAS-1;

	//Rellenar convenientemente segun se configuren los escavlos y cuantos haya y cambiar valores timer en la libreria

	uint16_t orden_TDMA_Esclavos [NUM_NODO_ESCLAVO] = {9,8};

	float X_Esclavos[NUM_NODO_ESCLAVO]={-1,-1};
	float Y_Esclavos[NUM_NODO_ESCLAVO]={-1,-1};

 	uint8_t counter=0;
	uint16_t IDNodoCercano=0;
	uint8_t IncideNodoCercano=-1;
	float posicion_Cercano[2] = {0,0};

	float X_Direccion=0;
	float Y_Direccion=0;

	uint16_t ID_ack_cambio_canal[NUM_NODO_ANCLA]={0,0,0,0};

	uint8_t channelWanted;
	uint8_t channelCurrent=AM_CANAL;

	bool backToStart=FALSE;
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

event void Timer_Localizacion.fired() {

  	int i=0;
  	TMDA_Localizacion_Msg* men_send = (TMDA_Localizacion_Msg*)(call Packet.getPayload(&pkt, sizeof(TMDA_Localizacion_Msg)));

  	if (men_send == NULL) {
  		return;
  	}

  	men_send->nodeid = TOS_NODE_ID;
  	for(i=0;i<NUM_NODO_ANCLA;i++){

  		men_send->orden_TDMA[i]=orden_TDMA_ancla[i];
  	}

  	setLeds(2);
  	printf("Mensaje TDMA ancla %u\n",num_muestras);
  	printfflush();
  	if (call AMSend.send(AM_BROADCAST_ADDR,
  		&pkt, sizeof(TMDA_Localizacion_Msg)) == SUCCESS) {
  		busy = TRUE;
  }
}

event void Timer_Ubicacion.fired() {

  	int i=0;
  	TMDA_Ubicacion_Msg* men_send = (TMDA_Ubicacion_Msg*)(call Packet.getPayload(&pkt, sizeof(TMDA_Ubicacion_Msg)));

  	if (men_send == NULL) {
  		return;
  	}

  	men_send->nodeid=TOS_NODE_ID;
  	for(i=0;i<NUM_NODO_ESCLAVO;i++){

  		men_send->orden_TDMA[i]=orden_TDMA_Esclavos[i];
  	}

  	setLeds(2);
  	printf("Mensaje TDMA esclavo\n");
  	printfflush();
  	if (call AMSend.send(AM_BROADCAST_ADDR,
  		&pkt, sizeof(TMDA_Ubicacion_Msg)) == SUCCESS) {
  		busy = TRUE;
  }
}

event void Timer_Cambio_Canal.fired() {

	int i=0;
	Cambio_Canal_Msg* men_send = (Cambio_Canal_Msg*)(call Packet.getPayload(&pkt, sizeof(Cambio_Canal_Msg)));
	bool cambio_canal=TRUE;
	if (men_send == NULL) {
  		return;
  	}



	setLeds(7);
	printf("Timer_Cambio_Canal\n");
	printfflush();

	for(i=0;i<NUM_NODO_ANCLA;i++){

  		if(ID_ack_cambio_canal[i]==0){

  			cambio_canal=FALSE;
  		}
  	}

  	if(cambio_canal==TRUE){

  		for(i=0;i<NUM_NODO_ANCLA;i++){

  			ID_ack_cambio_canal[i]=0;
  		}

  		setLeds(0);
  		men_send->nodeid=TOS_NODE_ID;
  		men_send->canal=channelWanted;

  		if (call AMSend.send(AM_BROADCAST_ADDR,
  			&pkt, sizeof(Cambio_Canal_Msg)) == SUCCESS) {
  			busy = TRUE;
  		}

	  	call CC2420Config.setChannel(channelWanted);
	  	call CC2420Config.sync();
	  	call Timer_Cambio_Canal.stop();
	}else{

		men_send->nodeid=TOS_NODE_ID;
		men_send->canal=channelWanted;

		printf("Enviado mensaje cambio de canal %u\n",channelWanted);
		printfflush();

		if (call AMSend.send(AM_BROADCAST_ADDR,
  		&pkt, sizeof(Cambio_Canal_Msg)) == SUCCESS) {
  		busy = TRUE;
  		}
	}
}

event void Timer_Recoleccion_Datos.fired() {
    if(counter<NUM_REINTETOS){
    	Datos* men_send = (Datos*)(call Packet.getPayload(&pkt, sizeof(Datos)));

		if (men_send == NULL) {
	  		return;
	  	}

		men_send->datos=0;

	  	//printf("Pidiendo datos de %u reintento: %u \n",IDNodoCercano,counter);
	  	////printfflush();
	  	//counter++;
	  	//Seleccionamos una baja potencia de transmision
	  	call CC2420Packet.setPower(&pkt,10);

	  	if (call AMSend.send(IDNodoCercano,
	  		&pkt, sizeof(Datos)) == SUCCESS) {
	  		busy = TRUE;
	  	}

    }else{
    	//Si superamos el numero de reintento pasamos al siguiente nodo
	    call Timer_Recoleccion_Datos.stop();
		//Calculamos el siguiente nodo
		calculateNodeNearest();
		channelWanted=AM_CANAL_MOVIL;
		counter=0;
		call CC2420Config.setChannel(channelWanted);
		call CC2420Config.sync();
    }

}

event void AMControl.startDone(error_t err) {

	if (err == SUCCESS) {
		call Timer_Localizacion.startPeriodic(TIMER_PERIOD_MILLI_LOCALIZACION);
	}
	else {
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

event void CC2420Config.syncDone( error_t error ) {

	if (error==SUCCESS){

		num_muestras=MUESTRAS-1;//Quitar o no

		//printf("Cambiado al canal %u\n",channelWanted);
		////printfflush()//;

		channelCurrent=channelWanted;

		if(channelCurrent==AM_CANAL && !otro_nodo){
			if(backToStart){

				posicion_Cercano[0]=0;
				posicion_Cercano[1]=0;
				backToStart=FALSE;

				//printf("Volvemos al inicio de flujo\n");
				////printfflush();
				//call Timer_Localizacion.startPeriodic(TIMER_PERIOD_MILLI_LOCALIZACION); //Ver la posibilidad de un ajuste de la velocidad

				}else{
				//Diferencial entre dato o inicio
				call Timer_Recoleccion_Datos.startPeriodic(TIMER_PERIOD_MILLI_DATA);
			}
			}else{

			call Timer_Localizacion.startPeriodic(TIMER_PERIOD_MILLI_LOCALIZACION); //Ver la posibilidad de un ajuste de la velocidad
			}
		}else{

		call CC2420Config.setChannel(channelWanted);//AM_CANAL_MOVIL
		call CC2420Config.sync();
	}
}

event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len){

	if (len == sizeof(Ancla_Localizacion_Msg)) {

		receiveAncla_Localizacion(msg,payload);
	}

	if (len == sizeof(TMDA_Ubicacion_Esclavo_Msg)) {

		receiveUbicacion_Esclavo(msg,payload);
	}

	if(len == sizeof(Cambio_Canal_Msg)){

		receive_Cambio_Canal_Msg(msg,payload);
	}

	if(len == sizeof(Datos)){

		receive_Datos(msg,payload);
	}

	return msg;
}

void receiveAncla_Localizacion(message_t* msg, void* payload){
	int i=0;
	Ancla_Localizacion_Msg* men_revc = (Ancla_Localizacion_Msg*) payload;
	int16_t rssiMedido = getRssi(msg);
	int16_t rssiMaestro = men_revc->rssi;
	float rssiMedio = ((float) rssiMedido + (float) rssiMaestro)/2;
	float exponente = (rssiMedio + 1.678)/(-10.302);
	float distancia = powf(10, exponente);

	//printf("Mensaje recibido del ancla %u \n",men_revc->nodeid);
	//printfflush();
	//Guardamos la distancia el vector de distancias
	for(i=0; i< NUM_NODO_ANCLA; i=i+1){
		if(men_revc->nodeid==orden_TDMA_ancla[i]){
			//Deberiamos tener al menos 10 muestrass y hacer la media
				distancia_nodos_muestas[i][num_muestras]=distancia;
			}
		}

	setLeds(0x01);

	//Final del TDMA de localizacion
	if (orden_TDMA_ancla[NUM_NODO_ANCLA-1]==men_revc->nodeid){
		if (num_muestras<=0){

			for(i=0; i< NUM_NODO_ANCLA; i=i+1){
				//Calculo la mediana de las 10 muestras de distancia calculcula con el rssi
				distancia_nodos[i]=median(MUESTRAS,distancia_nodos_muestas[i]);
			}

			//Una vez tenemos todas las distancias, triangulamos
			if(distancia_nodos[0] != 0 && distancia_nodos[1] != 0
				&& distancia_nodos[2] != 0  && distancia_nodos[3] != 0){

	     		calculatePosition();
		     	 //Comprobamos que no estamos en la rutina de movimiento
				if(posicion_Cercano[0]==0 && posicion_Cercano[1]==0){

		     	 	//Dejamo de enviar el mensaje TMDA_Localizacion_Msg
		     	 	call Timer_Localizacion.stop();
     				//Comenezamos el flujo de cambio de canal
     				call  Timer_Ubicacion.startPeriodic(TIMER_PERIOD_MILLI_UBICACION_MAESTRO);
					}else{

					calculateDirection();
					//Comprobaremos que estamos dentro del rango/10 de comunicacion con el nodo
					//Para eso calcularemos el modulo del vector que nos marca la direccion y sentido
					if (IDNodoCercano==0){

						if (channelCurrent==AM_CANAL_MOVIL){

							backToStart=TRUE;
							num_muestras=MUESTRAS-1;
							channelWanted=AM_CANAL;
							call Timer_Localizacion.stop();
							call Timer_Cambio_Canal.startPeriodic(TIMER_PERIOD_MILLI_CHANGE_CHANNEL);
						}
						}else{
						if (sqrtf(powf(X_Direccion,2)+powf(Y_Direccion,2))<=RANGO/10){
							moverRobot(0);
								X_Esclavos[IncideNodoCercano]=-1;
							    Y_Esclavos[IncideNodoCercano]=-1;
							for(i=0; i< NUM_NODO_ESCLAVO; i=i+1){

									if(IDNodoCercano==orden_TDMA_Esclavos[i]){
										//Estamos dentro del rango/10 de comunicacion del nodo mas cercano
										X_Esclavos[i]=-1;
										Y_Esclavos[i]=-1;
										channelWanted=AM_CANAL;
										call CC2420Config.setChannel(channelWanted);
										otro_nodo=TRUE;
										calculateNodeNearest();
										call CC2420Config.sync();
										//Una vez que cambiemos al canal principal empezaremos a tomar datos
										call Timer_Localizacion.stop();
									}
							}
						}else{
							moverRobot(1);
						}
					}
				}
			}
			num_muestras=MUESTRAS-1;
			}else{

			num_muestras--;
		}
	}
}

void receiveUbicacion_Esclavo(message_t* msg,void* payload){
	int i;
	TMDA_Ubicacion_Esclavo_Msg* men_revc = (TMDA_Ubicacion_Esclavo_Msg*) payload;

	for(i=0; i< NUM_NODO_ESCLAVO; i=i+1){

		if(men_revc->nodeid==orden_TDMA_Esclavos[i]){

			X_Esclavos[i]=men_revc->x;
			Y_Esclavos[i]=men_revc->y;

			printf("ID_esclavo: %u ",men_revc->nodeid);
			printf("Posicion: (");
			printfFloat( X_Esclavos[i]);
			printf(",");
			printfFloat( Y_Esclavos[i]);
			printf(")\n");
			//Lanzamo un nuevo calculo del nodo mas cercano al recibir un mensaje nuevo
			calculateNodeNearest();
			call Timer_Ubicacion.stop();//Paro el timer cuando ha llegado al menos un esclavo
			call Timer_Localizacion.startPeriodic(TIMER_PERIOD_MILLI_LOCALIZACION);
		}
	}

	counter++;

	if(counter>=NUM_NODO_ESCLAVO){
		counter=0;
		//Lamar a rutina de movimiento timer x
	}
}

void receive_Cambio_Canal_Msg(message_t* msg, void* payload){
	Cambio_Canal_Msg* men_revc = (Cambio_Canal_Msg*) payload;
  	int i=0;

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

void receive_Datos(message_t* msg, void* payload){
	Datos* men_revc = (Datos*) payload;

	printf("Datos %u\n",men_revc->datos );
	printfflush();
	call Timer_Recoleccion_Datos.stop();
	//Calculamos el siguiente nodo
	calculateNodeNearest();
	channelWanted=AM_CANAL_MOVIL;
	counter=0;
	if (IDNodoCercano==0)
	{
		backToStart=TRUE;
		otro_nodo=FALSE;
	}else{
		otro_nodo=TRUE;
	}

	
	call CC2420Config.setChannel(channelWanted);
	call CC2420Config.sync();
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

void calculatePosition(){
	int i=0;
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


	posicion[0] = sumMultiX/sumwij;
	//Calculo la posicion y
	posicion[1] = sumMultiY/sumwij;

	printf("Posicion: (");
	printfFloat( posicion[0]);
	printf(",");
	printfFloat( posicion[1]);
	printf(")\n");
	printfflush();
}

//Se ejecutara cada vez que se quiera cambiar al siguiente nodo
//Cuando se obtenga las medidas de un nodo pondra a -1 X_Esclavos[i] Y_Esclavos[i]
void calculateNodeNearest(){
	int i;
	float modulo_minimo=100000000;
	float modulo_calculado;
	posicion_Cercano[0]=-1;
	posicion_Cercano[1]=-1;
	IDNodoCercano=0;
	IncideNodoCercano=-1;
	for ( i= 0; i < NUM_NODO_ESCLAVO; i++){

		if(X_Esclavos[i]!=-1 && Y_Esclavos[i]!=-1){

			modulo_calculado=sqrtf(powf(X_Esclavos[i]-posicion[0],2)+powf(Y_Esclavos[i]-posicion[1],2));
			if (modulo_calculado<modulo_minimo){

					posicion_Cercano[0]=X_Esclavos[i];
					posicion_Cercano[1]=Y_Esclavos[i];
					IDNodoCercano=orden_TDMA_Esclavos[i];
					IncideNodoCercano=i;
			}
		}
	}

	if(IDNodoCercano!=0){

		printf("El ID del nodo mas cercano es %u\n",IDNodoCercano);
		//Si IDNodoCercano==0 hacer que empieze el ciclo
	}
}

//Se ejecutara cada vez que se quiera cambiar al siguiente nodo
void calculateDirection(){

	X_Direccion=posicion_Cercano[0]-posicion[0];
	Y_Direccion=posicion_Cercano[1]-posicion[1];

	printf("Direccion (");
	printfFloat(X_Direccion);
	printf(",");
	printfFloat(Y_Direccion);
	printf(")\n");
	printfflush();

	printf("\n");

	printf("Posicion cercano (");
	printfFloat(posicion_Cercano[0]);
	printf(",");
	printfFloat(posicion_Cercano[1]);
	printf(")\n");
	printfflush();

	//Cambiamo el canal para solo se comunique ancla y robot mientras se este moviendo
	if (channelCurrent==AM_CANAL){

		num_muestras=MUESTRAS-1;
		channelWanted=AM_CANAL_MOVIL;

		call Timer_Localizacion.stop();
		call Timer_Cambio_Canal.startPeriodic(TIMER_PERIOD_MILLI_CHANGE_CHANNEL);
	}
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
    	}else{
        return x[lenght/2];
	}
}


void moverRobot(int mover){
		/*
			enablePin:
				set = andar
				clr = parar

			Pin1 | Pin2
			set			set		: Arriba
			set			clr		: Izquierda
			clr			set		: Derecha
			clr			clr		: Abajo

		*/

		//Parar
		if(mover==0){
			printf("Parar\n");
			printfflush();

			call enablePin.makeOutput();
			call enablePin.clr();
		}else {
			//Arriba
			if(Y_Direccion > (0 + RANGO/10)){
				printf("Adelante\n");
				printfflush();

				call enablePin.makeOutput();
				call enablePin.set();
				call Pin2.makeOutput();
				call Pin2.set();
				call Pin1.makeOutput();
				call Pin1.set();
			}
			//Abajo
			else if(Y_Direccion < (0 - RANGO/10)){
				printf("Abajo\n");
				printfflush();
				call enablePin.makeOutput();
				call enablePin.set();
				call Pin2.makeOutput();
				call Pin2.clr();
				call Pin1.makeOutput();
				call Pin1.clr();
			}else{
				//Derecha
				if(X_Direccion > (0 + RANGO/10)){
					printf("Derecha\n");
					printfflush();
					call enablePin.makeOutput();
					call enablePin.set();
					call Pin2.makeOutput();
					call Pin2.set();
					call Pin1.makeOutput();
					call Pin1.clr();
				}
				//Izquierda
				else if(X_Direccion < (0 - RANGO/10)){
					printf("Izquierda\n");
					printfflush();
					call enablePin.makeOutput();
					call enablePin.set();
					call Pin2.makeOutput();
					call Pin2.clr();
					call Pin1.makeOutput();
					call Pin1.set();
				}else{
				moverRobot(0);
			}

			}
		}
}

}

/*Solo queda hacer uso de setPower
command void setPower(message_t *p_msg, uint8_t power)
Set transmission power for a given packet. Valid ranges are between 0 and 31.
Parameters:
p_msg - the message.
power - transmission power.
no por debajo de 20%

call CC2420Packet.setPower(&msg,10)
 */

 /*
 Mira un timer para el reenvio de TMDA_Ubicacion sino recibo ningun mensaje de esclavo
 */
