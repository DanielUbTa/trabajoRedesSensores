#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include <RadioLib.h>
#include <TinyGPSPlus.h>
#include <TimeLib.h>   	// Para conversion a tiempo UNIX
#include <nrf52840.h>  	// Para el watchdog

#include <nrf.h>				// Para el control de consumo
#include <nrf_gpio.h>


#include "configNodo.h"


#if (placaSiendoFlasheada == wisblock)
	#include "SparkFunLIS3DH.h" //http://librarymanager/All#SparkFun-LIS3DH
#endif

#if (placaSiendoFlasheada == wistag)
	#include "SparkFun_LIS2DH12.h" //http://librarymanager/All#SparkFun_LIS2DH12
#endif

#include "controlRadio.h"
#include "radioMesh.h"
#include <Trabajo_inferencing.h>

/*
extern "C" {
  #include "nrfx_timer.h"
}
*/







////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////                                                        /////////////////////
///////////////////////           Corregir problema de TX a veces crashea      /////////////////////
///////////////////////                                                        /////////////////////
///////////////////////               parece que está corregido y              /////////////////////
///////////////////////               hay un watchdog, pero revisar            /////////////////////
///////////////////////                                                        /////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////




// TODO en caso de emergencia, interesaría pasar a un modo de super bajo consumo en el que el minero pudiese pulsar el botón periodicamente para saber si sigue bien






////////////////////////////////////////////////////////////
//-------------------Globales generales-------------------//
////////////////////////////////////////////////////////////
	
#define LED_GREEN 35
#define LED_BLUE 36

//SemaphoreHandle_t radioMutex = NULL; // Para que la radio no de problemas de que distintas tareas le piden recibir y mandar al mismo tiempo

constexpr uint32_t periodoWatchdog = 20; // segundos después de los cuáles saltará el Watchdog


// texto para las notificaciones
char textoRecibido[256];
uint32_t  textoRecibidoLength = 0;




#if usarAcelerometro
//////////////////////////////////////// Globales Acelerómetro
	float buffCircularMedidas[bCMtamano][3];
	uint32_t pEscrituraBCM = 0;
	uint32_t pLecturaBCM = 0; // actualizado en "taskMeshtasticMandarTexto"
	
	#if (placaSiendoFlasheada == wisblock)
	LIS3DH myIMU(I2C_MODE, 0x18);
	#endif

	#if (placaSiendoFlasheada == wistag)
	SPARKFUN_LIS2DH12 accel;
	#endif
//////////////////////////////////////// 



//////////////////////////////////////// Cosas procesado Red neuronal
	float entradaRedNeuronal[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE]; // 400*2*3
	uint32_t pEntradaRed = 0;

	enum EstadoMovimiento {
		ESTADO_CAMINANDO,
		ESTADO_ESTACIONARIO,
		ESTADO_PICANDO,
		ESTADO_CAIDA,
		ESTADO_NO_RECONOCIDO
	};

	volatile EstadoMovimiento estadoMovimientoGlobal = ESTADO_NO_RECONOCIDO;
////////////////////////////////////////
#endif



//////////////////////////////////////// Cosas radio y Meshtastic
	SX1262 radio = new Module(42, 47, 38, 46, SPI); // Estos pines son iguales para ambas placas
	ControlRadio controlRadio(radio);
	MeshtasticRadio radioMesh(controlRadio);

	char bufferMeshtastic[128];
	int bufferMeshtasticLength = 0;
////////////////////////////////////////



//////////////////////////////////////// Cosas calculo de batería y voltage
	uint32_t vbat_pin = PIN_A0;

	#define VBAT_MV_PER_LSB   (0.73242188F)   // 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096
	#define VBAT_DIVIDER      (0.5F)          // 150K + 150K voltage divider on VBAT
	#define VBAT_DIVIDER_COMP (2.0F)          // Compensation factor for the VBAT divider
	#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB *   0.8386850668)				//Multiplicador obtenido experimentalmente
////////////////////////////////////////



#if usarBluetooth
//////////////////////////////////////// Globales Bluetooth
	// Forward declarations for functions
	void ble_connect_callback(uint16_t conn_handle);
	void ble_disconnect_callback(uint16_t conn_handle, uint8_t reason);

	
	//	@brief  BLE UART service
	//	@note   Used for BLE UART communication
	
	BLEUart g_BleUart;

	// Flag if BLE UART client is connected 
	bool g_BleUartConnected = false;
//////////////////////////////////////// 
#endif



#if usarGPS
//////////////////////////////////////// Globales GPS
	TinyGPSPlus gps;
	//static const uint32_t GPSBaud = 9600;
//////////////////////////////////////// 
#endif



////////////////////////////////////////////////////////////








///////////////////////////////////////////////////
//-------------------Funciones-------------------//
///////////////////////////////////////////////////



#if (placaSiendoFlasheada == wistag)
//////////////////////////////////////// Cosas específicas de la Tag. Incluye interrupciones por botón y modo de sleep en cuant se manda un mensaje de emergencia
	#define BUZZER 21
	#define BUTTON 9

	volatile bool botonPulsado = false;
	volatile bool emergenciaSolicitada = false;
	volatile uint32_t cuandoFuePulsado = 0xFFFFFFFF; // Tiempo máximo 

	constexpr const char* mensajeAlarma = "!ALARMA de ";
	char mensajeAlarmaConNombre[64];

	void botonHaSidoPulsado(void){
		if(digitalRead(BUTTON) == LOW){
			botonPulsado = true;
			cuandoFuePulsado = millis();
			//Serial.printf("Boton pulsado\n");
		}else{
			if((unsigned)millis() - cuandoFuePulsado >= 3000){	// Se ha pulsado el botón durante más de 3 segundos, eso significa emergencia
				emergenciaSolicitada = true;
				
				/*size_t len1 = strlen(mensajeAlarma);
    		size_t len2 = strlen(longName);
				memcpy(mensajeAlarmaConNombre, mensajeAlarma, len1);
    		memcpy(mensajeAlarmaConNombre + len1, longName, len2 + 1);  // +1 para copiar también '\0'
				radioMesh.mandarTexto(mensajeAlarmaConNombre, len1 + len2 + 1);

				// Pasar a un modo especial de emergencia, en este se pasa a un modo de bajo consumo: 
				// 		se apagan los periféricos y se deja al micro en un estado de sleep hasta que se pulsa el botón
				// 		cuando se pulsa el boton, se manda un mensaje de meshtastic y se vuelve a sleep
				// el micro permanecerá en este estado hasta que se reinicie el dispositivo
				entrarModoBajoConsumoSYSTEM_ONboton();

				//deshabilitado de momento*/
			}
		}
	}

	void gestionarEmergenciaSolicitada() {
		if (!emergenciaSolicitada) {
			return;
		}

		emergenciaSolicitada = false;

		detachInterrupt(digitalPinToInterrupt(BUTTON));

		size_t len1 = strlen(mensajeAlarma);
		size_t len2 = strlen(longName);

		memcpy(mensajeAlarmaConNombre, mensajeAlarma, len1);
		memcpy(mensajeAlarmaConNombre + len1, longName, len2 + 1);

		//radioMesh.mandarAlarma(mensajeAlarmaConNombre, len1 + len2 + 1); 	//Las alarmas no se suben a MQTT
		radioMesh.mandarTexto(mensajeAlarmaConNombre, len1 + len2 + 1);

		delay(500);
		feedWatchdog();

		suspenderTasksNoNecesariasEmergencia();

		while (digitalRead(BUTTON) == LOW) {
			delay(10);
			feedWatchdog();
		}

		delay(200);
		feedWatchdog();

		entrarModoBajoConsumoSYSTEM_ONboton();
	}
////////////////////////////////////////
#endif



//////////////////////////////////////// Funciones del Watchdog
	void setupWatchdogTimer(void) {
			NRF_WDT->CONFIG         = 0x00;     										// Configure WDT to not run when CPU is asleep
			NRF_WDT->CRV            = periodoWatchdog*32768 + 1;  	// timeout[s] = (CRV-1)/32768
			NRF_WDT->RREN           = 0x01;     										// Enable the RR[0] reload register
			NRF_WDT->TASKS_START    = 1;        										// Start WDT   
	}

	//Feed WDT frencuently enough to avoid WDT trigger
	void feedWatchdog(void) {
			NRF_WDT->RR[0] = WDT_RR_RR_Reload;
	}
////////////////////////////////////////


//////////////////////////////////////// Control de alimentación
	void setupAlimentacionSlots(){
		#if (placaSiendoFlasheada == wisblock)
			//GPS (slots A y B)
			pinMode(WB_IO1, OUTPUT);
			pinMode(WB_IO2, OUTPUT);
				digitalWrite(WB_IO1, (usarGPS)? HIGH:LOW);
				digitalWrite(WB_IO2, (usarGPS)? HIGH:LOW);

			//Acelerometro (slot C)
			pinMode(WB_IO3, OUTPUT);
			pinMode(WB_IO4, OUTPUT);
				digitalWrite(WB_IO3, (usarAcelerometro)? HIGH:LOW);
				digitalWrite(WB_IO4, (usarAcelerometro)? HIGH:LOW);
		#endif

		#if (placaSiendoFlasheada == wistag)
			#define PIN_3V3_EN (34)
			pinMode(PIN_3V3_EN, OUTPUT);
				digitalWrite(PIN_3V3_EN, (usarAcelerometro || usarGPS)? HIGH:LOW);
		#endif

		delay(10);
	}
//////////////////////////////////////// 


//////////////////////////////////////// Modo de bajo consumo

	#define PIN_BOTON_WAKEUP BUTTON

	volatile bool botonWakeupPulsado = false;

	void configurarBotonWakeupSystemOn() {
		pinMode(PIN_BOTON_WAKEUP, INPUT_PULLUP);

		nrf_gpio_cfg_sense_input(
			g_ADigitalPinMap[PIN_BOTON_WAKEUP],
			NRF_GPIO_PIN_PULLUP,
			NRF_GPIO_PIN_SENSE_LOW
		);
	}

	void apagarPerifericosModoBoton() {
		pinMode(LED_BLUE, OUTPUT);
		pinMode(LED_GREEN, OUTPUT);

		digitalWrite(LED_BLUE, LOW);
		digitalWrite(LED_GREEN, LOW);

		radio.sleep();

		Wire.end();
		Wire1.end();
		SPI.end();

		#if (placaSiendoFlasheada == wistag)

			digitalWrite(PIN_3V3_EN, LOW);
			
		#elif (placaSiendoFlasheada == wisblock)
			//GPS (slots A y B)
			digitalWrite(WB_IO1, LOW);
			digitalWrite(WB_IO2, LOW);

			//Acelerometro (slot C)
			digitalWrite(WB_IO3, LOW);
			digitalWrite(WB_IO4, LOW);
		#endif
	}

	void encenderPerifericosModoBoton() {
		Wire.begin();
		Wire1.begin();
		SPI.begin();
		radio.standby();
		delay(20);

		#if (placaSiendoFlasheada == wisblock)
			//GPS (slots A y B)
			digitalWrite(WB_IO1, (usarGPS)? HIGH:LOW);
			digitalWrite(WB_IO2, (usarGPS)? HIGH:LOW);

			//Acelerometro (slot C)
			digitalWrite(WB_IO3, (usarAcelerometro)? HIGH:LOW);
			digitalWrite(WB_IO4, (usarAcelerometro)? HIGH:LOW);
		#elif (placaSiendoFlasheada == wistag)

			digitalWrite(PIN_3V3_EN, (usarAcelerometro || usarGPS)? HIGH:LOW);

		#endif
	}

	void esperarEventoBajoConsumo() {
		// Limpia eventos pendientes
		__SEV();
		__WFE();
		__WFE();
		// Aquí la CPU se duerme hasta que llegue un evento/interrupción
	}

	void entrarModoBajoConsumoSYSTEM_ONboton() {
		configurarBotonWakeupSystemOn();
		apagarPerifericosModoBoton();

		while (true) {
			esperarEventoBajoConsumo();
			feedWatchdog();

			if (digitalRead(PIN_BOTON_WAKEUP) == LOW) {
				delay(30); // antirrebote

				if (digitalRead(PIN_BOTON_WAKEUP) == LOW) {
					encenderPerifericosModoBoton();

					char mensaje[64] = "";
					strcpy(mensaje, "!BOTON ");
					strcat(mensaje, longName);

					//radioMesh.mandarAlarma(mensaje, strlen(mensaje) + 1); //Las alarmas no se suben a MQTT
					radioMesh.mandarTexto(mensaje, strlen(mensaje) + 1);

					delay(500);
					feedWatchdog();

					float battVoltageMiliVolt = readVBAT();
					char battPercentage = mvToPercent(battVoltageMiliVolt);
					radioMesh.mandarTelemetry(battPercentage, battVoltageMiliVolt*0.001,-1.0,-1.0);
					
					delay(500);
					feedWatchdog();

					DPRINTLN("Boton pulsado en modo de bajo consumo");
					#if mandarSerial
						Serial.flush();
					#endif

					radio.sleep();

					while (digitalRead(PIN_BOTON_WAKEUP) == LOW) {
						delay(10);
						feedWatchdog();
					}					

					apagarPerifericosModoBoton();
					configurarBotonWakeupSystemOn();
					feedWatchdog();
				}
			}
		}
	}
////////////////////////////////////////


//////////////////////////////////////// Tasks de FreeRTOS
	TaskHandle_t HANDLEtaskIMU = NULL;
	TaskHandle_t HANDLEtaskProcesado = NULL;
	TaskHandle_t HANDLEtaskMeshtasticMandarTexto = NULL;
	TaskHandle_t HANDLEtaskMeshtasticMandarGPS = NULL;
	TaskHandle_t HANDLEtaskLeerGPS = NULL;
	TaskHandle_t HANDLEtaskAlarmaTagBuzzerYled = NULL;
	TaskHandle_t HANDLEtaskMeshtasticRecibir = NULL;
	TaskHandle_t HANDLEtaskMeshtasticMandarTelemetriaYnodeID = NULL;

	SemaphoreHandle_t 	imuBufferMutex = NULL; 										// Para bloquear la edición del puntero "pLecturaBCM"

	void crearTasks(){
		#if usarAcelerometro
			imuBufferMutex = xSemaphoreCreateMutex();
			if (imuBufferMutex == NULL) {
				Serial.println("Error creando imuBufferMutex");
			}
			xTaskCreate(taskIMU,"TaskIMU",512,NULL,11,&HANDLEtaskIMU);

			xTaskCreate(taskProcesado,"taskProcesado",2048,NULL,1,&HANDLEtaskProcesado);			
		#endif

		#if mandarMeshtastic
			xTaskCreate(taskMeshtasticMandarTexto,"TaskMensajeMeshtastic",2048,NULL,4,&HANDLEtaskMeshtasticMandarTexto);
		#endif

		#if (mandarMeshtastic && usarGPS)
			xTaskCreate(taskMeshtasticMandarGPS,"TaskMensajeGPS",2048,NULL,3,&HANDLEtaskMeshtasticMandarGPS);

			xTaskCreate(taskLeerGPS,"TaskLeerGPS",2048,NULL,2,&HANDLEtaskLeerGPS);
		#endif

		xTaskCreate(taskAlarmaTagBuzzerYled,"TaskAlarmaTagBuzzerYled",512,NULL,7,&HANDLEtaskAlarmaTagBuzzerYled); // Puede tener prioridad alta porque ocupa poco tiempo de cómputo y así no se entrecorta el zumbador

		xTaskCreate(taskMeshtasticRecibir,"taskMeshtasticRecibir",2048,NULL,6,&HANDLEtaskMeshtasticRecibir); // Puede tener mensajes de alarma, alta prioridad

		xTaskCreate(taskMeshtasticMandarTelemetriaYnodeID,"taskMeshtasticMandarTelemetriaYnodeID",2048,NULL,2,&HANDLEtaskMeshtasticMandarTelemetriaYnodeID);
	}

	void suspenderTasksNoNecesariasEmergencia() {
		if (HANDLEtaskIMU != NULL) {
			vTaskSuspend(HANDLEtaskIMU);
		}

		if (HANDLEtaskProcesado != NULL) {
			vTaskSuspend(HANDLEtaskProcesado);
		}

		if (HANDLEtaskMeshtasticMandarTexto != NULL) {
			vTaskSuspend(HANDLEtaskMeshtasticMandarTexto);
		}

		if (HANDLEtaskMeshtasticMandarGPS != NULL) {
			vTaskSuspend(HANDLEtaskMeshtasticMandarGPS);
		}

		if (HANDLEtaskLeerGPS != NULL) {
			vTaskSuspend(HANDLEtaskLeerGPS);
		}

		if (HANDLEtaskAlarmaTagBuzzerYled != NULL) {
			vTaskSuspend(HANDLEtaskAlarmaTagBuzzerYled);
		}

		if (HANDLEtaskMeshtasticRecibir != NULL) {
			vTaskSuspend(HANDLEtaskMeshtasticRecibir);
		}

		if (HANDLEtaskMeshtasticMandarTelemetriaYnodeID != NULL) {
			vTaskSuspend(HANDLEtaskMeshtasticMandarTelemetriaYnodeID);
		}
	}
//////////////////////////////////////// 


//////////////////////////////////////// Cosas RadioLib
		/*!
			\brief  Inicializa la interfaz SPI, configura la radio SX1262 con los parámetros definidos y deja el módulo en modo recepción
			\returns devuelve true si la inicialización de la radio se ha realizado correctamente y false si se produce algún error durante la configuración inicial
		*/
  bool setupLoRa(){
    SPI.setPins(45, 43, 44); // Estos pines son iguales para ambas placas
		SPI.begin();

		/*if(xSemaphoreTake(radioMutex, portMAX_DELAY) != pdTRUE){
			Serial.println(F("Intentando iniciar LoRa pero el semaforo está cogido"));
			return false;
		}*/

		if (!controlRadio.begin(freqMHz, bw, sf, cr, syncWord, potencia, preamble)) {
			Serial.println("Error iniciando ControlRadio");
			return false;
		}
    return true;
  }
//////////////////////////////////////// 



//////////////////////////////////////// Calculo de batería y voltage
	float readVBAT(void) { // Devuelve miliVoltios
		float raw;

		// Set the analog reference to 3.0V (default = 3.6V)
		analogReference(AR_INTERNAL_3_0);

		// Set the resolution to 12-bit (0..4095)
		analogReadResolution(12); // Can be 8, 10, 12 or 14

		// Let the ADC settle
		delay(1);

		// Get the raw 12-bit, 0..3000mV ADC value
		raw = analogRead(vbat_pin);

		// Set the ADC back to the default settings
		analogReference(AR_DEFAULT);
		analogReadResolution(10);

		// Convert the raw value to compensated mv, taking the resistor-
		// divider into account (providing the actual LIPO voltage)
		// ADC range is 0..3000mV and resolution is 12-bit (0..4095)
		return raw * REAL_VBAT_MV_PER_LSB; // Devuelve miliVoltios
	}

	uint8_t mvToPercent(float mvolts) {
		if(mvolts<3300.0)
			return 0;

		if(mvolts <3600.0) {
			mvolts -= 3300;
			return mvolts/30.0;
		}

		mvolts -= 3600;
		int percent = 10 + (mvolts * 0.15F );  // thats mvolts /6.66666666
		if(percent > 101){
			percent = 101;
		}
		return percent;
	}
////////////////////////////////////////




#if usarBluetooth
//////////////////////////////////////// Funciones BLE
	void setupBLE(){
		// Config the peripheral connection with maximum bandwidth
		// more SRAM required by SoftDevice
		// Note: All config***() function must be called before begin()
		Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
		Bluefruit.configPrphConn(92, BLE_GAP_EVENT_LENGTH_MIN, 16, 16);

		Bluefruit.begin(1, 0);
		// Set max power. Accepted values are: -40, -30, -20, -16, -12, -8, -4, 0, 4
		Bluefruit.setTxPower(4);
		// Set the BLE device name
		Bluefruit.setName("RAK4631_UART");

		Bluefruit.Periph.setConnectCallback(ble_connect_callback);
		Bluefruit.Periph.setDisconnectCallback(ble_disconnect_callback);

		// Configure and Start BLE Uart Service
		g_BleUart.begin();

		// Set up and start advertising
		// Advertising packet
		Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
		Bluefruit.Advertising.addTxPower();
  	Bluefruit.Advertising.addService(g_BleUart);
		Bluefruit.Advertising.addName();

		// Start Advertising
		//	- Enable auto advertising if disconnected
		//	- Interval:  fast mode = 20 ms, slow mode = 152.5 ms
		//	- Timeout for fast mode is 30 seconds
		//	- Start(timeout) with timeout = 0 will advertise forever (until connected)

		//	For recommended advertising interval
		//	https://developer.apple.com/library/content/qa/qa1931/_index.html
		
		Bluefruit.Advertising.restartOnDisconnect(true);
		Bluefruit.Advertising.setInterval(32, 244); // in unit of 0.625 ms
		Bluefruit.Advertising.setFastTimeout(30);   // number of seconds in fast mode
		Bluefruit.Advertising.start(0);             // 0 = Don't stop advertising after n seconds
	}

	
	//	@brief  Callback when client connects
	//	@param  conn_handle: Connection handle id

	void ble_connect_callback(uint16_t conn_handle){
			(void)conn_handle;
			g_BleUartConnected = true;

			Serial.println("BLE client connected");
	}

	
	//	@brief  Callback invoked when a connection is dropped
	//	@param  conn_handle: connection handle id
	//	@param  reason: disconnect reason
	
	void ble_disconnect_callback(uint16_t conn_handle, uint8_t reason){
			(void)conn_handle;
			(void)reason;
			g_BleUartConnected = false;

			Serial.println("BLE client disconnected");
	}
//////////////////////////////////////// 
#endif


#if usarGPS
//////////////////////////////////////// Cosas GPS
	void beginGPS(){
 		Serial1.begin(9600);
		while (!Serial1);
  	Serial.println("Inicialización de GPS ok!");

		//Serial1.print("$PCAS01,1*1D"); //Configurar baud rate a 9600 baudios

		Serial1.print("$PCAS02,1000*2E"); //Configurar periodo a 1000 ms, o 1 Hz

		//$PCAS03,1,1,1,1,1,1,1,1,0,0,,, 0,0*02
			//$PCAS03,<nGGA>,<nGLL>,<nGSA>,<nGSV>,<nRMC>,<nVTG>,<nZDA>,<nANT>,<Res>,<Res>,<Res>,<Res >,<Res>,<Res>*<Checksum>
	}

	void serializarGPShastaXms(unsigned long ms){
		unsigned long start = millis();
		char recv;
		#if debugGPS
			Serial.println();
		#endif
		do{
			while (Serial1.available()){
				recv = Serial1.read();
				gps.encode(recv);
				#if debugGPS
					Serial.print(recv);
				#endif
			}
		} while (millis() - start < ms);
	}

	bool posicionValidaGPS(){
		return 	(gps.hdop.isValid() && (gps.hdop.hdop() < 20.0)) && 
						gps.location.isValid() && (gps.location.age() <= 2000) && /*hace 2 segundos máximo*/
						gps.altitude.isValid();
	}

	uint32_t gpsUtcEpochOr0(){
		if (!(gps.date.isValid() && gps.time.isValid())) return 0;
		// opcional pero recomendable: solo actualizar cuando llegue una trama nueva
		if (!(gps.date.isUpdated() && gps.time.isUpdated())) return (uint32_t)now();

		setTime(gps.time.hour(), gps.time.minute(), gps.time.second(),
						gps.date.day(), gps.date.month(), gps.date.year() );
		return (uint32_t)now();
	}
//////////////////////////////////////// 
#endif


#if usarAcelerometro
//////////////////////////////////////// Funciones IMU
	void setupAcel(){
		#if (placaSiendoFlasheada == wisblock)
  	Wire.begin();

		myIMU.settings.adcEnabled = 0;
		myIMU.settings.tempEnabled = 0;
		myIMU.settings.accelSampleRate = freqIMU;  //Hz.  Can be: 0,1,10,25,50,100,200,400,1600,5000 Hz
		myIMU.settings.accelRange = 4;      //Max G force readable.  Can be: 2, 4, 8, 16
		myIMU.settings.xAccelEnabled = 1;
		myIMU.settings.yAccelEnabled = 1;
		myIMU.settings.zAccelEnabled = 1;
		
		myIMU.settings.fifoEnabled = 1;
		myIMU.settings.fifoThreshold = 30;  //Can be 0 to 32
		myIMU.settings.fifoMode = 1;  //FIFO mode.
		//fifoMode can be:
		//  0 (Bypass mode, FIFO off)
		//  1 (FIFO mode)
		//  3 (FIFO until full)
		//  4 (FIFO when trigger)
		
		//Call .begin() to configure the IMU (except for the fifo)
		if (myIMU.begin() != 0) {
			#if (placaSiendoFlasheada == wisblock)
				Serial.println("Problem starting IMU at 0x18"); Serial.flush();
			#elif (placaSiendoFlasheada == wistag)
				Serial.println("Problem starting IMU at 0x19"); Serial.flush();
			#endif
			return;
		}
		Serial.println("IMU started"); Serial.flush();

		Serial.print("Configuring FIFO with no error checking...");
		myIMU.fifoBegin(); //Configure fifo
		Serial.print("Done!\n");
		
		Serial.print("Clearing out the FIFO...");
		myIMU.fifoClear();
		Serial.print("Done!\n");
		myIMU.fifoStartRec(); //cause fifo to start taking data (re-applies mode bits)
		
		



		#elif (placaSiendoFlasheada == wistag)
		Wire1.end();
		Wire1.setPins(25, 24);
		Wire1.begin();

    if (accel.begin(0x19, Wire1) == false) {
      Serial.println("Problem starting LIS2DH12 at 0x19 on Wire1");
      Serial.flush();
      return;
    }

    Serial.println("IMU started");
    Serial.flush();

    accel.setScale(LIS2DH12_4g);
    accel.setDataRate(freqIMU);

		#endif
	}

	void configInterrupts(){
		#if (placaSiendoFlasheada == wisblock)
			uint8_t dataToWrite = 0;

			//LIS3DH_INT1_CFG   
			//dataToWrite |= 0x80;//AOI, 0 = OR 1 = AND
			//dataToWrite |= 0x40;//6D, 0 = interrupt source, 1 = 6 direction source
			//Set these to enable individual axes of generation source (or direction)
			// -- high and low are used generically
			dataToWrite |= 0x20;//Z high
			dataToWrite |= 0x10;//Z low
			dataToWrite |= 0x08;//Y high
			dataToWrite |= 0x04;//Y low
			dataToWrite |= 0x02;//X high
			dataToWrite |= 0x01;//X low
			myIMU.writeRegister(LIS3DH_INT1_CFG, dataToWrite);
			
			//LIS3DH_INT1_THS   
			dataToWrite = 0;
			//Provide 7 bit value, 0x7F always equals max range by accelRange setting
			dataToWrite |= 0x40; // 1/2 range
			myIMU.writeRegister(LIS3DH_INT1_THS, dataToWrite);
			
			//LIS3DH_INT1_DURATION  
			dataToWrite = 0;
			//minimum duration of the interrupt
			//LSB equals 1/(sample rate)
			dataToWrite |= 0x04; // 4 muestras
			myIMU.writeRegister(LIS3DH_INT1_DURATION, dataToWrite);


			//LIS3DH_CLICK_CFG   
			dataToWrite = 0;
			//Set these to enable individual axes of generation source (or direction)
			// -- set = 1 to enable
			//dataToWrite |= 0x20;//Z double-click
			dataToWrite |= 0x10;//Z click
			//dataToWrite |= 0x08;//Y double-click 
			dataToWrite |= 0x04;//Y click
			//dataToWrite |= 0x02;//X double-click
			dataToWrite |= 0x01;//X click
			myIMU.writeRegister(LIS3DH_CLICK_CFG, dataToWrite);
			
			/* No se usa nada de los clicks

				//LIS3DH_CLICK_SRC
				dataToWrite = 0;
				//Set these to enable click behaviors (also read to check status)
				// -- set = 1 to enable
				//dataToWrite |= 0x20;//Enable double clicks
				dataToWrite |= 0x04;//Enable single clicks
				//dataToWrite |= 0x08;//sine (0 is positive, 1 is negative)
				dataToWrite |= 0x04;//Z click detect enabled
				dataToWrite |= 0x02;//Y click detect enabled
				dataToWrite |= 0x01;//X click detect enabled
				myIMU.writeRegister(LIS3DH_CLICK_SRC, dataToWrite);
				
				//LIS3DH_CLICK_THS   
				dataToWrite = 0;
				//This sets the threshold where the click detection process is activated.
				//Provide 7 bit value, 0x7F always equals max range by accelRange setting
				dataToWrite |= 0x3F; // 1/2 range
				myIMU.writeRegister(LIS3DH_CLICK_THS, dataToWrite);
				
				//LIS3DH_TIME_LIMIT  
				dataToWrite = 0;
				//Time acceleration has to fall below threshold for a valid click.
				//LSB equals 1/(sample rate)
				dataToWrite |= 0x08; // 8 * 1/50 s = 160ms
				myIMU.writeRegister(LIS3DH_TIME_LIMIT, dataToWrite);
				
				//LIS3DH_TIME_LATENCY
				dataToWrite = 0;
				//hold-off time before allowing detection after click event
				//LSB equals 1/(sample rate)
				dataToWrite |= 0x08; // 4 * 1/50 s = 160ms
				myIMU.writeRegister(LIS3DH_TIME_LATENCY, dataToWrite);
				
				//LIS3DH_TIME_WINDOW 
				dataToWrite = 0;
				//hold-off time before allowing detection after click event
				//LSB equals 1/(sample rate)
				dataToWrite |= 0x10; // 16 * 1/50 s = 320ms
				myIMU.writeRegister(LIS3DH_TIME_WINDOW, dataToWrite);
			*/

			//LIS3DH_CTRL_REG5
			//Int1 latch interrupt and 4D on  int1 (preserve fifo en)
			myIMU.readRegister(&dataToWrite, LIS3DH_CTRL_REG5);
			dataToWrite &= 0xF3; //Clear bits of interest
			dataToWrite |= 0x08; //Latch interrupt (Cleared by reading int1_src)
			//dataToWrite |= 0x04; //Pipe 4D detection from 6D recognition to int1?
			myIMU.writeRegister(LIS3DH_CTRL_REG5, dataToWrite);

			//LIS3DH_CTRL_REG3
			//Choose source for pin 1
			dataToWrite = 0;
			//dataToWrite |= 0x80; //Click detect on pin 1
			dataToWrite |= 0x40; //AOI1 event (Generator 1 interrupt on pin 1)
			//dataToWrite |= 0x20; //AOI2 event ()
			//dataToWrite |= 0x10; //Data ready
			//dataToWrite |= 0x04; //FIFO watermark
			//dataToWrite |= 0x02; //FIFO overrun
			myIMU.writeRegister(LIS3DH_CTRL_REG3, dataToWrite);
		
			/* No se usa nada de clicks
				//LIS3DH_CTRL_REG6
				//Choose source for pin 2 and both pin output inversion state
				dataToWrite = 0;
				dataToWrite |= 0x80; //Click int on pin 2
				//dataToWrite |= 0x40; //Generator 1 interrupt on pin 2
				//dataToWrite |= 0x10; //boot status on pin 2
				//dataToWrite |= 0x02; //invert both outputs
				myIMU.writeRegister(LIS3DH_CTRL_REG6, dataToWrite);
			*/




		#elif (placaSiendoFlasheada == wistag)
			// INT1_CFG: evento high en X/Y/Z
			//writeLIS2DH12Register(0x30, 0x3F); // ZHIE | ZLOE | YHIE | YLOE | XHIE | XLOE

			// INT1_THS: mitad del fondo de escala
			//writeLIS2DH12Register(0x32, 0x3F);

			// INT1_DURATION: 4 muestras
			//writeLIS2DH12Register(0x33, 0x04);

			// CTRL_REG5: latch INT1
			//uint8_t reg5 = readLIS2DH12Register(0x24);
			//reg5 |= 0x08;
			//writeLIS2DH12Register(0x24, reg5);

			// CTRL_REG3: AOI1 en pin INT1
			//writeLIS2DH12Register(0x22, 0x40);
  	#endif			
	}

	void leerAcelerometro(){
		#if (placaSiendoFlasheada == wisblock)

			uint8_t fifoStatus = myIMU.fifoGetStatus();
			uint8_t muestrasFIFO = fifoStatus & 0x1F;

			if(muestrasFIFO == 0) {
				DPRINTLN("---- No hay datos en la FIFO");  //en myIMU.fifoGetStatus() & 0x1F está cuantos datos hay en la fifo, si es 0 no hay nada nuevo
				//return;
			}

			// //Quitar la medida vieja
			// myIMU.readFloatAccelX();
			// myIMU.readFloatAccelY();
			// myIMU.readFloatAccelZ();

			if (xSemaphoreTake(imuBufferMutex, portMAX_DELAY) == pdTRUE) {
				for (char i = 0; i < muestrasFIFO; i++) {
					buffCircularMedidas[pEscrituraBCM][0] = myIMU.readFloatAccelX();
					buffCircularMedidas[pEscrituraBCM][1] = myIMU.readFloatAccelY();
					buffCircularMedidas[pEscrituraBCM][2] = myIMU.readFloatAccelZ();

					pEscrituraBCM = (pEscrituraBCM + 1) % bCMtamano;

					// Si el buffer está lleno, borrar el dato más antiguo
					if(pEscrituraBCM == pLecturaBCM) {
						pLecturaBCM = (pLecturaBCM + 1) % bCMtamano;
					}
				}

				xSemaphoreGive(imuBufferMutex);
			}

		#elif (placaSiendoFlasheada == wistag)		
		
			if (xSemaphoreTake(imuBufferMutex, portMAX_DELAY) == pdTRUE) {

				buffCircularMedidas[pEscrituraBCM][0] = accel.getX() * 0.001; // Cambiar a miliGs 
				buffCircularMedidas[pEscrituraBCM][1] = accel.getY() * 0.001;
				buffCircularMedidas[pEscrituraBCM][2] = accel.getZ() * 0.001;

				pEscrituraBCM = (pEscrituraBCM + 1) % bCMtamano;

				if(((pEscrituraBCM + 1) % bCMtamano) == pLecturaBCM) {
					pLecturaBCM = (pLecturaBCM + 1) % bCMtamano;
				}

				xSemaphoreGive(imuBufferMutex);
			}

		#endif
	}

	void printBuffer(){
		if (xSemaphoreTake(imuBufferMutex, portMAX_DELAY) == pdTRUE) {
			while(pEscrituraBCM != pLecturaBCM){
				char buffer[128];
				sprintf(buffer, "%.3f; %.3f; %.3f\n", buffCircularMedidas[pLecturaBCM][0], buffCircularMedidas[pLecturaBCM][1], buffCircularMedidas[pLecturaBCM][2]);
				Serial.print(buffer);
				#if usarBluetooth
					if(g_BleUartConnected){
						g_BleUart.print(buffer);
					}
				#endif
				pLecturaBCM = (pLecturaBCM + 1) % bCMtamano;
			}
			xSemaphoreGive(imuBufferMutex);
		}
		Serial.flush();
	}
////////////////////////////////////////

//////////////////////////////////////// Cosas procesado Red neuronal
	void prepararArrayRed() {
		uint32_t inicio = (pEscrituraBCM + bCMtamano - EI_CLASSIFIER_RAW_SAMPLE_COUNT) % bCMtamano;

		for (uint32_t j = 0; j < EI_CLASSIFIER_RAW_SAMPLE_COUNT; j++) {
			uint32_t idxBuffer = (inicio + j) % bCMtamano;
			uint32_t idxRed = j * EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME;

			entradaRedNeuronal[idxRed + 0] = buffCircularMedidas[idxBuffer][0]; // Acc X
			entradaRedNeuronal[idxRed + 1] = buffCircularMedidas[idxBuffer][1]; // Acc Y
			entradaRedNeuronal[idxRed + 2] = buffCircularMedidas[idxBuffer][2]; // Acc Z
		}
	}

	static int getSignalDataEntradaRed(size_t offset, size_t length, float *out_ptr) {
		if ((offset + length) > EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
			return EIDSP_OUT_OF_MEM;
		}

		memcpy(out_ptr, entradaRedNeuronal + offset, length * sizeof(float));
		return EIDSP_OK;
	}

	EstadoMovimiento obtenerEstadoDesdeResultado(const ei_impulse_result_t &result) {
		float mejorValor = 0.0f;
		EstadoMovimiento mejorEstado = ESTADO_NO_RECONOCIDO;

		for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
			const char *label = result.classification[i].label;
			float value = result.classification[i].value;

			if (value > mejorValor) {
				if (strcmp(label, "Caminando") == 0) {
					mejorEstado = ESTADO_CAMINANDO;
					mejorValor = value;
				}
				else if (strcmp(label, "Estacionario") == 0) {
					mejorEstado = ESTADO_ESTACIONARIO;
					mejorValor = value;
				}
				else if (strcmp(label, "Picando") == 0) {
					mejorEstado = ESTADO_PICANDO;
					mejorValor = value;
				}
				else if (strcmp(label, "Caidas") == 0 || strcmp(label, "Caida") == 0) {
					mejorEstado = ESTADO_CAIDA;
					mejorValor = value;
				}
			}
		}

		if (mejorValor < UMBRAL_ESTADO_RED_NEURONAL) {
			return ESTADO_NO_RECONOCIDO;
		}

		return mejorEstado;
	}
////////////////////////////////////////
#endif




void setup(){
	time_t timeout = millis();
	Serial.begin(1000000);
	while (!Serial){
		if ((millis() - timeout) < 1000){      delay(100);    }
    else														{      break;    			}
	}
	
	/*radioMutex = xSemaphoreCreateMutex();
	if(radioMutex == NULL){
		Serial.println("Error creando radioMutex");
		while(true){}
	}*/
	
	Serial.println("Configurando alimentación de los slots");
	setupAlimentacionSlots();

	#if usarBluetooth
		Serial.println("Iniciando BLE");
		setupBLE();
	#endif


	#if usarGPS
		Serial.println("Iniciando GPS");
		beginGPS();
	#endif


	Serial.println("Iniciando LoRa y Meshtastic");

	if (!setupLoRa()) {
		Serial.println("Error iniciando LoRa");
  }
	if(!radioMesh.editarConfig(nodeNum, longName, shortName, hopLimit, wantAck, viaMqtt)) { // Todos estos valores en configNodo.h
		Serial.println("Error en la inicialización de radioMesh");
	}

	#if usarAcelerometro
		Serial.println("Configurando acelerómetro");		Serial.flush();
		setupAcel();
	#endif

	pinMode(LED_GREEN, OUTPUT);
	pinMode(LED_BLUE, OUTPUT);

	#if (placaSiendoFlasheada == wistag)
		//#define BUZZER 21
		pinMode(BUZZER, OUTPUT);
		
		pinMode(BUTTON, INPUT_PULLUP);
		attachInterrupt(digitalPinToInterrupt(BUTTON), botonHaSidoPulsado, CHANGE);
	#endif

	crearTasks();
	#if mandarMeshtastic
		radioMesh.mandarNodeID();
	#endif

	setupWatchdogTimer();
	// Apagado de momento, ya que cuando es pasa a modo "sleep hasta que se pulse el botón" el watchdog sigue contanoo y este no se puede parar una vez se ha iniciado

	delay(250);
}



bool LEDencendido = false;
uint32_t ultimosMilis = 0;
uint32_t milisON = 10, milisPeriodo = 1000; // 10% duty

void loop(){ // Se usa para cambiar el LED (para ver que la placa no haya crasheado) y para alimentar el Watchdog

  gestionarEmergenciaSolicitada(); // Por si se ha pulsado el botón durante más de 3 segundos, pasando a un modo de bajo consumo

	// máquina de estados simple
	if(LEDencendido){
		if(ultimosMilis + milisON <= millis()){
			ultimosMilis += milisON;
			LEDencendido = false;
			digitalWrite(LED_GREEN, LOW);
		}
	}else{
		if(ultimosMilis + milisPeriodo - milisON <= millis()){
			ultimosMilis += milisPeriodo - milisON;
			LEDencendido = true;
			digitalWrite(LED_GREEN, HIGH);

			feedWatchdog();
		}
	}	
}





void taskIMU(void *pvParameters){
  TickType_t lastWakeTime = xTaskGetTickCount();
	while(true){
		#if usarAcelerometro
			leerAcelerometro();
		#endif

		#if printBufferFIFO
			printBuffer();
		#endif

		vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(periodoAccel));
	}
}

void taskProcesado(void * pvParameters) {
	TickType_t lastWakeTime = xTaskGetTickCount();

	delay(1000);

	while (true) {
		#if usarAcelerometro

			if (xSemaphoreTake(imuBufferMutex, portMAX_DELAY) == pdTRUE) {
				prepararArrayRed();
				xSemaphoreGive(imuBufferMutex);
			}
			else {
				estadoMovimientoGlobal = ESTADO_NO_RECONOCIDO;
				vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(500));
				continue;
			}

			signal_t signal;
			signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
			signal.get_data = &getSignalDataEntradaRed;

			ei_impulse_result_t result = { 0 };

			EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);

			if (err == EI_IMPULSE_OK) {
				estadoMovimientoGlobal = obtenerEstadoDesdeResultado(result);

				#if mandarSerial
					Serial.print("Estado red neuronal: ");

					switch (estadoMovimientoGlobal) {
						case ESTADO_CAMINANDO:
							Serial.println("caminando");
							break;

						case ESTADO_ESTACIONARIO:
							Serial.println("estacionario");
							break;

						case ESTADO_PICANDO:
							Serial.println("picando");
							break;

						case ESTADO_CAIDA:
							Serial.println("caida");
							break;

						case ESTADO_NO_RECONOCIDO:
						default:
							Serial.println("no reconocido");
							break;
					}

					for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
						Serial.print(result.classification[i].label);
						Serial.print(": ");
						Serial.println(result.classification[i].value, 4);
					}
				#endif
			}
			else {
				estadoMovimientoGlobal = ESTADO_NO_RECONOCIDO;

				#if mandarSerial
					Serial.print("Error ejecutando run_classifier: ");
					Serial.println((int)err);
				#endif
			}

		#endif

		vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(500));
	}
}

void taskMeshtasticMandarTexto(void *pvParameters) {
	TickType_t lastWakeTime = xTaskGetTickCount();

	#if usarAcelerometro
		constexpr uint8_t N_ESTADOS_FILTRO = 7;
		constexpr uint8_t MIN_ESTADOS_CAMBIO = 5;
		constexpr uint8_t MIN_CAIDAS_CAMBIO = 2;

		EstadoMovimiento historicoEstados[N_ESTADOS_FILTRO];
		uint8_t indiceHistoricoEstados = 0;
		uint8_t estadosGuardados = 0;

		EstadoMovimiento ultimoEstadoEnviado = ESTADO_NO_RECONOCIDO;
		EstadoMovimiento estadoFiltradoActual = ESTADO_NO_RECONOCIDO;

		bool primerEstadoEnviado = true;
		uint32_t ultimoMensajeMandadoMs = 0;
		bool yaSeHaMandadoMensaje = false;
	#endif

	char bufferString[128];

	while (true) {
		#if usarAcelerometro

			EstadoMovimiento estadoLeido = estadoMovimientoGlobal;

			historicoEstados[indiceHistoricoEstados] = estadoLeido;
			indiceHistoricoEstados = (indiceHistoricoEstados + 1) % N_ESTADOS_FILTRO;

			if (estadosGuardados < N_ESTADOS_FILTRO) {
				estadosGuardados++;
			}

			uint8_t contadorCaminando = 0;
			uint8_t contadorEstacionario = 0;
			uint8_t contadorPicando = 0;
			uint8_t contadorCaida = 0;
			uint8_t contadorNoReconocido = 0;

			for (uint8_t i = 0; i < estadosGuardados; i++) {
				switch (historicoEstados[i]) {
					case ESTADO_CAMINANDO:
						contadorCaminando++;
						break;

					case ESTADO_ESTACIONARIO:
						contadorEstacionario++;
						break;

					case ESTADO_PICANDO:
						contadorPicando++;
						break;

					case ESTADO_CAIDA:
						contadorCaida++;
						break;

					case ESTADO_NO_RECONOCIDO:
					default:
						contadorNoReconocido++;
						break;
				}
			}

			EstadoMovimiento nuevoEstadoFiltrado = estadoFiltradoActual;
			bool hayNuevoEstadoFiltrado = false;

			if (contadorCaida >= MIN_CAIDAS_CAMBIO) {
				nuevoEstadoFiltrado = ESTADO_CAIDA;
				hayNuevoEstadoFiltrado = true;
			}
			else if (contadorCaminando >= MIN_ESTADOS_CAMBIO) {
				nuevoEstadoFiltrado = ESTADO_CAMINANDO;
				hayNuevoEstadoFiltrado = true;
			}
			else if (contadorEstacionario >= MIN_ESTADOS_CAMBIO) {
				nuevoEstadoFiltrado = ESTADO_ESTACIONARIO;
				hayNuevoEstadoFiltrado = true;
			}
			else if (contadorPicando >= MIN_ESTADOS_CAMBIO) {
				nuevoEstadoFiltrado = ESTADO_PICANDO;
				hayNuevoEstadoFiltrado = true;
			}
			else if (contadorNoReconocido >= MIN_ESTADOS_CAMBIO) {
				nuevoEstadoFiltrado = ESTADO_NO_RECONOCIDO;
				hayNuevoEstadoFiltrado = true;
			}

			if (hayNuevoEstadoFiltrado) {
				estadoFiltradoActual = nuevoEstadoFiltrado;
			}

			bool estadoHaCambiado = primerEstadoEnviado || (estadoFiltradoActual != ultimoEstadoEnviado);

			if (estadoHaCambiado) {
				const char *estadoTexto = "no reconocido";

				switch (estadoFiltradoActual) {
					case ESTADO_CAMINANDO:
						estadoTexto = "caminando";
						break;

					case ESTADO_ESTACIONARIO:
						estadoTexto = "estacionario";
						break;

					case ESTADO_PICANDO:
						estadoTexto = "picando";
						break;

					case ESTADO_CAIDA:
						estadoTexto = "caida";
						break;

					case ESTADO_NO_RECONOCIDO:
					default:
						estadoTexto = "no reconocido";
						break;
				}

				bool mandarMensaje = false;

				if (estadoFiltradoActual == ESTADO_CAIDA) {
					mandarMensaje = true;
				}
				else {
					if (!yaSeHaMandadoMensaje || ((uint32_t)(millis() - ultimoMensajeMandadoMs) >= tiempoMinimoEntreMensajesMeshtastic)) {
						mandarMensaje = true;
					}
				}

				if (mandarMensaje) {
					memset(bufferString, 0, sizeof(bufferString));

					if (estadoFiltradoActual == ESTADO_CAIDA) {
						snprintf(
							bufferString,
							sizeof(bufferString),
							"!ALARMA caída detectada de %s",
							longName
						);
					}
					else {
						snprintf(
							bufferString,
							sizeof(bufferString),
							"Estado de %s: %s",
							longName,
							estadoTexto
						);
					}

					bufferString[sizeof(bufferString) - 1] = '\0';

					bufferMeshtasticLength = strlen(bufferString);

					if (bufferMeshtasticLength >= sizeof(bufferMeshtastic)) {
						bufferMeshtasticLength = sizeof(bufferMeshtastic) - 1;
					}

					memcpy(bufferMeshtastic, bufferString, bufferMeshtasticLength);
					bufferMeshtastic[bufferMeshtasticLength] = '\0';

					DPRINTLN(bufferMeshtastic);

					/*#if usarBluetooth
						if(g_BleUartConnected){
							g_BleUart.println(bufferMeshtastic);
						}
					#endif*/

					if (bufferMeshtasticLength > 0) {
						if (estadoFiltradoActual == ESTADO_CAIDA) {
							//radioMesh.mandarAlarma(bufferMeshtastic, bufferMeshtasticLength); 	//Las alarmas no se suben a MQTT
							radioMesh.mandarTexto(bufferMeshtastic, bufferMeshtasticLength);
						}else{
							radioMesh.mandarTexto(bufferMeshtastic, bufferMeshtasticLength);
						}

						ultimoEstadoEnviado = estadoFiltradoActual;
						primerEstadoEnviado = false;
						ultimoMensajeMandadoMs = millis();
						yaSeHaMandadoMensaje = true;
					}
				}
			}

		#endif

		vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(250));
	}
}

void taskMeshtasticMandarGPS(void *pvParameters){
  TickType_t lastWakeTime = xTaskGetTickCount();
	while(true){
		#if (mandarMeshtastic && usarGPS)
		if(posicionValidaGPS()){					
			radioMesh.mandarPosicion((float)gps.location.lat(), (float)gps.location.lng(), gps.altitude.meters(), gpsUtcEpochOr0());
		}else{
			DPRINTLN("GPS no valido");
		}
		#endif
		vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(periodoMensajePosition));
	}
}

void taskMeshtasticMandarTelemetriaYnodeID(void *pvParameters){
  TickType_t lastWakeTime = xTaskGetTickCount();
	while(true){
		#if mandarMeshtastic
			radioMesh.mandarNodeID();
			vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(periodoMensajeTelemetriaYnodeID/2));
		#endif
		#if (mandarMeshtastic && mandarTelemetria)			
			float battVoltageMiliVolt = readVBAT();
			char battPercentage = mvToPercent(battVoltageMiliVolt);
			radioMesh.mandarTelemetry(battPercentage, battVoltageMiliVolt*0.001,-1.0,-1.0);
			DPRINTF("Mandado mensaje de telemetria. Bateria: %.4f V, %d%%\n",battVoltageMiliVolt*0.001, battPercentage);
		#endif
		vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(periodoMensajeTelemetriaYnodeID/2));
	}
}

void taskLeerGPS(void *pvParameters){
	while(true){
		#if usarGPS
		serializarGPShastaXms(250);
		#endif
		vTaskDelay(pdMS_TO_TICKS(4000));
	}
}

void taskMeshtasticRecibir(void *pvParameters) {
  while (true) {
    ControlRadio::Message msg;

    if (controlRadio.popRx(msg)) {
      if (msg.len > 16) {
        char texto[256];
        uint32_t textoLen = 0;

        if (radioMesh.decodificarSiEsTexto(msg.data + 16, msg.len - 16, texto, sizeof(texto), &textoLen)) {
          DPRINT("Recibido: ");
          DPRINT(texto);

					/*#if usarBluetooth
						if(g_BleUartConnected){
							DPRINT("		Mandando por Bluetooth");
							g_BleUart.println(texto);
						}
					#endif*/

					DPRINTLN();

          memcpy(textoRecibido, texto, textoLen + 1);
          textoRecibidoLength = textoLen;
        }

				if (radioMesh.decodificarSiEsAlarma(msg.data + 16, msg.len - 16, texto, sizeof(texto), &textoLen)) {
          DPRINT("Recibida Alarma: ");
          DPRINT(texto);

					// #if usarBluetooth
					// 	if(g_BleUartConnected){
					// 		DPRINT("		Mandando por Bluetooth");
					// 		g_BleUart.println(texto);
					// 	}
					// #endif

					DPRINTLN();

          memcpy(textoRecibido, texto, textoLen + 1);
          textoRecibidoLength = textoLen;
        }
      }
			// Se ha recibido un mensaje
			if (HANDLEtaskAlarmaTagBuzzerYled != NULL) {				xTaskNotifyGive(HANDLEtaskAlarmaTagBuzzerYled);			}
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void taskAlarmaTagBuzzerYled(void * pvParameters) {
	while (true) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Esperar hasta que alguien le solicite

		digitalWrite(LED_BLUE, HIGH);
		#if placaSiendoFlasheada == wistag
			botonPulsado = false;	
		#endif

		if(strstr(textoRecibido, "!ALARMA") == NULL){ // Si en la string recivida no dice "!ALARMA"
			#if false //(!calladitoEstaMasGuapo) && (placaSiendoFlasheada == wistag)
				
				do{
					tone(BUZZER, 440);
					vTaskDelay(pdMS_TO_TICKS(300));
					noTone(BUZZER);
					vTaskDelay(pdMS_TO_TICKS(50));
					for(int i=0; i<2; i++){
						tone(BUZZER, 2000);
						vTaskDelay(pdMS_TO_TICKS(50));
						noTone(BUZZER);
						vTaskDelay(pdMS_TO_TICKS(50));
					}
					vTaskDelay(pdMS_TO_TICKS(500));
				}while(!botonPulsado);

			#else

					for(int i=0; i<12; i++){
						digitalWrite(LED_BLUE, HIGH);
						vTaskDelay(pdMS_TO_TICKS(100));
						digitalWrite(LED_BLUE, LOW);
						vTaskDelay(pdMS_TO_TICKS(100));
					}

			#endif

		}else{

			#if (!calladitoEstaMasGuapo) && (placaSiendoFlasheada == wistag)

				do{
					tone(BUZZER, 4000);
					vTaskDelay(pdMS_TO_TICKS(1000));
					tone(BUZZER, 3000);
					vTaskDelay(pdMS_TO_TICKS(1000));
				}while(!botonPulsado);
				noTone(BUZZER);
				
			#else 

				for(int i=0; i<4; i++){
					digitalWrite(LED_BLUE, HIGH);
					vTaskDelay(pdMS_TO_TICKS(1000));
					digitalWrite(LED_BLUE, LOW);
					vTaskDelay(pdMS_TO_TICKS(1000));
				}

			#endif
		}

		#if placaSiendoFlasheada == wistag
			botonPulsado = false;	
		#endif
		digitalWrite(LED_BLUE, LOW);
	}
}

////////////////////////////////////////////////////////////