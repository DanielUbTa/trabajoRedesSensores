#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include <RadioLib.h>
#include <TinyGPSPlus.h>
#include <TimeLib.h>   // Para conversion a tiempo UNIX
#include <nrf52840.h>  // Para el watchdog

#include "controlRadio.h"
#include "radioMesh.h"

/*
extern "C" {
  #include "nrfx_timer.h"
}
*/


#include "configNodo.h"





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






////////////////////////////////////////////////////////////
//-------------------Globales generales-------------------//
////////////////////////////////////////////////////////////

//SemaphoreHandle_t radioMutex = NULL; // Para que la radio no de problemas de que distintas tareas le piden recibir y mandar al mismo tiempo

constexpr uint32_t periodoWatchdog = 30; // segundos después de los cuáles saltará el Watchdog


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
#endif

//////////////////////////////////////// Cosas radio y Meshtastic
	SX1262 radio = new Module(42, 47, 38, 46, SPI); // Estos pines son iguales para ambas placas
	ControlRadio controlRadio(radio);
	MeshtasticRadio radioMesh(controlRadio);

	char bufferMeshtastic[128];
	int bufferMeshtasticLength = 0;
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



//////////////////////////////////////// Funciones del Watchdog
	void setupWatchdogTimer(void) {
			NRF_WDT->CONFIG         = 0x01;     										// Configure WDT to run when CPU is asleep
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


TaskHandle_t 				HANDLEtaskAlarmaTagBuzzerYled = NULL;			// Para saber cuando 
SemaphoreHandle_t 	imuBufferMutex = NULL; 										// Para bloquear la edición del puntero "pLecturaBCM"
//////////////////////////////////////// Tasks de FreeRTOS
	void crearTasks(){
		#if usarAcelerometro
			imuBufferMutex = xSemaphoreCreateMutex();
			if (imuBufferMutex == NULL) {
				Serial.println("Error creando imuBufferMutex");
			}
			xTaskCreate(taskIMU,"TaskIMU",2048,NULL,11,NULL);
		#endif

		#if mandarMeshtastic
			xTaskCreate(taskMeshtasticMandarTexto,"TaskMensajeMeshtastic",2048,NULL,4,NULL);
		#endif

		#if (mandarMeshtastic && usarGPS)
			xTaskCreate(taskMeshtasticMandarGPS,"TaskMensajeGPS",2048,NULL,3,NULL);
			xTaskCreate(taskLeerGPS,"TaskLeerGPS",2048,NULL,2,NULL);
		#endif

		#if (placaSiendoFlasheada == wistag)
			xTaskCreate(taskAlarmaTagBuzzerYled,"TaskAlarmaTagBuzzerYled",512,NULL,7,&HANDLEtaskAlarmaTagBuzzerYled); // Puede tener prioridad alta porque ocupa poco tiempo de cómputo y así no se entrecorta el zumbador
		#endif

		xTaskCreate(taskMeshtasticRecibir,"taskMeshtasticRecibir",8192,NULL,6,NULL); // Puede tener mensajes de alarma, alta prioridad
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
				pLecturaBCM = (pLecturaBCM + 1) % bCMtamano;
			}
			xSemaphoreGive(imuBufferMutex);
		}
		Serial.flush();
	}
//////////////////////////////////////// 
#endif


#if (placaSiendoFlasheada == wistag)
//////////////////////////////////////// Cosas específicas de la Tag
	#define BUZZER 21
	#define LED_GREEN 35
	#define LED_BLUE 36
	#define BUTTON 9

	volatile bool botonPulsado = false;
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
				size_t len1 = strlen(mensajeAlarma);
    		size_t len2 = strlen(longName);
				memcpy(mensajeAlarmaConNombre, mensajeAlarma, len1);
    		memcpy(mensajeAlarmaConNombre + len1, longName, len2 + 1);  // +1 para copiar también '\0'
				radioMesh.mandarTexto(mensajeAlarmaConNombre, len1 + len2 + 1);
			}
		}
	}
////////////////////////////////////////
#endif




void setup(){
	time_t timeout = millis();
	Serial.begin(1000000);
	while (!Serial){
		if ((millis() - timeout) < 5000){      delay(100);    }
    else														{      break;    			}
	}

	setupWatchdogTimer();

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


	#if (placaSiendoFlasheada == wistag)
		//#define BUZZER 21
		pinMode(BUZZER, OUTPUT);
		
		//#define LED_GREEN 35
		//#define LED_BLUE 36
		pinMode(LED_GREEN, OUTPUT);
		pinMode(LED_BLUE, OUTPUT);

		pinMode(BUTTON, INPUT_PULLUP);
		attachInterrupt(digitalPinToInterrupt(BUTTON), botonHaSidoPulsado, CHANGE);
	#endif

	crearTasks();
	#if mandarMeshtastic
		radioMesh.mandarNodeID();
	#endif
	delay(500);
}

void loop(){
	//suspendLoop();
	#if (placaSiendoFlasheada == wistag)
		digitalWrite(LED_GREEN, HIGH);
		delay(300);
		digitalWrite(LED_GREEN, LOW);
		delay(700);
	#else
		delay(1000);
	#endif

	feedWatchdog();
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

void taskMeshtasticMandarTexto(void *pvParameters){
  TickType_t lastWakeTime = xTaskGetTickCount();

	char bufferString[128];
	int bufferStringLength = 0;

	while(true){
		#if usarAcelerometro
			float accX, accY, accZ;

			if (xSemaphoreTake(imuBufferMutex, portMAX_DELAY) == pdTRUE) { // Con semáforo para evitar que "pLecturaBCM" se pueda editar desde varias funciones
				accX = buffCircularMedidas[pLecturaBCM][0];
				accY = buffCircularMedidas[pLecturaBCM][1];
				accZ = buffCircularMedidas[pLecturaBCM][2];

				pLecturaBCM = pEscrituraBCM;

				xSemaphoreGive(imuBufferMutex);
			}

			int bufferStringLength = 0;
			for(int i=0; i<sizeof(bufferString); i++){
				bufferString[i] = 0;
			}
			sprintf(bufferString, " X(g) = %.4f,  Y(g) = %.4f,  Z(g) = %.4f", accX, accY, accZ);
			while(bufferString[bufferStringLength] != 0){		bufferStringLength++;		}
					
			DPRINTLN(bufferString);	

			for(int i=0; i<bufferStringLength; i++){
				bufferMeshtastic[i] = bufferString[i];
			}
			bufferMeshtasticLength = bufferStringLength;

			if(bufferMeshtasticLength > 0){
				radioMesh.mandarTexto(bufferMeshtastic, bufferMeshtasticLength);
			}
		#endif
		vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(periodoMensajeMeshtastic));
	}
}

void taskMeshtasticMandarGPS(void *pvParameters){
  TickType_t lastWakeTime = xTaskGetTickCount();
	while(true){
		#if usarGPS
		if(posicionValidaGPS()){					
			radioMesh.mandarPosicion((float)gps.location.lat(), (float)gps.location.lng(), gps.altitude.meters(), gpsUtcEpochOr0());
		}else{
			DPRINTLN("GPS no valido");
		}
		#endif
		vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(periodoMensajePosition));
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

        if (radioMesh.decodificarSiEsTexto(msg.data + 16,
                                           msg.len - 16,
                                           texto,
                                           sizeof(texto),
                                           &textoLen)) {
          DPRINT("Recibido: ");
          DPRINTLN(texto);

					#if usarBluetooth
						if(g_BleUartConnected){
							DPRINTLN("Mandando por Bluetooth");
							g_BleUart.print("Recibido: ");
							g_BleUart.println(texto);
						}
					#endif

          memcpy(textoRecibido, texto, textoLen + 1);
          textoRecibidoLength = textoLen;
        }
      }
			#if (placaSiendoFlasheada == wistag)
			if (HANDLEtaskAlarmaTagBuzzerYled != NULL) {
				xTaskNotifyGive(HANDLEtaskAlarmaTagBuzzerYled);
			}
			#endif
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

#if (placaSiendoFlasheada == wistag)
void taskAlarmaTagBuzzerYled(void * pvParameters) {
	while (true) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Esperar hasta que alguien le solicite

		digitalWrite(LED_BLUE, HIGH);
		botonPulsado = false;		
		if(strstr(textoRecibido, "!ALARMA") == NULL){ // Si en la string recivida no dice "!ALARMA"
			do{
				//digitalWrite(LED_GREEN, HIGH);
				#if !calladitoEstaMasGuapo
					tone(BUZZER, 440);
				#endif
				vTaskDelay(pdMS_TO_TICKS(300));

				//digitalWrite(LED_GREEN, LOW);
				#if !calladitoEstaMasGuapo
					noTone(BUZZER);
				#endif
				vTaskDelay(pdMS_TO_TICKS(50));

				for(int i=0; i<2; i++){
					//digitalWrite(LED_GREEN, HIGH);
					#if !calladitoEstaMasGuapo
						tone(BUZZER, 2000);
					#endif
					vTaskDelay(pdMS_TO_TICKS(50));

					//digitalWrite(LED_GREEN, LOW);
					#if !calladitoEstaMasGuapo
						noTone(BUZZER);
					#endif
					vTaskDelay(pdMS_TO_TICKS(50));
				}
				vTaskDelay(pdMS_TO_TICKS(500));
			}while(!botonPulsado);
		}else{
			do{
				tone(BUZZER, 4000);
				vTaskDelay(pdMS_TO_TICKS(1000));
				tone(BUZZER, 3000);
				vTaskDelay(pdMS_TO_TICKS(1000));
			}while(!botonPulsado);
			noTone(BUZZER);
		}
		botonPulsado = false;
		digitalWrite(LED_BLUE, LOW);
	}
}
#endif

////////////////////////////////////////////////////////////