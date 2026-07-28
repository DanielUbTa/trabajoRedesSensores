



////////////////////////////////////////Placa que se va a flashear

  #define wisblock 0
  #define wistag 1

  #define placaSiendoFlasheada wistag

  #if((placaSiendoFlasheada != wisblock) && (placaSiendoFlasheada != wistag))
    #error Placa siendo flasheada desconocida
  #endif

////////////////////////////////////////



////////////////////////////////////////Identificacion del nodo en Meshtastic

	#if (placaSiendoFlasheada == wisblock)
    constexpr uint32_t  nodeNum   = 0x01020304;
    constexpr char*     longName  = "WisMesh_wisblock_Arduino_1";
    constexpr char*     shortName = "0304";
  #elif (placaSiendoFlasheada == wistag)
    constexpr uint32_t  nodeNum   = 0x01020301;
    constexpr char*     longName  = "Wistag_Pecho";
    constexpr char*     shortName = "0301";
    //0x01020300;               0x01020301;       0x010203FF;
    //"Wistag_Casco_Minero";    "Wistag_Pecho";   "Wistag_Abierta";
    //"0300";                   "0301";           "03FF";
  #endif
  constexpr uint8_t   hopLimit  = 7;
  constexpr bool      wantAck   = false;
  constexpr bool      viaMqtt   = false;

////////////////////////////////////////



#define usarBluetooth false
			#if usarBluetooth	
			#include <bluefruit.h>
			#endif


#define usarGPS false
      // Añade 36 mA de consumo
      #define debugGPS false      



#define usarAcelerometro true 
			#if usarAcelerometro
        #define printBufferFIFO true
                    // Printea todo lo que se puede leer del buffer. Dejar apagado si no se está usando porque consume mucho tiempo de consumo. Independiente de "printearSerial"
                    // Si BLuetooth está encendido también se imprimirá por allí
			#endif



#define debugRedNeuronal false

#define simularRedNeuronal false                                 //En simulación no se generarán caídas
      #define probabilidadCambioEstadoEsUnoEntreEsto 100



#define mandarMeshtastic true

#define mandarTelemetria true


#define printearSerial true
      ////////////////////////////////////////  Serial.print toggleable
        #if printearSerial
          #define DPRINT(...) Serial.print(__VA_ARGS__)
          #define DPRINTLN(...)  Serial.println(__VA_ARGS__)
          #define DPRINTF(...)    Serial.printf(__VA_ARGS__)
        #else
          #define DPRINT(...)
          #define DPRINTLN(...)
          #define DPRINTF(...)
        #endif
      //////////////////////////////////////// 

#if (placaSiendoFlasheada == wistag)

  #define calladitoEstaMasGuapo true
        // true para que no pite la wistag

#endif



////////////////////////////////////////Configuración RadioLib

  constexpr float 		potencia = -9; // int8_t
  constexpr uint16_t 	preamble = 12;
  constexpr uint8_t	 	syncWord = 0x2B;
  constexpr float  		freqMHz  = 866.0;
  constexpr uint8_t 	sf 			 = 7;
  constexpr float 		bw       = 250.0;
  constexpr uint8_t	 	cr 			 = 5;

////////////////////////////////////////



////////////////////////////////////////Configuración periodos de tareas

  constexpr uint32_t periodoAccel = 160;			// ms de periodicidad

  constexpr uint32_t periodoMensajePosition = 5*60*1000;		// ms de periodicidad

  constexpr uint32_t tiempoMinimoEntreMensajesMeshtastic  = 30*1000;				// ms de periodicidad

  constexpr uint32_t periodoMensajeTelemetriaYnodeID  = 20*60*1000;				// ms de periodicidad

////////////////////////////////////////



  constexpr uint32_t bCMtamano = 128;    // Debe ser tan grande como para poder guardar 2 segundos de datos a 25 Hz

			#if (placaSiendoFlasheada == wisblock)
        constexpr uint32_t freqIMU = 25;	  						// Hz.  Can be: 0,1,10,25,50,100,200,400,1600,5000 Hz
			#elif (placaSiendoFlasheada == wistag)
				#define            freqIMU LIS2DH12_ODR_25Hz
          // Can be LIS2DH12_ODR_1Hz, LIS2DH12_ODR_1Hz, LIS2DH12_ODR_10Hz, LIS2DH12_ODR_25Hz, LIS2DH12_ODR_50Hz, LIS2DH12_ODR_100Hz, LIS2DH12_ODR_200Hz, LIS2DH12_ODR_400Hz, LIS2DH12_ODR_1kHz620_LP, LIS2DH12_ODR_5kHz376_LP_1kHz344_NM_HP, 
			#endif
  
	constexpr float UMBRAL_ESTADO_RED_NEURONAL = 0.60f;

  constexpr uint32_t milisMinEstadoRedNeuronal = 60*1000;