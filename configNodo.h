



////////////////////////////////////////Placa que se va a flashear

  #define wisblock 0
  #define wistag 1

  #define placaSiendoFlasheada wisblock

  #if((placaSiendoFlasheada != wisblock) && (placaSiendoFlasheada != wistag))
    #error Placa siendo flasheada desconocida
  #endif

////////////////////////////////////////



#define usarBluetooth false
			#if usarBluetooth	
			#include <bluefruit.h>
			#endif


#define usarGPS true
      // Añade 36 mA de consumo
      #define debugGPS false      



#define usarAcelerometro true 
			#if usarAcelerometro
			#if (placaSiendoFlasheada == wisblock)
				#include "SparkFunLIS3DH.h" //http://librarymanager/All#SparkFun-LIS3DH
					// Añade 15.6 mA de consumo a pesar de que debería consumir uA. Igual es alguna librería?
			#endif

			#if (placaSiendoFlasheada == wistag)
				#include "SparkFun_LIS2DH12.h" //http://librarymanager/All#SparkFun_LIS2DH12
			#endif

      #define printBufferFIFO false
            // Printea todo lo que se puede leer del buffer. Dejar apagado si no se está usando porque consume mucho tiempo de consumo
			#endif

#define mandarMeshtastic true
	    // No cambiar a false

#define mandarSerial true
      ////////////////////////////////////////  Serial.print toggleable
        #if mandarSerial
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

  #define calladitoEstaMasGuapo false
        // true para que no pite la wistag

#endif




////////////////////////////////////////Identificacion del nodo en Meshtastic

  constexpr uint32_t  nodeNum   = 0x01020301;
  constexpr char*     longName  = "WisMesh_wistag_Arduino_2";
  constexpr char*     shortName = "0301";
  constexpr uint8_t   hopLimit  = 3;
  constexpr bool      wantAck   = false;
  constexpr bool      viaMqtt   = false;

////////////////////////////////////////



////////////////////////////////////////Configuración RadioLib

  constexpr float 		potencia = 22.0;
  constexpr uint16_t 	preamble = 12;
  constexpr uint8_t	 	syncWord = 0x2B;
  constexpr float  		freqMHz  = 866.0;
  constexpr uint8_t 	sf 			 = 5;
  constexpr float 		bw       = 250.0;
  constexpr uint8_t	 	cr 			 = 5;

////////////////////////////////////////



////////////////////////////////////////Configuración periodos de tareas

  constexpr uint32_t periodoAccel = 10;			//ms de periodicidad. Es bajo por que el IMU va a 1600 Hz, con 10 ms significa que habrá 16 datos en la fifo

  //La frecuencia de recogida de datos es de 1 Hz
  constexpr uint32_t periodoMensajePosition = 30*1000;		//ms de periodicidad

  constexpr uint32_t periodoMensajeMeshtastic  = 30*1000;				//ms de periodicidad

////////////////////////////////////////



  constexpr uint32_t bCMtamano = 8192;    // Debe ser tan grande como para poder guardar 2 segundos de datos a 1600 Hz

			#if (placaSiendoFlasheada == wisblock)
        constexpr uint32_t freqIMU = 200;	  						//Hz.  Can be: 0,1,10,25,50,100,200,400,1600,5000 Hz
			#elif (placaSiendoFlasheada == wistag)
				#define            freqIMU LIS2DH12_ODR_200Hz
          // Can be LIS2DH12_ODR_1Hz, LIS2DH12_ODR_1Hz, LIS2DH12_ODR_10Hz, LIS2DH12_ODR_25Hz, LIS2DH12_ODR_50Hz, LIS2DH12_ODR_100Hz, LIS2DH12_ODR_200Hz, LIS2DH12_ODR_400Hz, LIS2DH12_ODR_1kHz620_LP, LIS2DH12_ODR_5kHz376_LP_1kHz344_NM_HP, 
			#endif