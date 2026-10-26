

////////////////////////////////////////Placa que se va a flashear

  // Cuál se está grabando. Cambiar esto para cada placa
  #define nodoSeleccionado WisMesh_wisblock_Arduino_1



  // Tipos de placas
  #define wisblock 0
  #define wistag 1


  #define WisMesh_wisblock_Arduino_1    0
  #define Wistag_Casco_Minero           1
  #define Wistag_Pecho                  2
  #define Wistag_Abierta                3
  #define Wistag_Normal_1               4
  #define Wistag_Normal_2               5
  #define Wistag_Normal_3               6
  #define Wistag_Normal_4               7
  #define Wistag_Normal_5               8
  #define Wistag_Normal_6               9
  #define Wistag_Normal_7               10

  
  // Para ver qué tipo de placa es
  #define placaSiendoFlasheada \
    ((nodoSeleccionado == WisMesh_wisblock_Arduino_1) ? wisblock : wistag)



  struct DatosNodo {
    const uint32_t nodeNum;
    const char* longName;
    const char* shortName;
    const uint8_t tipo;

    // Periodos de envio (ms)
    const uint32_t periodoMensajePosition;        //ms
    const uint32_t periodoMensajeTelemetria;      //ms
    const uint32_t periodoMensajeNodeID;          //ms
    const uint32_t periodoPeticionRouting;        //ms
    const uint32_t tiempoEntreBinariosPrivados;   //ms
  };


  #define MIN *60*1000
  #define SEG *1000
  constexpr DatosNodo listaNodos[] = {
    //                                nodeNum         longName                          shortName   tipo          perPositi    perTelem      perNodeID     perRouting    perBinario 
    /*WisMesh_wisblock_Arduino_1*/   {0x01020304,     "WisMesh_wisblock_Arduino_1",     "0304",     wisblock,     30 MIN,      20 MIN,       20 MIN,       20 MIN,       1 MIN},
    /*Wistag_Casco_Minero*/          {0x01020300,     "Wistag_Casco_Minero",            "0300",     wistag,       5 MIN,       20 MIN,       20 MIN,       10 MIN,       5 SEG},
    /*Wistag_Pecho*/                 {0x01020301,     "Wistag_Pecho",                   "0301",     wistag,       5 MIN,       20 MIN,       20 MIN,       10 MIN,       5 SEG},
    /*Wistag_Abierta*/               {0x010203FF,     "Wistag_Abierta",                 "03FF",     wistag,       5 MIN,       20 MIN,       20 MIN,       10 MIN,       5 SEG},
    /*Wistag_Normal_1*/              {0x01020305,     "Wistag_Normal_1",                "0305",     wistag,       5 MIN,       20 MIN,       20 MIN,       5 MIN,        5 SEG},
    /*Wistag_Normal_2*/              {0x01020306,     "Wistag_Normal_2",                "0306",     wistag,       5 MIN,       20 MIN,       20 MIN,       5 MIN,        5 SEG},
    /*Wistag_Normal_3*/              {0x01020307,     "Wistag_Normal_3",                "0307",     wistag,       5 MIN,       20 MIN,       20 MIN,       5 MIN,        5 SEG},
    /*Wistag_Normal_4*/              {0x01020308,     "Wistag_Normal_4",                "0308",     wistag,       5 MIN,       20 MIN,       20 MIN,       5 MIN,        5 SEG},
    /*Wistag_Normal_5*/              {0x01020309,     "Wistag_Normal_5",                "0309",     wistag,       5 MIN,       20 MIN,       20 MIN,       5 MIN,        5 SEG},
    /*Wistag_Normal_6*/              {0x0102030A,     "Wistag_Normal_6",                "030A",     wistag,       5 MIN,       20 MIN,       20 MIN,       5 MIN,        5 SEG},
    /*Wistag_Normal_7*/              {0x0102030B,     "Wistag_Normal_7",                "030B",     wistag,       5 MIN,       20 MIN,       20 MIN,       5 MIN,        5 SEG},
  };
  #undef MIN
  #undef SEG



  #if((placaSiendoFlasheada != wisblock) && (placaSiendoFlasheada != wistag))
    #error Placa siendo flasheada desconocida
  #endif

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
        #define printBufferFIFO false
                    // Printea todo lo que se puede leer del buffer. Dejar apagado si no se está usando porque consume mucho tiempo de consumo. Independiente de "printearSerial"
                    // Si BLuetooth está encendido también se imprimirá por allí
      #endif



#define debugRedNeuronal true

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





//////////////////////////////////////// Guardado de datos en la flash

  #define FILENAME    "/config.bin"   // Se guarda un struct de tipo "estructuraDatosGuardados"

////////////////////////////////////////





//////////////////////////////////////// El NodeID de la gateway, para que se pueda hacer mandado de mensajes por Next-Hop

  constexpr uint32_t  gatewayNodeNum   = 0xb3690d07;

////////////////////////////////////////Identificacion del nodo en Meshtastic

  constexpr uint32_t nodeNum = listaNodos[nodoSeleccionado].nodeNum;
  const char* const longName = listaNodos[nodoSeleccionado].longName;
  const char* const shortName = listaNodos[nodoSeleccionado].shortName;

  constexpr uint8_t   hopLimit  = 7;
  constexpr bool      wantAck   = false;
  constexpr bool      viaMqtt   = false;

////////////////////////////////////////Configuración RadioLib

  constexpr uint16_t 	preamble = 12;
  constexpr uint8_t	 	syncWord = 0x2B;
  int8_t              potencia = -9;
  float               freqMHz  = 866.0;
  uint8_t             sf 			 = 7;
  float               bw       = 250.0;
  uint8_t             cr 			 = 5;

////////////////////////////////////////






////////////////////////////////////////Configuración periodos de tareas

  constexpr uint32_t tiempoMinimoEntreMensajesMeshtastic  = 40*1000;				// ms de periodicidad entre actualizaciones del estado de la placa
  constexpr uint32_t periodoAccel = 1000 / 25;			// ms de periodicidad para conseguir 25 Hz
  constexpr uint32_t tiempoEntreMensajesPeriodicos = 500;   // ms

  // Mensajes periódicos
    constexpr uint32_t tiempoEntreBinariosPrivados  = listaNodos[nodoSeleccionado].tiempoEntreBinariosPrivados;
    constexpr uint32_t periodoMensajePosition       = listaNodos[nodoSeleccionado].periodoMensajePosition;
    constexpr uint32_t periodoMensajeTelemetria     = listaNodos[nodoSeleccionado].periodoMensajeTelemetria;
    constexpr uint32_t periodoMensajeNodeID         = listaNodos[nodoSeleccionado].periodoMensajeNodeID;
    constexpr uint32_t periodoPeticionRouting       = listaNodos[nodoSeleccionado].periodoPeticionRouting;

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