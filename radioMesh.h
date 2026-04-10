#pragma once

#include <Arduino.h>
#include <RadioLib.h>

// nanopb + protobufs de Meshtastic-arduino
#include <pb.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include <meshtastic/mesh.pb.h>
#include <meshtastic/portnums.pb.h>
#include <meshtastic/telemetry.pb.h>

#include "controlRadio.h"



class MeshtasticRadio {
public:
  // Identificación del nodo por defecto
  static constexpr uint32_t       NODE_NUM            = 0x01234567;        // Identificador del nodo en la mesh
  static constexpr const char*    LONG_NAME           = "RAK4630_Arduino"; // Nombre largo para la aplicación
  static constexpr const char*    SHORT_NAME          = "4567";            // Nombre corto para la aplicación, como un callsign, normalmente los dos últimos bytes del ID, pasados a ASCII

  static constexpr uint8_t        DEFAULT_MAC[6] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB}; // Cambiar para que cada uno tenga su propia MAC

  // Canales por defecto
  static constexpr uint8_t        DEFAULT_CHANNEL_QUANTITY = 1;            // De 1 a 8
  static constexpr const char*    DEFAULT_CHANNEL_NAME     = "Custom";
  static constexpr uint8_t        DEFAULT_CHANNEL_HASH     = 0x33;         // 0x33 = 51 decimal (si no recalcula)
  static constexpr const uint8_t* DEFAULT_CHANNEL_PSK      = nullptr;      // Canal sin cifrado
  static constexpr uint8_t        DEFAULT_CHANNEL_PSK_LEN  = 0;

  // Configuración de envío por defecto
  static constexpr uint8_t     DEFAULT_HOP_LIMIT         = 3;               // De 0 a 7
  static constexpr bool        DEFAULT_WANT_ACK          = false;
  static constexpr bool        DEFAULT_VIA_MQTT          = false;

  struct MeshConfig {
    uint32_t nodeNum;           // Identificador del nodo, se usa en la sección "From" del mensaje de Meshtastic
    char     longName[32];      // Nombre largo para la aplicación
    char     shortName[8];      // Nombre corto para la aplicación, como un callsign

    uint8_t  defaultHopLimit;   // 0..7
    bool     defaultWantAck;    // Usado para los DMs
    bool     defaultViaMqtt;

    // Array de los 8 canales
    uint8_t  cantidadCanales;   // Hasta 8 canales
    uint8_t  hashCanal[8];      // 1 byte, depende del nombre del canal y de la PSK
    char     channelName[8][16];
    uint8_t  psk[8][32];
    uint8_t  pskLen[8];
  };

  struct ChannelConfig {
    const char*    name;
    const uint8_t* psk;
    uint8_t        pskLen;
  };



  /*!
    \brief  Constructor. Asocia la clase a una instancia ya creada de ControlRadio
    \param  mailbox: referencia al buzón radio encargado del envío/recepción LoRa
  */
  explicit MeshtasticRadio(ControlRadio& mailbox);

  /*!
    \brief  Devuelve la configuración actual de Meshtastic
    \returns referencia constante a la configuración interna
  */
  const MeshConfig& getConfig() const;

  /*!
    \brief  Edita la configuración del nodo y de todos los canales activos
    \param  nodeNum: nodeNum del nodo (se usa en "From")
    \param  longName: nombre largo con el que aparecerá el nodo en la app
    \param  shortName: nombre corto, como un call sign
    \param  hopLimit: número máximo de saltos (0..7)
    \param  wantAck: solicitar ACK por defecto
    \param  viaMqtt: marcar el flag "via MQTT" por defecto
    \param  channels: array con la configuración de canales
    \param  cantidadCanales: número de canales activos (1..8)
    \returns true si se ha aplicado la configuración; false en caso contrario
  */
  bool editarConfig(uint32_t nodeNum = NODE_NUM,
                    const char* longName = LONG_NAME,
                    const char* shortName = SHORT_NAME,
                    uint8_t hopLimit = DEFAULT_HOP_LIMIT,
                    bool wantAck = DEFAULT_WANT_ACK,
                    bool viaMqtt = DEFAULT_VIA_MQTT,
                    const ChannelConfig* channels = nullptr,
                    uint8_t cantidadCanales = DEFAULT_CHANNEL_QUANTITY);

  /*!
    \brief  Para mandar mensajes de tipo NodeID
    \param  channelIndex: índice de canal (0..7). Por defecto 0
    \param  toNodeNum: destino (0xFFFFFFFF = broadcast)
    \param  nextHop: último byte del nodeNum del relé preferido (0 = flooding normal)
    \param  hopLimit: número máximo de saltos (0..7). Si es -1 se usa el de MeshConfig
    \param  wantAck: solicitar ACK. Si es -1 se usa el de MeshConfig
    \returns true si el mensaje se ha codificado y encolado correctamente; false en caso contrario
  */
  bool mandarNodeID(uint8_t channelIndex = 0,
                    uint32_t toNodeNum = 0xFFFFFFFF,
                    uint8_t nextHop = 0,
                    int8_t hopLimit = -1,
                    int8_t wantAck = -1);

  /*!
    \brief  Para mandar mensajes de tipo Position
    \param  lat_deg: latitud en grados (WGS84), por ejemplo 41.7071104
    \param  lon_deg: longitud en grados (WGS84), por ejemplo -0.8650752
    \param  alt_m: altitud en metros
    \param  utcTime: segundos desde el 1/1/1970
    \param  channelIndex: índice de canal (0..7). Por defecto 0
    \param  toNodeNum: destino (0xFFFFFFFF = broadcast)
    \param  nextHop: último byte del nodeNum del relé preferido (0 = flooding normal)
    \param  hopLimit: número máximo de saltos (0..7). Si es -1 se usa el de MeshConfig
    \param  wantAck: solicitar ACK. Si es -1 se usa el de MeshConfig
    \returns true si el mensaje se ha codificado y encolado correctamente; false en caso contrario
  */
  bool mandarPosicion(float lat_deg, float lon_deg, int alt_m,
                      uint32_t utcTime,
                      uint8_t channelIndex = 0,
                      uint32_t toNodeNum = 0xFFFFFFFF,
                      uint8_t nextHop = 0,
                      int8_t hopLimit = -1,
                      int8_t wantAck = -1);

  /*!
    \brief  Para mandar mensajes de tipo Telemetry (DeviceMetrics)
    \param  battery_pct: porcentaje de batería (0-100), 101 para indicar que está alimentado externamente
    \param  voltage: tensión en voltios
    \param  channel_util: utilización del canal (0.0 - 1.0)
    \param  air_util_tx: airtime TX (0.0 - 1.0)
    \param  channelIndex: índice de canal (0..7). Por defecto 0
    \param  toNodeNum: destino (0xFFFFFFFF = broadcast)
    \param  nextHop: último byte del nodeNum del relé preferido (0 = flooding normal)
    \param  hopLimit: número máximo de saltos (0..7). Si es -1 se usa el de MeshConfig
    \param  wantAck: solicitar ACK. Si es -1 se usa el de MeshConfig
    \returns true si el mensaje se ha codificado y encolado correctamente; false en caso contrario
  */
  bool mandarTelemetry(uint8_t battery_pct, float voltage,
                       float channel_util, float air_util_tx,
                       uint8_t channelIndex = 0,
                       uint32_t toNodeNum = 0xFFFFFFFF,
                       uint8_t nextHop = 0,
                       int8_t hopLimit = -1,
                       int8_t wantAck = -1);

  /*!
    \brief  Para mandar mensajes de texto (TextMessageApp)
    \param  txt: string en ASCII/UTF-8 con el texto a enviar
    \param  length: longitud de la string a mandar
    \param  channelIndex: índice de canal (0..7). Por defecto 0
    \param  toNodeNum: destino (0xFFFFFFFF = broadcast)
    \param  nextHop: último byte del nodeNum del relé preferido (0 = flooding normal)
    \param  hopLimit: número máximo de saltos (0..7). Si es -1 se usa el de MeshConfig
    \param  wantAck: solicitar ACK. Si es -1 se usa el de MeshConfig
    \returns true si el mensaje se ha encolado correctamente; false en caso contrario
  */
  bool mandarTexto(const char* txt, uint32_t length,
                   uint8_t channelIndex = 0,
                   uint32_t toNodeNum = 0xFFFFFFFF,
                   uint8_t nextHop = 0,
                   int8_t hopLimit = -1,
                   int8_t wantAck = -1);

  /*!
    \brief  Para mandar mensajes binarios por el PortNum de PRIVATE, 256
    \param  data: array de bytes a enviar
    \param  length: longitud del array a mandar
    \param  channelIndex: índice de canal (0..7). Por defecto 0
    \param  toNodeNum: destino (0xFFFFFFFF = broadcast)
    \param  nextHop: último byte del nodeNum del relé preferido (0 = flooding normal)
    \param  hopLimit: número máximo de saltos (0..7). Si es -1 se usa el de MeshConfig
    \param  wantAck: solicitar ACK. Si es -1 se usa el de MeshConfig
    \returns true si el mensaje se ha encolado correctamente; false en caso contrario
  */
  bool mandarBinario(const uint8_t* data, uint32_t length,
                     uint8_t channelIndex = 0,
                     uint32_t toNodeNum = 0xFFFFFFFF,
                     uint8_t nextHop = 0,
                     int8_t hopLimit = -1,
                     int8_t wantAck = -1);

    /*!
    \brief  Para iniciar un proceso de descubrimiento de ruta hacia un nodo concreto de la malla
    \param  toNodeNum: destino al que está dirigido el Routing
    \param  channelIndex: índice de canal (0..7). Por defecto 0
    \param  nextHop: último byte del nodeNum del relé preferido (0 = flooding normal)
    \param  hopLimit: número máximo de saltos (0..7). Si es -1 se usa el de MeshConfig
    \param  wantAck: solicitar ACK. Si es -1 se usa el de MeshConfig
    \returns true si el mensaje se ha encolado correctamente; false en caso contrario
  */
  bool pedirRouting(uint32_t toNodeNum,
                    uint8_t channelIndex = 0,
                    uint8_t nextHop = 0,
                    int8_t hopLimit = -1,
                    int8_t wantAck = -1);

  /*!
    \brief  Decodifica un protobuf de tipo meshtastic_Data y extrae su contenido si es un mensaje de texto
    \param  PBarray: array de bytes que contiene el protobuf serializado
    \param  PBlength: número de bytes del protobuf de entrada
    \param  StringArray: buffer de salida donde se copiará el texto decodificado
    \param  StringArraySize: tamaño del buffer de salida
    \param  StringLength: puntero donde se devolverá la longitud del texto extraído
    \returns true si el protobuf contiene un mensaje de texto válido; false en caso contrario
  */
  bool decodificarSiEsTexto(const uint8_t* PBarray, uint32_t PBlength,
                            char* StringArray, uint32_t StringArraySize,
                            uint32_t* StringLength);

  /*!
    \brief  Decodifica un protobuf Data si contiene una payload binaria PRIVATE_APP
    \param  PBarray buffer con el protobuf Data recibido
    \param  PBlength longitud del protobuf Data recibido
    \param  BinaryArray buffer de salida donde se copiará la carga binaria
    \param  BinaryArraySize tamaño del buffer de salida
    \param  BinaryLength longitud real de los datos copiados
    \return true si el mensaje era PRIVATE_APP y se ha decodificado correctamente; false en caso contrario
  */
  bool decodificarSiEsBinario(const uint8_t* PBarray, uint32_t PBlength,
                              uint8_t* BinaryArray, uint32_t BinaryArraySize,
                              uint32_t* BinaryLength);

                              
private:
  static const uint8_t MESHTASTIC_DEFAULT_KEY_16[16];

  ControlRadio& _mailbox;
  MeshConfig _config{};
  

  /*!
    \brief  Mete un dato a la cola para que sea transmitido
    \param  data: array de bytes a transmitir
    \param  len: cuantos bytes tiene el array a transmitir
    \returns devuelve true si ni ha habido problema y false si el canal está muy ocupado (ha hecho CAD 3 veces y ha fallado), o si ha habido problemas en la transmisión
  */
  bool scanTransmit_(const uint8_t* data, size_t len);

  
  /*!
    \brief  Construye el byte flags del header Meshtastic
    \param  hopLimit: bits 0-2 (0..7)
    \param  wantAck: bit 3
    \param  viaMqtt: bit 4
    \returns flags
  */
  static uint8_t buildFlags_(uint8_t hopLimit, bool wantAck, bool viaMqtt);


  /*!
    \brief  Construye el header LoRa de 16 bytes con el formato esperado por Meshtastic
    \param  out16: buffer de 16 bytes donde se escribirá el header (dest, from, packetId, flags, channelHash, nextHop, relay)
    \param  toNodeNum: destino (0xFFFFFFFF = broadcast)
    \param  flags: byte flags ya construido
    \param  nextHop: último byte del nodeNum del relé preferido (0 = flooding normal)
  */
  static void buildLoraHeader_(uint8_t out16[16], const MeshConfig&,
                               uint8_t channelIndex, uint32_t toNodeNum,
                               uint8_t flags, uint8_t nextHop);

  
  /*!
    \brief  Codifica un mensaje protobuf (nanopb) en un buffer binario
    \param  msg: descriptor del mensaje nanopb (por ejemplo meshtastic_Data_fields)
    \param  src: puntero al struct con los campos rellenos
    \param  out: buffer de salida donde se escribirá el protobuf codificado
    \param  outMax: capacidad máxima del buffer de salida (bytes)
    \param  outLen: referencia donde se devolverá la longitud codificada final (bytes)
    \returns true si la codificación se ha realizado correctamente; false en caso contrario
  */
  static bool encodePb_(const pb_msgdesc_t* msg, const void* src,
                        uint8_t* out, size_t outMax, size_t& outLen);


  /*!
    \brief  Envía por LoRa un subpaquete Meshtastic (protobuf Data) precedido por el header de 16 bytes
    \param  subpkt: puntero al buffer con el subpaquete (protobuf ya codificado)
    \param  subpktLen: longitud del subpaquete en bytes
    \param  toNodeNum: destino (0xFFFFFFFF = broadcast)
    \param  nextHop: último byte del nodeNum del relé preferido (0 = flooding normal)
    \param  hopLimit: número máximo de saltos (0..7)
    \param  wantAck: solicitar ACK
    \returns true si la transmisión se ha iniciado correctamente; false en caso contrario
  */
  bool sendSubPacketOverLora_(const uint8_t* subpkt, size_t subpktLen,
                              uint8_t channelIndex,
                              uint32_t toNodeNum, uint8_t nextHop,
                              int8_t hopLimit, int8_t wantAck);


  /*!
    \brief  Construye un mensaje Meshtastic de tipo Data (portnum + payload) y lo envía por LoRa
    \param  portnum: tipo de aplicación (TEXT_MESSAGE_APP, POSITION_APP, NODEINFO_APP, TELEMETRY_APP, etc.)
    \param  payload: puntero a los bytes del payload (por ejemplo User/Position/Telemetry codificados)
    \param  payloadLen: longitud del payload en bytes
    \param  toNodeNum: destino (0xFFFFFFFF = broadcast)
    \param  nextHop: último byte del nodeNum del relé preferido (0 = flooding normal)
    \param  hopLimit: número máximo de saltos (0..7)
    \param  wantAck: solicitar ACK
    \returns true si el mensaje se ha codificado y enviado correctamente; false en caso contrario
  */
  bool sendData_(meshtastic_PortNum portnum, const uint8_t* payload, size_t payloadLen,
                 uint8_t channelIndex,
                 uint32_t toNodeNum, uint8_t nextHop,
                 int8_t hopLimit, int8_t wantAck);


  /*!
    \brief  Hace el xor de todos los bytes de un array
    \param  p: address del array
    \param  len: longiud del array
    \returns xor de todos los bytes del array
  */
  static uint8_t xorHash_(const uint8_t* p, size_t len);


  /*!
    \brief  Calcula el channelHash (1 byte) que Meshtastic usa en el header LoRa para identificar el canal
    \param  channelName: nombre del canal (por ejemplo "LongFast"). Si es nullptr o vacío, se usará un nombre por defecto
    \param  psk: clave PSK del canal. Puede ser:
                    - nullptr con pskLen=0 para canal sin cifrado
                    - 1 byte (shorthand): 0x00 = sin cifrado; 0x01 = default; 0x02.. = “simpleN”
                    - 16 bytes (AES-128) o 32 bytes (AES-256)
                    - longitudes intermedias: se rellenan con ceros hasta 16 o 32 según corresponda
    \param  pskLen: longitud de la clave PSK en bytes
    \returns el hash (XOR de bytes del nombre del canal y de la PSK expandida, si aplica)
  */
  static uint8_t calcularChannelHash_(const char* channelName, const uint8_t* psk, size_t pskLen);
};

