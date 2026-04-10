#pragma once

#include <Arduino.h>
#include <RadioLib.h> //http://librarymanager/RadioLib

// nanopb + protobufs de Meshtastic-arduino
////http://librarymanager/All#Meshtastic
#include <pb.h>
#include <pb_encode.h>
#include <meshtastic/mesh.pb.h>
#include <meshtastic/portnums.pb.h>
#include <meshtastic/telemetry.pb.h>
#include <pb_decode.h>




// Identificación del nodo
static constexpr uint32_t       NODE_NUM = 0x01234567;                  // Debe coincidir con el "from" del header
static constexpr const char*    LONG_NAME = "RAK4630_Arduino";          // Nombre largo para la aplicación
static constexpr const char*    SHORT_NAME = "4567";                    // Nombre corto para la aplicación, como un callsign

// Canales
static constexpr uint8_t        DEFAULT_CHANNEL_QUANTITY = 1;            // De 1 a 8
static constexpr const char*    DEFAULT_CHANNEL_NAME     = "Custom";
static constexpr uint8_t        DEFAULT_CHANNEL_HASH     = 0x33;         // 0x33 = 51 decimal (si no recalcula)
static constexpr const uint8_t* DEFAULT_CHANNEL_PSK    = nullptr;        // Canal sin cifrado
static constexpr uint8_t        DEFAULT_CHANNEL_PSK_LEN  = 0;

// Configuración de mandado de mensajes
static constexpr uint8_t        DEFAULT_HOP_LIMIT = 3;                   // De 0 a 7
static constexpr bool           DEFAULT_WANT_ACK  = false;
static constexpr bool           DEFAULT_VIA_MQTT  = false;




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

extern MeshConfig mesh_configStruct;

extern SemaphoreHandle_t radioMutex;


extern uint8_t txBuffer[256];
extern int txLength;
extern bool txPending;


//////////////////////Funciones

  /*!
    \brief  Para editar la configuración del nodo y del canal 0 (identidad, parámetros por defecto y canal base)
    \param  cfg: estructura de configuración a editar
    \param  nodeNum: nodeNum del nodo (se usa en "From")
    \param  longName: nombre largo con el que aparecerá el nodo en la app
    \param  shortName: nombre corto, como un call sign
    \param  hopLimit: número máximo de saltos (0..7)
    \param  wantAck: solicitar ACK por defecto (recomendado en DMs)
    \param  viaMqtt: marcar el flag "via MQTT" por defecto
    \param  cantidadCanales: número de canales activos (1..8)
    \param  channel0Name: nombre del canal 0
    \param  channel0Psk: PSK del canal 0 (nullptr si sin cifrado)
    \param  channel0PskLen: longitud de la PSK (0..32)
    \returns true si se ha aplicado la configuración; false si algún parámetro está fuera de rango
  */
bool mesh_editarConfig(MeshConfig& cfg = mesh_configStruct,
                       uint32_t nodeNum = NODE_NUM,
                       const char* longName = LONG_NAME,
                       const char* shortName = SHORT_NAME,
                       uint8_t hopLimit = DEFAULT_HOP_LIMIT,
                       bool wantAck = DEFAULT_WANT_ACK,
                       bool viaMqtt = DEFAULT_VIA_MQTT,
                       uint8_t cantidadCanales = DEFAULT_CHANNEL_QUANTITY,
                       const char* channel0Name = DEFAULT_CHANNEL_NAME,
                       const uint8_t* channel0Psk = DEFAULT_CHANNEL_PSK,
                       uint8_t channel0PskLen = DEFAULT_CHANNEL_PSK_LEN);



  /*!
    \brief  Para mandar mensajes de tipo NodeID. Identificador para cuando se une por primera vez a la malla
    \param  cfg: estructura con la configuración del nodo/canales a utilizar
    \param  channelIndex: índice de canal (0..7). Por defecto 0
    \param  toNodeNum: destino (0xFFFFFFFF = broadcast)
    \param  nextHop: último byte del nodeNum del relé preferido (0 = flooding normal)
    \param  hopLimit: número máximo de saltos (0..7). Si es -1 se usa cfg.defaultHopLimit
    \param  wantAck: solicitar ACK. Si es -1 se usa cfg.defaultWantAck
    \returns true si el mensaje se ha codificado y enviado correctamente; false en caso contrario
  */
bool mesh_mandarNodeID(const MeshConfig& cfg = mesh_configStruct,
                       uint8_t channelIndex = 0,
                       uint32_t toNodeNum = 0xFFFFFFFF, uint8_t nextHop = 0,
                       int8_t hopLimit = -1, int8_t wantAck = -1);


  /*!
    \brief  Para mandar mensajes de tipo Position
    \param  lat_deg: latitud en grados (WGS84), por ejemplo 41.7071104
    \param  lon_deg: longitud en grados (WGS84), por ejemplo -0.8650752
    \param  alt_m: altitud en metros
    \param  utcTime: segundos desde el 1/1/1970
    \param  cfg: estructura con la configuración del nodo/canales a utilizar
    \param  channelIndex: índice de canal (0..7). Por defecto 0
    \param  toNodeNum: destino (0xFFFFFFFF = broadcast)
    \param  nextHop: último byte del nodeNum del relé preferido (0 = flooding normal)
    \param  hopLimit: número máximo de saltos (0..7). Si es -1 se usa cfg.defaultHopLimit
    \param  wantAck: solicitar ACK. Si es -1 se usa cfg.defaultWantAck
    \returns true si el mensaje se ha codificado y enviado correctamente; false en caso contrario
  */
bool mesh_mandarPosicion(float lat_deg, float lon_deg, int alt_m,
                         uint32_t utcTime,
                         const MeshConfig& cfg = mesh_configStruct,
                         uint8_t channelIndex = 0,
                         uint32_t toNodeNum = 0xFFFFFFFF, uint8_t nextHop = 0,
                         int8_t hopLimit = -1, int8_t wantAck = -1);


  /*!
    \brief  Para mandar mensajes de tipo Telemetry (DeviceMetrics)
    \param  battery_pct: porcentaje de batería (0-100), 101 para indicar que está alimentado externamente
    \param  voltage: tensión en voltios
    \param  channel_util: utilización del canal (0.0 - 1.0)
    \param  air_util_tx: airtime TX (0.0 - 1.0)
    \param  cfg: estructura con la configuración del nodo/canales a utilizar
    \param  channelIndex: índice de canal (0..7). Por defecto 0
    \param  toNodeNum: destino (0xFFFFFFFF = broadcast)
    \param  nextHop: último byte del nodeNum del relé preferido (0 = flooding normal)
    \param  hopLimit: número máximo de saltos (0..7). Si es -1 se usa cfg.defaultHopLimit
    \param  wantAck: solicitar ACK. Si es -1 se usa cfg.defaultWantAck
    \returns true si el mensaje se ha codificado y enviado correctamente; false en caso contrario
  */
bool mesh_mandarTelemetry(uint8_t battery_pct, float voltage, float channel_util, float air_util_tx,
                          const MeshConfig& cfg = mesh_configStruct,
                          uint8_t channelIndex = 0,
                          uint32_t toNodeNum = 0xFFFFFFFF, uint8_t nextHop = 0,
                          int8_t hopLimit = -1, int8_t wantAck = -1);


  /*!
    \brief  Para mandar mensajes de texto (TextMessageApp)
    \param  txt: string en ASCII/UTF-8 con el texto a enviar
    \param  length: longitud de la string a mandar
    \param  cfg: estructura con la configuración del nodo/canales a utilizar
    \param  channelIndex: índice de canal (0..7). Por defecto 0
    \param  toNodeNum: destino (0xFFFFFFFF = broadcast)
    \param  nextHop: último byte del nodeNum del relé preferido (0 = flooding normal)
    \param  hopLimit: número máximo de saltos (0..7). Si es -1 se usa cfg.defaultHopLimit
    \param  wantAck: solicitar ACK. Si es -1 se usa cfg.defaultWantAck
    \returns true si el mensaje se ha enviado correctamente; false en caso contrario
  */
bool mesh_mandarTexto(const char* txt, uint32_t length,
                      const MeshConfig& cfg = mesh_configStruct,
                      uint8_t channelIndex = 0,
                      uint32_t toNodeNum = 0xFFFFFFFF, uint8_t nextHop = 0,
                      int8_t hopLimit = -1, int8_t wantAck = -1);


  /*!
    \brief  Decodifica un protobuf de tipo meshtastic_Data y extrae su contenido si es un mensaje de texto
    \param  PBarray: array de bytes que contiene el protobuf serializado
    \param  PBlength: número de bytes del protobuf de entrada
    \param  StringArray: buffer de salida donde se copiará el texto decodificado
    \param  StringArraySize: tamaño del buffer de salida
    \param  StringLength: puntero donde se devolverá la longitud del texto extraído, sin contar el carácter nulo final
    \returns devuelve true si el protobuf se ha decodificado correctamente y contiene un mensaje de texto, y false en caso contrario
  */
bool mesh_decodificarSiEsTexto(const uint8_t* PBarray, uint32_t PBlength,
                               char* StringArray, uint32_t StringArraySize,
                               uint32_t* StringLength);


