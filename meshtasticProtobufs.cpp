#include "meshtasticProtobufs.h"

#include <cstring>  // memcpy, strncpy
#include <cstdio>   // snprintf
#include <cmath>    // llround


static const uint8_t MESHTASTIC_DEFAULT_KEY_16[16] = {
  0xD4, 0xF1, 0xBB, 0x3A, 0x20, 0x29, 0x07, 0x59,
  0xF0, 0xBC, 0xFF, 0xAB, 0xCF, 0x4E, 0x69, 0x01
};





MeshConfig mesh_configStruct = {
  0x01234567,          // nodeNum
  "RAK4630_Arduino",   // longName
  "4567",              // shortName
	3,                   // defaultHopLimit
  false,               // defaultWantAck
  false,               // defaultViaMqtt
  1,                   // cantidadCanales
  { 0x33 },            // hashCanal[0] = 0x33, resto 0
  { "Custom" },        // channelName[0] = "Custom", resto ""
  { { 0 } },           // psk[0] todo a 0
  { 0 }                // pskLen[0] = 0 (sin PSK), resto 0
};






// --------- Funciones "helpers" privadas --------- //


  /*!
    \brief  Mete un dato a la cola para que sea transmitido
    \param  data: array de bytes a transmitir
    \param  len: cuantos bytes tiene el array a transmitir
    \returns devuelve true si ni ha habido problema y false si el canal está muy ocupado (ha hecho CAD 3 veces y ha fallado), o si ha habido problemas en la transmisión
  */
  static bool scanTransmit(const uint8_t* data, size_t len) {
    if(txPending == true){
      return false; // La cola está ocupada todavía
    }
    if(len <= 0 || len > (int)sizeof(txBuffer)) {
      return false;
    }

    memcpy(txBuffer, data, len);
    txLength = len;
    txPending = true;

    return true;
  }


  /*!
    \brief  Construye el byte flags del header Meshtastic
    \param  hopLimit: bits 0-2 (0..7)
    \param  wantAck: bit 3
    \param  viaMqtt: bit 4
    \returns flags
  */
  static uint8_t mesh_buildFlags(uint8_t hopLimit, bool wantAck, bool viaMqtt) {
    hopLimit &= 0x07;
    const uint8_t hopStart = hopLimit; // bits 5-7
    return (uint8_t)(
        (hopLimit) |
        (wantAck ? 0x08 : 0x00) |
        (viaMqtt ? 0x10 : 0x00) |
        (hopStart << 5)
    );
  }


  /*!
    \brief  Construye el header LoRa de 16 bytes con el formato esperado por Meshtastic
    \param  out16: buffer de 16 bytes donde se escribirá el header (dest, from, packetId, flags, channelHash, nextHop, relay)
    \param  toNodeNum: destino (0xFFFFFFFF = broadcast)
    \param  flags: byte flags ya construido
    \param  nextHop: último byte del nodeNum del relé preferido (0 = flooding normal)
  */
  static void mesh_buildLoraHeader(uint8_t out16[16], const MeshConfig& cfg, uint8_t channelIndex, uint32_t toNodeNum, uint8_t flags, uint8_t nextHop){
    // to (little-endian)
    out16[0] = (uint8_t)(toNodeNum & 0xFF);
    out16[1] = (uint8_t)((toNodeNum >> 8) & 0xFF);
    out16[2] = (uint8_t)((toNodeNum >> 16) & 0xFF);
    out16[3] = (uint8_t)((toNodeNum >> 24) & 0xFF);

    // from (little-endian)
    out16[4] = (uint8_t)(cfg.nodeNum & 0xFF);
    out16[5] = (uint8_t)((cfg.nodeNum >> 8) & 0xFF);
    out16[6] = (uint8_t)((cfg.nodeNum >> 16) & 0xFF);
    out16[7] = (uint8_t)((cfg.nodeNum >> 24) & 0xFF);

    // packetId (8..11) se randomiza por paquete
    for (int i = 8; i < 12; i++){
      out16[i] = (uint8_t)random(0, 256);
    }

    out16[12] = flags;

    // channelHash (1 byte) del canal seleccionado
    out16[13] = cfg.hashCanal[channelIndex];

    // nextHop solo tiene sentido en unicast
    out16[14] = (toNodeNum == 0xFFFFFFFF) ? 0x00 : nextHop;

    // relay: originador = 0
    out16[15] = 0x00;
  }

  /*!
    \brief  Codifica un mensaje protobuf (nanopb) en un buffer binario
    \param  msg: descriptor del mensaje nanopb (por ejemplo meshtastic_Data_fields)
    \param  src: puntero al struct con los campos rellenos
    \param  out: buffer de salida donde se escribirá el protobuf codificado
    \param  outMax: capacidad máxima del buffer de salida (bytes)
    \param  outLen: referencia donde se devolverá la longitud codificada final (bytes)
    \returns true si la codificación se ha realizado correctamente; false en caso contrario
  */
  static bool mesh_encodePb(const pb_msgdesc_t* msg, const void* src, uint8_t* out, size_t outMax, size_t& outLen) {
    pb_ostream_t s = pb_ostream_from_buffer(out, outMax); //Destino
    if (!pb_encode(&s, msg, src)) return false;
    outLen = s.bytes_written;
    return true;
  }


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
  static bool mesh_sendSubPacketOverLora(const uint8_t* subpkt, size_t subpktLen,
                                        const MeshConfig& cfg, uint8_t channelIndex,
                                        uint32_t toNodeNum, uint8_t nextHop,
                                        int8_t hopLimit, int8_t wantAck) 
  {
    if (channelIndex >= cfg.cantidadCanales) return false;

    const uint8_t effHop = (hopLimit  == -1) ? cfg.defaultHopLimit : (uint8_t)hopLimit;   // Si no se introduce nada, en el ".h" el valor por defecto es -1
    const bool    effAck = (wantAck == -1) ? cfg.defaultWantAck  : (wantAck != 0);        // Introducir un -1 significa usar los parámetros del struct "cfg"

    uint8_t tx[256];
    uint8_t hdr[16];

    const uint8_t flags = mesh_buildFlags(effHop, effAck, cfg.defaultViaMqtt);
    mesh_buildLoraHeader(hdr, cfg, channelIndex, toNodeNum, flags, nextHop);

    if (16 + subpktLen > sizeof(tx)) return false;

    memcpy(tx, hdr, 16);
    memcpy(tx + 16, subpkt, subpktLen);

    bool mandado = scanTransmit(tx, 16 + subpktLen);
    return mandado;
  }


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
  static bool mesh_sendData(meshtastic_PortNum portnum, const uint8_t* payload, size_t payloadLen,
                            const MeshConfig& cfg, uint8_t channelIndex,
                            uint32_t toNodeNum, uint8_t nextHop,
                            int8_t hopLimit, int8_t wantAck)
  {
    // Construye un protobuf tipo Data que encapsula el mensaje que se quiere mandar
    
    meshtastic_Data d = meshtastic_Data_init_zero;
    d.portnum = portnum;

    if (payloadLen > sizeof(d.payload.bytes)) return false;
    d.payload.size = payloadLen;
    memcpy(d.payload.bytes, payload, payloadLen);

    d.has_bitfield = true;
    d.bitfield = 1;

    uint8_t subpkt[256];
    size_t subLen = 0;
    if (!mesh_encodePb(meshtastic_Data_fields, &d, subpkt, sizeof(subpkt), subLen)) return false;

    return mesh_sendSubPacketOverLora(subpkt, subLen, cfg, channelIndex, toNodeNum, nextHop, hopLimit, wantAck);
  }


  /*!
    \brief  Hace el xor de todos los bytes de un array
    \param  p: address del array
    \param  len: longiud del array
    \returns xor de todos los bytes del array
  */
  static uint8_t xorHash(const uint8_t* p, size_t len) {
    uint8_t h = 0;
    for (size_t i = 0; i < len; i++) h ^= p[i];
    return h;
  }


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
  static uint8_t mesh_calcularChannelHash(const char* channelName, const uint8_t* psk, size_t pskLen) {
    // En firmware, el nombre debe coincidir entre nodos para poder comunicarse
    // y el hash se basa en ese nombre
    if (!channelName || channelName[0] == '\0') channelName = "LongFast";

    uint8_t h = xorHash((const uint8_t*)channelName, strlen(channelName));

    // Expandir PSK si hace falta y XOR del PSK expandido
    uint8_t keyBuf[32] = {0};
    size_t keyLen = 0;

    if (!psk || pskLen == 0) {
      keyLen = 0; // sin cifrado
    } else if (pskLen == 1) {
      // Shorthand:
      // 0x00 => sin cifrado
      // 0x01 => default
      // 0x02.. => “simpleN” (último byte cambia)
      uint8_t idx = psk[0];
      if (idx == 0) {
        keyLen = 0;
      } else {
        keyLen = 16;
        memcpy(keyBuf, MESHTASTIC_DEFAULT_KEY_16, 16);
        keyBuf[15] = idx; // equivale a DEFAULT_KEY[:-1] + idx
      }
    } else if (pskLen < 16) {
      // AES128 “corto”: se rellena con ceros hasta 16
      keyLen = 16;
      memcpy(keyBuf, psk, pskLen);
    } else if (pskLen == 16 || pskLen == 32) {
      keyLen = pskLen;
      memcpy(keyBuf, psk, pskLen);
    } else if (pskLen < 32) {
      // AES256 “corto”: se rellena con ceros hasta 32
      keyLen = 32;
      memcpy(keyBuf, psk, pskLen);
    } else {
      keyLen = 32;
      memcpy(keyBuf, psk, 32);
    }

    h ^= xorHash(keyBuf, keyLen);
    return h;
  }

// --------------------------------------------- //




// --------- Funciones publicas ---------

  bool mesh_editarConfig(MeshConfig& cfg,
                        uint32_t nodeNum,
                        const char* longName,
                        const char* shortName,
                        uint8_t hopLimit,
                        bool wantAck,
                        bool viaMqtt,
                        uint8_t cantidadCanales,
                        const char* channel0Name,
                        const uint8_t* channel0Psk,
                        uint8_t channel0PskLen)
  {
    // Validaciones básicas
    if (hopLimit > 7) return false;
    if (cantidadCanales < 1 || cantidadCanales > 8) return false;
    if (channel0PskLen > 32) return false;

    // Limpieza total para evitar basura en arrays
    memset(&cfg, 0, sizeof(MeshConfig));

    // Identidad
    cfg.nodeNum = nodeNum;

    strncpy(cfg.longName, longName ? longName : "", sizeof(cfg.longName) - 1);
    cfg.longName[sizeof(cfg.longName) - 1] = '\0';

    strncpy(cfg.shortName, shortName ? shortName : "", sizeof(cfg.shortName) - 1);
    cfg.shortName[sizeof(cfg.shortName) - 1] = '\0';

    // Defaults de envío
    cfg.defaultHopLimit = hopLimit;
    cfg.defaultWantAck  = wantAck;
    cfg.defaultViaMqtt  = viaMqtt;

    // Canales
    cfg.cantidadCanales = cantidadCanales;

    // Canal 0
    strncpy(cfg.channelName[0], channel0Name ? channel0Name : "", sizeof(cfg.channelName[0]) - 1);
    cfg.channelName[0][sizeof(cfg.channelName[0]) - 1] = '\0';

    cfg.pskLen[0] = channel0PskLen;
    if (channel0Psk && channel0PskLen > 0) {
      memcpy(cfg.psk[0], channel0Psk, channel0PskLen);
    }

    // Hash del canal 0
    if (cfg.channelName[0][0] == '\0') {
      cfg.hashCanal[0] = DEFAULT_CHANNEL_HASH;
    } else {
      // Si channel0PskLen=0 y channel0Psk=nullptr => canal sin cifrado (hash depende solo del nombre)
      cfg.hashCanal[0] = mesh_calcularChannelHash(cfg.channelName[0],
                                                  (channel0PskLen > 0) ? cfg.psk[0] : nullptr,
                                                  cfg.pskLen[0]);
    }
    return true;
  }




  /* EXPLICACIÓN DEL FUNCIONAMIENTO A GRANDES RASGOS DE LAS SIGUIENTES FUNCIONES

    Para todas estas funciones el proceso es:
      -- Usar un struct que se crea cuando se compila la implementación de Protobuffs de Mestastic
          Este será "meshtastic_Position" para los de posición, "meshtastic_Telemetry" para telemetría
          En este struct están los distintos campos que puede tener el mensaje. Si solo hay un campo como para mandar un mensaje de texto no hace falta un struct
      -- Rellenar ese struct con los datos que se quieran mandar
      -- Pasar el struct a "mesh_encodePb()" a la cual se le dice qué tipo de struct es, y esta rellena un array de bytes con la conversión de Protobuff
      -- Pasar el array de bytes (un binario) a mesh_sendData(), este llama a "mesh_encodePb()" que vuelve a meter el binario en otro protobuff
          Este segundo protobuf es uno de "Data" el cual contiene el portnum y la payload como tal. Un ejemplo podría ser este mensaje de posición:            
              0x08, 0x03, //Portnum, POSITION_APP 
              0x12, 0x1B, //Payload con 27 bytes
                0x0D, 0x00, 0x00, 0xDC, 0x18, //latitud 417071104 → 41.7071104°
                0x15, 0x00, 0x00, 0x7C, 0xFF, //longitud -8650752 → -0.8650752°
                0x18, 0x5B, //altura 91 metros
                0x25, 0x4C, 0x90, 0xAA, 0x69, //tiempo 1772785740 → 2026-03-06 08:29:00 UTC
                0x28, 0x01, //location source, manual
                0x78, 0x00, //velocidad en el suelo
                0x80, 0x01, 0x00, //"ground-track"
                0xB8, 0x01, 0x0D, //bits de precision, 13
              0x48, 0x01 //Has bitfield            
          El protobuf de "Data" es el que está más tabulado a la izquierda y contiene el portnum, la payload y un campo que indica que hay un bitfield (el mensaje de posición)
          Cuando el mensaje ya ha sido encapsulado en el protobuf "Data", se puede mandar por LoRa
      -- Dentro de mesh_sendData() se manda el mensaje por LoRa. Se le añaden los 16 bytes que preceden a todas las payloads de Meshtastic (To, From, Id...) y se manda
          El resto de variables que se meten a mesh_sendData() sirven para variar los 16 bytes de la cabecera.
*/

  bool mesh_mandarNodeID(const MeshConfig& cfg, uint8_t channelIndex,
                        uint32_t toNodeNum, uint8_t nextHop,
                        int8_t hopLimit, int8_t wantAck) 
  {
    meshtastic_User u = meshtastic_User_init_zero;

    char id[16];
    snprintf(id, sizeof(id), "!%08lx", (unsigned long)cfg.nodeNum);
    strncpy(u.id, id, sizeof(u.id));

    strncpy(u.long_name, cfg.longName, sizeof(u.long_name));
    strncpy(u.short_name, cfg.shortName, sizeof(u.short_name));

    const uint8_t mac[6] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB};
    memcpy(u.macaddr, mac, sizeof(mac));

    u.hw_model = meshtastic_HardwareModel_RAK4631;
    u.is_licensed = false;
    u.role = meshtastic_Config_DeviceConfig_Role_CLIENT;

    uint8_t userPb[128];
    size_t userLen = 0;
    if (!mesh_encodePb(meshtastic_User_fields, &u, userPb, sizeof(userPb), userLen)) return false;

    return mesh_sendData(meshtastic_PortNum_NODEINFO_APP, userPb, userLen,
                        cfg, channelIndex, toNodeNum, nextHop, hopLimit, wantAck);
  }

  bool mesh_mandarPosicion(float lat_deg, float lon_deg, int alt_m,
                          uint32_t utcTime,
                          const MeshConfig& cfg, uint8_t channelIndex,
                          uint32_t toNodeNum, uint8_t nextHop,
                          int8_t hopLimit, int8_t wantAck)
  {
    meshtastic_Position p = meshtastic_Position_init_zero;

    p.has_latitude_i = true;
    p.latitude_i = (int32_t)llround((double)lat_deg * 1e7);

    p.has_longitude_i = true;
    p.longitude_i = (int32_t)llround((double)lon_deg * 1e7);

    p.has_altitude = true;
    p.altitude = alt_m;

    p.location_source = meshtastic_Position_LocSource_LOC_MANUAL;
    p.altitude_source = meshtastic_Position_AltSource_ALT_MANUAL;
    p.time = utcTime;

    uint8_t posPb[128];
    size_t posLen = 0;
    if (!mesh_encodePb(meshtastic_Position_fields, &p, posPb, sizeof(posPb), posLen)) return false;

    return mesh_sendData(meshtastic_PortNum_POSITION_APP, posPb, posLen,
                        cfg, channelIndex, toNodeNum, nextHop, hopLimit, wantAck);
  }

  bool mesh_mandarTelemetry(uint8_t battery_pct, float voltage, float channel_util, float air_util_tx,
                            const MeshConfig& cfg, uint8_t channelIndex,
                            uint32_t toNodeNum, uint8_t nextHop,
                            int8_t hopLimit, int8_t wantAck)
  {
    meshtastic_Telemetry t = meshtastic_Telemetry_init_zero;
    t.time = 0;

    t.which_variant = meshtastic_Telemetry_device_metrics_tag;
    t.variant.device_metrics = meshtastic_DeviceMetrics_init_zero;

    t.variant.device_metrics.has_battery_level = true;
    t.variant.device_metrics.battery_level = battery_pct;

    t.variant.device_metrics.has_voltage = true;
    t.variant.device_metrics.voltage = voltage;

    t.variant.device_metrics.has_channel_utilization = true;
    t.variant.device_metrics.channel_utilization = channel_util;

    t.variant.device_metrics.has_air_util_tx = true;
    t.variant.device_metrics.air_util_tx = air_util_tx;

    t.variant.device_metrics.has_uptime_seconds = true;
    t.variant.device_metrics.uptime_seconds = millis() / 1000;

    uint8_t telPb[128];
    size_t telLen = 0;
    if (!mesh_encodePb(meshtastic_Telemetry_fields, &t, telPb, sizeof(telPb), telLen)) return false;

    return mesh_sendData(meshtastic_PortNum_TELEMETRY_APP, telPb, telLen,
                        cfg, channelIndex, toNodeNum, nextHop, hopLimit, wantAck);
  }

  bool mesh_mandarTexto(const char* txt, uint32_t length,
                        const MeshConfig& cfg, uint8_t channelIndex,
                        uint32_t toNodeNum, uint8_t nextHop,
                        int8_t hopLimit, int8_t wantAck) 
  {
    return mesh_sendData(meshtastic_PortNum_TEXT_MESSAGE_APP,
                        (const uint8_t*)txt, (size_t)length,
                        cfg, channelIndex, toNodeNum, nextHop, hopLimit, wantAck);
  }

  bool mesh_mandarBinario(const char* array, uint32_t length,
                          const MeshConfig& cfg, uint8_t channelIndex,
                          uint32_t toNodeNum, uint8_t nextHop,
                          int8_t hopLimit, int8_t wantAck) 
  {
    return mesh_sendData(meshtastic_PortNum_PRIVATE_APP, // Tiene que ser del 256 al 511 y no puede ser 257 que ese ya está usado por ATAK, este es el 256
                        (const uint8_t*)array, (size_t)length,
                        cfg, channelIndex, toNodeNum, nextHop, hopLimit, wantAck);
  }




  bool mesh_decodificarSiEsTexto(const uint8_t* PBarray, uint32_t PBlength,
                                 char* StringArray, uint32_t StringArraySize,
                                 uint32_t* StringLength)
  {
    if(PBarray == nullptr || StringArray == nullptr || StringLength == nullptr || StringArraySize == 0) {
      return false;
    }

    StringArray[0] = '\0';
    *StringLength = 0;

    meshtastic_Data msg = meshtastic_Data_init_zero;
    pb_istream_t s1 = pb_istream_from_buffer(PBarray, PBlength);
    if(!pb_decode(&s1, meshtastic_Data_fields, &msg)) {
      return false;
    }

    if(msg.portnum != meshtastic_PortNum_TEXT_MESSAGE_APP) {
      return false;
    }

    uint32_t textLen = msg.payload.size;

    if(textLen >= StringArraySize) {
      textLen = StringArraySize - 1;
    }

    memcpy(StringArray, msg.payload.bytes, textLen);
    StringArray[textLen] = '\0';
    *StringLength = textLen;

    return true;
  }

// --------------------------------------------- //







/*
  void sendMeshtastic(char* bytes, int length){
    byte transmitBuffer[256];

    char header[] = {0xFF, 0xFF, 0xFF, 0xFF, 0x67, 0x45, 0x23, 0x01, 0xE1, 0xEB, 0x61, 0x05, 0x63, 0x33, 0x00, 0x67};
    int point=0;
    for(int i=0; i<(sizeof(header)); i++){
      transmitBuffer[point] = header[i];
      point++;
    }

    for(int i=8; i<12; i++){
      transmitBuffer[i] ^= random()&0xFF; //Generar un ID aleatorio, para que interpreten que es un mensaje nuevo
    }

    for(int i=0; i<(length); i++){
      transmitBuffer[point] = bytes[i];
      point++;
    }

    for(int i=0; i<point; i++){		
      DPRINTF("%X ", transmitBuffer[i]);
    }
    DPRINTLN();

    //radio.standby();
    transmissionState = radio.startTransmit(transmitBuffer, point);
    DPRINTLN(transmissionState);
    //radio.sleep();
  }

  void sendMeshtasticString(char* string, int length){
    if(length <= 0){
      return; //Tiene que ser positiva y mayor a 0 para que tenga sentido
    }

    char transmitBuffer[256];

    char header[] = {0x08, 0x01, 0x12};
    int point=0;
    for(int i=0; i<(sizeof(header)); i++){
      transmitBuffer[point] = header[i];
      point++;
    }

    int lengthCopia = length;
    while(lengthCopia > 0){
      transmitBuffer[point] = lengthCopia & 0x7F;
      if(lengthCopia > 0x7F){transmitBuffer[point] |= 0x80;	}

      lengthCopia >>= 7;
      point++;
    }	

    for(int i=0; i<(length); i++){
      transmitBuffer[point] = string[i];
      point++;
    }

    transmitBuffer[point] = 0x48;
    point++;
    transmitBuffer[point] = 0x01;
    point++;

    for(int i=0; i<point; i++){		
      DPRINTF("%c", transmitBuffer[i]);
    }
    DPRINTLN();

    for(int i=0; i<length; i++){		
      DPRINTF("%X ", string[i]);
    }
    DPRINTF(" %d", length);
    DPRINTLN();

    sendMeshtastic(transmitBuffer, point);
  }

  void sendMeshtasticNodeID(){
    char buffer[] = {
      0x08, 0x04, // Data.portnum = NODEINFO_APP
      0x12, 0x32, // Data.payload length = 50
        0x0A, 0x09, 0x21, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,   // "!01234567"
        0x12, 0x0F, 0x52, 0x41, 0x4B, 0x34, 0x36, 0x33, 0x30, 0x5F, 0x41, 0x72, 0x64, 0x75, 0x69, 0x6E, 0x6F, // User.long_name = "RAK4630_Arduino"
        0x1A, 0x04, 0x34, 0x35, 0x36, 0x37, // User.short_name = "4567"
        0x22, 0x06, 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, // User.macaddr
        0x28, 0x69, //Id de la placa, el mismo que las RAK4630 oficiales
        0x30, 0x01, //Booleano de si está registrado, para que meshtastic crea que es una placa con firmware de Meshtastic
        0x38, 0x01, // User.role = CLIENT_MUTE
        0x48, 0x00,
      0x48, 0x01 //Para sacar por MQTT
    };

    sendMeshtastic(buffer, sizeof(buffer));
  }

  void sendMeshtasticPositonApp(){	
    char buffer[] = {
      0x08, 0x03, //Portnum, POSITION_APP 
      0x12, 0x1B, //Payload con 27 bytes
        0x0D, 0x00, 0x00, 0xDC, 0x18, //latitud 417071104 → 41.7071104°
        0x15, 0x00, 0x00, 0x7C, 0xFF, //longitud -8650752 → -0.8650752°
        0x18, 0x5B, //altura 91 metros
        0x25, 0x4C, 0x90, 0xAA, 0x69, //tiempo 1772785740 → 2026-03-06 08:29:00 UTC
        0x28, 0x01, //location source, manual
        0x78, 0x00, //velocidad en el suelo
        0x80, 0x01, 0x00, //"ground-track"
        0xB8, 0x01, 0x0D, //bits de precision, 13
      0x48, 0x01 //Has bitfield
    };

    sendMeshtastic(buffer, sizeof(buffer));
  }
*/