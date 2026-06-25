#include "radioMesh.h"

#include <cstring>
#include <cstdio>
#include <cmath>

const uint8_t MeshtasticRadio::MESHTASTIC_DEFAULT_KEY_16[16] = { // REVISAR
  0xD4, 0xF1, 0xBB, 0x3A, 0x20, 0x29, 0x07, 0x59,
  0xF0, 0xBC, 0xFF, 0xAB, 0xCF, 0x4E, 0x69, 0x01
};

MeshtasticRadio::MeshtasticRadio(ControlRadio& mailbox) : _mailbox(mailbox) {
  editarConfig();
}

const MeshtasticRadio::MeshConfig& MeshtasticRadio::getConfig() const {
  return _config;
}

bool MeshtasticRadio::editarConfig(uint32_t nodeNum,
                                   const char* longName,
                                   const char* shortName,
                                   uint8_t hopLimit,
                                   bool wantAck,
                                   bool viaMqtt,
                                   const ChannelConfig* channels,
                                   uint8_t cantidadCanales)
{
  if (hopLimit > 7) return false;
  if (cantidadCanales < 1 || cantidadCanales > 8) return false;

  const ChannelConfig defaultChannels[1] = {
    { DEFAULT_CHANNEL_NAME, DEFAULT_CHANNEL_PSK, DEFAULT_CHANNEL_PSK_LEN }
  };

  if (channels == nullptr) {
    channels = defaultChannels;
    cantidadCanales = 1;
  }

  for (uint8_t i = 0; i < cantidadCanales; i++) {
    if (channels[i].pskLen > 32) return false;
    if ((channels[i].psk == nullptr) && (channels[i].pskLen > 0)) return false;
  }

  memset(&_config, 0, sizeof(MeshConfig));

  _config.nodeNum = nodeNum;

  strncpy(_config.longName, longName ? longName : "", sizeof(_config.longName) - 1);
  _config.longName[sizeof(_config.longName) - 1] = '\0';

  strncpy(_config.shortName, shortName ? shortName : "", sizeof(_config.shortName) - 1);
  _config.shortName[sizeof(_config.shortName) - 1] = '\0';

  _config.defaultHopLimit = hopLimit;
  _config.defaultWantAck  = wantAck;
  _config.defaultViaMqtt  = viaMqtt;
  _config.cantidadCanales = cantidadCanales;

  for (uint8_t i = 0; i < cantidadCanales; i++) {
    const char* effectiveName = channels[i].name;

    if ((effectiveName == nullptr) || (effectiveName[0] == '\0')) {
      effectiveName = (i == 0) ? DEFAULT_CHANNEL_NAME : "LongFast";
    }

    strncpy(_config.channelName[i], effectiveName, sizeof(_config.channelName[i]) - 1);
    _config.channelName[i][sizeof(_config.channelName[i]) - 1] = '\0';

    _config.pskLen[i] = channels[i].pskLen;

    if ((channels[i].psk != nullptr) && (channels[i].pskLen > 0)) {
      memcpy(_config.psk[i], channels[i].psk, channels[i].pskLen);
    }

    _config.hashCanal[i] = calcularChannelHash_(
      _config.channelName[i],
      (_config.pskLen[i] > 0) ? _config.psk[i] : nullptr,
      _config.pskLen[i]
    );
  }

  return true;
}




bool MeshtasticRadio::scanTransmit_(const uint8_t* data, size_t len) {
  return _mailbox.pushTx(data, len);
}

uint8_t MeshtasticRadio::buildFlags_(uint8_t hopLimit, bool wantAck, bool viaMqtt) {
  hopLimit &= 0x07;
  const uint8_t hopStart = hopLimit;
  return (uint8_t)(
    (hopLimit) |
    (wantAck ? 0x08 : 0x00) |
    (viaMqtt ? 0x10 : 0x00) |
    (hopStart << 5)
  );
}

void MeshtasticRadio::buildLoraHeader_(uint8_t out16[16], const MeshConfig& cfg,
                                       uint8_t channelIndex, uint32_t toNodeNum,
                                       uint8_t flags, uint8_t nextHop)
{
  out16[0] = (uint8_t)(toNodeNum & 0xFF);
  out16[1] = (uint8_t)((toNodeNum >> 8) & 0xFF);
  out16[2] = (uint8_t)((toNodeNum >> 16) & 0xFF);
  out16[3] = (uint8_t)((toNodeNum >> 24) & 0xFF);

  out16[4] = (uint8_t)(cfg.nodeNum & 0xFF);
  out16[5] = (uint8_t)((cfg.nodeNum >> 8) & 0xFF);
  out16[6] = (uint8_t)((cfg.nodeNum >> 16) & 0xFF);
  out16[7] = (uint8_t)((cfg.nodeNum >> 24) & 0xFF);

  for (int i = 8; i < 12; i++) {
    out16[i] = (uint8_t)random(0, 256);
  }

  out16[12] = flags;
  out16[13] = cfg.hashCanal[channelIndex];
  out16[14] = (toNodeNum == 0xFFFFFFFF) ? 0x00 : nextHop;
  out16[15] = 0x00;
}

bool MeshtasticRadio::encodePb_(const pb_msgdesc_t* msg, const void* src,
                                uint8_t* out, size_t outMax, size_t& outLen)
{
  pb_ostream_t s = pb_ostream_from_buffer(out, outMax);
  if (!pb_encode(&s, msg, src)) return false;
  outLen = s.bytes_written;
  return true;
}

bool MeshtasticRadio::sendSubPacketOverLora_(const uint8_t* subpkt, size_t subpktLen,
                                             uint8_t channelIndex,
                                             uint32_t toNodeNum, uint8_t nextHop,
                                             int8_t hopLimit, int8_t wantAck)
{
  if (channelIndex >= _config.cantidadCanales) return false;

  const uint8_t effHop = (hopLimit == -1) ? _config.defaultHopLimit : (uint8_t)hopLimit;
  const bool    effAck = (wantAck == -1) ? _config.defaultWantAck : (wantAck != 0);

  uint8_t tx[256];
  uint8_t hdr[16];

  const uint8_t flags = buildFlags_(effHop, effAck, _config.defaultViaMqtt);
  buildLoraHeader_(hdr, _config, channelIndex, toNodeNum, flags, nextHop);

  if (16 + subpktLen > sizeof(tx)) return false;

  memcpy(tx, hdr, 16);
  memcpy(tx + 16, subpkt, subpktLen);

  return scanTransmit_(tx, 16 + subpktLen);
}

bool MeshtasticRadio::sendData_(meshtastic_PortNum portnum, const uint8_t* payload, size_t payloadLen,
                                uint8_t channelIndex,
                                uint32_t toNodeNum, uint8_t nextHop,
                                int8_t hopLimit, int8_t wantAck)
{
  meshtastic_Data d = meshtastic_Data_init_zero;
  d.portnum = portnum;

  if (payloadLen > sizeof(d.payload.bytes)) {
    return false;

  }

  if ((payload == nullptr) && (payloadLen > 0)) {
    return false;
  }

  d.payload.size = payloadLen;
  if (payloadLen > 0) {
    memcpy(d.payload.bytes, payload, payloadLen);
  }

  d.has_bitfield = true;
  d.bitfield = 1;

  //d.has_dest = true;
  //d.dest = toNodeNum;

  uint8_t subpkt[256];
  size_t subLen = 0;
  if (!encodePb_(meshtastic_Data_fields, &d, subpkt, sizeof(subpkt), subLen)) {
    return false;
  }

  return sendSubPacketOverLora_(subpkt, subLen, channelIndex, toNodeNum, nextHop, hopLimit, wantAck);
}

uint8_t MeshtasticRadio::xorHash_(const uint8_t* p, size_t len) {
  uint8_t h = 0;
  for (size_t i = 0; i < len; i++) h ^= p[i];
  return h;
}

uint8_t MeshtasticRadio::calcularChannelHash_(const char* channelName, const uint8_t* psk, size_t pskLen) {
  if (!channelName || channelName[0] == '\0') channelName = "LongFast";

  uint8_t h = xorHash_((const uint8_t*)channelName, strlen(channelName));

  uint8_t keyBuf[32] = {0};
  size_t keyLen = 0;

  if (!psk || pskLen == 0) {
    keyLen = 0;
  } else if (pskLen == 1) {
    uint8_t idx = psk[0];
    if (idx == 0) {
      keyLen = 0;
    } else {
      keyLen = 16;
      memcpy(keyBuf, MESHTASTIC_DEFAULT_KEY_16, 16);
      keyBuf[15] = idx;
    }
  } else if (pskLen < 16) {
    keyLen = 16;
    memcpy(keyBuf, psk, pskLen);
  } else if (pskLen == 16 || pskLen == 32) {
    keyLen = pskLen;
    memcpy(keyBuf, psk, pskLen);
  } else if (pskLen < 32) {
    keyLen = 32;
    memcpy(keyBuf, psk, pskLen);
  } else {
    keyLen = 32;
    memcpy(keyBuf, psk, 32);
  }

  h ^= xorHash_(keyBuf, keyLen);
  return h;
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

bool MeshtasticRadio::mandarNodeID(uint8_t channelIndex,
                                   uint32_t toNodeNum, uint8_t nextHop,
                                   int8_t hopLimit, int8_t wantAck)
{
  meshtastic_User u = meshtastic_User_init_zero;

  char id[16];
  snprintf(id, sizeof(id), "!%08lx", (unsigned long)_config.nodeNum);
  strncpy(u.id, id, sizeof(u.id) - 1);

  strncpy(u.long_name, _config.longName, sizeof(u.long_name) - 1);
  strncpy(u.short_name, _config.shortName, sizeof(u.short_name) - 1);

  memcpy(u.macaddr, DEFAULT_MAC, sizeof(DEFAULT_MAC));

  u.hw_model = meshtastic_HardwareModel_RAK4631;
  u.is_licensed = false;
  u.role = meshtastic_Config_DeviceConfig_Role_CLIENT;

  uint8_t userPb[128];
  size_t userLen = 0;
  if (!encodePb_(meshtastic_User_fields, &u, userPb, sizeof(userPb), userLen)) return false;

  return sendData_(meshtastic_PortNum_NODEINFO_APP, userPb, userLen,
                   channelIndex, toNodeNum, nextHop, hopLimit, wantAck);
}

bool MeshtasticRadio::mandarPosicion(float lat_deg, float lon_deg, int alt_m,
                                     uint32_t utcTime,
                                     uint8_t channelIndex,
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
  if (!encodePb_(meshtastic_Position_fields, &p, posPb, sizeof(posPb), posLen)) return false;

  return sendData_(meshtastic_PortNum_POSITION_APP, posPb, posLen,
                   channelIndex, toNodeNum, nextHop, hopLimit, wantAck);
}

bool MeshtasticRadio::mandarTelemetry(uint8_t battery_pct, float voltage,
                                      float channel_util, float air_util_tx,
                                      uint8_t channelIndex,
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
  if (!encodePb_(meshtastic_Telemetry_fields, &t, telPb, sizeof(telPb), telLen)) return false;

  return sendData_(meshtastic_PortNum_TELEMETRY_APP, telPb, telLen,
                   channelIndex, toNodeNum, nextHop, hopLimit, wantAck);
}

bool MeshtasticRadio::mandarTexto(const char* txt, uint32_t length,
                                  uint8_t channelIndex,
                                  uint32_t toNodeNum, uint8_t nextHop,
                                  int8_t hopLimit, int8_t wantAck)
{
  return sendData_(meshtastic_PortNum_TEXT_MESSAGE_APP,
                   reinterpret_cast<const uint8_t*>(txt), (size_t)length,
                   channelIndex, toNodeNum, nextHop, hopLimit, wantAck);
}

bool MeshtasticRadio::mandarAlarma(const char* txt, uint32_t length,
                                  uint8_t channelIndex,
                                  uint32_t toNodeNum, uint8_t nextHop,
                                  int8_t hopLimit, int8_t wantAck)
{
  return sendData_((meshtastic_PortNum)11, /* Este es un Portnum específico para mandar alarmas, pero no está en la librería de los protobufs*/
                   reinterpret_cast<const uint8_t*>(txt), (size_t)length,
                   channelIndex, toNodeNum, nextHop, hopLimit, wantAck);
}

bool MeshtasticRadio::mandarBinario(const uint8_t* data, uint32_t length,
                                    uint8_t channelIndex,
                                    uint32_t toNodeNum, uint8_t nextHop,
                                    int8_t hopLimit, int8_t wantAck)
{
  return sendData_(meshtastic_PortNum_PRIVATE_APP,
                   data, (size_t)length,
                   channelIndex, toNodeNum, nextHop, hopLimit, wantAck);
}

bool MeshtasticRadio::pedirRouting(uint32_t toNodeNum, 
                                   uint8_t channelIndex, uint8_t nextHop,
                                   int8_t hopLimit, int8_t wantAck)
{
  meshtastic_Routing r = meshtastic_Routing_init_zero;

  r.which_variant = meshtastic_Routing_route_request_tag;
  //r.variant.route_request = meshtastic_RouteDiscovery_init_zero; // Estoi da problemas no se porque

  // Opcional: meter el nodo local como primer salto conocido
  // r.variant.route_request.route_count = 1;
  // r.variant.route_request.route[0] = _config.nodeNum;

  uint8_t routingPb[128];
  size_t routingLen = 0;

  if (!encodePb_(meshtastic_Routing_fields, &r,
                 routingPb, sizeof(routingPb), routingLen)) {
    return false;
  }

  return sendData_(meshtastic_PortNum_TRACEROUTE_APP,
                   routingPb, routingLen,
                   channelIndex, toNodeNum, nextHop, hopLimit, wantAck);
}






bool MeshtasticRadio::decodificarSiEsTexto(const uint8_t* PBarray, uint32_t PBlength,
                                           char* StringArray, uint32_t StringArraySize,
                                           uint32_t* StringLength)
{
  if (PBarray == nullptr || StringArray == nullptr || StringLength == nullptr || StringArraySize == 0) {
    return false;
  }

  StringArray[0] = '\0';
  *StringLength = 0;

  meshtastic_Data msg = meshtastic_Data_init_zero;
  pb_istream_t s1 = pb_istream_from_buffer(PBarray, PBlength);
  if (!pb_decode(&s1, meshtastic_Data_fields, &msg)) {
    return false;
  }

  if (msg.portnum != meshtastic_PortNum_TEXT_MESSAGE_APP) {
    return false;
  }

  uint32_t textLen = msg.payload.size;
  if (textLen >= StringArraySize) {
    textLen = StringArraySize - 1;
  }

  memcpy(StringArray, msg.payload.bytes, textLen);
  StringArray[textLen] = '\0';
  *StringLength = textLen;

  return true;
}


bool MeshtasticRadio::decodificarSiEsAlarma(const uint8_t* PBarray, uint32_t PBlength,
                                            char* StringArray, uint32_t StringArraySize,
                                            uint32_t* StringLength)
{
  if (PBarray == nullptr || StringArray == nullptr || StringLength == nullptr || StringArraySize == 0) {
    return false;
  }

  StringArray[0] = '\0';
  *StringLength = 0;

  meshtastic_Data msg = meshtastic_Data_init_zero;
  pb_istream_t s1 = pb_istream_from_buffer(PBarray, PBlength);
  if (!pb_decode(&s1, meshtastic_Data_fields, &msg)) {
    return false;
  }

  if (msg.portnum != (meshtastic_PortNum)11 /* Este es un Portnum específico para mandar alarmas, pero no está en la librería de los protobufs*/) {
    return false;
  }

  uint32_t textLen = msg.payload.size;
  if (textLen >= StringArraySize) {
    textLen = StringArraySize - 1;
  }

  memcpy(StringArray, msg.payload.bytes, textLen);
  StringArray[textLen] = '\0';
  *StringLength = textLen;

  return true;
}


bool MeshtasticRadio::decodificarSiEsBinario(const uint8_t* PBarray, uint32_t PBlength,
                                             uint8_t* BinaryArray, uint32_t BinaryArraySize,
                                             uint32_t* BinaryLength)
{
  if (PBarray == nullptr || BinaryArray == nullptr || BinaryLength == nullptr) {
    return false;
  }

  *BinaryLength = 0;

  meshtastic_Data msg = meshtastic_Data_init_zero;
  pb_istream_t s1 = pb_istream_from_buffer(PBarray, PBlength);
  if (!pb_decode(&s1, meshtastic_Data_fields, &msg)) {
    return false;
  }

  if (msg.portnum != meshtastic_PortNum_PRIVATE_APP) {
    return false;
  }

  uint32_t dataLen = msg.payload.size;
  if (dataLen > BinaryArraySize) {
    return false;
  }

  if (dataLen > 0) {
    memcpy(BinaryArray, msg.payload.bytes, dataLen);
  }

  *BinaryLength = dataLen;
  return true;
}