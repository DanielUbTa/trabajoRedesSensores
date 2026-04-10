#pragma once

#include <Arduino.h>
#include <RadioLib.h>
#include <FreeRTOS.h>
#include <task.h>

class ControlRadio {
public:
  static constexpr size_t MAX_MSG_LEN  = 256;
  static constexpr size_t RX_QUEUE_LEN = 8;
  static constexpr size_t TX_QUEUE_LEN = 8;

  struct Message {
    uint16_t len = 0;
    uint8_t  data[MAX_MSG_LEN] = {0};
    int16_t  rssi = 0;
    int8_t   snr = 0;
  };

  enum State : uint8_t { // Máquina de estados del SX1262
    STATE_REARM_RX = 0,
    STATE_AWAITING_RADIO,
    STATE_RX_PROCESS,
    STATE_TX_START
  };

  /*!
    \brief  Constructor de la clase. Asocia la librería a una instancia ya creada de SX1262
    \param  radio: referencia al objeto SX1262 que gestionará la librería
  */
  ControlRadio(SX1262& radio);


  /*!
    \brief  Inicializa la radio, configura las interrupciones y crea la task interna de FreeRTOS
    \param  freqMHz: frecuencia LoRa en MHz
    \param  bw: ancho de banda LoRa
    \param  sf: spreading factor
    \param  cr: coding rate
    \param  syncWord: palabra de sincronización
    \param  power: potencia de transmisión
    \param  preambleLength: longitud del preámbulo
    \param  tcxoVoltage: tensión del TCXO utilizada por RadioLib
    \param  useRegulatorLDO: selecciona el regulador interno usado por el chip
    \param  taskStack: tamaño de pila de la task interna
    \param  taskPriority: prioridad de la task interna
    \returns true si la inicialización ha sido correcta; false en caso contrario
  */
  bool begin(float freqMHz,
             float bw,
             uint8_t sf,
             uint8_t cr,
             uint8_t syncWord,
             int8_t power,
             uint16_t preambleLength,
             float tcxoVoltage = 1.6,
             bool useRegulatorLDO = false,
             uint16_t taskStack = 4096,
             UBaseType_t taskPriority = 3);
  
  

  /*!
    \brief  Encola un mensaje binario para su transmisión asíncrona
    \param  data: puntero al buffer de datos a transmitir
    \param  len: longitud del mensaje en bytes
    \returns true si el mensaje se ha encolado correctamente; false en caso contrario
  */
  bool pushTx(const uint8_t* data, size_t len);

  /*!
    \brief  Encola un mensaje de texto terminado en null para su transmisión asíncrona
    \param  text: puntero a la cadena de texto a transmitir
    \returns true si el mensaje se ha encolado correctamente; false en caso contrario
  */
  bool pushTx(const char* text);



  /*!
    \brief  Indica si hay al menos un mensaje pendiente de leer en la cola RX
    \returns true si hay mensajes disponibles en RX; false en caso contrario
  */
  bool availableRx() const;



  /*!
    \brief  Extrae el siguiente mensaje recibido de la cola RX
    \param  msg: estructura donde se copiará el mensaje extraído
    \returns true si se ha extraído un mensaje correctamente; false si la cola RX está vacía
  */
  bool popRx(Message& msg);

  /*!
    \brief  Extrae el siguiente mensaje recibido de la cola RX y lo copia a un buffer del usuario
    \param  out: buffer de salida donde se copiarán los datos recibidos
    \param  maxLen: tamaño máximo disponible en el buffer de salida
    \param  outLen: longitud real del mensaje copiado
    \param  rssi: puntero opcional donde devolver el RSSI del mensaje
    \param  snr: puntero opcional donde devolver el SNR del mensaje
    \returns true si se ha extraído y copiado el mensaje correctamente; false en caso contrario
  */
  bool popRx(uint8_t* out, size_t maxLen, size_t& outLen, int16_t* rssi = nullptr, int8_t* snr = nullptr);



  /*!
    \brief  Permite inspeccionar el siguiente mensaje recibido sin extraerlo de la cola RX
    \param  msg: estructura donde se copiará el mensaje consultado
    \returns true si existe un mensaje en RX y se ha copiado correctamente; false en caso contrario
  */
  bool peekRx(Message& msg);

  /*!
    \brief  Permite inspeccionar el siguiente mensaje recibido sin extraerlo de la cola RX, copiándolo a un buffer del usuario
    \param  out: buffer de salida donde se copiarán los datos recibidos
    \param  maxLen: tamaño máximo disponible en el buffer de salida
    \param  outLen: longitud real del mensaje copiado
    \param  rssi: puntero opcional donde devolver el RSSI del mensaje
    \param  snr: puntero opcional donde devolver el SNR del mensaje
    \returns true si existe un mensaje en RX y se ha copiado correctamente; false en caso contrario
  */
  bool peekRx(uint8_t* out, size_t maxLen, size_t& outLen, int16_t* rssi = nullptr, int8_t* snr = nullptr);

  /*!
    \brief  Indica si la librería tiene una transmisión en curso
    \returns true si hay una transmisión activa; false en caso contrario
  */
  bool isBusy() const;

  /*!
    \brief  Devuelve el estado actual de la máquina de estados interna
    \returns estado actual de la librería
  */
  State getState() const;

  /*!
    \brief  Devuelve el último código de error registrado por la librería o por RadioLib
    \returns último código de error registrado
  */
  int16_t getLastError() const;

  /*!
    \brief  Devuelve cuántos mensajes RX se han descartado por falta de espacio en la cola
    \returns número de mensajes RX descartados
  */
  uint32_t getRxDroppedCount() const;

  /*!
    \brief  Devuelve cuántos mensajes TX se han descartado por falta de espacio en la cola
    \returns número de mensajes TX descartados
  */
  uint32_t getTxDroppedCount() const;

  /*!
    \brief  Devuelve cuántos mensajes se han recibido correctamente
    \returns número de recepciones correctas
  */
  uint32_t getRxOkCount() const;

  /*!
    \brief  Devuelve cuántas transmisiones se han completado correctamente
    \returns número de transmisiones correctas
  */
  uint32_t getTxOkCount() const;

  /*!
    \brief  Devuelve cuántos errores de recepción se han producido
    \returns número de errores de recepción
  */
  uint32_t getRxErrorCount() const;

  /*!
    \brief  Devuelve cuántos errores de transmisión se han producido
    \returns número de errores de transmisión
  */
  uint32_t getTxErrorCount() const;

  /*!
    \brief  ISR estática asociada a la interrupción de la radio. Marca el evento y despierta la task interna
  */
  static void radioIsrRouter();

private:
  SX1262& _radio;
  static ControlRadio* _instance;

  TaskHandle_t _taskHandle = nullptr;

  volatile bool _irqFlag = false;
  volatile bool _txInProgress = false;
  volatile State _state = STATE_REARM_RX;

  volatile int16_t _lastError = RADIOLIB_ERR_NONE;
  volatile int16_t _lastTxStartState = RADIOLIB_ERR_NONE;

  Message _rxQueue[RX_QUEUE_LEN];
  volatile uint8_t _rxHead = 0;
  volatile uint8_t _rxTail = 0;
  volatile uint8_t _rxCount = 0;

  Message _txQueue[TX_QUEUE_LEN];
  volatile uint8_t _txHead = 0;
  volatile uint8_t _txTail = 0;
  volatile uint8_t _txCount = 0;

  uint8_t _radioRxBuffer[MAX_MSG_LEN] = {0};

  volatile uint32_t _rxDropped = 0;
  volatile uint32_t _txDropped = 0;
  volatile uint32_t _rxOk = 0;
  volatile uint32_t _txOk = 0;
  volatile uint32_t _rxErr = 0;
  volatile uint32_t _txErr = 0;

  static void taskEntry(void* arg);
  void taskLoop();

  bool pushTx_(const uint8_t* data, size_t len);
  bool popTx_(Message& msg);
  bool pushRx_(const uint8_t* data, size_t len, int16_t rssi, int8_t snr);
  bool popRx_(Message& msg);
  bool peekRx_(Message& msg) const;

  inline uint8_t nextIndex_(uint8_t idx, uint8_t size) const {
    return static_cast<uint8_t>((idx + 1U) % size);
  }
};