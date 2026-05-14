#include "controlRadio.h"

ControlRadio* ControlRadio::_instance = nullptr;

ControlRadio::ControlRadio(SX1262& radio) : _radio(radio) {
  _instance = this;
}

// Lo que se ejecuta cuando hay una interrupción del SX1262
void ControlRadio::radioIsrRouter() {
  if (_instance != nullptr) {
    _instance->_irqFlag = true;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (_instance->_taskHandle != nullptr) {
      vTaskNotifyGiveFromISR(_instance->_taskHandle, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  }
}

bool ControlRadio::begin(float freqMHz, float bw, uint8_t sf, uint8_t cr, 
                         uint8_t syncWord, int8_t power, uint16_t preambleLength,
                         float tcxoVoltage, bool useRegulatorLDO,
                         uint16_t taskStack, UBaseType_t taskPriority) 
{
  _lastError = _radio.begin(freqMHz, bw, sf, cr, syncWord, power, preambleLength, tcxoVoltage, useRegulatorLDO);
  if (_lastError != RADIOLIB_ERR_NONE) {
    return false;
  }

  _radio.setDio2AsRfSwitch(true);
  _radio.setDio1Action(ControlRadio::radioIsrRouter);

  _state = STATE_REARM_RX;
  _txInProgress = false;
  _irqFlag = false;

  BaseType_t ok;

  ok = xTaskCreate(
    ControlRadio::taskEntry,
    "ControlRadio",
    taskStack,
    this,
    taskPriority,
    &_taskHandle
  );
  return (ok == pdPASS);
}

// Truco porque FreeRTOS no puede arrancar directamente un método no estático de una clase
void ControlRadio::taskEntry(void* arg) {
  ControlRadio* self = static_cast<ControlRadio*>(arg);
  self->taskLoop();
}





// La que se ocupa de todo
bool ControlRadio::pushTx_(const uint8_t* data, size_t len) {
  if ((data == nullptr) || (len == 0) || (len > MAX_MSG_LEN)) {
    return false;
  }

  taskENTER_CRITICAL();       // Se modifican variables qe se puden modificar desde otras tareas, entonces bloquear otras tareas
  if (_txCount >= TX_QUEUE_LEN) {
    taskEXIT_CRITICAL();
    _txDropped++;
    return false;
  }

  Message& slot = _txQueue[_txHead]; // Dirección del dato que se va a escribir = dirección de fifo[i]
  slot.len = static_cast<uint16_t>(len);
  memcpy(slot.data, data, len);

  _txHead = nextIndex_(_txHead, TX_QUEUE_LEN);
  _txCount++;
  taskEXIT_CRITICAL();

  if (_taskHandle != nullptr) {
    xTaskNotifyGive(_taskHandle);
  }

  return true;
}

// Wrapper público de pushTx_
bool ControlRadio::pushTx(const uint8_t* data, size_t len) {
  return pushTx_(data, len);
}

// Wrapper público de pushTx_
bool ControlRadio::pushTx(const char* text) {
  if (text == nullptr) {
    return false;
  }
  return pushTx_(reinterpret_cast<const uint8_t*>(text), strlen(text));
}

// Para cuando hay que mandar
bool ControlRadio::popTx_(Message& msg) {
  taskENTER_CRITICAL();
  if (_txCount == 0) {
    taskEXIT_CRITICAL();
    return false;
  }

  msg = _txQueue[_txTail];
  _txTail = nextIndex_(_txTail, TX_QUEUE_LEN);
  _txCount--;
  taskEXIT_CRITICAL();

  return true;
}




// Para cuando se ha recibido un mensaje
bool ControlRadio::pushRx_(const uint8_t* data, size_t len, int16_t rssi, int8_t snr) {
  if ((data == nullptr) || (len == 0) || (len > MAX_MSG_LEN)) {
    return false;
  }

  taskENTER_CRITICAL();
  if (_rxCount >= RX_QUEUE_LEN) {
    taskEXIT_CRITICAL();
    _rxDropped++;
    return false;
  }

  Message& slot = _rxQueue[_rxHead];
  slot.len = static_cast<uint16_t>(len);
  memcpy(slot.data, data, len);
  slot.rssi = rssi;
  slot.snr = snr;

  _rxHead = nextIndex_(_rxHead, RX_QUEUE_LEN);
  _rxCount++;
  taskEXIT_CRITICAL();

  return true;
}

// La que se ocupa de todo
bool ControlRadio::popRx_(Message& msg) {
  taskENTER_CRITICAL();
  if (_rxCount == 0) {
    taskEXIT_CRITICAL();
    return false;
  }

  msg = _rxQueue[_rxTail];
  _rxTail = nextIndex_(_rxTail, RX_QUEUE_LEN);
  _rxCount--;
  taskEXIT_CRITICAL();

  return true;
}

bool ControlRadio::availableRx() const {
  return (_rxCount > 0);
}

// Wrapper público de popRx_
bool ControlRadio::popRx(Message& msg) {
  return popRx_(msg);
}

// Wrapper público de popRx_
bool ControlRadio::popRx(uint8_t* out, size_t maxLen, size_t& outLen, int16_t* rssi, int8_t* snr) {
  outLen = 0;

  if ((out == nullptr) || (maxLen == 0)) {
    return false;
  }

  Message msg;
  if (!peekRx_(msg)) {
    return false;
  }

  if (msg.len > maxLen) {
    return false;
  }

  if (!popRx_(msg)) {
    return false;
  }

  memcpy(out, msg.data, msg.len);
  outLen = msg.len;

  if (rssi != nullptr) *rssi = msg.rssi;
  if (snr != nullptr) *snr = msg.snr;

  return true;
}



// La que se ocupa de todo, pero sin extraer de la cola
bool ControlRadio::peekRx_(Message& msg) const {
  taskENTER_CRITICAL();
  if (_rxCount == 0) {
    taskEXIT_CRITICAL();
    return false;
  }

  msg = _rxQueue[_rxTail];
  taskEXIT_CRITICAL();

  return true;
}

// Wrapper público de peekRx_
bool ControlRadio::peekRx(Message& msg) {
  return peekRx_(msg);
}

// Wrapper público de peekRx_
bool ControlRadio::peekRx(uint8_t* out, size_t maxLen, size_t& outLen, int16_t* rssi, int8_t* snr) {
  outLen = 0;

  if ((out == nullptr) || (maxLen == 0)) {
    return false;
  }

  Message msg;
  if (!peekRx_(msg)) {
    return false;
  }

  if (msg.len > maxLen) {
    return false;
  }

  memcpy(out, msg.data, msg.len);
  outLen = msg.len;

  if (rssi != nullptr) *rssi = msg.rssi;
  if (snr  != nullptr) *snr  = msg.snr;

  return true;
}




bool ControlRadio::isBusy() const {  return _txInProgress;  }
ControlRadio::State ControlRadio::getState() const {  return static_cast<State>(_state);  }
int16_t ControlRadio::getLastError() const {  return _lastError;  }

uint32_t ControlRadio::getRxDroppedCount() const { return _rxDropped; }
uint32_t ControlRadio::getTxDroppedCount() const { return _txDropped; }
uint32_t ControlRadio::getRxOkCount() const { return _rxOk; }
uint32_t ControlRadio::getTxOkCount() const { return _txOk; }
uint32_t ControlRadio::getRxErrorCount() const { return _rxErr; }
uint32_t ControlRadio::getTxErrorCount() const { return _txErr; }


/*  La maquina de estados para controlar la radio. Tiene los siguientes estados y transiciones:
    -- STATE_REARM_RX : Volver a activar modo de recepción
        Si ha habido un error en la recepción se incrementa el contador de recepciones erroneas
        En cuanto se haya ejecutado este estado se pasará a STATE_AWAITING_RADIO
    -- STATE_AWAITING_RADIO : Como un IDLE. Esperando a que la radio acabe de hacer algo o arrancar transmisiones pendientes
        Se mira si se ha ejecutado una interrupción del SX1262, esta indicaría que se ha finalizado una tarea, ya sea recibir, transmitir...
          Si lo que se ha finalizado era una transmisión se mira si no ha habido problemas y se vuelve a STATE_REARM_RX
          Si se ha recibido un mensaje se pasa al estado STATE_RX_PROCESS, en el que se guardarán los datos recibidos
          Si hay mensajes pendientes de mandar se pasa al estado STATE_TX_START
    -- STATE_RX_PROCESS : Procesar el mensaje que se ha recibido y meterlo en el buffer de mensajes recibidos
        Se revisa que la longitud sea correcta, si no se vuelve a STATE_REARM_RX
        Se mete el mensaje, con su RSSI y SNR, al buffer de mensajes recibidos. Si ha habido un error en la recepción del mensaje se incrementa el correspondiente contador
        Se finaliza la recepción y se vuelve a STATE_REARM_RX
    -- STATE_TX_START : mandar el mensaje del buffer de mensajes a mandar
        Se revisa que haya algún mensaje que mandar, si lo hay se manda
          Si la radio no ha dado problemas se pasa al estado STATE_AWAITING_RADIO, que esperará a que la radio termine de transmitir
          Si la radio ha dado problemas, se pasa a STATE_REARM_RX
*/
void ControlRadio::taskLoop() {
  for (;;) {
    switch (_state) {
      case STATE_REARM_RX: {
        _lastError = _radio.startReceive();

        if (_lastError == RADIOLIB_ERR_NONE) {
          _state = STATE_AWAITING_RADIO;
        } else {
          _rxErr++;
          ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
        }
        break;
      }

      case STATE_AWAITING_RADIO: {
        bool irqPending = false;
        bool hasTx = false;

        taskENTER_CRITICAL();
        if (_irqFlag) {
          _irqFlag = false;
          irqPending = true;
        }
        hasTx = (_txCount > 0);
        taskEXIT_CRITICAL();

        if (irqPending) {
          if (_txInProgress) { // La operación que la radio estaba realizando era una transmisón
            _txInProgress = false;

            if (_lastTxStartState == RADIOLIB_ERR_NONE) {
              _txOk++;
            } else {
              _txErr++;
            }

            _radio.finishTransmit();
            _state = STATE_REARM_RX;
          } else {
            _state = STATE_RX_PROCESS;
          }
          break;
        }

        if (hasTx) {
          _state = STATE_TX_START;
          break;
        }

        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));
        break;
      }

      case STATE_RX_PROCESS: {
        const size_t len = _radio.getPacketLength();

        if ((len == 0U) || (len > MAX_MSG_LEN)) {
          _lastError = RADIOLIB_ERR_PACKET_TOO_LONG;
          _rxErr++;
          _radio.finishReceive();
          _state = STATE_REARM_RX;
          break;
        }

        _lastError = _radio.readData(_radioRxBuffer, len);

        if (_lastError == RADIOLIB_ERR_NONE) {
          const bool queued = pushRx_(
            _radioRxBuffer,
            len,
            static_cast<int16_t>(_radio.getRSSI()),
            static_cast<int8_t>(_radio.getSNR())
          );

          if (queued) {
            _rxOk++;
          } else {
            _rxErr++;
          }
        } else {
          _rxErr++;
        }

        _radio.finishReceive();
        _state = STATE_REARM_RX;
        break;
      }

      case STATE_TX_START: {
        Message msg;

        if (!popTx_(msg)) {
          _state = STATE_AWAITING_RADIO;
          break;
        }

        _radio.finishReceive(); // Salimos explícitamente de RX antes de transmitir. 

        _lastTxStartState = _radio.startTransmit(msg.data, msg.len);
        _lastError = _lastTxStartState;

        if (_lastTxStartState == RADIOLIB_ERR_NONE) {
          _txInProgress = true;
          _state = STATE_AWAITING_RADIO;
        } else {
          _txInProgress = false;
          _txErr++;
          _state = STATE_REARM_RX;
        }
        break;
      }

      default: {
        _txInProgress = false;
        _state = STATE_REARM_RX;
        break;
      }
    }
  }
}