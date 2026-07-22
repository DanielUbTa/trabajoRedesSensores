# RAK4630 Accel BLE Meshtastic

Proyecto para una placa RAK4630/WisMesh que lee datos de acelerómetro, ejecuta una red neuronal de Edge Impulse para clasificar el estado de movimiento y envía información por una malla compatible con Meshtastic usando LoRa SX1262 y RadioLib.

El código está preparado para funcionar en dos variantes de placa:

- `wisblock`: usa acelerómetro LIS3DH.
- `wistag`: usa acelerómetro LIS2DH12, buzzer y botón físico.

La configuración principal se cambia desde `configNodo.h`.

---

## Estructura de archivos

```text
RAK4630_accel_ble.ino   Programa principal: setup, tareas FreeRTOS, sensores, IA, alarmas y lógica general.
configNodo.h            Configuración del nodo, placa, periféricos, LoRa, tiempos y red neuronal.
controlRadio.h/.cpp     Capa de control asíncrono de la radio SX1262 con colas RX/TX y máquina de estados.
radioMesh.h/.cpp        Capa de empaquetado Meshtastic: cabeceras, protobufs, texto, posición, telemetría y binarios.
```

---

## Arquitectura general

El proyecto está dividido en tres capas principales:

```text
Aplicación principal
RAK4630_accel_ble.ino
  ├─ Lee acelerómetro/GPS/batería
  ├─ Ejecuta la red neuronal
  ├─ Decide cuándo enviar mensajes
  ├─ Gestiona alarmas, LED, buzzer y botón
  ↓
Capa Meshtastic
radioMesh.cpp/.h
  ├─ Construye mensajes Meshtastic
  ├─ Codifica protobufs
  ├─ Añade cabecera LoRa de 16 bytes
  ├─ Decodifica mensajes recibidos
  ↓
Capa RadioLib asíncrona
controlRadio.cpp/.h
  ├─ Controla el SX1262
  ├─ Usa colas TX/RX
  ├─ Gestiona interrupciones DIO1
  ├─ Alterna RX/TX mediante una máquina de estados
  ↓
Radio física LoRa SX1262
```

La idea principal es que el programa principal no accede directamente a `RadioLib` para mandar mensajes. En su lugar llama a `radioMesh`, que empaqueta el mensaje en formato Meshtastic, y `radioMesh` lo pasa a `controlRadio`, que se encarga de transmitirlo sin bloquear el resto del sistema.

---

## Configuración principal

La mayor parte de los cambios se hacen en `configNodo.h`.

### Selección de placa

```cpp
#define wisblock 0
#define wistag 1
#define placaSiendoFlasheada wistag
```

Esto selecciona qué partes del código se compilan. La `wistag` activa elementos específicos como buzzer, botón y el acelerómetro LIS2DH12. La `wisblock` usa otro acelerómetro y gestión de alimentación por slots.

### Activación de módulos

```cpp
#define usarBluetooth false
#define usarGPS false
#define usarAcelerometro true
#define mandarMeshtastic true
#define mandarTelemetria true
#define printearSerial true
```

Estos `#define` permiten compilar o excluir partes del sistema. No son simples flags en ejecución: al ser directivas de preprocesador, el código asociado se incluye o se elimina durante la compilación.

### Identificación Meshtastic

```cpp
constexpr uint32_t nodeNum = 0x01020301;
constexpr char* longName  = "WisMesh_wistag_Arduino_1";
constexpr char* shortName = "0301";
```

Cada nodo debe tener un `nodeNum` diferente. Este valor se usa como identificador del nodo en la cabecera Meshtastic. `longName` y `shortName` se envían mediante mensajes `NodeInfo`.

### Parámetros LoRa

```cpp
constexpr float freqMHz = 866.0;
constexpr uint8_t sf = 5;
constexpr float bw = 250.0;
constexpr uint8_t cr = 5;
constexpr uint8_t syncWord = 0x2B;
constexpr float potencia = 22.0;
```

Estos valores configuran el SX1262 mediante RadioLib. Todos los nodos que tengan que comunicarse entre sí deben usar parámetros compatibles: frecuencia, ancho de banda, spreading factor, coding rate y sync word.

---

## Funcionamiento del programa principal

`RAK4630_accel_ble.ino` inicializa alimentación, radio, sensores, watchdog y tareas FreeRTOS.

En `setup()` se ejecuta, de forma resumida:

1. Inicio de `Serial`.
2. Activación del watchdog.
3. Configuración de alimentación de slots o alimentación 3V3 de la tag.
4. Inicio opcional de BLE y GPS.
5. Inicio de LoRa mediante `setupLoRa()`.
6. Configuración de `radioMesh` con `editarConfig()`.
7. Configuración del acelerómetro.
8. Creación de tareas FreeRTOS.
9. Envío inicial de `NodeID`.

El `loop()` queda casi vacío funcionalmente: solo parpadea el LED verde y alimenta el watchdog. El trabajo real lo hacen las tareas FreeRTOS.

---

## Tareas FreeRTOS

### `taskIMU`

Lee el acelerómetro periódicamente y guarda las muestras en `buffCircularMedidas`, un buffer circular de aceleraciones X, Y y Z.

En `wistag` el retardo se fuerza a 5 ms, mientras que en `wisblock` se usa `periodoAccel`.

### `taskProcesado`

Toma las últimas muestras del buffer circular, prepara el array de entrada de Edge Impulse y ejecuta:

```cpp
run_classifier(&signal, &result, false);
```

Después convierte el resultado de la red neuronal en un estado interno:

- `ESTADO_CAMINANDO`
- `ESTADO_ESTACIONARIO`
- `ESTADO_PICANDO`
- `ESTADO_CAIDA`
- `ESTADO_NO_RECONOCIDO`

Solo acepta una clasificación si la probabilidad supera `UMBRAL_ESTADO_RED_NEURONAL`.

### `taskMeshtasticMandarTexto`

Envía mensajes de estado por Meshtastic, pero no manda cada clasificación directamente. Primero aplica un filtro temporal para evitar falsos cambios puntuales.

Mantiene un histórico de los últimos 7 estados:

```cpp
N_ESTADOS_FILTRO = 7
```

Para cambiar a un estado normal hacen falta 5 apariciones de ese estado en el histórico:

```cpp
MIN_ESTADOS_CAMBIO = 5
```

Para cambiar a caída bastan 2 apariciones:

```cpp
MIN_CAIDAS_CAMBIO = 2
```

Esto evita que un único estado erróneo, por ejemplo `picando` entre varios `estacionario`, genere un mensaje incorrecto.

La lógica de envío es:

- Si el estado no ha cambiado, no se envía nada.
- Si cambia a un estado normal, se envía solo si ha pasado `periodoMensajeMeshtastic` desde el último mensaje.
- Si cambia a caída, se envía inmediatamente, sin esperar al periodo mínimo.

Formato de mensajes normales:

```text
Estado de <longName>: <estado>
```

Formato de caída:

```text
!ALARMA caída detectada de <longName>
```

### `taskMeshtasticMandarGPS`

Si el GPS está activado y la posición es válida, envía un mensaje Meshtastic de posición con latitud, longitud, altitud y tiempo UTC.

### `taskMeshtasticMandarTelemetriaYnodeID`

Envía periódicamente:

- `NodeID`, para que otros nodos conozcan el nombre e identificación del dispositivo.
- Telemetría de batería, tensión y uptime.

Actualmente `channel_utilization` y `air_util_tx` se mandan como `-1.0` si no están disponibles.

### `taskMeshtasticRecibir`

Extrae mensajes recibidos desde la cola RX de `controlRadio`. Si el mensaje tiene más de 16 bytes, descarta la cabecera LoRa Meshtastic y pasa el resto a:

```cpp
radioMesh.decodificarSiEsTexto(...)
```

Si el mensaje es texto, lo copia a `textoRecibido`. Después notifica a `taskAlarmaTagBuzzerYled` para activar LED, buzzer o alarma.

### `taskAlarmaTagBuzzerYled`

Gestiona la respuesta local ante mensajes recibidos.

- Si el texto recibido contiene `!ALARMA`, se activa la alarma fuerte.
- Si no contiene `!ALARMA`, se activa una notificación normal.
- En `wistag`, el botón puede silenciar la alarma.

---

## Cómo funciona `controlRadio`

`controlRadio` es una capa para usar la radio SX1262 de forma asíncrona.

El objetivo es evitar que varias tareas llamen directamente a `RadioLib` y provoquen conflictos entre recepción y transmisión.

### Colas internas

Tiene dos colas circulares:

- Cola TX: mensajes pendientes de transmitir.
- Cola RX: mensajes recibidos pendientes de leer.

Cuando una tarea quiere transmitir, llama a:

```cpp
controlRadio.pushTx(data, len);
```

El mensaje se copia a la cola TX y la tarea interna de radio lo enviará cuando pueda.

Cuando se recibe un paquete, `controlRadio` lo guarda en la cola RX. La aplicación lo lee con:

```cpp
controlRadio.popRx(msg);
```

### Máquina de estados

La task interna de `controlRadio` usa cuatro estados:

```cpp
STATE_REARM_RX
STATE_AWAITING_RADIO
STATE_RX_PROCESS
STATE_TX_START
```

Su funcionamiento es:

1. `STATE_REARM_RX`: activa recepción con `startReceive()`.
2. `STATE_AWAITING_RADIO`: espera una interrupción de la radio o un mensaje pendiente de TX.
3. `STATE_RX_PROCESS`: lee el paquete recibido, RSSI y SNR, y lo mete en la cola RX.
4. `STATE_TX_START`: saca un mensaje de la cola TX y llama a `startTransmit()`.

La interrupción DIO1 no procesa el paquete directamente. Solo marca un flag y despierta la task de radio. Esto es importante porque la ISR debe ser lo más corta posible.

---

## Cómo funciona `radioMesh`

`radioMesh` construye mensajes compatibles con Meshtastic sobre LoRa.

No controla la radio directamente: recibe una referencia a `ControlRadio` y usa su cola TX.

### Configuración

La función principal es:

```cpp
radioMesh.editarConfig(nodeNum, longName, shortName, hopLimit, wantAck, viaMqtt);
```

Guarda la configuración del nodo y calcula el hash de canal a partir del nombre del canal y la PSK.

### Cabecera LoRa Meshtastic

Antes de cada protobuf Meshtastic se añaden 16 bytes de cabecera:

```text
0..3    To
4..7    From
8..11   Packet ID aleatorio
12      Flags
13      Channel hash
14      Next hop
15      Relay node
```

Los flags incluyen:

- `hopLimit`
- `wantAck`
- `viaMqtt`
- `hopStart`

Después de construir la cabecera, el mensaje se manda a `controlRadio.pushTx()`.

### Protobufs

Para enviar datos, `radioMesh` usa los protobufs de Meshtastic mediante nanopb.

El proceso general es:

1. Crear el struct del tipo de mensaje: `meshtastic_Position`, `meshtastic_Telemetry`, `meshtastic_User`, etc.
2. Rellenar sus campos.
3. Codificarlo con `pb_encode()`.
4. Encapsularlo dentro de un `meshtastic_Data` con su `PortNum`.
5. Añadir la cabecera LoRa de 16 bytes.
6. Encolar el paquete final para transmisión.

Funciones principales:

```cpp
mandarNodeID();
mandarPosicion(...);
mandarTelemetry(...);
mandarTexto(...);
mandarBinario(...);
pedirRouting(...);
```

También permite decodificar mensajes recibidos:

```cpp
decodificarSiEsTexto(...);
decodificarSiEsBinario(...);
```

---

## Watchdog

El watchdog se configura para reiniciar la placa si el programa deja de ejecutarse correctamente durante más de `periodoWatchdog` segundos.

Actualmente se alimenta desde `loop()` cuando parpadea el LED verde:

```cpp
feedWatchdog();
```

Si el sistema se bloquea completamente, el LED dejará de parpadear y el watchdog reiniciará el microcontrolador.

---

## Flujo completo de envío de estado

```text
Acelerómetro
  ↓
Buffer circular de muestras
  ↓
Preparación de entrada para Edge Impulse
  ↓
Clasificación de la red neuronal
  ↓
Filtro de últimos 7 estados
  ↓
Decisión de si hay cambio real de estado
  ↓
Construcción del texto
  ↓
radioMesh.mandarTexto()
  ↓
Protobuf Meshtastic Data
  ↓
Cabecera LoRa Meshtastic de 16 bytes
  ↓
controlRadio.pushTx()
  ↓
Cola TX
  ↓
SX1262 transmite por LoRa
```

---

## Flujo completo de recepción

```text
SX1262 recibe paquete
  ↓
Interrupción DIO1
  ↓
Task interna de controlRadio
  ↓
Lectura del paquete, RSSI y SNR
  ↓
Cola RX
  ↓
taskMeshtasticRecibir
  ↓
Se descartan los 16 bytes de cabecera
  ↓
radioMesh.decodificarSiEsTexto()
  ↓
textoRecibido
  ↓
taskAlarmaTagBuzzerYled
  ↓
LED / buzzer / alarma
```

---

## Dependencias

El proyecto usa, como mínimo:

- Arduino framework para nRF52840.
- FreeRTOS.
- RadioLib.
- TinyGPSPlus.
- TimeLib.
- Librerías SparkFun para LIS3DH o LIS2DH12.
- Meshtastic protobufs.
- nanopb.
- Librería generada por Edge Impulse, incluida como `Trabajo_inferencing.h`.

---

## Notas importantes

- Todos los nodos deben usar parámetros LoRa compatibles.
- Cada nodo debe tener un `nodeNum` diferente.
- `longName` se usa tanto para `NodeInfo` como para los mensajes de estado y alarma.
- La radio no debe controlarse directamente desde varias tareas. Para transmitir, se debe usar `radioMesh` o `controlRadio.pushTx()`.
- Los mensajes recibidos tienen una cabecera Meshtastic de 16 bytes antes del protobuf `Data`.
- El filtro de estados evita falsos positivos puntuales antes de enviar cambios de estado.
- Las caídas tienen prioridad: requieren menos confirmaciones y no respetan el periodo mínimo entre mensajes.

---

## Ejemplo de configuración rápida

Para compilar una tag con acelerómetro y Meshtastic, sin GPS ni BLE:

```cpp
#define placaSiendoFlasheada wistag
#define usarBluetooth false
#define usarGPS false
#define usarAcelerometro true
#define mandarMeshtastic true
#define mandarTelemetria true
```

Después se deben ajustar:

```cpp
constexpr uint32_t nodeNum = 0x01020301;
constexpr char* longName  = "WisMesh_wistag_Arduino_1";
constexpr char* shortName = "0301";
```

