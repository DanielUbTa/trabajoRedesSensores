# trabajoRedesSensores
Documentos y código para la asignatura del máster "Redes de sensores electrónicos"

# meshtasticProtobufs
Esta es una librería que ya no está en uso que englobava el control de la radio de LoRa y la generación de mensajes de Meshtastic. Esto se ha subdividido en dos clases, controlRadio para el control del SX1262, y radioMesh para generar los protobufs y pasarselos al buffer de controlRadio

# controlRadio
Se introduce una tarea de FreeRTOS que implementa una máquina de estados para el control de la radio, esta se  ocupa de que cuando la radio no esté transmitiendo datos  esté recibiendo, a grandes rasgos. 

Además de esto, la librería crea una clase con un objeto con dos buffers. Uno para los mensajes de LoRa que se quieren mandar y otro para los mensajes que se han recbido.

# radioMesh
Como un wrapper que se ocupa de transformar los mensajes que se quieren mandar, como podrían ser mensajes de texto, de posición... a protobufs usando la librería de Meshtastic, y transformar esos protobufs a arrays de bytes que se pueden mandar por LoRa. También hay funciones para decodificar mensajes de LoRa según algunos protobufs y obtener los datos (para poder leer mensajes de LoRa).