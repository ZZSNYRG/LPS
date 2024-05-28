# Sistema de Posicionamiento Local

 ## Condiciones de uso
- Para los marcadores circulares de las esquinas se recomienda un color amarillo fosforescente, los cuales deben tener un diámetro de 15 a 20 cm. **Nota:** Puede variar tanto de tamaño y color dependiendo de la distancia de la cámara al objeto, y el color e iluminación del suelo (las condiciones esperadas son de 3m de altura de la cámara en relación con el objeto).
- Para el objeto se necesita un marcador con forma de flecha o similar a la proporcionada en la siguiente imagen. La flecha debe tener un mismo color (se recomienda verde obscuro) en su base y punta, pero diferente tamaño en las anteriores mencionadas (se recomienda una relación de 3:1). A continuación se proporciona el ejemplo recomendado: 
![flecha.png](flecha.png)

## Parámetros de transmisor de radiofrecuencia

  

Al iniciar el transmisor de radiofrecuencia o al resetear el mismo (botón negro de Nucleo64) se hará un dump de la configuración de los registros del NRF24L01+. Los parámetros a resaltar son el canal de frecuencia **RF_CH** (**52**) y la dirección de transmisión **TX_ADDR** (```0x11223344AA```), la última de las cuales tiene que coincidir con la configuración de la dirección de alguna de las pipes de recepción. A continuación se presenta un dump completo de los registros:

  

```

CRC:

Enabled, 2 Bytes

ENAA:

P0: 0

P1: 0

P2: 0

P3: 0

P4: 0

P5: 0

EN_RXADDR:

P0: 1

P1: 1

P2: 0

P3: 0

P4: 0

P5: 0

SETUP_AW:

5 bytes

RF_CH:

52 CH

Data Rate:

2Mbps

RF_PWR:

0dB

RX_Pipe0 Addrs:

11,22,33,44,AA

RX_Pipe1 Addrs:

C2,C2,C2,C2,C2

RX_Pipe2 Addrs:

xx,xx,xx,xx,C3

RX_Pipe3 Addrs:

xx,xx,xx,xx,C4

RX_Pipe4 Addrs:

xx,xx,xx,xx,C5

RX_Pipe5 Addrs:

xx,xx,xx,xx,C6

TX Addrs:

11,22,33,44,AA

RX_PW_P0:

32 bytes

RX_PW_P1:

0 bytes

RX_PW_P2:

0 bytes

RX_PW_P3:

0 bytes

RX_PW_P4:

0 bytes

RX_PW_P5:

0 bytes

DYNPD_pipe:

P0: 1

P1: 1

P2: 1

P3: 1

P4: 1

P5: 1

EN_DPL:

Enabled

EN_ACK_PAY:

Disabled

```

  

## Manual de uso (main.exe)

1. Ejecutar main.exe

2. Calibrar manualmente los parámetros indicados:

  

	2.1 **Teclear largo** del área de interés definida por los marcadores en centímetros. Esta medida funcionará como referencia para obtener las coordenadas en el eje X horizontal. Por default es asignado 3 metros.

  

	2.2 **Teclear ancho** de la misma área de interés en centímetros. Esta medida funcionará como referencia para obtener las coordenadas en el eje Y vertical. Por default es asignado 2 metros.

  

	2.3 **Teclear número de cámara** a usar. Asumiendo que no hay cámaras conectadas mas que la de interés, podemos teclear Enter. Por default se abre la número **0**. Si existen más cámaras conectadas se debe buscar la de interés por prueba y error tecleando de 0 a n-1, siendo n el número de cámaras conectadas.

  

	2.4 **Teclear número de puerto serial** que corresponde al transmisor de radiofrecuencia (tarjeta Nucleo64). Para lo anterior es necesario abrir Administrador de dispositivos>Ports (COM & LPT) y buscar nuestro dispositivo marcado como STMicroelectronics STLink Virtual Com Port (COMX), donde X representa el número a teclear. **Nota**: Si hay mas de una tarjeta de STM32 conectada entonces conectar y desconectar la tarjeta transmisora para detectarla.

  

3. Al abrir la ventana "*Original*", seleccionar esquinas con ctrl + clic izquierdo. Se abre la ventana "*Warped*" y en la ventana "*Original*" se marca el contorno de las esquinas.

  

4. Seleccionar centro de objeto en "*Warped*" con clic izquierdo. El centro corresponde a la base de la flecha (forma de menor tamaño).

  

5. Las coordenadas del objeto se visualizan en la ventana "*Warped*". Si el mensaje se envió por radiofrecuencia exitosamente, la consola imprime el payload en hexadecimal en crudo. La estructura del payload es la siguiente:

  

```b'XX XX YY YY AA AA \r\n'```

  

donde:

- ```XX XX``` es la representación hexadecimal de las coordenadas en X en formato de unsigned integer de 4 bytes.

- ```YY YY``` es la representación hexadecimal de las coordenadas en Y en formato de unsigned integer de 4 bytes.

- ```AA AA``` es la representación hexadecimal del ángulo en formato de unsigned integer de 4 bytes.

  

En dado caso que las coordenadas no se encuentren (mensaje de "*Not found*"), el payload será el siguiente:

  

```b'0F FF 0F FF 0F FF \r\n'```

  

## Errata

- Si los valores de las coordenadas no coinciden con el payload (valores shifteados a la derecha), resetear la tarjeta Nucleo64 transmisora presionando el botón negro dentro de la misma.