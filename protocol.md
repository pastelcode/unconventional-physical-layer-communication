## Especificación Técnica del Protocolo Híbrido Matricial de 4 Canales (PHM-4)

**Versión:** 1.0

**Plataforma de Desarrollo:** 2 × ESP32 (arquitectura de capas integradas en un único firmware por nodo)

**Topología Física:** Enlace punto a punto bidireccional (4 canales TX + 4 canales RX por nodo sobre al menos 3 medios físicos no convencionales)

**Frecuencia Base de Operación:** Configurable ($f_{\text{base}} = 1\text{ Hz}$, $T = 1000\text{ ms}$ para auditoría humana de `LOG`, escalable hasta el límite de asentamiento de los sensores)

---

## 1. Arquitectura General y Asignación de Hardware

El sistema implementa un protocolo de comunicación híbrido **paralelo-serial** en Capa Física (PHY), evitando la serialización pura o la modulación por conteo de pulsos. Las cuatro capas lógicas requeridas (**Aplicación, Presentación, Enlace de Datos y Física**) residen en un mismo ejecutable por estación, estrictamente desacopladas mediante interfaces internas.

### 1.1 Restricción de Medios Físicos y Asignación Recomendada

La especificación prohíbe medios cableados convencionales y radiofrecuencia (*wireless*), exigiendo un mínimo de **3 medios físicos distintos** cuya salida acondicionada entregue niveles binarios discretos (`LOW = 0 V`, `HIGH = 3.3 V`) compatibles con los pines GPIO del ESP32. Para cumplir la norma (reemplazando cualquier conducción eléctrica directa por cable), se asignan los 4 canales por sentido de la siguiente forma:

| Canal | Función Lógica | Medio Físico Asignado | Actuador (TX) / Sensor (RX) | Justificación Técnica |
| --- | --- | --- | --- | --- |
| **`CH0`** | **`CLK`** (Reloj maestro) | **Infrarrojo (IR)**

 | LED IR + Fototransistor / Receptor IR

 | Medio de menor latencia y mayor limpieza de flanco para el *strobe* de muestreo.

 |
| **`CH1`** | **`D1`** (Dato paralelo 1) | **Ultrasonido**

 | Transductor 40 kHz + etapa comparadora

 | Independiente de interferencias ópticas o magnéticas.

 |
| **`CH2`** | **`D2`** (Dato paralelo 2) | **Magnético**

 | Electroimán / bobina + Sensor Hall digital (ej. A3144)

 | Inmune a bloqueos visuales o acústicos.

 |
| **`CH3`** | **`Pr`** (Paridad horizontal) | **Luz visible / Láser** (ópticamente aislado de `CH0`)

 | Diodo láser / LED colimado + Fotodiodo con comparador

 | Permite aislamiento físico respecto a `CH1` y `CH2` para evitar fallos correlacionados en datos y paridad.

 |

> **Requisito bidireccional:** Cada nodo integra **8 circuitos físicos** (4 emisores TX y 4 receptores RX), sumando **16 circuitos en total** entre ambas estaciones, físicamente apantallados para prevenir diafonía (*cross-talk*) entre canales.
> 
> 

### 1.2 Mapa de Pines Sugerido (ESP32)

Todos los pines seleccionados evitan pines de *strapping* conflictivos o exclusivos de flash:

| Dirección | Señal | GPIO Sugerido | Modo Eléctrico |
| --- | --- | --- | --- |
| **TX (Salidas)** | `TX_CLK`, `TX_D1`, `TX_D2`, `TX_PR` | `GPIO 16`, `17`, `18`, `19` | `OUTPUT` (estado inicial `LOW`) |
| **RX (Entradas)** | `RX_CLK`, `RX_D1`, `RX_D2`, `RX_PR` | `GPIO 32`, `33`, `25`, `26` | `INPUT` (con pull-down externo/interno según comparador) |

---

## 2. Especificación de la Capa Física (PHY)

### 2.1 Temporización Dinámica y Mitigación de *Skew* entre Medios

Debido a la heterogeneidad física de los canales (la luz se propaga a $\approx 3 \times 10^8\text{ m/s}$, mientras el ultrasonido viaja a $\approx 343\text{ m/s}$ y presenta amortiguamiento mecánico, y las bobinas tienen constante de tiempo inductiva $L/R$), las líneas nunca conmutan de forma simultánea. El canal `CLK` gobierna la sincronización mediante un ciclo de período configurable $T = 1/f$:

1. **Fase de actualización (Flanco de bajada de `CLK`, $t = 0$):** El transmisor coloca `TX_CLK = LOW` y actualiza `TX_D1`, `TX_D2` y `TX_PR` con los nuevos niveles lógicos.


2. **Ventana de asentamiento ($0 < t < 0.5T$):** Durante medio período (500 ms a 1 Hz), los actuadores y sensores receptores convergen a estados estables `0` o `1`, absorbiendo retardos acústicos, *ringing* y transitorios inductivos.


3. **Instante de muestreo (Flanco de subida de `CLK`, $t = 0.5T$):** El transmisor coloca `TX_CLK = HIGH`. El receptor detecta el flanco ascendente exclusivamente por transición (`LOW -> HIGH`).


4. **Filtro anti-rebote y voto por mayoría en RX:** Tras detectar el flanco ascendente de `RX_CLK`, el receptor aguarda un margen de guarda $t_{\text{guard}} = \min(20\text{ ms},\; 0.05T)$ y ejecuta **5 lecturas consecutivas** espaciadas por $\Delta t = \min(4\text{ ms},\; 0.01T)$ sobre los 4 pines de entrada. Si `RX_CLK` no se mantiene en `HIGH` en al menos 4 de 5 muestras, el flanco se descarta como ruido espurio (*glitch*). Los valores finales de `(D1, D2, Pr)` se fijan por mayoría simple ($\ge 3/5$).

### 2.2 Alfabeto Físico y Señalización Fuera de Banda (OOB)

Con tres líneas de datos/paridad `(D1, D2, Pr)` existen $2^3 = 8$ vectores binarios posibles por ciclo de reloj. Al imponer la invariante de paridad par $P_r = D_1 \oplus D_2$ para todo dato válido, el espacio se particiona exactamente en dos subconjuntos disjuntos:

* **4 Vectores de Datos Válidos ($P_r = D_1 \oplus D_2$):** `(0,0,0)`, `(0,1,1)`, `(1,0,1)`, `(1,1,0)`.


* **4 Vectores de Control Físico OOB ($P_r \neq D_1 \oplus D_2$):** `(1,1,1)`, `(0,1,0)`, `(1,0,0)`, `(0,0,1)`.



> **Propiedad matemática clave:** Todos los comandos físicos OOB poseen una **distancia de Hamming $d_H = 2$ entre sí**. Ningún error simple de 1 bit puede transformar un comando físico en otro comando físico válido (por ejemplo, `SYN_ACK` jamás puede mutar a `PHY_ABORT` por una interferencia de 1 bit).

| Vector `(D1, D2, Pr)` | Condición de `CLK` | Símbolo Físico | Duración | Función en el Protocolo |
| --- | --- | --- | --- | --- |
| `(0, 0, 0)` | `LOW` continuo (sin flancos) | **`IDLE`** | Indefinida | Bus en reposo.

 |
| `(D1, D2, D1 ^ D2)` | Flanco de subida | **`DATA_SLICE`** | $1T$ | Fila válida de una matriz $3 \times 3$.

 |
| **`(1, 1, 1)`** | 1 flanco (bus de ida, desde `IDLE`) | **`SYN`** | $1T$ | Despierta al receptor y solicita inicio de trama.

 |
| **`(0, 1, 0)`** | 1 flanco (bus de retorno) | **`SYN_ACK`** | $1T$ | Confirma enlace físico activo y reloj íntegro antes de emitir matrices. |
| **`(1, 0, 0)`** | 1 flanco (bus de retorno) | **`PHY_ABORT`** | $1T$ | Interrupción inmediata por fallo matricial irrecuperable, `SFD` inválido o `CLOCK_LOSS`.

 |
| **`(0, 0, 1)`** | 1 flanco (bus de retorno) | **`PHY_BUSY`** | $1T$ | Reservado: indica saturación momentánea de búfer en RX (solicita espera de $2T$). |

### 2.3 Unidad de Transporte: Matriz Espacio-Temporal $3 \times 3$

Cada bloque lógico de **4 bits** ($b_0, b_1, b_2, b_3$) se codifica en paralelo-serial mediante una matriz espacio-temporal de 3 columnas físicas (`D1`, `D2`, `Pr`) por 3 ciclos de reloj ($t_1, t_2, t_3$):

$$M = \begin{bmatrix}
b_0 & b_1 & P_{r1} \\
b_2 & b_3 & P_{r2} \\
P_{c1} & P_{c2} & P_{\text{total}}
\end{bmatrix} = \begin{bmatrix}
b_0 & b_1 & (b_0 \oplus b_1) \\
b_2 & b_3 & (b_2 \oplus b_3) \\
(b_0 \oplus b_2) & (b_1 \oplus b_3) & (b_0 \oplus b_1 \oplus b_2 \oplus b_3)
\end{bmatrix}$$

#### Evaluación de Síndromes y Decodificación en Tiempo Real

En cada flanco de reloj, el receptor evalúa los síndromes de paridad:

* **En $t_1$:** $S_{r1} = D_1(t_1) \oplus D_2(t_1) \oplus P_r(t_1)$. Si $S_{r1} = 1$, incrementa `row_errors`.
* **En $t_2$:** $S_{r2} = D_1(t_2) \oplus D_2(t_2) \oplus P_r(t_2)$. Si $S_{r2} = 1$, incrementa `row_errors`.
* **Corte temprano en $t_2$:** Si `row_errors == 2`, el receptor **no espera a $t_3$**; dispara **`PHY_ABORT`** inmediatamente al final del segundo 2.




* **En $t_3$:** Evalúa síndromes de columna y de la propia fila 3:

$$S_{c1} = D_1(t_1) \oplus D_1(t_2) \oplus P_{c1}, \quad S_{c2} = D_2(t_1) \oplus D_2(t_2) \oplus P_{c2}$$


$$S_{c3} = P_r(t_1) \oplus P_r(t_2) \oplus P_{\text{total}}, \quad S_{r3} = P_{c1} \oplus P_{c2} \oplus P_{\text{total}}$$



Sea $R_{\text{err}} = S_{r1} + S_{r2} + S_{r3}$ el total de filas con síndrome $1$, y $C_{\text{err}} = S_{c1} + S_{c2} + S_{c3}$ el total de columnas con síndrome $1$:

| $R_{\text{err}}$ | $C_{\text{err}}$ | Diagnóstico Matemático | Acción Inmediata del Receptor |
| --- | --- | --- | --- |
| `0` | `0` | Matriz libre de errores | Extrae $(b_0, b_1, b_2, b_3)$ y avanza a la siguiente matriz.

 |
| `1` | `1` | **Error único en celda $(r, c)$** | Si $r \in \{1,2\}$ y $c \in \{1,2\}$: **invierte y corrige** el bit en $(r,c)$ y registra en LOG. Si $r=3$ o $c=3$: el error cayó solo en redundancia; datos íntegros, deja pasar.

 |
| `2` o `3` | Cualquiera | Error múltiple ($\ge 2$ bits) | Irrecuperable ($t=1$). Dispara **`PHY_ABORT` `(1,0,0)**` y reinicia RX a `IDLE`.

 |
| Cualquiera | `2` o `3` | Error múltiple ($\ge 2$ bits) | Irrecuperable (incluye fallo doble simultáneo en una fila o en fila 3). Dispara **`PHY_ABORT` `(1,0,0)**`.

 |
| `1` / `0` | `0` / `1` | Inconsistencia imposible para 1 bit (error triple con cancelación parcial) | Dispara **`PHY_ABORT` `(1,0,0)**` por seguridad. |

*(Nota de implementación: al evaluar los 3 síndromes de fila y los 3 de columna completos, cualquier error de 1 bit en cualquiera de las 9 celdas siempre produce exactamente $R_{\text{err}} = 1$ y $C_{\text{err}} = 1$, haciendo la detección 100% uniforme y libre de ambigüedades).*

---

## 3. Estructura de Trama y Optimización de Bits

El protocolo respeta la secuencia obligatoria **`SYN` $\to$ `SFD` $\to$ `Control` $\to$ `DATA` $\to$ `CRC**`, alineando cada campo a múltiplos exactos de 4 bits (0 bits de relleno desperdiciados) para que la capa PHY opere como un *pipeline* limpio de matrices $3 \times 3$ sin mezcla de campos entre bloques:

### 3.1 Trama de Datos (`24 bits lógicos = 6 matrices`)

| Orden | Campo | Tamaño Lógico | Unidades PHY | Contenido / Propósito |
| --- | --- | --- | --- | --- |
| 0 | **`SYN`** | OOB | 1 tick ($1T$) | Pulso `(1,1,1)` del emisor para despertar al receptor.

 |
| 0b | **`SYN_ACK`** | OOB (retorno) | 1 tick ($1T$) | Pulso `(0,1,0)` del receptor validando `CLK` y disponibilidad. |
| 1 | **`SFD`** | 4 bits | Matriz 1 ($3T$) | Delimitador fijo `1010` (`0xA`). Confirma inicio exacto de trama.

 |
| 2 | **`Control`** | 4 bits | Matriz 2 ($3T$) | Empaquetado como `[Tipo: 3 bits |
| 3 | **`DATA` (Alto)** | 4 bits | Matriz 3 ($3T$) | Nibble alto ($d_7..d_4$) del byte `CHAR`.

 |
| 4 | **`DATA` (Bajo)** | 4 bits | Matriz 4 ($3T$) | Nibble bajo ($d_3..d_0$) del byte `CHAR`.

 |
| 5 | **`CRC-8` (Alto)** | 4 bits | Matriz 5 ($3T$) | Nibble alto ($r_7..r_4$) del checksum calculado sobre `Control` + `DATA`.

 |
| 6 | **`CRC-8` (Bajo)** | 4 bits | Matriz 6 ($3T$) | Nibble bajo ($r_3..r_0$) del checksum.

 |

* **Tiempo total trama de datos (a 1 Hz):** $1\text{ s (SYN)} + 1\text{ s (SYN\_ACK)} + 18\text{ s (6 matrices)} = \mathbf{20\text{ segundos}}$.

### 3.2 Trama Corta de Control (`ACK` / `NACK` / `PING`: `16 bits lógicos = 4 matrices`)

Para optimizar latencia sin sacrificar integridad, las tramas de control de Capa 2 omiten el campo vacío `DATA` (ahorrando 6 segundos) y prescinden del pulso `SYN_ACK` porque el nodo que las espera ya se encuentra despierto en estado `WAIT_L2_RESP`:

```
[SYN (1T)] -> [M1: SFD (3T)] -> [M2: Control (3T)] -> [M3: CRC_H (3T)] -> [M4: CRC_L (3T)]

```

* **Tiempo total trama `ACK`/`NACK` (a 1 Hz):** $1\text{ s} + 12\text{ s} = \mathbf{13\text{ segundos}}$ (reducción del 35% respecto a una trama completa).
* **Compatibilidad estricta opcional:** El firmware incluye la directiva `#define FIXED_24BIT_CONTROL_FRAMES 0`. Si se requiere tamaño estrictamente fijo de 24 bits incluso en `ACK`/`NACK`, basta cambiarlo a `1` para insertar `DATA = 0x00`.

### 3.3 Codificación del Campo `Control` (4 bits)

El bit menos significativo (`bit 0`) transporta el **bit alternante de secuencia (`SEQ`)** sin consumir bits extra, mientras los 3 bits superiores (`bits 3..1`) definen **6 tipos de trama** (dentro del rango exigido de 3 a 15 tipos):

| Bits `[3:1]` (Tipo) | Bit `[0]` (`SEQ`) | Nombre del Tipo | Descripción y Uso |
| --- | --- | --- | --- |
| `000` | `0` / `1` | **`DATA_ASCII`** | Carácter individual de 1 byte en modo Extended ASCII.

 |
| `001` | `0` / `1` | **`DATA_UTF8_CONT`** | Byte inicial o intermedio de una secuencia UTF-8 multibyte (2 a 4 bytes).

 |
| `010` | `0` / `1` | **`DATA_UTF8_END`** | Byte único o final de un carácter UTF-8 (dispara entrega a Capa de Aplicación).

 |
| `011` | `0` / `1` | **`ACK`** | Confirmación positiva de la trama de datos con ese `SEQ`.

 |
| `100` | `0` / `1` | **`NACK`** | Rechazo por error de `CRC-8`; solicita retransmisión inmediata del `SEQ`.

 |
| `101` | `0` | **`PING`** | Sonda de diagnóstico de enlace físico para el `LOG`.

 |
| `110`–`111` | — | Reservado | Expansión futura (p. ej. negociación de frecuencia). |

---

## 4. Capa de Enlace de Datos (Control de Flujo, Errores y Temporizadores)

### 4.1 Stop-and-Wait con Bit Alternante (`SEQ`)

El enlace opera en modo **Stop-and-Wait** con bit de secuencia alternante (`tx_seq` y `expected_rx_seq` alternan `0 -> 1 -> 0` por cada trama de datos confirmada). Este mecanismo de 1 bit (costo cero en tiempo) garantiza entrega exactamente una vez (*exactly-once*) incluso si un `ACK` se pierde físicamente.

### 4.2 Recuperación Jerárquica en Tres Niveles

1. **Nivel 1 — Autocorrección Local (FEC Matricial):** Corrige 1 bit dañado por cada matriz de 9 bits en vuelo sin intervención del transmisor (0 s de penalización).


2. **Nivel 2 — Corte Físico Temprano (`PHY_ABORT` + `SYN_ACK`):** Aborta errores de 2 o 3 bits, desalineaciones de `SFD` y pérdidas de reloj en segundos, y evita transmitir matrices al vacío si `CLK` permanece bloqueado.


3. **Nivel 3 — Validación Integral (`CRC-8` + `ACK`/`NACK`):** Protege contra patrones de error de orden superior (4 bits en rectángulo perfecto) que burlan las paridades ortogonales, activando retransmisión controlada por Capa 2.



### 4.3 Algoritmos `CRC-8` Seleccionables en Vivo

La Capa de Enlace implementa los 4 estándares exigidos, seleccionables desde la interfaz web (ambas estaciones deben coincidir en el algoritmo seleccionado):

| Estándar | Polinomio | Valor Inicial (`Init`) | `RefIn` | `RefOut` | `XorOut` | Check (`"123456789"`) |
| --- | --- | --- | --- | --- | --- | --- |
| **`SMBUS`**

 | `0x07` | `0x00` | `false` | `false` | `0x00` | `0xF4` |
| **`AUTOSAR`**

 | `0x2F` | `0xFF` | `false` | `false` | `0xFF` | `0xDF` |
| **`SAE-J1850`**

 | `0x1D` | `0xFF` | `false` | `false` | `0xFF` | `0x4B` |
| **`MAXIM-DOW`**

 | `0x31` | `0x00` | `true` | `true` | `0x00` | `0xA1` |

*En tramas de datos, el `CRC-8` se computa sobre los 2 bytes `[Control_byte, DATA_byte]` (donde `Control_byte = Control & 0x0F`). En tramas cortas de control, se computa sobre `[Control_byte]`.*

### 4.4 Temporizadores y Watchdogs Parametrizados ($T = 1/f$)

Ningún temporizador está fijo en milisegundos; todos escalan automáticamente con la frecuencia configurada `clock_hz`:

| Temporizador | Fórmula | Valor a 1 Hz | Función |
| --- | --- | --- | --- |
| **`T_SETTLE`** | $0.5 \times T$ | 500 ms | Estabilización física de medios antes del flanco de subida.

 |
| **`T_CLK_WATCHDOG`** | $1.5 \times T$ | 1500 ms | Máximo tiempo entre flancos de `CLK` durante una recepción activa; su vencimiento dispara `CLOCK_LOSS`.

 |
| **`T_SYN_ACK`** | $1.5 \times T$ | 1500 ms | Espera del emisor tras pulsar `SYN` para recibir `SYN_ACK` `(0,1,0)`. |
| **`T_ACK_START`** | $2.5 \times T$ | 2500 ms | Ventana máxima tras terminar una trama de datos para recibir el pulso `SYN` del `ACK`/`NACK` de retorno. |
| **`T_BACKOFF(k)`** | $\min(k + 1,\; 4) \times T$ | 2 s, 3 s, 4 s... | Espera progresiva tras un `PHY_ABORT` o `SYN_TIMEOUT` en el reintento $k$. |
| **`T_UTF8_FLUSH`** | $30 \times T$ | 30 s | Purga de búfer multibyte UTF-8 si una secuencia queda truncada por caída del enlace. |
| **`MAX_RETRIES`** | Configurable (def. `5`) | 5 intentos | Límite de reintentos antes de declarar `LINK_DOWN` sin colgar el sistema.

 |

---

## 5. Capas de Presentación y Aplicación

### 5.1 Capa de Presentación (`Extended ASCII` y `UTF-8` Completo)

* **Modo `Extended ASCII` (ISO-8859-1 / CP1252):** Cada carácter ocupa exactamente 1 byte (`0x00`–`0xFF`) y genera 1 trama `DATA_ASCII`. Caracteres fuera de rango (p. ej., emojis) se reemplazan por `'?'` (`0x3F`) con advertencia en el `LOG`.


* **Modo `UTF-8` (1 a 4 bytes, RFC 3629):** Descompone cada carácter Unicode en $m \in \{1, 2, 3, 4\}$ bytes.


* Los primeros $m-1$ bytes se envían con tipo `DATA_UTF8_CONT`.
* El último byte (o el único byte si $m=1$, compatible al 100% con ASCII `0x00`–`0x7F`) se envía con tipo `DATA_UTF8_END`.


* El receptor valida la estructura binaria de cabecera (`0xxxxxxx`, `110xxxxx`, `1110xxxx`, `11110xxx`) y bytes de continuación (`10xxxxxx`) antes de entregar el carácter ensamblado a la Capa de Aplicación.



### 5.2 Capa de Aplicación (Web Server, Sala de Chat y Consola de `LOG`)

Cada ESP32 aloja un servidor web local (HTTP + WebSockets/polling sobre Wi-Fi únicamente para interfaz local con el usuario, **nunca** como medio de transmisión entre equipos) que provee:

* **Sala de Chat:** Envío y visualización de mensajes recibidos en tiempo real.


* **Panel de Control en Vivo:** Selector de algoritmo `CRC-8` (4 opciones), selector de codificación (`UTF-8` / `Extended ASCII`), control de frecuencia `clock_hz` (por defecto `1.0 Hz`), selector de `MAX_RETRIES` y botón de **`Reset Link / Reintentar`**.


* **Consola de `LOG` Estructurado:** Registra con marca de tiempo cada pulso `SYN`/`SYN_ACK`, cada matriz $3 \times 3$ recibida, bit corregido por coordenada $(r,c)$, eventos `PHY_ABORT` y `CLOCK_LOSS`, resultado de `CRC-8`, `ACK`/`NACK`, descartes por duplicado y retransmisiones.



---

## 6. Máquinas de Estados Finitos (FSM)

### 6.1 FSM del Transmisor (TX)

```text
[TX_IDLE]
   |-- (Byte en cola de envío) --> envía SYN (1,1,1) --> [TX_WAIT_SYN_ACK]
[TX_WAIT_SYN_ACK]
   |-- (RX = SYN_ACK (0,1,0)) --------------> [TX_SEND_MATRICES]
   |-- (RX = SYN (1,1,1)) ------------------> [TX_COLLISION_RESOLVE]
   |-- (RX = PHY_BUSY (0,0,1)) -------------> espera 2T --> reenvía SYN
   |-- (Vence T_SYN_ACK = 1.5T) ------------> retry++ --> [TX_BACKOFF]
[TX_SEND_MATRICES]
   |-- (Antes de cada tick: RX = PHY_ABORT) -> corta transmisión, retry++ --> [TX_BACKOFF]
   |-- (Completó las 6 matrices) -----------> inicia T_ACK_START --> [TX_WAIT_L2_RESP]
[TX_WAIT_L2_RESP]
   |-- (Recibe trama ACK con mismo SEQ) ----> invierte tx_seq, retry=0 --> [TX_IDLE]
   |-- (Recibe trama NACK / CRC malo) ------> retry++ --> [TX_BACKOFF]
   |-- (Vence T_ACK_START = 2.5T) ----------> retry++ --> [TX_BACKOFF]
[TX_BACKOFF]
   |-- (retry < MAX_RETRIES) ---------------> espera T_BACKOFF(retry) --> envía SYN --> [TX_WAIT_SYN_ACK]
   |-- (retry >= MAX_RETRIES) --------------> LOG [FATAL: LINK_DOWN], libera bus --> [TX_IDLE]

```

### 6.2 FSM del Receptor (RX)

```text
[RX_IDLE]
   |-- (Flanco CLK + (1,1,1) [SYN] con nodo en TX_WAIT_L2_RESP)
                                              omite SYN_ACK (el transmisor ya está despierto)
                                              inicia T_CLK_WATCHDOG --> [RX_RECV_SFD]
   |-- (Flanco CLK + (1,1,1) [SYN]) --------> envía SYN_ACK (0,1,0) en bus retorno
                                              inicia T_CLK_WATCHDOG --> [RX_RECV_SFD]
   |-- (Cualquier otro vector en IDLE) -----> ignora y registra ruido espurio
[RX_RECV_SFD]
   |-- (Matriz 1 OK y valor == 1010) -------> [RX_RECV_CONTROL]
   |-- (Matriz 1 OK y valor != 1010) -------> envía PHY_ABORT (1,0,0) --> [RX_IDLE]
[RX_RECV_CONTROL]
   |-- (Matriz 2 OK: Tipo == DATA_*) -------> [RX_RECV_DATA] (2 matrices)
   |-- (Matriz 2 OK: Tipo == ACK/NACK/PING)-> [RX_RECV_CRC]  (2 matrices)
   |-- (Tipo inválido) ---------------------> envía PHY_ABORT (1,0,0) --> [RX_IDLE]
[RX_RECV_DATA] -> [RX_RECV_CRC]
   |-- (En cualquier estado activo: 2 filas rotas en t2, error múltiple en t3, o vence T_CLK_WATCHDOG)
       -------------------------------------> LOG error, envía PHY_ABORT (1,0,0) --> [RX_IDLE]
   |-- (Completa CRC-8):
       |-- CRC válido y Tipo == DATA_*:
       |      si SEQ == expected_rx_seq ----> entrega byte a Presentación, invierte expected_rx_seq
       |      si SEQ != expected_rx_seq ----> descarta duplicado (LOG [DUP])
       |      envía trama corta ACK(SEQ) ---> [RX_IDLE]
       |-- CRC válido y Tipo == PING --------> registra [PING_OK] en LOG y envía ACK(0) --> [RX_IDLE]
       |-- CRC inválido y Tipo == DATA_* ---> envía trama corta NACK(SEQ) --> [RX_IDLE]

```

---

## 7. Catálogo Exhaustivo de Casos Borde (*Edge Cases*) y Manejo de Fallos

### 7.1 Bloqueo Prolongado del Canal `CLK` Después de un `PHY_ABORT` (Caso Crítico)

**Escenario:** Durante la transmisión de la Matriz 3 ocurre un fallo doble. El receptor dispara `PHY_ABORT` `(1,0,0)` y vuelve a `RX_IDLE`. Acto seguido, el evaluador bloquea de forma continua el sensor de `CLK` de ida durante 10 a 15 segundos.

**Por qué fallaría un diseño ingenuo:** Sin confirmación física de `SYN`, el transmisor volvería a enviar las 6 matrices completas (18 segundos) hacia un receptor dormido, y además esperaría el timeout de `ACK` en cada intento, desperdiciando más de 21 segundos por reintento.

**Cómo lo resuelve PHM-4:**

1. Al recibir `PHY_ABORT`, el emisor detiene la trama al instante, pone sus salidas en `IDLE` `(0,0,0)` y espera `T_BACKOFF(1) = 2T` (2 s).
2. En el reintento, el emisor transmite **exclusivamente el pulso `SYN` `(1,1,1)` ($1T$)** y aguarda `SYN_ACK` `(0,1,0)` durante `T_SYN_ACK = 1.5T` (1.5 s).
3. Como `CLK` sigue tapado, el receptor nunca detecta `SYN` y no emite `SYN_ACK`.
4. Al cumplirse **1.5 s**, el emisor aborta el intento **sin enviar ninguna matriz de datos**, registra `[WARN] SYN_TIMEOUT: Canal CLK bloqueado o receptor inactivo (Intento k/5)` e ingresa a *backoff* progresivo ($2T, 3T, 4T$).
5. **Recuperación automática o cierre limpio:** Si el bloqueo se retira antes de agotar `MAX_RETRIES`, el siguiente pulso `SYN` obtiene `SYN_ACK` en 1 s y la transmisión se completa sola sin reiniciar placas. Si el bloqueo supera todos los intentos, se declara `[FATAL] LINK_DOWN`, se limpian las líneas a `LOW` y el enlace queda disponible para reintento manual.



### 7.2 Matriz Completa de Escenarios de Fallo y Respuesta del Protocolo

| # | Caso Borde / Condición de Fallo | Detección | Mecanismo de Recuperación Exacto |
| --- | --- | --- | --- |
| **1** | **Error de 1 bit en `D1` o `D2**` (en $t_1$ o $t_2$) | $R_{\text{err}}=1,\; C_{\text{err}}=1$ con $r,c \in \{1,2\}$ | **Autocorrección local:** se invierte el bit en $(r,c)$ al cerrar $t_3$; 0 s de penalización, no se aborta.

 |
| **2** | **Error de 1 bit en redundancia** (`Pr` en $t_1/t_2$, o cualquier bit de $t_3$) | $R_{\text{err}}=1,\; C_{\text{err}}=1$ con $r=3$ o $c=3$ | El algoritmo identifica que los 4 bits de datos están intactos; registra ruido en canal de paridad y continúa sin abortar.

 |
| **3** | **2 bits dañados en filas distintas** ($t_1$ y $t_2$, incluso si ambos fueron en `Pr`) | `row_errors == 2` al muestrear $t_2$ | **Corte en segundo 2:** RX no espera a $t_3$; dispara `PHY_ABORT` `(1,0,0)` de inmediato y vuelve a `RX_IDLE`.

 |
| **4** | **2 bits dañados simultáneamente en la misma fila** (`D1` y `D2` invertidos a la vez) | En $t_1/t_2$ pasa el XOR; en $t_3$ da $R_{\text{err}}=0,\; C_{\text{err}}=2$ | Síndrome ilegal de 2 columnas rotas en $t_3$ $\to$ dispara `PHY_ABORT` `(1,0,0)`.

 |
| **5** | **2 bits dañados en la Fila 3** (paridad vertical) | $R_{\text{err}}=0,\; C_{\text{err}}=2$ | Indistinguible del caso 4; por seguridad matemática dispara `PHY_ABORT` `(1,0,0)` en $t_3$.

 |
| **6** | **3 bits dañados en una matriz** | $d_{\min}=4$ garantiza síndrome no nulo distinto de 1 bit | Nunca produce $R_{\text{err}}=0, C_{\text{err}}=0$; si no coincide con corrección simple, dispara `PHY_ABORT`. (Si por patrón degenerado coincide con un falso error simple, el `CRC-8` final atrapa el bit mal corregido). |
| **7** | **4 bits dañados en rectángulo perfecto** (`D1` y `D2` invertidos en $t_1$ y $t_2$) | Las paridades ortogonales se cancelan; falla **`CRC-8`** al final de la trama | La Capa de Enlace rechaza la trama y envía **`NACK(SEQ)`**, forzando retransmisión completa. Si el rectángulo de 4 bits ocurre sobre las matrices del `CRC-8` (o simultáneamente sobre `DATA` y `CRC-8`), el checksum no coincide en el 99.61% (255/256) de los casos y dispara `NACK(SEQ)` de forma segura.

 |
| **8** | **Bloqueo momentáneo de `CLK` durante trama activa** | Vence `T_CLK_WATCHDOG` ($1.5T$) en RX | RX registra `[ERROR] CLOCK_LOSS`, dispara `PHY_ABORT` `(1,0,0)` y regresa a `RX_IDLE`.

 |
| **9** | **Pérdida del pulso `PHY_ABORT` o bloqueo del `CLK` de retorno** | TX no ve `PHY_ABORT` y completa las matrices | RX, ya en `RX_IDLE`, ignora las filas restantes de datos (paridad válida $\neq (1,1,1)$). Al terminar, TX vence `T_ACK_START` ($2.5T$) sin recibir `SYN` del `ACK` y reintenta desde `SYN` (donde `SYN_ACK` verificará si el bus de retorno sigue bloqueado). |
| **10** | **Corrupción de 1 bit sobre un comando OOB** (`SYN`, `SYN_ACK`, `PHY_ABORT`) | Distancia de Hamming $d_H = 2$ entre comandos OOB | Un error de 1 bit convierte un comando OOB en un vector de paridad válida (nunca en otro comando OOB). Se descarta con seguridad y el watchdog (`T_SYN_ACK` o `T_ACK_START`) resuelve el intento en $\le 2.5T$. |
| **11** | **Ruido espurio en reposo (`IDLE`) simula `SYN` `(1,1,1)**` | RX despierta y valida Matriz 1 (`SFD`) | Si los siguientes 3 ticks no forman `SFD == 1010` (o vence `T_CLK_WATCHDOG` en 1.5 s porque no hay más pulsos de reloj), RX aborta en $\le 3T$ y retorna a `RX_IDLE`. |
| **12** | **Ruido a mitad de trama genera `(1,1,1)**` | FSM de RX está en estado activo ($\neq$ `RX_IDLE`) | **No reinicia la trama:** se procesa únicamente como fallo de paridad de fila ($1 \oplus 1 \neq 1$) para intentar autocorrección.

 |
| **13** | **Colisión bidireccional simultánea** (ambos nodos pulsan "Enviar" a la vez) | Ambos transmiten `SYN` `(1,1,1)` y leen `(1,1,1)` en lugar de `SYN_ACK` `(0,1,0)` | **Desempate determinista en $1T$:** cada placa tiene un `NODE_ID` fijo (`0` = prioritario, `1` = secundario). El Nodo 1 cede inmediatamente, limpia su TX a `IDLE` y pasa a escucha; el Nodo 0 espera $1T$ y reenvía `SYN`. Cero *deadlock*. |
| **14** | **Pérdida o corrupción de la trama `ACK**` | TX vence `T_ACK_START` o recibe `ACK` con `CRC-8` inválido y retransmite con el mismo `SEQ` | RX valida el `CRC-8`, detecta `SEQ != expected_rx_seq` (duplicado), **no entrega el carácter repetido al chat**, registra `[DUP]` en el LOG y reenvía `ACK(SEQ)`.

 |
| **15** | **Canal `CLK` pegado en `HIGH` (*Stuck-at-1*) por luz ambiente** | Detección estricta por flanco ascendente (`LOW -> HIGH`), no por nivel | Un nivel constante `HIGH` jamás genera lecturas fantasma; simplemente cesan los flancos y se activa `T_CLK_WATCHDOG` / `T_SYN_ACK` de forma determinista. |
| **16** | **Canal de datos pegado (*Stuck-at-0/1*) por falla de sensor** | Múltiples correcciones/abortos consecutivos en la misma columna | El `LOG` acumula un contador por canal (`err_ch1`, `err_ch2`, `err_ch3`) y emite diagnóstico explícito (`[DIAG] Degradación persistente en CH1 - Ultrasonido`). |
| **17** | **Secuencia `UTF-8` multibyte truncada** (el enlace cae tras 2 de 4 bytes) | Vence `T_UTF8_FLUSH` o llega nuevo inicio `DATA_ASCII` / cabecera UTF-8 | La Capa de Presentación purga los bytes huérfanos en `utf8_buffer`, registra `[WARN] UTF8_INCOMPLETE_FLUSH` y evita corromper los mensajes posteriores. |

---

## 8. Sustento Matemático y Análisis de Eficiencia

### 8.1 Teoría de Códigos: Código Producto Lineal $[9, 4, 4]$

Cada matriz $3 \times 3$ implementa formalmente el producto tensorial de dos códigos de paridad simple $C_1(n_1=3, k_1=2, d_1=2)$ y $C_2(n_2=3, k_2=2, d_2=2)$:

$$C = C_1 \otimes C_2 \implies [n = 9,\; k = 4,\; d_{\min} = d_1 \cdot d_2 = 4]$$

* **Tasa de código:** $R = \frac{k}{n} = \frac{4}{9} \approx 0.444$.
* **Capacidad de corrección garantizada por matriz:**

$$t = \left\lfloor \frac{d_{\min} - 1}{2} \right\rfloor = \left\lfloor \frac{4 - 1}{2} \right\rfloor = 1\text{ bit por matriz}$$


* **Capacidad de detección garantizada por matriz:**

$$e = d_{\min} - 1 = 3\text{ bits por matriz}$$



### 8.2 Reducción Probabilística de Retransmisiones

Bajo un modelo de canal binario simétrico con probabilidad independiente de error de bit $p$ (BER), la probabilidad de que una matriz de 9 bits sufra un fallo no corregible ($\ge 2$ bits erróneos) es:

$$P_{\text{matriz\_fallida}}(p) = 1 - \left[(1-p)^9 + \binom{9}{1}p(1-p)^8\right] \approx 36p^2 \quad (\text{para } p \ll 1)$$

La probabilidad de fallo de una trama completa de 24 bits (6 matrices) frente a una transmisión sin corrección en vuelo ($P_{\text{sin\_FEC}} = 1 - (1-p)^{24}$) es:

| BER del Canal ($p$) | Fallo de Trama sin Matriz (Serial / Solo CRC) | Fallo por Matriz $3 \times 3$ | Fallo de Trama Completa con PHM-4 (6 matrices) | Factor de Mejora |
| --- | --- | --- | --- | --- |
| **$1\%$ ($0.01$)** | $21.43\%$ | $0.34\%$ | **$2.04\%$** | **$10.5\times$ menos retransmisiones** |
| **$5\%$ ($0.05$)** | $70.80\%$ | $7.12\%$ | **$35.81\%$** | **$2.0\times$ menos retransmisiones** |

### 8.3 Ahorro Temporal Frente a Protocolos Clásicos (Tipo TCP / Stop-and-Wait sin OOB)

Evaluando el costo temporal en segundos a $f = 1\text{ Hz}$:

1. **Fallo doble irrecuperable en la matriz promedio ($k = 3.5$):**
* *Sin `PHY_ABORT` (esperar trama + `NACK`):* $20\text{ s} + 13\text{ s} = \mathbf{33\text{ s}}$ perdidos.
* *Con `PHY_ABORT`:* $1\text{ s (SYN)} + 1\text{ s (SYN\_ACK)} + 3(3.5)\text{ s} + 1\text{ s (ABORT)} = \mathbf{13.5\text{ s}}$ perdidos (**59.1% menos tiempo desperdiciado**).


2. **Bloqueo prolongado de `CLK` durante 3 reintentos consecutivos:**
* *Sin `SYN_ACK` (transmitiendo tramas al vacío):* $3 \times (19\text{ s} + 2.5\text{ s}) = \mathbf{64.5\text{ s}}$ desperdiciados.
* *Con `SYN_ACK` de 1 ciclo + backoff de 2 s:* $3 \times (1\text{ s} + 1.5\text{ s}) + 2 \times 2\text{ s} = \mathbf{11.5\text{ s}}$ (**82.2% menos tiempo muerto**).



---

## 9. Protocolo de Validación Presencial (Pruebas Físicas de Rúbrica)

Para demostrar todos los requisitos de la rúbrica presencial con interferencia estrictamente física a **1 Hz**, ejecute la siguiente secuencia verificable en el `LOG`:

1. **Prueba de Autocorrección en Vuelo (Nivel 1 — Datos y Paridad):**
* *Acción física:* Obstruir **1 solo canal** (p. ej., tapar `CH1` Ultrasonido) durante exactamente 1 segundo mientras se transmite la Matriz 3 (`DATA`).
* *Resultado esperado:* El `LOG` muestra el error de fila en $t_1$, calcula la intersección $(r,c)$ en $t_3$, corrige el bit y entrega el carácter intacto en el WebChat sin retransmisión. Repetir tapando `CH3` (`Pr`) durante 1 segundo para evidenciar inmunidad ante ruido exclusivo en redundancia.




2. **Prueba de Aborto Temprano (Nivel 2 — `PHY_ABORT`):**
* *Acción física:* Obstruir `CH1` durante 2 segundos seguidos (corrompiendo $t_1$ y $t_2$ de la misma matriz).
* *Resultado esperado:* Al segundo 2 de esa matriz, el receptor dispara `PHY_ABORT` `(1,0,0)`; el transmisor corta la trama inmediatamente sin completar los 20 s y reintenta de forma limpia.




3. **Prueba de Bloqueo Prolongado de Reloj (`CLOCK_LOSS` + `SYN_TIMEOUT`):**
* *Acción física:* Tapar el emisor IR de `CH0` (`CLK`) durante 6 segundos en medio de una trama.
* *Resultado esperado:* En 1.5 s el receptor registra `CLOCK_LOSS` y envía `PHY_ABORT`. Al intentar reenviar, el transmisor detecta `SYN_TIMEOUT` en 1.5 s sin desperdiciar matrices; al retirar la mano, el siguiente reintento completa la entrega automáticamente.


4. Prueba Obligatoria de `CRC-8` $\to$ `NACK` $\to$ Retransmisión (Nivel 3, totalmente física):


* *Acción física:* Durante los 2 primeros ticks ($t_1$ y $t_2$) de la Matriz 3 (`DATA_H`), obstruir **simultáneamente los dos canales de datos (`CH1` y `CH2`)** (manteniendo libres `CLK` y `Pr`), y liberar ambos canales antes de $t_3$.
* *Fundamento físico-matemático:* Esta obstrucción invierte en rectángulo los 4 bits de datos de la matriz, preservando todas las paridades horizontales y verticales ($R_{\text{err}}=0, C_{\text{err}}=0$). La matriz $3 \times 3$ deja pasar el nibble corrupto, demostrando el límite teórico $d_{\min}=4$, y al recibir las Matrices 5–6 el **`CRC-8` detecta la corrupción, dispara la trama `NACK` y obliga a la retransmisión exitosa**.




5. Prueba de Codificación Completa y Selectores:


* Enviar `"Hola ñ € 🚀"` en modo `UTF-8` (demostrando tramas de 1, 2, 3 y 4 bytes) y cambiar en vivo a `Extended ASCII` y a cada uno de los 4 algoritmos `CRC-8` (`SMBUS`, `AUTOSAR`, `SAE-J1850`, `MAXIM-DOW`) verificando los registros en el `LOG`.