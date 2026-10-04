# rat_sw

Firmware de micromouse para un robot con STM32F103: explora un
laberinto desconocido, lo mapea, calcula la ruta más rápida y la recorre.

- **Búsqueda** (modo 1): explora hasta la meta y, al llegar, guarda el mapa
  en flash (3 parpadeos lentos: guardado); sigue explorando solo las celdas
  que aún podrían acortar el camino rápido hasta que el mejor camino queda
  verificado, vuelve a la salida explorando y, si aprendió algo, lo guarda
  otra vez.
- **Carrera rápida** (modo 2, carreras 4-6): recorre el camino verificado con las rectas
  fusionadas a velocidad `FAST`, frenando antes de cada giro; en la meta
  guarda el mapa si cambió (nunca al volver; si no llega a la meta, no
  guarda) y vuelve a la salida sola.
- El planificador minimiza **tiempo** (celdas y giros), no solo celdas.
- El mapa, la meta y los parámetros sobreviven a reinicios.

## Uso

| Botón | Parado | Durante un run |
|---|---|---|
| SELECT | tras el arranque (LEDs en vaivén) elige el modo 1; luego cambia de modo (1: LEDs 1-2, 2: 3-4, 3: 5-6); en el menú de carreras, cambia de carrera (LEDs 1..n) | — |
| START | lanza el modo tras 5 s de cuenta atrás, con todos los LEDs parpadeando rápido (ignorado hasta elegir modo); en el modo 2 abre el menú de carreras, y ahí lanza la carrera | detiene el run |

Modos: 1 búsqueda, 2 carrera, 3 borrar mapa y volver a la meta por defecto
(pide pulsar START otra vez en 3 s). Menú de carreras (parpadea lento; solo se sale con reset): 1 seguidor
de pared izquierda, 1-2 derecha (pasan por la meta y siguen hasta STOP), 1-3 sin curvas (camino
verificado, rectas a `FAST` 800, gira parado; ida y vuelta), 1-4 `FAST 800
CURVE 300`, 1-5 `FAST 900 CURVE 400`, 1-6 `FAST 900 CURVE 480`. Elegir una
carrera fija `FAST` y `CURVE`; por consola se pueden cambiar antes de START.

Flujo de competición: `ERASE` (o modo 3) si el laberinto es nuevo → modo 1 →
carrera 2.4, y las más rápidas cuando la anterior salga bien.

LEDs: modo seleccionado (con un parpadeo por segundo = vivo); durante las
rectas, 1-3 = centrando con la pared izquierda, 4-6 = con la derecha; mapa
guardado en flash = 3 parpadeos lentos; fallo = 3 parpadeos rápidos (también:
START sin camino verificado, o el mapa no se pudo guardar y solo está en RAM:
no reinicies ni apagues); parpadeo continuo rápido = `Error_Handler`; parpadeo lento
continuo = fallo grave de CPU (motores parados).

## Monitor en vivo

```
python3 tools/robot_monitor.py                 # /dev/rfcomm0 a 9600 baudios
python3 tools/robot_monitor.py --replay tools/logs/<sesion>.log --speed 4
```

Dibuja el laberinto mientras el robot lo explora: paredes confirmadas y
dudosas, celdas visitadas (en amarillo las de este run), el robot con su
orientación, la **ruta que va a seguir** (calculada con el mismo planificador
y costes que el firmware), las celdas candidatas mientras optimiza y, parado,
el camino rápido verificado. Al lado, el log con colores y la consola.

- Teclas: Enter envía, ↑/↓ historial, RePág/AvPág desplazan el log, Tab
  muestra la ayuda, **Ctrl+X manda STOP**, Esc sale. Locales: `/full`
  (16x16 siempre), `/ascii`, `/clear`, `/nota`.
- Cada sesión se graba en `tools/logs/` y se puede reproducir con `--replay`.
- El robot envía la telemetría (líneas `@…` de ~20 bytes) solo entre
  acciones; el monitor solo transmite cuando escribes (más un `SYNC` al
  conectar). No frena al robot. `TELEM OFF` la desactiva.
- Abre el puerto en exclusiva: si otro programa lo usa, lo dice.

## Consola Bluetooth

Un comando por línea (en el monitor o en cualquier terminal serie), en
mayúsculas o minúsculas. El firmware no trae ayuda (`HELP`) para ahorrar
flash: esta tabla es la referencia. "Parado" = se rechaza durante un run.

| Comando | Qué hace | Parado |
|---|---|---|
| `STATUS` | estado, parámetros, pila, causa del reinicio, registros de reloj y flash, chip, huecos libres de la flash y tiempos de su última escritura; tras una búsqueda, lo que tardó la peor decisión en marcha (también se imprime al acabar el run) | |
| `MODE n [k]` | elige el modo: 1 búsqueda, 2 k carrera k (1/2 seguidor izquierda/derecha, 3 sin curvas, 4 800/300, 5 900/400, 6 900/480), 3 borrar mapa | sí |
| `START` | lanza el modo elegido tras 2 s (como el botón) | |
| `STOP` | detiene el run (como START durante el run) | |
| `PAUSE`, `RESUME` | frena y espera; sigue tras una pausa o un paso | |
| `STEP ON\|OFF` (o `DEBUG`) | paso a paso: pausa tras cada acción | |
| `SPD n` | mm/s de crucero en la búsqueda y en la vuelta de la carrera rápida | |
| `FAST n` | mm/s de crucero en las rectas de la carrera rápida | |
| `CURVE n` | mm/s en las curvas de la carrera rápida (los motores lo limitan a ~480) | |
| `ACCEL n` | mm/s² de aceleración y frenada en recta | |
| `TURN n`, `TACCEL n` | grados/s máximos y grados/s² de los giros en el sitio | |
| `TURNTICKS n` | ticks de encoder de un giro de 90° (menos = gira menos) | |
| `KP f` | centrado: grados de rumbo por mm descentrado | |
| `KI f` | centrado: corrige el rumbo torcido (grados por mm y metro; 0 = apagado) | |
| `TUNE [nombre valor]` | ajusta en vivo una constante del control (no se guarda); `TUNE` solo las lista | |
| `LOG 0-2` | detalle del log | |
| `TELEM ON\|OFF` | líneas `@` para el mapa en vivo del monitor | |
| `CONT ON\|OFF` | búsqueda con rectas sin parar (ON, por defecto) o parando en cada celda; hasta reiniciar | sí |
| `DEFAULTS` | vuelve a los parámetros por defecto | |
| `IR` | sensores en mm y crudos, y encoders | |
| `WALLS` | detecta las paredes ahora mismo | sí |
| `MAP` | dibuja el mapa ASCII con el camino rápido | sí |
| `GOAL x y [x1 y1]` | celdas meta (p. ej. `GOAL 7 7 8 8` para 16x16) | sí |
| `SAVE` | guarda mapa, meta y parámetros (si la flash no tiene hueco, la compacta) | sí |
| `ERASE` | borra el mapa en RAM y en flash y vuelve a la meta por defecto de la compilación (16x16: 7 7 8 8) | sí |
| `HOME` | "el robot está en la salida mirando al norte" | sí |
| `SYNC` | reenvía mapa y estado al monitor | sí |
| `CAL …` | pruebas de calibración (ver abajo) | sí |
| `RESET` | reinicia el micro (se niega si la flash está bloqueada: apaga y enciende) | |

## Datos de calibración

El robot graba encoders, PWM aplicado y los 4 IR en crudo cada 2-10 ms
durante una prueba (si no cabe, espacia las muestras en vez de cortar el
final) y al acabar los vuelca; el monitor los guarda como CSV en
`tools/calib_data/` con todas las constantes del firmware. Las pruebas que
mueven el robot esperan 2 s (START o STOP cancelan).

| Prueba | Qué hacer | Para calibrar |
|---|---|---|
| `CAL NOISE [ms]` | robot quieto (no se mueve) | ruido de los sensores |
| `CAL STRAIGHT [celdas] [pwm]` | en un pasillo; luego `/nota medido <mm> mm` | distancia por celda, centrado KP/KD |
| `CAL TURN [±cuartos]` | en el sitio; luego `/nota angulo <grados>` | `TURNTICKS`, sobregiro |
| `CAL STEP [pwm] [ms]` | espacio libre delante | modelo del motor, frenada |
| `CAL IR [mm]` | pegado a una pared de frente; `/nota inicio <mm> mm` | curva de los IR frontales |
| vueltas de `CAL NOISE` + `CAL TURN 1` (x4) | en una celda cerrada por 3 lados, centrado delante-detrás; una vuelta por posición, moviendo el robot 10-15 mm a cada lado entre vueltas | pendiente y centro (`SIDE_CENTER_L/R_MM`) de los IR laterales |
| `CAL DUMP` | — | reenviar la última grabación |

```
python3 tools/calib_analyze.py tools/calib_data/*.csv
```

resume cada fichero y, con las notas, propone valores concretos (por
ejemplo `CELL_TICKS`/`MOVE_EXTRA_TICKS` combinando rectas de 1 y 3 celdas, o
coeficientes nuevos de los IR listos para `infrared.c`).

## Puesta a punto tras el rework

Las calibraciones de siempre se conservan (`src/robot_config.h`). En el
laberinto real conviene validar, en este orden:

1. `LOG 2` + una búsqueda en el laberinto de práctica: los avances de una
   celda y los giros usan las mismas fórmulas, velocidades y puntos de parada
   que antes (la parada por IR se confirma ahora durante 3 ms).
2. Carrera rápida con `FAST` igual a `SPD` (150): comprueba que las rectas
   largas terminan centradas en la celda. Si terminan largas o cortas, ajusta
   el reparto entre `CELL_TICKS` y `MOVE_EXTRA_TICKS`.
3. Sube `FAST` poco a poco (220 por defecto) y guarda con `SAVE`.
4. Opcional: `KE 0.5` para mantener el rumbo donde no hay paredes laterales.
5. Sensores laterales (van en el morro a 15°): si el robot se para
   demasiado cerca de una pared frontal o torcido, su lectura de "hay pared"
   en el lado expuesto se marca dudosa (`?` en el log y en `WALLS`) y no se
   apunta en el mapa. Para que detecte bien el giro, calibra
   `FRONT_SQUARE_OFFSET_MM`: robot centrado en una celda y recto frente a una
   pared, `CAL NOISE`, y `calib_analyze.py` te da el valor.

## Compilar, flashear y probar

```
pio run                  # firmware del robot
pio run -t upload        # flashear (ST-Link con openocd, ver AGENTS.md)
make -C test/host        # tests en PC: planificador, estrategias y simulador
test/host/build/host_tests --demo   # log y mapa de una búsqueda simulada
python3 -m unittest discover -s tools -p 'test_*.py'   # monitor y análisis
```

Entornos de prueba de hardware: `pio run -e uart_test` y `pio run -e diag_test`
(este último con `tools/dashboard.py`). `pio run -e virtual -t upload`:
robot virtual, el firmware del robot con un laberinto 16x16 grabado;
búsqueda y carreras se recorren en él a velocidad real y guardan en la
flash de verdad (para probar los guardados sin laberinto). Las ruedas no se
mueven salvo con `TUNE RUEDAS 1` (solo con el robot en alto: giran con cada
movimiento). Cada carrera empieza con el robot virtual en la salida. Al
volver al firmware real, su mapa se ignora (manda `ERASE`). Detalles de arquitectura y
convenciones en [AGENTS.md](AGENTS.md).
