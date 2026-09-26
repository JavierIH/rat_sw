# rat_sw

Firmware de micromouse para un robot con STM32F103 ("Blue Pill"): explora un
laberinto desconocido, lo mapea, calcula la ruta más rápida y la recorre.

- **Búsqueda** (modo 1): explora hasta la meta, sigue explorando solo las
  celdas que aún podrían acortar el camino rápido hasta que el mejor camino
  queda verificado, vuelve a la salida explorando y guarda el mapa en flash.
- **Carrera rápida** (modo 2): recorre el camino verificado con las rectas
  fusionadas a velocidad `FAST`, frenando antes de cada giro; vuelve a la
  salida sola.
- El planificador minimiza **tiempo** (celdas y giros), no solo celdas.
- El mapa, la meta y los parámetros sobreviven a reinicios.

## Uso

| Botón | Parado | Durante un run |
|---|---|---|
| SELECT | cambia de modo (LED n encendido = modo n) | — |
| START | lanza el modo tras 2 s de cuenta atrás | detiene el run |

Modos: 1 búsqueda, 2 carrera rápida, 3 monitor de sensores (motores apagados;
paredes en los LEDs), 4 borrar mapa (pide pulsar START otra vez en 3 s).

Flujo de competición: `ERASE` (o modo 4) si el laberinto es nuevo → modo 1 →
modo 2 tantas veces como quieras (subiendo `FAST` entre carreras).

LEDs: modo seleccionado (con un parpadeo por segundo = vivo); durante las
rectas, 1-3 = centrando con la pared izquierda, 4-6 = con la derecha; fallo =
3 parpadeos rápidos; parpadeo continuo rápido = `Error_Handler`; parpadeo lento
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

Un comando por línea (en el monitor o en cualquier terminal serie); `HELP`
los lista.

| Comando | Qué hace |
|---|---|
| `MODE n`, `START`, `STOP` | elegir, lanzar y detener modos |
| `PAUSE`, `RESUME`, `STEP ON/OFF` | pausa, y modo paso a paso (para tras cada acción) |
| `STATUS`, `MAP` | estado y parámetros; mapa ASCII con el camino rápido |
| `IR`, `WALLS` | sensores en mm y crudo; paredes detectadas ahora mismo |
| `SPD n`, `FAST n`, `TURN n` | PWM de crucero en búsqueda y en carrera rápida, y de giro (0-1000). Cada recta frena a 150 antes de parar, así que las paradas no dependen de ellos |
| `TURNTICKS n` | ticks de un giro de 90° (~5 por grado; menos = gira menos) |
| `KP f`, `KI f`, `KD f`, `KE f` | centrado en pasillo (`KI` corrige el desvío por motores desiguales); `KE` mantiene el rumbo sin paredes (0 = off) |
| `LOG 0-2`, `DEFAULTS` | detalle del log; parámetros por defecto |
| `GOAL x y [x1 y1]` | celdas meta (p. ej. `GOAL 7 7 8 8` para 16x16) |
| `SAVE`, `ERASE`, `HOME`, `RESET` | guardar, borrar mapa, "estoy en la salida", reiniciar |
| `SYNC`, `TELEM ON/OFF` | reenviar mapa y estado al monitor; activar la telemetría |
| `CAL …` | pruebas de calibración (ver abajo) |

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
(este último con `tools/dashboard.py`). Detalles de arquitectura y
convenciones en [AGENTS.md](AGENTS.md).
