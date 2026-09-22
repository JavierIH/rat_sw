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

Modos: 1 búsqueda, 2 carrera rápida, 3/4 seguidor de pared izquierda/derecha,
5 monitor de sensores (motores apagados; paredes en los LEDs), 6 borrar mapa
(pide pulsar START otra vez en 3 s).

Flujo de competición: `ERASE` (o modo 6) si el laberinto es nuevo → modo 1 →
modo 2 tantas veces como quieras (subiendo `FAST` entre carreras).

LEDs: modo seleccionado (con un parpadeo por segundo = vivo); durante las
rectas, 1-3 = centrando con la pared izquierda, 4-6 = con la derecha; fallo =
3 parpadeos rápidos; parpadeo continuo rápido = `Error_Handler`; parpadeo lento
continuo = fallo grave de CPU (motores parados).

## Consola Bluetooth

HC-05 a 9600 baudios (`/dev/rfcomm0`). Monitor recomendado:
`python3 tools/robot_monitor.py`. Un comando por línea; `HELP` los lista.

| Comando | Qué hace |
|---|---|
| `MODE n`, `START`, `STOP` | elegir, lanzar y detener modos |
| `PAUSE`, `RESUME`, `STEP ON/OFF` | pausa, y modo paso a paso (para tras cada acción) |
| `STATUS`, `MAP` | estado y parámetros; mapa ASCII con el camino rápido |
| `IR`, `WALLS` | sensores en mm y crudo; paredes detectadas ahora mismo |
| `SPD n`, `FAST n`, `TURN n` | PWM de búsqueda, crucero rápido y giro (0-1000) |
| `KP f`, `KD f`, `KE f` | centrado en pasillo; `KE` mantiene el rumbo sin paredes (0 = off) |
| `LOG 0-2`, `DEFAULTS` | detalle del log; parámetros por defecto |
| `GOAL x y [x1 y1]` | celdas meta (p. ej. `GOAL 7 7 8 8` para 16x16) |
| `SAVE`, `ERASE`, `HOME`, `RESET` | guardar, borrar mapa, "estoy en la salida", reiniciar |

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

## Compilar, flashear y probar

```
pio run                  # firmware del robot
pio run -t upload        # flashear (ST-Link con openocd, ver AGENTS.md)
make -C test/host        # tests en PC: planificador, estrategias y simulador
test/host/build/host_tests --demo   # log y mapa de una búsqueda simulada
```

Entornos de prueba de hardware: `pio run -e uart_test` y `pio run -e diag_test`
(este último con `tools/dashboard.py`). Detalles de arquitectura y
convenciones en [AGENTS.md](AGENTS.md).
