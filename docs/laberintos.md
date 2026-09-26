# Laberintos de prueba

## Resumen

El robot se prueba en varios laberintos pequeños antes de la competición 16x16. Todos son versiones del **laberinto de práctica 4×3** con la meta en **(3,2)** (esquina sureste), rearrangements de pared simple.

---

## Laberinto de Práctica (4×3)

**Tamaño:** 4×3 celdas  
**Meta:** (3,2) - celda única en la esquina sureste  
**Características:**
- Laberinto base para validación: búsqueda, control de velocidad, control de giro, centrado
- Recta de 9 celdas con 4 curvas suave en la ruta rápida (una U incluida)
- Búsqueda ~19.4 s a SPD 600, lo mismo cada vez, sin paredes dudosas
- Control: rectas de 3 celdas a 700 mm/s paran en ~2 mm del objetivo, giros <0.2°

**Verificado:**
- Curvas suave: 3.14 s hasta meta, 3.27 s de vuelta (FAST 700/CURVE 400)
- Giros en sitio: TURNTICKS 405 medido en tests CAL TURN
- Centrado: KP 0.70, KI 8.00 (FRONT_SQUARE_OFFSET_MM confirmado con CAL NOISE: -15.4)

---

## Layout B

**Base:** Laberinto de práctica 4×3 rearrangement simple  
**Meta:** (3,2)  
**Características:**
- Búsqueda ~7.4 s contra 8.5 s si se para en cada celda
- Piernas de búsqueda de una sola celda: sin paredes dudosas en los lados
- Control de velocidad: paradas del IR en ~3.5 mm del plan

**Próximos pasos:** validar piernas largas

---

## Layout C (validado 2026-09-26)

**Base:** Laberinto de práctica 4×3 rearrangement  
**Meta:** (3,2)  
**Características:**
- **Ruta:** 1D1I1D2 = una curva derecha, izquierda, derecha, 2 rectas
- **Escalera de curvas:** tres curvas en celdas consecutivas, luego 2 celdas a lo largo del borde norte
- Detectó: el centrado aprendía un inicio descentrado (8-12 mm) como sesgo de rumbo (~5°), lo que desviaba la última recta hacia la pared

**Problemas encontrados:**
- Velocidad de carrera 478 mm/s: las curvas giran ~2.4° menos de lo que dicen los encoders (4.9° para dos curvas)
- Llegada yawada 5.5° y 24 mm descentrada
- Se arrastró y quedó atascada girando en la meta

**Solucionado:**
- Observador de sesgo en lugar de integral del error lateral (commit 1472d96)
- Limitación de cuadratura a 12° máximo (commit 93370fc)

**Próximos pasos:** revalidar con compensación de `CURVE_SLIP` en layout D

---

## Layout D (próxima prueba)

**Base:** Layout C con una pared movida  
**Descripción:** Toma Layout C y desplaza la pared entre (0,1)/(0,2) a la posición (0,1)|(1,1)  
**Meta:** (3,2)  
**Ruta simplificada:** 
- **N:** 2 celdas hacia el norte
- **E:** 3 celdas hacia el este hasta la meta
- **Descripción:** Una curva seguida de 3 celdas rectas

**Propósito:**
- Validar `CURVE_SLIP` (compensación de giro en curvas) en `CAL RUN`
- Verificar que el centrado en la última recta tenga sesgo de rumbo promedio ~0°
- Prueba simple de decisión de puntos y muro frontal a 450 mm/s

**Cómo construirlo:** Parte del layout C, mueve una pared frontal. Mantén meta (3,2).

---

## Competición 16×16

**Tamaño:** 16×16 celdas  
**Meta:** Centro 2×2 bloque: (7,7)-(8,8)  
**Características:**
- Búsqueda exploratoria completa
- Ruta rápida optimizada en tiempo (no solo en celdas)
- Sin paredes dudosas al final (todas confirmadas)
- Curvas suave en toda la ruta rápida

---

## Tabla resumen

| Laberinto | Tamaño | Meta | Ruta | Búsqueda | Notas |
|-----------|--------|------|------|----------|-------|
| Práctica | 4×3 | (3,2) | 9 celdas, 4 curvas | ~19.4 s | Base de validación |
| Layout B | 4×3 | (3,2) | 7.4 s | 7.4 s | Piernas de búsqueda cortas |
| Layout C | 4×3 | (3,2) | 1D1I1D2 | - | Escalera de curvas, sesgo observado |
| Layout D | 4×3 | (3,2) | 1 curva + 3 rectas | - | Prueba CURVE_SLIP (próxima) |
| 16×16 | 16×16 | (7,7)-(8,8) | Variable | Variable | Competición real |

