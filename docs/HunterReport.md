# Reporte: Funcionamiento del Tanque + Estrategia Hunter

**Fecha**: 2026-05-31
**Repositorio**: https://github.com/francofreda2/wakuseibokan
**Agente bajo evaluación**: [`scripts/Hunter.py`](../scripts/Hunter.py) (commit `7fdb2a8`)
**Plataforma de prueba**: [`scripts/MockSimulator.py`](../scripts/MockSimulator.py) (commit `dc7d99f`)
**Escenario emulado**: Testcase 111 / 131 (duelo 1v1 de tanques Walrus/Otter)

---

## 1. Resumen ejecutivo

> El tanque **funciona correctamente**. La estrategia **gana 15 de 20 matches** (75 %), no pierde ninguno y empata únicamente contra el rival más simétrico (`sniper`).

| Métrica | Valor |
|---|---|
| Matches totales | **20** |
| Victorias | **15** |
| Derrotas | **0** |
| Empates | **5** |
| Win-rate | **75 %** |
| Loss-rate | **0 %** |
| HP promedio al final (Hunter) | 538 / 1000 |
| HP promedio al final (rival)  | 0 / 1000 |
| Total de hits propinados | **281** |
| Total de hits recibidos | **118** |
| Ratio damage out / damage in | **2.38x** |

---

## 2. Metodología

- **20 matches** en total: Hunter (tanque 1) vs cada uno de los 4 rivales del [OpponentsZoo](../scripts/OpponentsZoo.py), 5 matches por rival.
- Simulador: `MockSimulator.py`, escenario plano, 5000 ticks por match (250 s simulados), velocidad de cañón 600 m/s con drag, gravedad −9.81 m/s², damping linear 0.01, radio efectivo de daño 35 m, 80 daño por bala, vida y munición inicial 1000.
- Posiciones de spawn aleatorias en un cuadrado de 1500 m (distancia inicial entre tanques ≈ 1500 m).
- Comunicación UDP idéntica al simulador real: puertos 4501/4502 para comandos, 4601/4602 para telemetría, formato binario de 68 / 96 bytes.
- Criterio de victoria: HP final propio > HP rival + 10 (margen para evitar ambigüedades).

---

## 3. Resultados detallados

### 3.1 Tabla resumen por rival

| Rival | Tipo | W-L-D | Hits propinados | Hits recibidos | HP final Hunter (avg) |
|---|---|---:|---:|---:|---:|
| **smartlead** | Aproacher con lead balístico | **5-0-0** | 67 | 53 | 152 |
| **sniper** | Estático con apuntado fino | 0-0-5 | 65 | 65 | 0 (anihilación mutua) |
| **rusher** | Embiste a máxima velocidad | **5-0-0** | 75 | 0 | **1000** (sin daño) |
| **zigzag** | Anti-predicción aleatoria | **5-0-0** | 74 | 0 | **1000** (sin daño) |
| **Total** | | **15-0-5** | **281** | **118** | **538** |

### 3.2 Detalle match-por-match

**vs smartlead** (5 victorias, daño moderado recibido):
```
match 1: 200 / 0   hits 15-10
match 2: 200 / 0   hits 14-10
match 3: 200 / 0   hits 12-10
match 4: 120 / 0   hits 13-11
match 5:  40 / 0   hits 13-12
```
Hunter gana con margen pero recibe consistentemente ~10 hits. SmartLead ataca con lead balístico parecido, pero su estimación de velocidad rival es frame-a-frame sin iteración → falla más cuando Hunter zigzaguea.

**vs sniper** (5 empates de aniquilación mutua):
```
match 1-5: 0 / 0   hits 13-13 (cada match)
```
Sniper se queda quieto y apunta fino. Hunter cierra distancia para mejorar su propia precisión, pero queda dentro del campo de tiro del sniper. Resultado: ambos llegan a HP 0 al mismo tiempo, con cantidad de hits casi idéntica. **Aquí hay margen de mejora** (ver §6).

**vs rusher** (sweep perfecto, daño cero recibido):
```
match 1-5: 1000 / 0   hits 14-18 vs 0
```
Rusher viene a embestir con thrust máximo. Su aim es pésimo en movimiento. Hunter se queda a ~900 m, dispara al cuerpo presente, lo destroza. Hunter ni siquiera necesita zigzag.

**vs zigzag** (sweep perfecto):
```
match 1-5: 1000 / 0   hits 14-18 vs 0
```
El más sorprendente. Zigzag oscila la dirección cada 20-40 ticks intentando romper el lead. Hunter usa `solve_moving_intercept` con 4 iteraciones, así que recalcula el punto de impacto al ritmo del cambio de dirección. Zigzag sigue disparando pero sus propias rotaciones sacan al cañón de la línea hacia Hunter.

---

## 4. Estrategia del Hunter

### 4.1 Filosofía: simple gana

El Hunter es **257 líneas, sin PID, sin posturas, sin profiler**. Después de armar un agente complejo (`Strategist.py`, 600 líneas con 6 posturas, 2 PIDs, profiler de 5 categorías, anti-stuck, detector de fuego entrante) que **perdía 0-3** contra SmartLead, recuperamos un principio dolorosamente aprendido en robótica reactiva: **las capas que se contradicen meten lag y oscilación**.

### 4.2 Componentes

| Componente | Implementación |
|---|---|
| **Apuntería** | [`solve_moving_intercept`](../scripts/Ballistic.py#L262) con 4 iteraciones — predice dónde va a estar el rival al momento del impacto, recalcula el tiempo de vuelo según la nueva distancia, repite hasta converger. |
| **Estimador de velocidad rival** | Derivada directa frame-a-frame `(p_now - p_prev) / dt`. Sin EMA (probado: el filtro EMA introduce lag de 1-2 ticks que mata el aim). |
| **Heading control** | Bang-bang: si `\|heading_err\| > 8°` → steering = ±1.0 (rotación máxima). Sino, proporcional fino. |
| **Movimiento** | Mantener distancia ideal de **900 m** (compromiso entre tiempo de vuelo y precisión). Si rival a < 400 m, reversa. Si > 1050 m, full thrust adelante. |
| **Gate anti-spiral** | Si `\|heading_err\| > 25°` → thrust = 0. No avanzar mientras estás girando 90°. |
| **Disparo** | Cada tick que tenga solución balística válida y power > 20. Sin umbral de aim_err. La torreta rota instantáneamente al ángulo deseado. |
| **Anti-predicción** | Si recibió daño en el último tick, oscilar el setpoint de heading ±30° con período aleatorio entre 15-40 ticks. |

### 4.3 Lo que NO tiene (intencionalmente)

- **PIDs**: el chasis gira a 60 °/s configurado, no necesita feedback control sofisticado.
- **Clasificación de rival**: el profiler tomaba 30 ticks (1.5 s) de "observar". El rival aprovecha esos 1.5 s para tirar primero.
- **Posturas (SNIPER/LEAD/KITE/CHASE)**: cada switch reseteaba PIDs → 0.5-1 s de re-estabilización → más oportunidades para el rival.
- **Detección de incoming fire**: la implementación previa marcaba siempre TRUE por un bug, gatillando defensive override en bucle.
- **Anti-stuck con reversa**: detectaba como "stuck" la rotación en el lugar (porque `dx,dz ≈ 0`) y mandaba a reversa al máximo, alejándose del rival.

---

## 5. Funcionamiento físico del tanque

Esta sección documenta el comportamiento **observado** del tanque (no del agente). Información útil para entender el dominio.

### 5.1 Dinámica del chasis (Otter)

- Velocidad máxima: **28 m/s** (con thrust=1, en escenario plano y sin obstáculos).
- Rotación máxima: aproximadamente **60 °/s** con steering ±1 (medido en mock; en sim real varía con la pendiente y el contacto de las ruedas).
- Tiempo de respuesta del thrust a velocidad: ~0.5 s (control first-order con k=2).
- Inercia angular: el chasis tarda ~150 ms en empezar a girar tras cambio brusco de steering.
- Damping linear: 1.5 m/s² de deceleración natural cuando thrust=0.

### 5.2 Balística del cañón

- Velocidad inicial: **600 m/s** ([AdvancedWalrus.h:21](../src/units/AdvancedWalrus.h#L21))
- Salida de la boca: 40 m adelante + 2.3 m arriba del centro de masa ([AdvancedWalrus.cpp:40, :731](../src/units/AdvancedWalrus.cpp#L731))
- Drag (linear damping ODE): factor 0.9995 por tick → ~3 % de pérdida de velocidad sobre 2 segundos de vuelo
- Gravedad: −9.81 m/s² (eje y)
- TTL del proyectil: 500 ticks = 25 s, alcance máximo teórico ~12 km en declinación óptima
- **Declinación útil**: rango angosto de **0° a 3°** para combate en 0–3000 m. Disparos a 1500 m necesitan ~1.1°. Diferencias de 30 m de altura del blanco requieren ~2°.
- Cooldown entre disparos: 20 ticks = 1 s
- Daño por impacto: 80 HP (12.5 hits para matar)
- Radio efectivo: ~25-35 m (en el sim real depende de la geometría de la unidad)

### 5.3 Anomalías detectadas y corregidas durante el desarrollo

1. **Spawn wheel jam** ([testcase_111.cpp](../src/tests/testcase_111.cpp), [testcase_131.cpp](../src/tests/testcase_131.cpp), commit `652f6ee`)
   El testcase reseteaba las ruedas a la posición del chasis después de que `Wheel::attachTo()` ya las había colocado en su offset correcto. El joint hinge2 resolvía la violación con fuerzas enormes, dejando el tanque rocked o atorado. Sin este fix, el tanque quedaba completamente inmóvil durante un match entero.

2. **Convención de bearing invertida** (commit `24521dd`)
   El simulador C++ usa azimuth compass (azimuth 0 = +z norte, **+90 = oeste**, +270 = este). Mi Python usaba la convención matemática estándar (atan2(dx,dz) con +90 = este). Resultado: durante 4 commits las balas iban **al lado contrario** del rival. Verificado con `scripts/test_aim.py`. Fix: `atan2(-dx, dz)`.

3. **PID de distancia con signo invertido** (commit `1cec2c5`)
   El PIDController genera `error = setpoint - pv`. Para el caso de distancia, `dist > sp` significa "estoy lejos, debería avanzar (thrust > 0)" pero el error es negativo. La salida del PID iba en reversa cuando el rival se alejaba. Fix: negar la salida.

4. **Detector de incoming fire siempre activo** (commit `652f6ee`)
   La función actualizaba el timestamp antes de comparar contra él, por lo que `timer - last_change < 30` siempre era cero. INFIRE permanente → override de thrust → tanque siempre forzado a avanzar a 12 m/s aunque el agente quisiera frenar.

---

## 6. Limitaciones y trabajos a futuro

### 6.1 Aniquilación mutua vs Sniper (problema abierto)

El Sniper es un blanco estático que apunta perfecto. Hunter cierra distancia para aumentar su propia precisión, pero entra en el campo de tiro del sniper. Resultado: ambos llegan a 0 HP simultáneamente.

**Posibles soluciones**:
- Mantener distancia mayor (1500+ m) cuando se detecta rival sin movimiento.
- Aproach lateral en arco (siempre con velocidad tangencial), no en línea recta — rompe el lead del sniper si lo tiene.
- Detección de "rival estacionario" → forzar aproach con zigzag amplio.

Implementación estimada: 30 líneas adicionales al Hunter.

### 6.2 Brecha mock ↔ simulador real

El mock no implementa:
- Colisiones con terreno (relevante para TC=131 con colinas y warehouses).
- Daño por agua (caer fuera de la isla).
- Latencia variable de UDP (en el mock es 0).
- Inestabilidad de física ODE en condiciones extremas.

La estrategia del Hunter NO depende de ninguna de estas, así que el transfer al sim real debería ser directo. Pero el simulador real **crashea con frecuencia** durante el desarrollo: sin un wrapper robusto que lo relance, ejecutar 20 matches contra él es difícil.

### 6.3 No probado contra agentes humanos o con ML

Todos los rivales del zoo son agentes scripted con heurísticas simples. Contra un agente con RL entrenado o un humano hábil el resultado podría variar.

---

## 7. Conclusión

El tanque **funciona correctamente** y la **estrategia Hunter gana o empata el 100 % de los matches** contra el rango de rivales scripted probados.

El proceso de llegar a una estrategia ganadora requirió:
1. Encontrar y arreglar **4 bugs serios** (físico C++, dos convenciones invertidas, lógica de detección).
2. Construir un **simulador mock** UDP-compatible para iterar sin crashes y a 5× velocidad real.
3. **Desechar capas de complejidad** que no producían valor (PIDs con overshoot, posturas que se cancelaban, profiler con lag).

El agente final es simple, predecible y rápido. **257 líneas, sin estados internos profundos**, le gana a un agente parecido (SmartLead) consistentemente.

Quedan los empates contra Sniper como mejora identificable y el transfer a TC=131 con terreno irregular como prueba pendiente.

---

## Apéndice — Archivos relevantes

| Archivo | Propósito |
|---|---|
| [`scripts/Hunter.py`](../scripts/Hunter.py) | Agente principal |
| [`scripts/Ballistic.py`](../scripts/Ballistic.py) | Modelo balístico + lead solver iterativo |
| [`scripts/MockSimulator.py`](../scripts/MockSimulator.py) | Simulador UDP-compatible para benchmarking |
| [`scripts/OpponentsZoo.py`](../scripts/OpponentsZoo.py) | 4 rivales sintéticos (smartlead/sniper/rusher/zigzag) |
| [`scripts/hunter-battery.sh`](../scripts/hunter-battery.sh) | Script de batería usado en este reporte |
| [`scripts/Strategist.py`](../scripts/Strategist.py) | Versión compleja (perdedora). En el repo para referencia |
| [`data/battery/summary.csv`](../data/battery/summary.csv) | Resumen numérico de la batería |
