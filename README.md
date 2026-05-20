# panda_controllers — Branch MPC (Frankino NMPC)

Questo branch implementa un **Nonlinear Model Predictive Controller (NMPC)** per il robot Franka Emika Panda, sviluppato con [acados](https://github.com/acados/acados) e CasADi. Il controller ottimizza jerk di giunto su un orizzonte di 20 nodi, con vincoli di coppia, limiti cinematici e collision avoidance (capsule robot vs sfere/piano + autocollisioni).

---

## Struttura dei file sorgente

I file C++ principali si trovano in:

```
panda_controllers/src/
├── menu_acados.cpp          # Nodo MPC: menu interattivo + solver acados
├── mpc_integrator_node.cpp  # Nodo integratore: filtraggio, integrazione, watchdog
├── computed_torque.cpp      # Controller computed torque (plugin ros_control)
├── capsule_visualizer_node.cpp  # Nodo opzionale: visualizzazione capsule in RViz
└── ...
```

I file Python si trovano in:

```
panda_controllers/scripts/
├── build_frankino_solver.py # Genera il codice C del solver acados
├── DistanceFunctions.py     # Funzioni CasADi per distanze capsule-ostacolo
└── ...
```

---

## Dipendenze

### Sistema

| Dipendenza | Versione | Note |
|---|---|---|
| Ubuntu | 20.04 | Testato su Focal |
| ROS | Noetic | Con `ros-noetic-franka-ros` e `ros-noetic-franka-gazebo` |
| CMake | ≥ 3.10.2 | |
| Eigen3 | ≥ 3.3 | `sudo apt install libeigen3-dev` |
| libfranka | ≥ 0.9.2 | Vedi [guida ufficiale](https://frankaemika.github.io/docs/installation_linux.html) |
| yaml-cpp | qualsiasi | `sudo apt install libyaml-cpp-dev` |

### Python

```bash
pip install casadi numpy acados-template
```

> `acados-template` richiede che acados sia **già compilato** sul sistema e che la variabile d'ambiente `ACADOS_SOURCE_DIR` punti alla sua cartella sorgente. Segui la [guida ufficiale acados](https://docs.acados.org/installation/index.html).

> **Nota**: `DistanceFunctions.py` è un modulo **locale** del pacchetto (in `scripts/`). Non va installato con pip — viene importato automaticamente da `build_frankino_solver.py` tramite il path relativo. Assicurati che il file sia presente in `panda_controllers/scripts/DistanceFunctions.py`.

### Librerie C++ esterne (percorsi hardcoded)

Il `CMakeLists.txt` si aspetta CasADi e acados in percorsi fissi. **Prima di compilare**, apri `CMakeLists.txt` e aggiorna queste due righe con i percorsi reali sulla tua macchina:

```cmake
set(CASADI_ROOT_DIR "/percorso/alla/tua/installazione/casadi")   # riga ~20
set(ACADOS_INSTALL_DIR "/percorso/alla/tua/installazione/acados") # riga ~24
```

### File CasADi pre-generati (dinamica del robot)

Il solver richiede le funzioni simboliche pre-calcolate della dinamica del robot (M, C, G e trasformate cinematiche). Questa cartella **non è inclusa nel repository** e va generata separatamente con il pacchetto `thunder_frankino`.

Posizionala in `panda_controllers/frankino_generatedFiles/` con questa struttura:

```
frankino_generatedFiles/
├── M.casadi
├── C.casadi
├── G.casadi
├── T_0_0.casadi
├── T_0_1.casadi
├── ...
└── T_0_8.casadi
```

Aggiorna anche il percorso hardcoded in `scripts/build_frankino_solver.py`:

```python
path_to_files = "/percorso/assoluto/alla/tua/cartella/frankino_generatedFiles"  # riga ~15
```

---

## Installazione

### 1. Crea il workspace catkin e clona il repository (da rivedere insieme perchè ho paura di far generare troppe cartelle)

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone --branch students/em/mpc https://github.com/CentroEPiaggio/panda_controllers.git
```

### 2. Aggiorna i percorsi hardcoded

Prima di fare qualsiasi altra cosa, modifica i tre percorsi descritti nella sezione Dipendenze:

- `CMakeLists.txt` → `CASADI_ROOT_DIR`
- `CMakeLists.txt` → `ACADOS_INSTALL_DIR`
- `scripts/build_frankino_solver.py` → `path_to_files`

---

## Avvio della simulazione

La simulazione si avvia in **tre step sequenziali**.

### Step 1 — Genera il solver acados (Python)

Questo script formula il problema OCP con CasADi e genera il codice C del solver NMPC nella cartella `~/catkin_ws/c_generated_code_tracking/`. Va eseguito **una sola volta**, o ogni volta che si modifica la formulazione del problema.

```bash
cd ~/catkin_ws/src/panda_controllers
python3 scripts/build_frankino_solver.py
```

Output atteso:

```
Codice generato con successo in c_generated_code_tracking
Codice SIMULATORE generato con successo.
```

> Se il comando fallisce con `FileNotFoundError`, controlla il percorso `path_to_files` in `build_frankino_solver.py`.

### Step 2 — Compila il workspace catkin (C++)

Una volta generato il codice C, catkin lo compila assieme al resto del pacchetto:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

> Se `catkin_make` fallisce con errori su acados o casadi, verifica che i percorsi in `CMakeLists.txt` siano corretti e che le librerie `.so` siano presenti nelle cartelle indicate.

### Step 3 — Avvia la simulazione (ROS + Gazebo)

Servono **due terminali** (esegui `source devel/setup.bash` in entrambi).

**Terminale 1** — Avvia Gazebo, RViz, il controller e il nodo integratore:

```bash
roslaunch panda_controllers panda_controller_sim3.launch
```

Questo launch file avvia:
- **Gazebo** con il mondo `worlds/palla.world` e il Panda nella posizione iniziale predefinita
- **RViz** per la visualizzazione
- Il controller `computed_torque_controller`
- Il nodo `mpc_integrator_node` (vedi sezione Architettura)

Attendi che Gazebo sia completamente avviato prima di procedere.

**Terminale 2** — Avvia il nodo MPC con il menu interattivo:

```bash
roslaunch panda_controllers menu_acados.launch
```

All'avvio apparirà un menu testuale. Seleziona l'opzione **6** o **7** per avviare il controller NMPC con acados.

---

## Architettura dei nodi a regime (da rivedere se effettivamente comprensibile)

```
Gazebo
  ├─ /franka/joint_states_1khz  (1 kHz)
  │             │
  │             ▼
  │   [mpc_integrator_node]
  │     - Filtra q, dq con IIR passa-basso
  │     - Stima ddq per derivata numerica
  │     - Integra triple integrator a 1 kHz
  │     - Filtra posizione/velocità ostacoli da Gazebo
  │             │
  │             ├──► /mpc/filtered_joint_state  (1 kHz)
  │             │              │
  │             │              ▼
  │             │       [menu_acados]
  │             │         - Risolve NMPC (~20-50 Hz, dipende dal solve time)
  │             │         - Orizzonte N=20, jerk ottimo
  │             │              │
  │             │              └──► /mpc_solution
  │             │                        │
  │             │◄────────────────────────┘
  │             │    (reset stato con feedback reale)
  │             │
  │             ├──► /computed_torque_controller/command  (1 kHz)
  │             │                   │
  │             │                   ▼
  │             │                Gazebo
  │             │
  │             └──► /mpc/obstacle_status  (callback-driven)
  │
  └─ /gazebo/model_states  →  [mpc_integrator_node]  (posizione palla)
```

| Topic | Tipo | Frequenza | Descrizione |
|---|---|---|---|
| `/franka/joint_states_1khz` | `sensor_msgs/JointState` | 1 kHz | Stato grezzo da Gazebo/franka_ros |
| `/mpc/filtered_joint_state` | `sensor_msgs/JointState` | 1 kHz | Stato filtrato (q, dq, ddq stimata) verso MPC |
| `/mpc_solution` | `panda_controllers/MpcSolution` | ~20–50 Hz | Soluzione ottima: q_start, dq_start, ddq_start, jerk |
| `/computed_torque_controller/command` | `sensor_msgs/JointState` | 1 kHz | Traiettoria integrata al controller |
| `/mpc/obstacle_status` | `panda_controllers/ObstacleStatus` | callback-driven | Posizione e velocità filtrate dell'ostacolo (palla) |
| `/gazebo/set_model_state` | `gazebo_msgs/ModelState` | on demand | Posizionamento ostacoli in Gazebo | (caso di movimento autonomo dell'ostacolo)
| `/gazebo/model_states` | `gazebo_msgs/ModelStates` | Gazebo rate | Stato modelli Gazebo letto da `mpc_integrator_node` |

### Logica del nodo `mpc_integrator_node`

Il nodo svolge quattro ruoli in parallelo:

**Filtraggio stato robot (1 kHz):** legge lo stato grezzo da `/franka/joint_states_1khz`, applica un filtro IIR passa-basso su q e dq, e stima ddq per derivata numerica con clamp ai limiti fisici Franka. Pubblica immediatamente il risultato su `/mpc/filtered_joint_state`.

**Filtraggio ostacoli (callback-driven):** legge la posizione della palla da `/gazebo/model_states`, stima la velocità per derivata numerica e applica un filtro IIR separato su posizione e velocità. Pubblica il risultato su `/mpc/obstacle_status`.

**Integrazione (1 kHz):** non appena riceve una soluzione MPC tramite `/mpc_solution`, resetta lo stato integrato con il feedback reale filtrato e integra il jerk ottimo come triple integrator (jerk → ddq → dq → q) ad ogni ciclo di controllo. Pubblica il comando su `/computed_torque_controller/command`.

**Watchdog:** se non arriva una nuova soluzione MPC entro **70 ms**, l'integratore si arresta per sicurezza.

### Logica del nodo `menu_acados`

Il nodo gira in un loop a 20-50Hz e ad ogni iterazione:

1. Legge lo stato filtrato da `/mpc/filtered_joint_state`
2. Aggiorna i parametri degli ostacoli (posizione predetta per ogni nodo dell'orizzonte)
3. Imposta condizioni iniziali, riferimenti MinJerk e warm start nel solver acados
4. Chiama `frankino_tracking_mpc_acados_solve()`
5. Estrae il jerk ottimo al nodo 0 e pubblica su `/mpc_solution`
6. Integra `ddq0` con Eulero esplicito per mantenere una stima dell'accelerazione tra un solve e il successivo

La frequenza effettiva del loop dipende dal tempo di solve acados (tipicamente 5–20 ms su hardware moderno) e dal `loop_rate.sleep()` configurato a 1000 Hz: in pratica il collo di bottiglia è il solve stesso.

---

## Parametri chiave del solver

| Parametro | Valore |
|---|---|
| Orizzonte N | 20 nodi |
| Frequenza MPC effettiva | ~20–50 Hz (dipende dal solve time) |
| Loop di controllo | 1000 Hz |
| Watchdog MPC | 70 ms |
| Stato x | `[q, dq, ddq]` — 21 variabili |
| Ingresso u | jerk — 7 variabili |
| Solver QP | `FULL_CONDENSING_HPIPM` |
| Tipo NLP | `SQP_RTI` |
| Sfere ostacolo | 2 |
| Capsule robot | 10 |
| Piani | 1 |
| Coppie autocollisione attive in C++ | 6 (`N_autocollisioni = 6`) |
| Coppie autocollisione definite in Python | 6 (whitelist, alcune commentate) |

> **Nota sulle autocollisioni**: la whitelist in `build_frankino_solver.py` definisce 6 coppie candidate `(1,7)`, `(1,8)`, `(2,7)`, `(2,8)`, `(3,7)`, `(3,8)`. Il parametro `N_autocollisioni = 6` in `menu_acados.cpp` deve essere coerente con il numero di coppie effettivamente abilitate nel builder Python al momento della generazione del codice C.

---

## Problemi comuni

| Errore | Causa | Soluzione |
|---|---|---|
| `FileNotFoundError: Path non trovato` | `path_to_files` hardcoded errato | Modifica riga ~15 di `build_frankino_solver.py` |
| `ModuleNotFoundError: DistanceFunctions` | File non in `scripts/` o path Python non configurato | Verifica che `DistanceFunctions.py` sia in `panda_controllers/scripts/` |
| `FATAL_ERROR: libcasadi.so non trovata` | `CASADI_ROOT_DIR` errato nel CMakeLists | Aggiorna il percorso alla tua installazione CasADi |
| Errore di link su `acados` / `hpipm` / `blasfeo` | `ACADOS_INSTALL_DIR` errato nel CMakeLists | Aggiorna il percorso alla tua installazione acados |
| `c_generated_code_tracking` non trovata dal CMake | Step 1 non eseguito | Esegui prima `build_frankino_solver.py` |
| Il nodo MPC non riceve dati sullo stato | `mpc_integrator_node` non attivo o topic errato | Verifica che `panda_controller_sim3.launch` sia in esecuzione e che Gazebo pubblichi su `/franka/joint_states_1khz` |
| `acados status: 1` a runtime | Problema di convergenza del solver | Controlla le condizioni iniziali e i parametri degli ostacoli |
| MPC si arresta dopo ~70 ms | Watchdog scattato: nessuna soluzione ricevuta in tempo | Verifica che `menu_acados` stia pubblicando su `/mpc_solution` |
| `menu.yaml` non trovato | File di config mancante | Verifica che esista `config/menu.yaml` nel pacchetto |
| `/mpc/obstacle_status` non pubblicato | `mpc_integrator_node` non riceve da `/gazebo/model_states` | Verifica che Gazebo sia in esecuzione e che il modello si chiami esattamente `palla` |
| `N_autocollisioni` non coerente con il solver | Mismatch tra Python builder e C++ | Rigenera il codice acados dopo aver modificato la whitelist in `build_frankino_solver.py` |

---

## Autore

**Eligio Mansi** — Tesi di Laurea Magistrale, [2025/2026]  
Università di Pisa — Centro E.Piaggio  
Relatore: Prof. Paolo Salaris