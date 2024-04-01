# Controladores

La carpeta `pruebaMatrizDifeomorfismo` contiene el controlador individual que se encarga de procesar las velocidades generadas por el algoritmo y mediante el modelo del uniciclo transformar las velocidades lineales en radiales para cada llanta del agente. En Webots es el controlador (`pruebaMatrizDifeomorfismo.py`) que se selecciona para cada entidad de Robot e-puck.


El controlador Supervisor se encarga de ser el programa de control centralizado del algoritmo de inteligencia de enjambre. Este procesa toda la información, asignar velocidades a cada agente según sus posiciones, posiciones de los obstáculos y del objetivo, así como en la etapa del algoritmo en la que se encuentre. Este controlador se selecciona en un Robot arbitrario que no sea un agente, y este se encarga de correr el algoritmo. Se tienen diversas carpetas con nombre Supervisor, y corresponden a versiones anteriores del Supervisor que se generaron a lo largo del desarrollo. 

La carpeta `Supervisor_simulacion_y_fisico_v4` corresponde a la versión final del controlador con la que se realizaron las corridas finales y se generaron los datos estadísticos de la tésis.
Además esta versión genera figuras (PNG y eps), gif (animación de la corrida), una tabla comparativa entre simulación y físico, un excel con datos estadísticos, todo en una carpeta generada correspondiente a la corrida y su respectivo código en latex, con nombres y títulos generados automáticamente según el nombre del archivo.

La carpeta `Supervisor_simulacion_y_fisico_demo` contiene el mismo controlador y funciones que `Supervisor_simulacion_y_fisico_v4`, pero esta contiene solo los archivos necesarios para correr el algoritmo siendo estos `calibracion_markers_inicial.npy`, `funciones.py`, `funciones_conjunto.py`, `funVel.py` y `Supervisor_simulacion_y_fisico_demo.py`. Además se contienen 3 archivos `.npz` con los datos de algunas corridas finales, como ejemplo de nombramiento y para poder probar la comparación con corridas ya existentes. También se tienen dos carpetas dentro, `backup` y `finaltrials`, que contienen los datos y figuras respectivamente de las corridas finales que se realizaron para la parte estadística de la tésis. El propósito de `Supervisor_simulacion_y_fisico_demo` es servir como una demo.
## Carpetas

| Carpeta             | Contenido                                                                |
| ----------------- | ------------------------------------------------------------------ |
| outdated | Supervisores obsoletos y datos generados por los mismos |
| previous | Versiones previas del Supervisor_simulacion_y_fisico |
| pruebaMatrizDifeomorfismo | Controlador de agente individual para transformación de velocidades para llantas |
| Supervisor_simulacion_y_fisico_v3 | Versión 3 del Supervisor_simulacion_y_fisico |
| Supervisor_simulacion_y_fisico_v4 | Versión final del algoritmo |
| Supervisor_simulacion_y_fisico_demo | Es igual al v4, solo que que una carpeta más limpia y ordenada, con propósito de demonstración |

Los Supervisores `Supervisor_simulacion_y_fisico_v4` y `Supervisor_simulacion_y_fisico_demo` guardan en un archivo `.npz` las variables relevantes de la corrida como tal. A continuación se muestra una tabla con las variables guardadas que contiene este archivo `.npz` y la explicación de qué representa. (Es una versión más actualizada que contiene más variables que las que se muestran en el documento de tésis.)

## Variables

| Variable Name           | Comment                                                               |
|-------------------------|-----------------------------------------------------------------------|
| `trajectory_data`       | Agents positions register of the run.                                 |
| `velocity_data`         | Agents velocities register of the run.                                |
| `normV_data`            | Agents velocities norm register of the run.                           |
| `obj_data`              | Objective positions register of the run.                              |
| `obs_data`              | Obstacle positions register of the run.                               |
| `formation_mse_data`    | MSE (Mean Squared Error) register of the run for formation.           |
| `rot_data`              | Rotation data register of the run.                                     |
| `total_cycle`           | Total cycles of the run.                                              |
| `form_cycle`            | Cycle when formation construction began.                              |
| `obj_cycle`             | Cycle when the leader began following the objective.                  |
| `quantO`                | Total quantity of obstacles.                                          |
| `posObsAct`             | Last position of obstacles when the run ended.                        |
| `sizeO`                 | Size of the obstacles.                                                |
| `NStart`                | Lower limit of the interval of agents.                                |
| `N`                     | Higher limit of the interval of agents.                               |
| `NMax`                  | Maximum agent number that the formation shape can contain.            |
| `pObjVec`               | Last position of objective when the run ended.                        |
| `PosRealAgents`         | Position of the agents at the start of stage 1 (initial conditions).  |
| `RotRealAgents`         | Rotation of the agents at the start of stage 1 (initial conditions).  |
| `begin_alg_time`        | Cycle when the algorithm itself started (start of stage 1).           |
| `posIniPosVec`          | Position of initial position marks.                                   |
| `fisico`                | Physical (Robotat) or virtual (Webots) world run.                     |
| `r_initial_conditions`  | Real initial conditions active or not.                                |
| `r_obs`                 | Real or virtual obstacles.                                            |
| `r_obj`                 | Real or virtual objective.                                            |
| `TIME_STEP`             | Time step of the program.                                             |
| `agent_setup`           | Agent setup used for the run.                                         |
| `obs_active`            | Obstacles active or not.                                              |
| `initial_pos_setup`     | Initial position setup (random or not).                               |
| `r`                     | Radius to consider to avoid collisions.                               |
| `R`                     | Radar radius.                                                         |
| `MAX_SPEED`             | Maximum speed of agents' wheels allowed.                              |
| `form_shape`            | Shape of the formation.                                               |
| `rigidity_level`        | Rigidity level of the formation.                                      |
| `total_agent_number`    | Total number of agents involved in the run.                           |
| `obj_marker`            | Robotat marker used for the objective in real life.                   |
| `obs_start_marker`      | Robotat starting marker used for the obstacles in real life.          |
| `setup_starting_point`  | Beginning of the initial positions shape.                             |
| `setup_shape`           | Initial positions shape.                                              |
| `setup_shape_space`     | Space covered with the initial positions.                             |
| `formation_edge`        | Length of the edge of the formation.                                  |
| `r_f`                   | Robot dimensions for unicycle model (radius).                         |
| `l_f`                   | Robot dimensions for unicycle model (length).                         |
| `a_f`                   | Robot dimensions for unicycle model (additional parameter).          |
| `obj_success`           | Indicates if the objective was successful.                            |
| `obj_success_cycle`     | Objective success cycle.                                              |








`La comida sabe a comida.`