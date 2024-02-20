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
