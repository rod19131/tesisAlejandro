
# Algoritmo de sincronización y control de formaciones

El algoritmo se desarrolló por completo dentro del entorno de desarrollo de robots, Webots, para un funcionamiento híbrido en la vida real (Robotat). Se trabajó con una cantidad de agentes de 1 a 9.

Las características principales del algoritmo son las siguientes:
- Acercamiento de agentes entre sí
- Construcción de formación 
- Evasión de obstáculos
- Movimiento de un agente líder hacia un objetivo, arrastrando a los demás agentes, manteniendo formación
- Naturaleza dinámica

Además de las características previamente mencionadas, se tiene una etapa previa de posicionamiento inicial automático de agentes en un espacio determinado, para facilitar los experimentos.

El algoritmo fue diseñado para funcionar en un entorno controlado, específicamente el ecosistema Robotat con sistema de captura de movimiento OptiTrack, y comunicación TCP con un servidor para obtener información de posicionamiento de agentes y envío de instrucciones a los mismos. Los agentes utilizados son robots diferenciales Pololu 3Pi+ modificados para funcionar con un ESP32 que se encarga de la recepción inalámbrica de información.

El funcionamiento básico del algoritmo implementado en físico se resume en lo siguiente:

- En Webots se tiene dos programas controladores principales.
    - Supervisor: uno que sirve como la unidad de procesamiento centralizada del algoritmo, que calcula las trayectorias en base a las posiciones actuales.
    - Controlador de agente (pruebaMatrizDifeoMorfismo): Básicamente consistente en traducir las velocidades para el modelo del uniclo y enviarlo a los agentes.

## Resumen de carpetas

| Carpeta             | Contenido                                                                |
| ----------------- | ------------------------------------------------------------------ |
| Webots_integracion_fisica | Contiene el mundo de Webots basado en el ecosistema Robotat de la UVG, así como las variantes de programas de Supervisor del algoritmo |
| comunicacion_pololu | Contiene todos los programas de python para comunicarse con el servidor, obtener información de la pose de marcadores de interés y enviar instrucciones al Pololu 3Pi+ |
| antecedentes | Consiste en dos subcarpetas, una con las carpetas de la fase anterior del desarrollo del algoritmo por Andrea Maybell Peña, y otra carpeta con la restauración/migración del algoritmo y mundo de Webots de 2019 hacia 2023 |




