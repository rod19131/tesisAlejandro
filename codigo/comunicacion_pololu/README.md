
# Comunicación con el servidor del Robotat

Esta carpeta contiene los programas necesarios para comunicarse con el servidor del Robotat, adquirir la información de pose de los marcadores del OptiTrack, y enviar las instrucciones a los agentes Pololu 3Pi+.


| Carpeta             | Contenido                                                                |
| ----------------- | ------------------------------------------------------------------ |
| esp32_agent_program | Código que se carga al ESP32 montado en el Pololu 3Pi+, para recepción de instrucciones |
| markers_calibration | El archivo con los desfases a considerar para la calibración y corrección del ángulo de los marcadores montados en los agentes. Contiene los desfases del marcador 1 al 15 |
| miscellaneous_data | Datos recabados con los marcadores en pruebas iniciales |
| outdated_functions | Versiones desactualizadas de los programas, que ya no funcionan |
| robotat_3pi_matlab | Funciones previamente desarrolladas por catedráticos y estudiantes de la UVG, para comunicarse con el servidor y agentes Pololu, en Matlab |
| robotat_3pi_python | Funciones desarrolladas, migradas y adaptadas para su funcionamiento en Python |
| first_setup | Archivo de prueba con posiciones guardadas de los marcadores de la vida real para posicionar agentes, obstáculos y objetivo en el mundo de Webots |

