# ROS 2 Signal Processing - CH1

Este proyecto implementa un sistema de procesamiento de señales en **ROS 2**, compuesto por dos nodos principales:
- **`signal_generator.py`**: Genera una onda senoidal y publica los valores en ROS 2.
- **`process.py`**: Recibe la onda senoidal, le aplica transformaciones (desfase, ajuste de amplitud y desplazamiento), y publica la señal procesada.

## 📌 Requisitos
Este proyecto está desarrollado en **Ubuntu 22.04** con **ROS 2 Humble**. Antes de ejecutarlo, asegúrate de tener instalado:
- **Python 3**
- **ROS 2 Humble**
- **PlotJuggler (para visualizar señales)**

Instala `PlotJuggler` si aún no lo tienes:
```bash
sudo apt install ros-humble-plotjuggler-ros
