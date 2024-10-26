# ControlMapaROS

Aplicación Android que se comunica con un vehículo controlado por una Raspberry Pi utilizando WebSocket para recibir datos de sensores y tomar decisiones de control. La comunicación se realiza a través de ROS (Robot Operating System), y es necesario que tanto la Raspberry Pi como el dispositivo Android estén en la misma red.

## Requisitos

- **Dispositivo Android**: con la aplicación instalada y configurada con la dirección IP de la Raspberry Pi.
- **Raspberry Pi**: con ROS y los siguientes paquetes instalados:
  - `rosbridge_server` para la comunicación WebSocket
  - `rosserial_python` para la comunicación serial
- **Conexión Wi-Fi**: Ambos dispositivos deben estar conectados a la misma red.

## Configuración de la Raspberry Pi

1. **Verificar la dirección IP de la Raspberry Pi**: Conecta la Raspberry Pi a la red y ejecuta el siguiente comando para verificar la dirección IP asignada:

    ```bash
    ifconfig
    ```

2. **Asignar la dirección IP en la aplicación Android**: Una vez que obtengas la IP, reemplaza la dirección en el archivo `MainActivity` de la aplicación con la IP actual de la Raspberry Pi para que el WebSocket pueda conectarse correctamente.

3. **Conexión SSH a la Raspberry Pi**: Conéctate a la Raspberry Pi mediante SSH con el siguiente comando:

    ```bash
    ssh ubuntu@<RPI_IP>
    ```

   - **Usuario**: `ubuntu`
   - **Contraseña**: `ubuntu`

4. **Configurar los procesos en la Raspberry Pi**:

   Una vez conectado, abre tres terminales y ejecuta los siguientes comandos en cada una:

   - **Terminal 1**: Inicia el núcleo de ROS:

     ```bash
     roscore
     ```

     Si has ejecutado otro proceso de `roscore` anteriormente, puedes finalizarlo con:

     ```bash
     pkill -f roscore
     ```

   - **Terminal 2**: Inicia el nodo de ROS para la comunicación serial con el microcontrolador:

     ```bash
     rosrun rosserial_python serial_node.py /dev/ttyACM0
     ```

     Verifica el puerto asignado ejecutando:

     ```bash
     ls /dev/ttyACM*
     ```

     Si el puerto es correcto, deberías ver mensajes de publicación y suscripción en la terminal.

   - **Terminal 3**: Inicia el servidor WebSocket de ROS para que pueda recibir conexiones desde el dispositivo Android:

     ```bash
     roslaunch rosbridge_server rosbridge_websocket.launch
     ```

     Si aparece un error indicando que el puerto `9090` está ocupado, ejecuta:

     ```bash
     sudo lsof -i :9090
     ```

     Esto te mostrará el PID del proceso en uso. Luego, termina el proceso ejecutando:

     ```bash
     sudo kill <PID>
     ```

     Una vez iniciado, el servidor WebSocket esperará conexiones de clientes y notificará cuando uno se haya conectado y comience a recibir datos de suscriptores.
## Uso de la Aplicación

1. **Ejecutar la Aplicación Android**: Inicia la aplicación en tu dispositivo Android. La aplicación intentará conectarse al servidor WebSocket en la dirección IP de la Raspberry Pi configurada en `MainActivity`.

2. **Visualización de Datos**: La aplicación mostrará en pantalla los datos de los sensores del vehículo (distancia, sensor central, derecho, izquierdo y decisión) en tiempo real.

3. **Control del Vehículo**: La aplicación permite tomar decisiones de control en función de los datos recibidos del vehículo, incluyendo opciones de modo **Automático** y **Manual**. En el modo Automático, el vehículo sigue una secuencia de acciones pre-configuradas. En el modo Manual, el usuario detiene el vehículo manualmente.

4. **Trazado de la Trayectoria del Vehículo**: La aplicación muestra el recorrido del vehículo en tiempo real sobre un lienzo (`Canvas`). Cada cambio de dirección, ya sea desde los botones de control o de las decisiones automáticas, se representa en el lienzo, permitiendo visualizar el camino exacto seguido por el vehículo. La trayectoria se ajusta dinámicamente, expandiendo el lienzo según sea necesario para mantener todo el trayecto visible sin perder detalles de movimientos previos.
   
   - **Control de Dirección con Botones**: Los botones en la interfaz permiten al usuario dirigir el vehículo, y el trazado responde a cada cambio de dirección en tiempo real.
   - **Actualización en Base a Decisiones**: En el modo Automático, las decisiones del vehículo se trazan en el lienzo, mostrando el camino completo seguido. 

   La funcionalidad asegura que el trazado continúe fluidamente, sin reiniciarse ni interrumpirse, proporcionando una visualización continua y detallada de la trayectoria en el lienzo.

