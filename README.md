# KUKA Med LBR 7 - ROS 1 Bridge (Docker Setup)

Questo repository contiene l'infrastruttura Docker plug-and-play per comunicare con il KUKA Med LBR 7 utilizzando il ROS 1 Bridge verso ROS 2.

## 🛠 Fase 1: Compilazione (Build)
*Da eseguire solo la prima volta o se si modificano i file sorgenti.*

Apri il terminale nella cartella del progetto ed esegui:
```bash
docker build --network=host -t kuka_ros1_bridge .
```
🚀 Fase 2: Avvio Standard

Da eseguire sempre come punto di partenza.

Terminale 1 (Host):
```Bash

docker run -it --network=host kuka_ros1_bridge
```
(Il server si avvierà e resterà in attesa sulla porta 5005. Lascia questo terminale aperto).
🤖 Fase 3A: Esecuzione CON il Robot Reale (Laboratorio)

    Avvia l'applicazione Java (Sunrise) sul pad del KUKA.

    Inserisci l'IP di questo PC e la porta 5005.

    Verifica sul Terminale 1 che il robot sia connesso.

Terminale 2 (Opzionale - Per usare il telecomando demo):
```Bash

docker exec -it $(docker ps -q -l) bash
source /ros1_ws/devel/setup.bash
rosrun kuka_ros_node demo_node
```
💻 Fase 3B: Simulazione (SENZA Robot Reale)

Per testare l'infrastruttura software senza il braccio robotico.

Terminale 2 (Avvio del Demo Node):
```Bash

docker exec -it $(docker ps -q -l) bash
source /ros1_ws/devel/setup.bash
rosrun kuka_ros_node demo_node
```
(Il nodo rimarrà in attesa. Passa al Terminale 3 per sbloccarlo).

Terminale 3 (Sblocco simulato):
```Bash

docker exec -it $(docker ps -q -l) bash
source /opt/ros/noetic/setup.bash
source /ros1_ws/devel/setup.bash
rostopic pub -1 /robot/state/joint_states sensor_msgs/JointState "{name: ['lbr_joint_0', 'lbr_joint_1', 'lbr_joint_2', 'lbr_joint_3', 'lbr_joint_4', 'lbr_joint_5', 'lbr_joint_6'], position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```
(Ora il menu interattivo nel Terminale 2 sarà sbloccato).
📡 Fase 4: Lettura Dati su ROS 2 (Host PC)

Comandi da eseguire sul PC nativo (fuori da Docker) per verificare la comunicazione.

Verificare i topic disponibili:
```Bash

ros2 topic list
```
Ascoltare i comandi in uscita (es. verso il robot):
```Bash

ros2 topic echo /robot/cmd/joint_ptp
```
Leggere lo stato in ingresso (dal robot):
```Bash

ros2 topic echo /robot/state/joint_states
```
