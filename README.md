# Conception d'un Système Intelligent de Régulation de Vitesse (V2V & ROS2)

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue) 
![Python](https://img.shields.io/badge/Python-3.10%20%7C%203.12-blue) 
![Platform](https://img.shields.io/badge/Platform-Yocto%20Linux%20%7C%20Ubuntu-orange)
![License](https://img.shields.io/badge/License-MIT-green)

Ce projet, réalisé à l'**ENSA Fès** (Filière ISEIA), porte sur le développement d'une architecture de régulation de vitesse intelligente pour véhicules connectés. Il intègre une simulation 2D/3D (CARLA), un asservissement PID robuste et une communication temps réel avec un robot physique basé sur Raspberry Pi 4.

---

## 🚀 Fonctionnalités Clés
* **Caractérisation Dynamique** : Mesure expérimentale de l'accélération maximale ($a_{max} = 0,047 \text{ m/s}^2$) sur robot réel.
* **Asservissement PID** : Régulateur de vitesse discret avec gestion des saturations physiques et annulation de l'erreur statique.
* **Architecture Distribuée** : Communication TCP/IP asynchrone entre un client embarqué (Yocto) et un serveur ROS2.
* **Jumeau Numérique** : Synchronisation temps réel entre le robot physique et sa représentation dans RViz2.
* **Monitoring Avancé** : Visualisation des performances dynamiques via `rqt_plot`.

---

## 🛠 Stack Technique
* **Middleware** : ROS2 Humble Hawksbill.
* **Langage** : Python 3.10 / 3.12 (NumPy, Matplotlib).
* **OS Embarqué** : Yocto/Poky 5.0.15 (Scarthgap).
* **Gestion Matérielle** : `gpiod` (contrôle GPIO bas niveau).
* **Simulation** : CARLA Simulator (3D) et environnement 2D personnalisé.

---

## 🔌 Configuration Hardware
Le système repose sur une architecture maître/esclave communicant via WiFi :
* **Unité de calcul** : Raspberry Pi 4.
* **Puissance** : Driver L298N + Batterie Li-ion 7.4V dédiée.
* **Actionneur** : Moteur DC avec motoréducteur (1:48).
* **Capteur** : Encodeur infrarouge (20 trous/tour).

---

## 📂 Structure du Projet
```text
├── simulation_2d_ros/       # Package ROS2 principal
│   ├── simulation_node.py   # Calcul du PID et odométrie de simulation
│   ├── bridge_ros.py        # Serveur TCP et passerelle ROS2
│   ├── setup.py             # Configuration du build colcon
│   └── package.xml          # Dépendances du package
├── etape1_rpi/              # Scripts embarqués (Raspberry Pi)
│   ├── test_moteur_gpiod.py # Test de l'actionneur
│   ├── test_vitesse_amax.py # Script de calibration et calcul a_max
│   └── robot_client.py      # Client de télémétrie TCP vers ROS2
└── simulation_2d_standalone/# Prototypes de simulation hors ROS
