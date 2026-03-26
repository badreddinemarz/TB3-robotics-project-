# TB3 Robotics Project 🤖

> Système de navigation autonome pour TurtleBot3 sous ROS 2 Jazzy
link demo : https://www.youtube.com/watch?v=LTjpCx3EnGE

---

## 📋 Vue d'ensemble

Développement d'un système complet de navigation autonome pour TurtleBot3 basé sur **ROS 2** et **C++**, couvrant l'ensemble de la chaîne :

```
Perception (LiDAR) → SLAM → Cartographie → Planification (Nav2) → Contrôle (cmd_vel)
```

| Composant | Statut |
|---|---|
| 🟢 Simulation Gazebo (world SDF) | Opérationnel |
| 🟢 Intégration TurtleBot3 (URDF) | Opérationnel |
| 🟢 Bridge Gazebo ↔ ROS 2 | Opérationnel |
| 🟢 Nodes C++ (navigation + évitement) | Opérationnel |
| 🟢 SLAM Toolbox | Opérationnel |
| 🟡 Nav2 (planification globale) | En cours |

---

## 🌐 Présentation du projet

👉 **[Voir la page de présentation complète](https://badreddinemarz.github.io/TB3-robotics-project-/)**

---

## 🛠️ Environnement technique

| Outil | Version |
|---|---|
| OS | Ubuntu 24.04 |
| Framework | ROS 2 Jazzy |
| Simulateur | Gazebo Harmonic |
| Langage | C++ |

### Installation des dépendances

```bash
# SLAM
sudo apt install ros-jazzy-slam-toolbox

# Navigation
sudo apt install ros-jazzy-navigation2

# TurtleBot3
sudo apt install ros-jazzy-turtlebot3*
```

---

## 🏗️ Architecture du projet

```
TB3-robotics-project/
├── src/                        # Nodes C++
│   ├── navigation_node.cpp     # Navigation vers positions prédéfinies
│   └── obstacle_avoidance.cpp  # Évitement réactif LiDAR
├── launch/                     # Launch files ROS 2
│   ├── spawn_warehouse.launch.py
│   └── single_robot.launch.py
├── worlds/                     # Environnement de simulation
│   └── world.sdf
├── config/                     # Paramètres Nav2 / SLAM
│   └── nav2_params.yaml
└── index.html                  # Page de présentation
```

---

## 🚀 Étapes de développement

### 1️⃣ Environnement de simulation

Création d'un fichier `world.sdf` personnalisé avec obstacles dynamiques, lancé via :

```bash
ros2 launch tb3_project spawn_warehouse.launch.py
```

### 2️⃣ Intégration du robot

Chargement du TurtleBot3 avec description URDF complète :

```bash
ros2 launch tb3_project single_robot.launch.py
```

### 🔁 Bridge Gazebo ↔ ROS 2

Communication bidirectionnelle via les topics :

| Topic | Type | Rôle |
|---|---|---|
| `/cmd_vel` | geometry_msgs/Twist | Commande du robot |
| `/odom` | nav_msgs/Odometry | Localisation |
| `/scan` | sensor_msgs/LaserScan | Données LiDAR |

> **Problème rencontré** : Le topic `/scan` ne publiait aucune donnée.  
> **Cause** : Absence des plugins LiDAR dans `world.sdf`.  
> **Solution** : Ajout des plugins Gazebo nécessaires → chaîne de perception finalisée.

### 4️⃣ Nodes C++

**Navigation basique**
- Exploitation de `/odom`
- Commande via `/cmd_vel`
- Navigation vers des waypoints prédéfinis

**Évitement d'obstacles**
- Exploitation de `/scan`
- Logique d'évitement réactif basée sur les données LiDAR

### 5️⃣ Intégration SLAM & Nav2

- **SLAM Toolbox** : cartographie en temps réel ✅ opérationnel
- **Nav2** : planification globale et locale 🔄 en cours

---

## 💡 Points techniques notables

- Architecture **distribuée et modulaire** ROS 2
- Intégration et paramétrage de capteurs **LiDAR**
- Développement de **nodes C++** communicants via topics
- Débogage multi-composants (Gazebo + ROS 2 + capteurs)
- Conception d'un **pipeline complet** de navigation autonome

---

## 🧠 Compétences mobilisées

`ROS 2` `Gazebo` `SDF` `URDF` `C++` `SLAM Toolbox` `Nav2` `LiDAR` `Debugging` `Architecture modulaire`

---

## 👤 Auteur

**badreddinemarz** — Projet réalisé en autonomie complète
