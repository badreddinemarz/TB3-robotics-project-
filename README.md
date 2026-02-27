TB3 Robotics Project

 Système de navigation autonome pour TurtleBot3 sous ROS 2

Développement d’un système complet de navigation autonome pour TurtleBot3 basé sur ROS 2 et C++, intégrant :

Un environnement de simulation personnalisé (SDF) avec obstacles dynamiques

Une architecture prête pour la cartographie en temps réel via SLAM Toolbox

La planification de trajectoire et l’évitement d’obstacles avec Nav2

Une chaîne complète : Perception → Localisation → Planification → Contrôle

1) Description du projet

Ce projet a pour objectif de concevoir et intégrer une architecture complète de navigation autonome pour un robot mobile simulé sous Gazebo.

Objectifs

Concevoir un environnement de simulation réaliste au format SDF

Intégrer un robot TurtleBot3 avec description URDF complète

Mettre en place la communication bidirectionnelle entre Gazebo et ROS 2

Exploiter les données LiDAR pour la perception

Préparer l’intégration de SLAM pour la cartographie en ligne

Configurer Nav2 pour une navigation robuste

Le projet met en avant :

L’architecture distribuée et modulaire de ROS 2

L’intégration et le paramétrage de capteurs LiDAR

Le développement de nodes C++

La conception d’un pipeline complet de navigation autonome

2) Environnement technique
Système

Ubuntu 24.04

ROS 2 Jazzy

Gazebo Harmonic

Dépendances
sudo apt install ros-jazzy-slam-toolbox
sudo apt install ros-jazzy-navigation2
sudo apt install ros-jazzy-turtlebot3*

3) Architecture et étapes clés
1️⃣ Conception de l’environnement de simulation

Création d’un fichier world.sdf personnalisé intégrant des obstacles dynamiques

Intégration via le launch file : spawn_warehouse.launch.py

2️⃣ Intégration du robot

Utilisation du modèle SDF du TurtleBot3 depuis le package turtlebot3_gazebo

Chargement de la description URDF

Lancement via : single_robot.launch.py

À ce stade, le robot est correctement simulé dans Gazebo.

🔁 Bridge Gazebo ↔ ROS 2

Mise en place de la communication entre la simulation et ROS 2 via les topics :

/cmd_vel

/odom

/scan

Difficulté rencontrée : LiDAR non fonctionnel

Le topic /scan ne publiait pas de données.

Cause identifiée :
Absence des plugins nécessaires dans le fichier world.sdf.

Solution :
Ajout des plugins Gazebo permettant l’activation du capteur LiDAR.

Cette étape a permis de finaliser la chaîne de perception.

4) Développement des nodes C++
Navigation basique

Exploitation des données /odom

Commande du robot via /cmd_vel

Navigation vers des positions prédéfinies

Évitement d’obstacles

Exploitation des données /scan

Implémentation d’une logique d’évitement réactif

Bien que fonctionnelle, cette approche restait limitée (pas de planification globale).

5) Intégration SLAM & Nav2

La prochaine étape consistait à intégrer :

SLAM Toolbox pour la cartographie en temps réel

Nav2 pour la planification globale et locale

⚠️ État d’avancement actuel

L’architecture a été préparée pour intégrer SLAM et Nav2.
Cependant, l’intégration complète de SLAM n’a pas encore été finalisée et validée en simulation.

Le projet s’est arrêté à cette étape :

SLAM configuré mais non encore pleinement opérationnel

Nav2 en cours de préparation dépendant de la validation du SLAM

 Pipeline actuel

Perception (LiDAR)
→ Localisation (Odometry)
→ Contrôle bas niveau (cmd_vel)

Pipeline cible (en cours d’intégration) :

Perception
→ SLAM
→ Cartographie
→ Planification (Nav2)
→ Contrôle

 Compétences mobilisées:

ROS 2 (nodes, topics, launch files, architecture modulaire)

Gazebo & modélisation SDF

URDF

Intégration LiDAR

Développement C++

Debugging multi-composants

Gestion autonome d’un projet technique complexe

