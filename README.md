# spider-pfe
Ceci est le repo de mes travaux sur un robot nommé Spider dans le cadre d'un projet de fin d'études. Il s'agit d'un robot mobile doté de quatre pattes, à 12 degrés de liberté.
<img width="1920" height="1080" alt="render_spider" src="https://github.com/user-attachments/assets/bc16966b-5b29-47a3-9e61-2d81bcb0060f" />

## spider_ws

spider_ws est un dossier contenant le code source pour build les noeuds ROS du robot réel. Il est possible de s'abonner aux topics générés ici via la simulation sur isaacsim et ainsi voir en temps réel sur la simulation quels ordres sont envoyés aux moteurs du robot réel. Une petite interface graphique codée en Qt5 permet aussi de voir la position réelle des moteurs du robot.


## SpiderGeneration

C'est un dossier qui contient tous les fichiers nécessaires pour générer une scène sous isaacsim du robot. Il y a les fichiers pour générer les servomoteurs (des xm430 de la gamme dynamixel) et générer le robot, avec en plus des fonctions pour le faire se déplacer de manière basique.


## SpiderDynamixelRegister

Il s'agit d'un dossier qui contient tous les fichiers nécessaires pour lancer un apprentissage du robot dans isaaclab. On y retrouve un environnement pour apprendre au robot à marcher en ligne droite, et un autre pour lui apprendre à tourner sur lui-même
