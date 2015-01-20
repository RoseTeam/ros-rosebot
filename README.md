

### installation du paquet rosebot_description
Creer un lien dans votre dossier catkin/src vers le dossier rosebot_description.
puis lancer:
catkin_make


###installation des paquets joy_interpreter et mot_rosebot
Le paquet joy_interpreter s'abonne à un topic /joy lui même généré par une manette ou un joystick (controller ps3 dans le cas présent) et publie un message twist compatible et calibré pour notre robot. 
Le paquet mot_rosbot s'abonné au topic twist et transmet les commandes à la carte driver moteur via le port série. Il remonte les infs de l'odométrie via un topic odom.

Copier les deux dossiers dans votre catkin_ws/src et lancer un catkin_make


###lecture du programme NUCLEO F401 
Créer un compte mbed sur le site mBed puis importer le .zip.
C'est un compulateur/IDE en ligne :)
