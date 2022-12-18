# CO2-Laser-Cutter
Build (or improve) a CO2 laser

Ce projet détaille l'ensemble des étapes et pièces utilsées pour construire une découpeur/graveur laser au CO2.

Cette description est juste une illustration d'une conception de A à Z, pourvant servir d'exemple à d'autres réalisations.
Ce n'est ni un guide, ni un kit, mais les fichiers partagés peuvent bien sur être utilsés si ils sont utiles à un autre projet.

La partie contrôleur sera également décrite, mais à chacun de choisir le produit le plus adapté à son besoin.

## Les caractéristiques du produit décrit sont 
- Surface de travail : 780 * 480 mm
- réglage de Hauteur Z : 95mm
- Adaptable pour tubes de 40, 50 et 60W (au delà cela dépasse le cadre)
- Entrainement par courroies armées
- 2 moteurs Y avec arbre solidaire

## Le controleur
Cette version utilise le contrôleur suivant
- Carte Arduino Atmega 1250 + Shield Ramp1.4 + Ecran LCD
- firmware Marlin V2.1 (paramétré)
- Prise en charge d'un module Ratatif (compatible Comande A de Lightburn)
- 5 Commandes de moteur pas à pas 
  - 1 Axe X
  - 2 Axes Y
  - 1 Axe Y
  - 1 Axe A (Rotatif)  
- Compatible Lightburn

Modèle 3D crée sous Sketchup
![CO2_Laser](https://user-images.githubusercontent.com/84618082/208289881-955a83f9-7de3-4fcb-a703-7317397d6f9c.jpg)
toutes les pièces (découpe et impression 3D) ont été construites dans Sketchup

Version intermédiaure
![IMG_20221209_230923](https://user-images.githubusercontent.com/84618082/208289903-55186b29-1876-48af-91f3-a71b02f3ff6a.jpg)

Version finale
![IMG_20221211_155953](https://user-images.githubusercontent.com/84618082/208289911-7b8ddef4-6eff-49ed-9041-bee76f0724f8.jpg)

## Préparation 
https://github.com/bsfconception/CO2-Laser-Cutter/tree/main/0-preparation

## Liste des pièces détachées
https://github.com/bsfconception/CO2-Laser-Cutter/tree/main/1-BOM

## Structure générale
https://github.com/bsfconception/CO2-Laser-Cutter/blob/main/1-Frame/README.MD

## Montage de tête sur Chariot X
https://github.com/bsfconception/CO2-Laser-Cutter/tree/main/2-Head

## Axe X
https://github.com/bsfconception/CO2-Laser-Cutter/tree/main/4-XAxis

## Axe Y
https://github.com/bsfconception/CO2-Laser-Cutter/blob/main/5-YAxis/README.MD

## Axe Z
https://github.com/bsfconception/CO2-Laser-Cutter/tree/main/9-ZAxis

## Tube Laser 
https://github.com/bsfconception/CO2-Laser-Cutter/blob/main/6-tube/README.MD

## Evacuation
https://github.com/bsfconception/CO2-Laser-Cutter/tree/main/7-Exhaust

## Controle
https://github.com/bsfconception/CO2-Laser-Cutter/tree/main/8-Control

## Pièces 3D
https://github.com/bsfconception/CO2-Laser-Cutter/blob/main/10-3DFiles/README.MD


Afin de reproduire un système analogue, de bonnes connaissances mécaniques et un minimum d'outillage de qualité sont requis. Ce projet n'est pas envisageable pour un amateur "débutant".

A titre d'information, les outils et accessoires suivants ont été requis (non exhaustif)
- tournenis divers
- clefs plates
- clefs à pipe
- Niveau, mètre, outils de traçage, poinçonnage
- forets 
- perceuse à colonne
- scie à onglet avec lame aluminum
- scie à métaux
- limes et outils d'ébavurage
- Pied à coulisse
- Tarauds
