# Projet Industriel — Système de Détection de Collision pour Essaim Autonome (UAV)

**École des Sciences de l'Information — Programmation Avancée en C**  
**Professeur : Tarik HOUICHIME**

---

## Description

Ce projet implémente un système de détection de collision en temps réel pour un essaim de **10 000 micro-drones autonomes**.  
L'algorithme identifie instantanément les deux drones les plus proches l'un de l'autre dans l'espace 3D afin de déclencher une manœuvre d'évitement.

---

## Architecture Algorithmique

- **Algorithme** : Tri par projection axiale + Balayage linéaire (Sweep Line)
- **Complexité** : O(n log n)
- **Contrainte** : Aucune indexation par crochets — arithmétique pure des pointeurs

---

## Livrables

| Livrable | Description | Lien |
|----------|-------------|------|
| Livrable 1 | Code source en C | [drone_collision.c](./drone_collision.c) |
| Livrable 2 | Dossier de conception technique | [livrable2.pdf](./livrable2.pdf) |
| Livrable 3 | Audit asymptotique et Preuve | [livrable3.pdf](./livrable3.pdf) |

---

## Structure du Projet

```
Projet-Industriel/
├── README.md
├── drone_collision.c
├── livrable2.pdf
└── livrable3.pdf
```

---

## Compilation et Exécution

```bash
gcc drone_collision.c -o drone_collision -lm
./drone_collision
```

---

## Résultat Attendu

```
=======================================================
  SYSTÈME DE DÉTECTION DE COLLISION — ESSAIM UAV
  N = 10000 drones | Volume 1000 m³
=======================================================
[MEM] Entrepôt alloué : 160000 octets (156.2 Ko)
[INIT] 10000 drones initialisés.
[ALGO] Lancement du sweep line O(n log n)...
=======================================================
  RAPPORT D'ALERTE DE COLLISION
=======================================================
  Drone A  : ID=...
  Drone B  : ID=...
  Distance : ... mètres
  *** MANŒUVRE D'ÉVITEMENT DÉCLENCHÉE ***
=======================================================
```

---

## Auteur

**chourouk-ibrahimi**  
École des Sciences de l'Information
