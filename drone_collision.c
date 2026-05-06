/**
 * ============================================================
 * SYSTÈME DE DÉTECTION DE COLLISION POUR ESSAIM AUTONOME (UAV)
 * École des Sciences de l'Information — Programmation Avancée en C
 * Pr. Tarik HOUICHIME — Projet Industriel
 * ============================================================
 *
 * Architecture : Tri par projection axiale + balayage linéaire (Sweep Line)
 * Complexité   : O(n log n)
 * Contrainte   : AUCUNE indexation par crochets — arithmétique pure des pointeurs
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <time.h>

/* ============================================================
 * SECTION 1 — DÉFINITION DE LA STRUCTURE HÉTÉROGÈNE
 * ============================================================ */

/**
 * Structure représentant un drone dans l'espace 3D.
 * Taille : 4 + 4 + 4 + 4 = 16 octets (alignement naturel).
 */
struct Drone {
    int   id;   /* Identifiant unique du drone (1..N) */
    float x;    /* Coordonnée spatiale axe X (mètres) */
    float y;    /* Coordonnée spatiale axe Y (mètres) */
    float z;    /* Coordonnée spatiale axe Z (mètres) */
};

/* ============================================================
 * SECTION 2 — DÉFINITION DES CONSTANTES ET TYPES AUXILIAIRES
 * ============================================================ */

#define N_DRONES    10000       /* Taille de l'essaim                    */
#define ESPACE_MAX  1000.0f     /* Volume de vol : cube 1000m × 1000m × 1000m */

/**
 * Structure légère pour l'index de tri.
 * On trie des pointeurs vers les drones (pas les drones eux-mêmes)
 * pour éviter de copier 160 Ko de données à chaque permutation.
 */
typedef struct {
    struct Drone *ptr;  /* Pointeur vers le drone dans le tableau principal */
    float         val;  /* Valeur de projection sur l'axe de tri (axe X)   */
} IndexDrone;

/* ============================================================
 * SECTION 3 — CALCUL DE DISTANCE EUCLIDIENNE (INLINEABLE)
 * ============================================================ */

/**
 * distance_carree — Retourne le carré de la distance euclidienne 3D.
 *
 * On travaille avec le carré de la distance pour éviter sqrt() coûteux.
 * La comparaison d² < d_min² est équivalente à d < d_min (pour d,d_min ≥ 0).
 *
 * @param a  Pointeur vers le premier drone
 * @param b  Pointeur vers le second drone
 * @return   (xa-xb)² + (ya-yb)² + (za-zb)²
 */
static inline float distance_carree(const struct Drone *a, const struct Drone *b)
{
    float dx = a->x - b->x;
    float dy = a->y - b->y;
    float dz = a->z - b->z;
    return dx*dx + dy*dy + dz*dz;
}

/* ============================================================
 * SECTION 4 — COMPARATEUR POUR qsort (tri sur l'axe X)
 * ============================================================ */

/**
 * comparateur_x — Fonction de comparaison pour qsort.
 *
 * Trie les IndexDrone par valeur croissante de la projection X.
 * Signature imposée par la bibliothèque standard C.
 *
 * @param a  Pointeur générique vers un IndexDrone
 * @param b  Pointeur générique vers un IndexDrone
 * @return   Valeur négative, nulle, ou positive selon l'ordre
 */
static int comparateur_x(const void *a, const void *b)
{
    /* Cast sans déréférencement de crochets : on accède aux membres via -> */
    const IndexDrone *ia = (const IndexDrone *)a;
    const IndexDrone *ib = (const IndexDrone *)b;

    /*
     * Comparaison flottante robuste :
     * - On retourne -1/0/+1 plutôt que (ia->val - ib->val) pour éviter
     *   les erreurs d'arrondi qui pourraient inverser le signe.
     */
    if (ia->val < ib->val) return -1;
    if (ia->val > ib->val) return  1;
    return 0;
}

/* ============================================================
 * SECTION 5 — ALGORITHME PRINCIPAL : SWEEP LINE SUR AXE X
 * ============================================================ */

/**
 * trouver_paire_la_plus_proche — Algorithme O(n log n) de détection.
 *
 * PRINCIPE :
 *   1. Projeter tous les drones sur l'axe X et trier (O(n log n)).
 *   2. Parcourir le tableau trié avec deux pointeurs (gauche/droite).
 *      Pour chaque drone i, seuls les drones j tels que (xj - xi) < d_min
 *      peuvent être plus proches que d_min.
 *   3. Dès que (xj - xi)² >= d_min², le drone j et tous les suivants
 *      sont éliminés sans calcul de distance complète.
 *   Cette fenêtre glissante réduit le nombre de comparaisons effectives
 *   de O(n²) vers O(n log n) en pratique.
 *
 * @param essaim       Tableau principal des drones (allocation malloc)
 * @param n            Nombre de drones
 * @param[out] drone_a Adresse du pointeur vers le premier drone de la paire
 * @param[out] drone_b Adresse du pointeur vers le second drone de la paire
 * @return             Distance euclidienne minimale trouvée (sqrt de d²_min)
 */
float trouver_paire_la_plus_proche(
    struct Drone *essaim,
    int           n,
    struct Drone **drone_a,
    struct Drone **drone_b)
{
    /* --- 5.1 : Allocation du tableau d'index --- */
    IndexDrone *index = (IndexDrone *)malloc((size_t)n * sizeof(IndexDrone));
    if (index == NULL) {
        fprintf(stderr, "[ERREUR] Échec malloc index (%d entrées)\n", n);
        exit(EXIT_FAILURE);
    }

    /* --- 5.2 : Initialisation de l'index par arithmétique de pointeurs --- */
    /*
     * On initialise chaque entrée de l'index sans utiliser essaim[i].
     * (essaim + i) donne l'adresse du i-ème drone.
     * *(index + i) déréférence le i-ème IndexDrone.
     */
    for (int i = 0; i < n; i++) {
        (index + i)->ptr = essaim + i;          /* Adresse du drone i  */
        (index + i)->val = (essaim + i)->x;     /* Projection sur X    */
    }

    /* --- 5.3 : Tri par projection X en O(n log n) --- */
    qsort(index, (size_t)n, sizeof(IndexDrone), comparateur_x);

    /* --- 5.4 : Initialisation des variables de résultat --- */
    float d2_min = FLT_MAX;   /* Distance² minimale courante (initialisation à +∞) */
    *drone_a = NULL;
    *drone_b = NULL;

    /* --- 5.5 : Balayage avec fenêtre glissante (Sweep Line) --- */
    /*
     * Pour chaque drone i (pointeur gauche de la fenêtre),
     * on cherche parmi les drones j > i (pointeur droit)
     * dont la différence de projection X est inférieure à sqrt(d2_min).
     *
     * Invariant de boucle : tous les drones k < i ont déjà été comparés
     * avec i et ne peuvent plus améliorer d2_min (leur écart X dépasse
     * sqrt(d2_min)).
     */
    for (int i = 0; i < n - 1; i++) {

        /* Pointeur vers le drone courant (gauche de la fenêtre) */
        struct Drone *di = (index + i)->ptr;

        for (int j = i + 1; j < n; j++) {

            struct Drone *dj = (index + j)->ptr;

            /*
             * ÉLAGAGE AXIAL (pruning) :
             * Si l'écart sur X seul dépasse déjà d_min, tous les drones
             * j' > j seront encore plus éloignés sur X (le tableau est trié).
             * On sort immédiatement de la boucle interne.
             *
             * dx² = (xj - xi)²
             * Si dx² >= d2_min → impossible d'améliorer → break
             */
            float dx = dj->x - di->x;   /* dx ≥ 0 car trié croissant */
            if (dx * dx >= d2_min) break;

            /* Calcul complet de la distance² 3D uniquement si nécessaire */
            float d2 = distance_carree(di, dj);

            if (d2 < d2_min) {
                d2_min   = d2;
                *drone_a = di;
                *drone_b = dj;
            }
        }
    }

    /* --- 5.6 : Libération de l'index temporaire --- */
    free(index);

    /* Retour de la distance euclidienne vraie (sqrt appliqué une seule fois) */
    return sqrtf(d2_min);
}

/* ============================================================
 * SECTION 6 — GÉNÉRATION DE L'ESSAIM (SIMULATION CAPTEUR RADAR)
 * ============================================================ */

/**
 * initialiser_essaim — Remplit le tableau de drones avec des coordonnées
 * aléatoires dans le volume [0, ESPACE_MAX]³.
 *
 * Navigation strictement par arithmétique de pointeurs.
 *
 * @param essaim  Pointeur vers le début du tableau alloué
 * @param n       Nombre de drones à initialiser
 */
void initialiser_essaim(struct Drone *essaim, int n)
{
    srand((unsigned int)time(NULL));

    for (int i = 0; i < n; i++) {
        /*
         * (essaim + i) est le pointeur vers le i-ème drone.
         * On accède à ses champs via l'opérateur flèche ->.
         * AUCUN crochet d'indexation n'est utilisé.
         */
        (essaim + i)->id = i + 1;
        (essaim + i)->x  = ((float)rand() / (float)RAND_MAX) * ESPACE_MAX;
        (essaim + i)->y  = ((float)rand() / (float)RAND_MAX) * ESPACE_MAX;
        (essaim + i)->z  = ((float)rand() / (float)RAND_MAX) * ESPACE_MAX;
    }
}

/* ============================================================
 * SECTION 7 — POINT D'ENTRÉE PRINCIPAL
 * ============================================================ */

int main(void)
{
    printf("=======================================================\n");
    printf("  SYSTÈME DE DÉTECTION DE COLLISION — ESSAIM UAV\n");
    printf("  N = %d drones | Volume %.0f m³\n", N_DRONES, ESPACE_MAX);
    printf("=======================================================\n\n");

    /* --- 7.1 : Allocation dynamique de l'entrepôt continu en mémoire --- */
    /*
     * malloc retourne un pointeur vers le PREMIER drone.
     * Les N_DRONES structures sont contiguës en mémoire (tas / heap).
     * L'adresse du k-ème drone est : essaim + k
     */
    struct Drone *essaim = (struct Drone *)malloc(
        (size_t)N_DRONES * sizeof(struct Drone)
    );

    if (essaim == NULL) {
        fprintf(stderr, "[ERREUR FATALE] Allocation mémoire échouée.\n");
        fprintf(stderr, "Mémoire requise : %zu octets\n",
                (size_t)N_DRONES * sizeof(struct Drone));
        return EXIT_FAILURE;
    }

    printf("[MEM] Entrepôt alloué : %zu octets (%.1f Ko) à l'adresse %p\n\n",
           (size_t)N_DRONES * sizeof(struct Drone),
           (double)(N_DRONES * sizeof(struct Drone)) / 1024.0,
           (void *)essaim);

    /* --- 7.2 : Simulation du flux capteur radar --- */
    printf("[INIT] Chargement des coordonnées radar...\n");
    initialiser_essaim(essaim, N_DRONES);
    printf("[INIT] %d drones initialisés.\n\n", N_DRONES);

    /* --- 7.3 : Exécution de l'algorithme de détection --- */
    struct Drone *drone_a = NULL;
    struct Drone *drone_b = NULL;

    printf("[ALGO] Lancement du sweep line O(n log n)...\n");

    clock_t debut = clock();
    float distance = trouver_paire_la_plus_proche(
        essaim, N_DRONES, &drone_a, &drone_b
    );
    clock_t fin = clock();

    double duree_ms = 1000.0 * (double)(fin - debut) / (double)CLOCKS_PER_SEC;

    /* --- 7.4 : Rapport de sécurité --- */
    printf("[ALGO] Analyse terminée.\n\n");
    printf("=======================================================\n");
    printf("  RAPPORT D'ALERTE DE COLLISION\n");
    printf("=======================================================\n");

    if (drone_a != NULL && drone_b != NULL) {
        printf("  Drone A  : ID=%d  (%.2f, %.2f, %.2f)\n",
               drone_a->id, drone_a->x, drone_a->y, drone_a->z);
        printf("  Drone B  : ID=%d  (%.2f, %.2f, %.2f)\n",
               drone_b->id, drone_b->x, drone_b->y, drone_b->z);
        printf("  Distance : %.4f mètres\n", distance);
        printf("  Durée    : %.3f ms\n", duree_ms);
        printf("\n  *** MANŒUVRE D'ÉVITEMENT DÉCLENCHÉE ***\n");
    } else {
        printf("  [ERREUR] Aucune paire trouvée.\n");
    }

    printf("=======================================================\n");

    /* --- 7.5 : Libération de l'entrepôt mémoire --- */
    free(essaim);
    essaim = NULL;

    return EXIT_SUCCESS;
}
