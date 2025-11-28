# TP de Synthèse – Autoradio

Projet réalisé dans le cadre du TP de synthèse de l’ENSEA.  
L’objectif est de concevoir un mini autoradio embarqué sur une carte **STM32 NUCLEO-L476RG**, utilisant **FreeRTOS**, un **CODEC audio SGTL5000**, et un **GPIO Expander** pour la visualisation par LED.

---

## 1. Objectifs

- Mettre en œuvre une application temps réel sous **FreeRTOS**.  
- Piloter des périphériques externes via **SPI**, **I2C** et **SAI (I2S)**.  
- Configurer et utiliser le **CODEC audio SGTL5000**.  
- Implémenter un **Shell UART** pour le contrôle du système.  
- Visualiser le signal audio avec un **VU-Mètre**.  
- Réaliser un **filtrage** et un **effet audio**.

---

## 2. Architecture matérielle

### Matériel utilisé
- STM32 NUCLEO-L476RG  
- CODEC audio SGTL5000 (I2C + I2S)  
- GPIO Expander (via SPI) pour le contrôle des LED  
- Interface série (USART2) via STLink pour le Shell  

### Schéma fonctionnel
```
STM32L476RG
├── USART2  → Shell (terminal série)
├── SPIx    → GPIO Expander → LEDs (VU-Mètre)
├── I2C1    → SGTL5000 (configuration)
└── SAI2    ↔ SGTL5000 (flux audio)
```

---

## 3. Étapes principales

### 3.1 Démarrage du projet
- Création du projet sous **STM32CubeIDE**.  
- Tests de la **LED LD2**, de **l’USART2**, et de la fonction `printf()`.  
- Activation de **FreeRTOS (CMSIS V1)**.  
- Mise en place d’un **Shell UART** dans une tâche FreeRTOS, avec gestion par interruptions.

- <img width="484" height="126" alt="image" src="https://github.com/user-attachments/assets/a1c2d2a2-7c07-4995-b3df-5dae59972871" />
<img width="684" height="491" alt="image" src="https://github.com/user-attachments/assets/4a207b2e-7af0-4c6b-9b52-8e14648cf6c9" />
Puis on active la fonction shell_run dans une tache afin de ne pas bloquer l'acces au scheduler.

<img width="406" height="651" alt="image" src="https://github.com/user-attachments/assets/8bdd0176-c412-4f4c-9c71-a99330a374fd" />


### 3.2 GPIO Expander et VU-Mètre
- Configuration du bus **SPI** pour le [GPIO Expander MCP23S17](https://github.com/Romeo-Lorenzo/TP_Freertos/blob/main/DOCS/MCP23017-Data-Sheet-DS20001952.pdf):
  - Sur le [schématique de la carte d'interface audio](https://github.com/Romeo-Lorenzo/TP_Freertos/blob/main/DOCS/audio_iface.pdf) on retrouve que les pistes SPI du GPIO Expander sont routées sur les pins PC10, PC11, PB5 et PB7 correspondant aux bus de l'SPI3 de la STM32L476RG.
  - Ainsi nous avons initialisé le SPI4 en Full-Duplex Master configuré comme suit:
    <img width="492" height="398" alt="image" src="https://github.com/user-attachments/assets/d46b0fc0-4a98-441f-981c-785e45309da5" />

- Test de clignotement et chenillard sur les LED.  
- Écriture d’un **driver structuré** (`gpio_expander.c / .h`).  
- Commande Shell pour allumer ou éteindre une LED.  
- Visualisation du volume audio sur les LED.

### 3.3 CODEC Audio SGTL5000
- Configuration de l’I2C pour la configuration du CODEC.  
- Configuration du SAI2 (I2S) :
  - Bloc A : Master avec MCLK  
  - Bloc B : Slave synchrone  
- Activation de l’horloge MCLK :
  ```c
  __HAL_SAI_ENABLE(&hsai_BlockA2);
  ```
- Lecture du registre **CHIP_ID** (0x0000, adresse I2C = 0x14).
  <img width="626" height="201" alt="image" src="https://github.com/user-attachments/assets/e6361832-2fad-4104-8646-50151e927062" />

  <img width="620" height="28" alt="image" src="https://github.com/user-attachments/assets/b07dbc6b-f77d-4603-8c10-2fc7d1860571" />


- Écriture des registres de configuration dans `sgtl5000.c`.  
- Vérification des signaux I2S à l’oscilloscope.  
- Implémentation :
  - Génération d’un signal triangulaire,  
  - Bypass numérique (ADC → DAC).

### 3.4 Filtre RC
- Implémentation dans `RCFilter.c / RCFilter.h`.  
- Structure :
  ```c
  typedef struct {
      uint32_t coeff_A;
      uint32_t coeff_B;
      uint32_t coeff_D;
      uint16_t out_prev;
  } h_RC_filter_t;
  ```
- Fonctions :
  ```c
  void RC_filter_init(h_RC_filter_t *h, uint16_t fc, uint16_t fs);
  uint16_t RC_filter_update(h_RC_filter_t *h, uint16_t input);
  ```
- Commande Shell pour ajuster la fréquence de coupure.

### 3.5 Effet audio
- Réalisation d’un effet simple au choix (ex. tremolo, distorsion, delay…).  
- Application de l’effet dans la chaîne de traitement audio.
  
```c

```
---

## 4. Organisation du projet

```
TP_Autoradio/
├── Core/
│   ├── Src/
│   └── Inc/
├── Drivers/
│   ├── gpio_expander/
│   ├── sgtl5000/
│   ├── RCFilter/
│   └── audio_effect/
├── FreeRTOS/
├── README.md
└── LICENSE
```

---

## 5. Validation

Chaque étape du TP doit être validée par le professeur avant de passer à la suivante.  
En cas de doute, demander validation avant de modifier le matériel ou le code.

---

## 6. Auteurs

- Mathieu POMMERY
- Lorenzo ROMEO

---

## 7. Références

- Datasheet SGTL5000  
- Documentation STM32L476RG  
- Exemple Shell FreeRTOS : [https://github.com/lfiack/rtos_td_shell](https://github.com/lfiack/rtos_td_shell)
