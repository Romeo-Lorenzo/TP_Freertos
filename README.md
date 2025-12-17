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
```c
void Startshelltask(void const * argument)
{
  /* USER CODE BEGIN Startshelltask */


	  shell_init(&shellstruct);
	  shell_add(&shellstruct, 'f', fonction, "Une fonction inutile");
	  shell_add(&shellstruct, 'l', fonction_led, "control des led");
  /* Infinite loop */
  for(;;)
  {

	shell_run(&shellstruct);
    osDelay(100);
  }
  /* USER CODE END Startshelltask */
}
```
On appelle shell_run() dans la tache afin de ne pas bloquer l'accès au scheduler

```c
int shell_run(h_shell_t * h_shell) {
	int reading = 0;
	int pos = 0;

	while (1) {
		h_shell->drv.transmit(prompt, 2);
		reading = 1;

		while(reading) {
			char c;
            if (xQueueReceive(uartRxQueue, &c, 1000) != pdTRUE) {
                continue;
            }
      [...]
      shell_exec(h_shell, h_shell->cmd_buffer);
	  }
	return 0;
}   
```
Nous avons modifié la fonction shell_run originelle du [TD shell RTOS](https://github.com/lfiack/rtos_td_shell) afin qu'elle soit compatible avec un fonctionnement en interruption.

```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
    	shell_uart_isr(&shellstruct);
    }
}
```

```c
void shell_uart_isr(h_shell_t * h_shell){

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xQueueSendFromISR(uartRxQueue, &h_shell->receive_car, &xHigherPriorityTaskWoken);
	h_shell->drv.receive(&h_shell->receive_car);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
```
 Notre nouveau shell se base sur une reception caractère par caractère en intérruption, chaque caractère est ensuite chargé dans une queu, la fonction de mise à jour du shell s'execute dans une tache freertos, elle recois les caractère par l'intermediaire de cette queue, cela nous permet de faire fonctionner le shell sans aucune attente bloquante pour les autres taches, ce qui est nécéssaire pour de l'audio.

 
<img width="419" height="342" alt="image" src="https://github.com/user-attachments/assets/80d0b630-aad8-4a9f-ab38-328fe7928c0b" />

Fonctionnement du shell endans une tache interruption validé.

Par ailleurs ce shell fonctionne grace à un systèmes de structures:
```c
typedef struct shell_func_struct
{
	char c;
	shell_func_pointer_t func;
	char * description;
} shell_func_t;

typedef struct drv_shell_struct
{
	UART_HandleTypeDef uart;
	void (*transmit)(char * buf, uint16_t size);
	void (*receive)(uint8_t * carac);
} drv_shell_t;

typedef struct h_shell_struct
{
	int func_list_size;
	shell_func_t func_list[SHELL_FUNC_LIST_MAX_SIZE];
	char print_buffer[BUFFER_SIZE];
	char cmd_buffer[BUFFER_SIZE];
	drv_shell_t drv;
	uint8_t receive_car;
} h_shell_t;
```

Assurant ainsi le fonctionnement avec driver sous forme de structure.

Il suffit donc d'inporter les fichiers shell.h et shell.c et de mettre en place la callback de réception de caractère, et d'activer le NVIC dans le .ioc.

### 3.2 GPIO Expander et VU-Mètre
- Configuration du bus **SPI** pour le [GPIO Expander MCP23S17](https://github.com/Romeo-Lorenzo/TP_Freertos/blob/main/DOCS/MCP23017-Data-Sheet-DS20001952.pdf):
  - Sur le [schématique de la carte d'interface audio](https://github.com/Romeo-Lorenzo/TP_Freertos/blob/main/DOCS/audio_iface.pdf) on retrouve que les pistes SPI du GPIO Expander sont routées sur les pins PC10, PC11, PB5 et PB7 correspondant aux bus de l'SPI3 de la STM32L476RG.
  - Ainsi nous avons initialisé le SPI4 en Full-Duplex Master configuré comme suit:
    
    <img width="492" height="398" alt="image" src="https://github.com/user-attachments/assets/d46b0fc0-4a98-441f-981c-785e45309da5" />

- Test de clignotement et chenillard sur les LED.  
- Écriture du driver du GPIO extender (`MCP25S17.c / .h`):

<img width="300" height="300" alt="image" src="https://github.com/user-attachments/assets/4651cf97-aa15-4b2f-8a51-75afdee23a64" />

 ```c
// Initialize MCP23S17 - turns off all LEDs
void MCP23S17_SetAllPinsLow(void) {
	MCP23S17_WriteRegister(MCP23S17_GPIOA, 0x0); // Set all GPIOA high
	MCP23S17_WriteRegister(MCP23S17_GPIOB, 0x0); // Set all GPIOB high
}

// Write to a register
void MCP23S17_WriteRegister(uint8_t reg, uint8_t value) {
	uint8_t txData[3] = {MCP23S17_WRITE, reg, value};
	HAL_GPIO_WritePin(MCP_CS_GPIO_PORT, MCP_CS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, txData, 3, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(MCP_CS_GPIO_PORT, MCP_CS_PIN, GPIO_PIN_SET);
}

// Read from a register
uint8_t MCP23S17_ReadRegister(uint8_t reg) {
	uint8_t txData[3] = {MCP23S17_READ, reg, 0x00};
	uint8_t rxData[3];
	
	HAL_GPIO_WritePin(MCP_CS_GPIO_PORT, MCP_CS_PIN, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi3, txData, rxData, 3, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(MCP_CS_GPIO_PORT, MCP_CS_PIN, GPIO_PIN_SET);
	
	return rxData[2];
}

// Set pins
void MCP23S17_SetPin( uint8_t pin, uint8_t state) {
	if (pin > 15) return;

	uint8_t reg = (pin < 8) ? MCP23S17_GPIOA : MCP23S17_GPIOB;
	uint8_t bit = (pin < 8) ? pin : (pin - 8);

	uint8_t current = 0;
	// Lire l'état actuel
	current=MCP23S17_ReadRegister(reg);

	if (state)
		current = ~(1 << bit); //set the bit at position bit to 0
	else
		current = 0xFF; //set all bits to 1
	MCP23S17_WriteRegister(reg, current);
}

void MCP23S17_Init(void){
	// Set CS high initially
	HAL_GPIO_WritePin(MCP_CS_GPIO_PORT, MCP_CS_PIN, GPIO_PIN_SET);

	// Mandatory
	HAL_GPIO_WritePin(MCP_RESET_GPIO_PORT, MCP_RESET_PIN, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(MCP_RESET_GPIO_PORT, MCP_RESET_PIN, GPIO_PIN_SET);
	HAL_Delay(1);

	// Configure IOCON
	MCP23S17_WriteRegister(MCP23S17_IOCONA, 0x20); // BANK=0, SEQOP=0, etc.

	// Set all pins as outputs
	MCP23S17_WriteRegister(MCP23S17_IODIRA, 0x00); // All GPIOA outputs
	MCP23S17_WriteRegister(MCP23S17_IODIRB, 0x00); // All GPIOB outputs

	for(int i=0; i<16;i++){
		MCP23S17_SetPin(i, 0);
	}
}
```

- Commande Shell pour allumer ou éteindre une LED:
  
```c
int fonction_led(h_shell_t * h_shell, int argc, char ** argv)
{
	if(argc==2){
		int lednum=atoi(argv[0]);
		int ledstate=atoi(argv[1]);
		MCP23S17_SetPin(lednum,ledstate );
		int size = snprintf(h_shell->print_buffer, BUFFER_SIZE, "LED n°%d\r\n",lednum);
		h_shell->drv.transmit(h_shell->print_buffer, size);
	}
	else if(argc==3){
		int lednum=atoi(argv[0])*10+atoi(argv[1]);
		int ledstate=atoi(argv[2]);
		MCP23S17_SetPin(lednum,ledstate );
		int size = snprintf(h_shell->print_buffer, BUFFER_SIZE, "LED n°%d\r\n",lednum);
			h_shell->drv.transmit(h_shell->print_buffer, size);
	}
	else{
		int size = snprintf(h_shell->print_buffer, BUFFER_SIZE, "mauvais arg\r\n");
		h_shell->drv.transmit(h_shell->print_buffer, size);
		return 0;
	}
```

- ajout de la fonction:
	
```c
shell_add(&shellstruct, 'l', fonction_led, "control des led");

```
Nous obtenons donc une maniere de visualiser le volume audio sur des barres de led, par ailleurs nous avons légèrement modifié le code précédent afin que lorsque l'on veuille allumer une led, cela garde l'état des led précédentes, de plus on recopie l'état des led 1 à 8 sur les leds 9 à 16 ainsi les deux barres évoluerons en même temps et de manière équivalente:

```c

void MCP23S17_SetPin(uint8_t pin, uint8_t state)
{
    if (pin > 15) return;

    uint8_t reg = (pin < 8) ? MCP23S17_GPIOA : MCP23S17_GPIOB;
    uint8_t bit = (pin < 8) ? pin : (pin - 8);

    // Lire l'état actuel du port
    uint8_t current = MCP23S17_ReadRegister(reg);

    if (state) {
        // mettre le bit à 1
        current |= (1u << bit);
    } else {
        // mettre le bit à 0
        current &= ~(1u << bit);
    }

    MCP23S17_WriteRegister(reg, current);
}

```

Le changement principal est qu'ici on écrase pas l'état précédent mais on change précisément le bit voulu sans toucher aux autres avec un masque.

### 3.3 CODEC Audio SGTL5000

Nous passons maintenant à la puce audio principale, qui contient un adc qui gère le microphone sur la carte d'extension, cette adc par la suite sors ses echantillons en i2s, elle possède aussi un préampli qui prends lui ses échantillons en I2S, nous allons voir sa mise en route: 


- Configuration de l’I2C pour la configuration du CODEC:
  	Le CODEC est relié à l'I2C sortant des pins PB10/PB11 du STM32 (cf. [schématique](https://github.com/Romeo-Lorenzo/TP_Freertos/blob/main/DOCS/audio_iface.pdf). Soit l'I2C2 du STM32L476RG.
  
  
- Configuration du SAI2 (I2S) :
  - Bloc A : Master avec MCLK  
  - Bloc B : Slave synchrone  
- Activation de l’horloge MCLK :
  ```c
  __HAL_SAI_ENABLE(&hsai_BlockA2);
  ```

  Remarque, il est important d'activer cette horloge avant toute configuration en i2c du sgtl car c'est grace à cette horloge qu'il cadence aussi ses bloc logique interne donc l'i2c ne répondra pas sans la présence de cette horloge externe.

  ![20251128_143331 (1)](https://github.com/user-attachments/assets/ed851e04-c306-402a-8596-3785f639a5b1)

  On peut voir à l'oscilloscope le signal d'horloge qui est un signal d'horloge non retouché( en dehors des PLL) donc pas de beau carré ici, sa fréquence est de 12.195Mhz ce qui correspond à 256 fois la fréquence d'echantillonage qui est de 48kHz, ce qui est cohérent.

  


  
- Lecture du registre **CHIP_ID** (0x0000, adresse I2C = 0x14).
  <img width="626" height="201" alt="image" src="https://github.com/user-attachments/assets/e6361832-2fad-4104-8646-50151e927062" />

  <img width="620" height="28" alt="image" src="https://github.com/user-attachments/assets/b07dbc6b-f77d-4603-8c10-2fc7d1860571" />

On lit rapidement le registre chip id qui nous retourne 0xA0, ce qui est bon la puce fonctionne donc bien.

- Écriture des registres de configuration dans `sgtl5000.c`.
```c
HAL_StatusTypeDef sgtl5000_i2c_write_register(h_sgtl5000_t * h_sgtl5000, sgtl5000_registers_t reg_address, uint16_t data)
{
	HAL_StatusTypeDef ret;
	uint8_t buffer[2];
	buffer[0] = (data >> 8) & 0xFF;
	buffer[1] = data & 0xFF;
	ret = HAL_I2C_Mem_Write(
			h_sgtl5000->hi2c,
			h_sgtl5000->dev_address,
			reg_address,
			I2C_MEMADD_SIZE_16BIT,
			buffer,
			2,
			HAL_MAX_DELAY		// WTF
	);
	return ret;
}
```

Voici donc notre fonction racine, que nous utilisons pour toute écriture de registre afin de configurer, pour ce qui est des valeurs que nous écrivons dans ces registres, se referer au fichier sgtl5000.h et sgtl5000.c dans le projet.


- Vérification des signaux I2S à l’oscilloscope.

  
 ![20251128_163624](https://github.com/user-attachments/assets/56ec98fa-472d-4db2-bc38-5a7df8045cc7)


On observe donc ici le frame clock, nos frame font 16bit donc un échantillons, on remarque qu'elles sont parfaitement cloquée à 48kHz ce qui est coérent, les signaux i2s sont des signaux numérique 3.3V, on aurait pu afficher le dataout en même temps bien que peu représentatif.


  
  - Génération d’un signal triangulaire: Validé par le professeur.
```c
void gen_triangle(void)
{
    int32_t step;
    uint32_t i;

    // montée de -A à +A sur TRI_LEN/2
    step = (2 * (int32_t)TRI_AMP) / ((TRI_LEN / 2) - 1);

    for (i = 0; i < TRI_LEN / 2; i++) {
        tri_buf[i] = (int16_t)(-TRI_AMP + step * (int32_t)i);
    }

    // descente de +A à -A sur TRI_LEN/2
    for (i = TRI_LEN / 2; i < TRI_LEN; i++) {
        tri_buf[i] = (int16_t)(TRI_AMP - step * (int32_t)(i - TRI_LEN / 2));
    }
}
```

```c
void build_sai_stereo_from_triangle(void)
{
    for (uint32_t i = 0; i < TRI_LEN; i++) {
        int16_t s = tri_buf[i];
        sai_buf[2 * i + 0] = s; // Left
        sai_buf[2 * i + 1] = s; // Right (même signal pour test)
    }
}
```

Pour ce qui est de la génération du signal triangulaire, on rempli un buffer d'entier signé sur 16bit avec un signal triangulaire, d'une amplitude max et d'une période donnée alors on lance le sai_transmit dma qui est configuré en circulaire ainsi le dma va prend une à une les valeurs dans le buffer de manière circulaire et les envoyer dans le bloc amplificateur du sgtl5000.


Le signal triangulaire à été validé par le professeur sur l'oscilloscope.



  - Bypass numérique (ADC → DAC).

```c
void StartTask02(void const * argument)
{
    HAL_SAI_Receive_DMA(&hsai_BlockB2,i2s_rx_buf,AUDIO_BUF_BYTES / 2);
    HAL_SAI_Transmit_DMA(&hsai_BlockA2,i2s_tx_buf,AUDIO_BUF_BYTES / 2);
  for(;;)
  {
	        if (rx_half_flag)
	        {
	            rx_half_flag = 0;
	            memcpy(&i2s_tx_buf[0],&i2s_rx_buf[0],AUDIO_HALF_BYTES);
	        }
	        if (rx_full_flag)
	        {
	            rx_full_flag = 0;
	            memcpy(&i2s_tx_buf[AUDIO_HALF_BYTES],&i2s_rx_buf[AUDIO_HALF_BYTES],AUDIO_HALF_BYTES);
	        }

    osDelay(1);
  }
}
```

Ce bloc est encore plus simple, ici on fait un bypass numérique, les samples arrivé du bloc adc du sgtl5000 par i2s sont directement renvoyé par i2s au bloc amplificateur, remarque le code précédent ne marchait pas bien car le memcpy est trop long, il vaut mieu faire une copy à la main du buffer dans la callback de dma avec une boucle for.

```c
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
    HAL_SAI_Receive_DMA(&hsai_BlockB2,(int16_t *)i2s_rx_buf,AUDIO_BUF_BYTES);

    HAL_SAI_Transmit_DMA(&hsai_BlockA2,(int16_t *)i2s_tx_buf,AUDIO_BUF_BYTES);
  /* Infinite loop */
  for(;;)
  {



	        if (rx_half_flag)
	        {
	            rx_half_flag = 0;
	            for(int i=0;i<AUDIO_HALF_BYTES;i++){
	            	i2s_tx_buf[i]=i2s_rx_buf[i];
	            }
	        }
	        if (rx_full_flag)
	        {
	            rx_full_flag = 0;
	            for(int i=0;i<AUDIO_HALF_BYTES;i++){
	            	i2s_tx_buf[i+AUDIO_HALF_BYTES]=i2s_rx_buf[i+AUDIO_HALF_BYTES];

	            }
	        }
  }
  /* USER CODE END StartTask02 */
}
```

Ci-dessus notre tache freertos utilisé pour le bypass numérique sans utiliser memcpy, le résultat est bien meilleure.



### 3.4 Filtre RC
Nous implémentons alors un filtrage passe bas sur notre buffer,afin d'améliorer la qualitée sonore.




### 3.5 Effet audio
- Réalisation d’un effet simple au choix (ex. tremolo, distorsion, delay…).  
- Application de l’effet dans la chaîne de traitement audio.
  
```c

```
---
## 4. Visualisation


## 5. Filtre RC


## 6. Effet numérique


## 7. Organisation du projet

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
