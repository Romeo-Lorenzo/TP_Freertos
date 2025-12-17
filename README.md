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

## 4. Visualisation
Maintenant que le sgtl est fonctionnel, est que nous obtenons des samples de l'adc relié au microphone, nous allons visualiser la puissance sonore: 

Pour cela on récupère juste dans une autre tache notre petit buffer circulaire de reception dma d'i2s, et on fait une puissance RMS moyenne, par la suite on la clamp puis on affiche lineairement dans notre cas les led de 0 allumé si la puissance est nulle à 8 allumée si elle est de 32767:
```c
void VU_UpdateFromBuffer(int16_t *buf, uint32_t len)
{
    int64_t sum_sq = 0;
    for (uint32_t i = 0; i < len; i++) {
        int32_t s = buf[i];
        sum_sq += (int64_t)s * (int64_t)s;
    }

    int32_t rms = (int32_t)sqrtf((float)sum_sq / (float)len);
    if (rms > 32767) rms = 32767;

    uint8_t level = (uint8_t)((rms * 8) / 32768);
    if (level > 8) level = 8;

    for (uint8_t i = 0; i < 8; i++) {
        uint8_t on = (i < level) ? 0 : 1;

        MCP23S17_SetPin(i,     on);
        MCP23S17_SetPin(i + 8, on);
    }
}
```
On obtient donc un vu mettre, il serait peu etre mieux d'adapter le coefficient P entre puissance recu et nombre de led allumée mais cela marche très bien actuellement: 



https://github.com/user-attachments/assets/e109a773-889d-464d-9e3e-81b22ae58576




## 5. Filtre RC
Nous implémentons alors un filtrage passe bas sur notre buffer,afin d'améliorer la qualitée sonore.

1. On a donc

 <img width="271" height="73" alt="image" src="https://github.com/user-attachments/assets/a550a378-1c70-4022-8c86-1bcbe098d60e" />

 Ainsi sous la forme demandé on a: X=RC et Y=1

2.On passe en temps discret afin de le traiter numériquement on à donc:
On pose:

<img width="116" height="32" alt="image" src="https://github.com/user-attachments/assets/38502fa7-996e-434b-b1e6-315e3650d8c3" />

3.Et alors:

<img width="367" height="42" alt="image" src="https://github.com/user-attachments/assets/00b429d8-61eb-4295-9472-37032ee3a9ef" />

Ainsi sous la forme demandé on a : A=1, B=K et D=1+K
donc: 

<img width="334" height="72" alt="image" src="https://github.com/user-attachments/assets/5f9b1ecd-1e8b-447a-a826-163de8e2110d" />

4.Dans notre cas nous opérons notre stm32l4 à une fréquence de 80Mhz, donc à 48kHz cela nous laisse 1666 cycle d'horloge pour faire ces calculs au maximum, ce qui est très large car nous somme ici sur une centaine de cycles d'horloge de calcul par echantillons et encore moins sur l4 avec un fpu hardware.

5.Nous implementons donc ce filtre dans un .c et un .h: 


```c
void RC_filter_init(h_RC_filter_t *h,uint16_t cutoff_frequency,uint16_t sampling_frequency)
{
    if (!h) return;

    if (cutoff_frequency == 0) cutoff_frequency = 1;
    if (sampling_frequency == 0) sampling_frequency = 1;

    float fs = (float)sampling_frequency;
    float fc = (float)cutoff_frequency;
    float K  = fs / (2.0f * (float)M_PI * fc);

    float A = 1.0f;
    float B = K;
    float D = 1.0f + K;

    h->coeff_A = (uint32_t)(A * (float)SCALE + 0.5f);
    h->coeff_B = (uint32_t)(B * (float)SCALE + 0.5f);
    h->coeff_D = (uint32_t)(D * (float)SCALE + 0.5f);

}

uint16_t RC_filter_update(h_RC_filter_t *h, uint16_t input)
{
    if (!h) return input;

    uint64_t num = 0;
    num += (uint64_t)h->coeff_A * (uint64_t)input;
    num += (uint64_t)h->coeff_B * (uint64_t)h->out_prev;

    uint32_t den = h->coeff_D;
    if (den == 0u) return input;

    uint32_t y = (uint32_t)((num + (uint64_t)(den / 2u)) / (uint64_t)den);

    if (y > 0xFFFFu) y = 0xFFFFu;//on clamp au cas ou
    h->out_prev = (uint16_t)y;//on update l'etat precedent
    return (uint16_t)y;
}
```
Pour ce qui est des coefficient, nous allons pouvoir changer la fréquence de coupure en initialisant à nouveau le filtre à la volé, pour cela on crée une fonction que l'on ajoute au shell, remarque ne mettons pas l'état de la valeurs precedente retenu en structure à zero volontairement pour cela:

```c 
int fonction_filter(h_shell_t * h_shell, int argc, char ** argv)
{
	if(argc>0){
		int freq=atoi(argv[1]);
		RC_filter_init(&filter_struct,(uint16_t ) freq,(uint16_t ) SAMPLING_FREQ);
		int size = snprintf(h_shell->print_buffer, BUFFER_SIZE, "filter cutoff freq at %d\r\n",freq);
		h_shell->drv.transmit(h_shell->print_buffer, size);
	}
	else{
		int size = snprintf(h_shell->print_buffer, BUFFER_SIZE, "mauvais arg\r\n");
		h_shell->drv.transmit(h_shell->print_buffer, size);
		return 0;
	}
	return 1;
}
```

Resultat:

<img width="301" height="181" alt="image" src="https://github.com/user-attachments/assets/e224ade6-790e-4a3d-bd87-ecf863b03276" />


On obtient alors un son meilleur avec moins de crépitement.Un autre amélioration possible serait de couper l'ampli lorsque le niveau sonore est en dessous d'un seuil minimum.

Puis il nous reste à appliquer le filtre pour cela on l'applique sur les buffer circulaire de la dma directement on obtient: 

```c
	  if (rx_half_flag)
	  {
	      rx_half_flag = 0;
	      for (int i = 0; i < AUDIO_HALF_BYTES; i++)
	      {
	          uint16_t in = i2s_rx_buf[i];
	          t0=DWT->CYCCNT;
	          i2s_tx_buf[i] = RC_filter_update(&filter_struct, in);
	          tps_filter=DWT->CYCCNT-t0;
	      }
	  }

	  if (rx_full_flag)
	  {
	      rx_full_flag = 0;
	      for (int i = 0; i < AUDIO_HALF_BYTES; i++)
	      {
	          uint16_t in = i2s_rx_buf[i + AUDIO_HALF_BYTES];
	          i2s_tx_buf[i + AUDIO_HALF_BYTES] =
	              RC_filter_update(&filter_struct, in);
	      }
	  }

```

Remarque comme vous pouvez le voir nous utilisons les compteur DWT pour connaitre précisément le temps d'horloge necessaire au calcul d'un échantillons:

<img width="503" height="24" alt="image" src="https://github.com/user-attachments/assets/22d10e25-f0a7-4370-9d50-a479e5c1fa33" />

Comme prévu nous avons ici 301 cycles d'horloges ce qui est bien inférieur au 1666 maximum autorisé, le procésseur ne travaille que 18% du temps sur le filtre RC .



On obtient alors un son meilleur avec moins de crépitement.Un autre amélioration possible serait de couper l'ampli lorsque le niveau sonore est en dessous d'un seuil minimum.

## 6. Effet numérique

Nous allons mettre en place l'effet du tremolo, il s'agit de creer une sorte d'envellope sinusoidale au son: 

<img width="404" height="685" alt="image" src="https://github.com/user-attachments/assets/62d5b4b2-49d6-4dc1-b33b-e57b21b05486" />

Pour cela Nous jouons directement sur les buffer entre le moment ou ils ont été recu et le moment ou ils vont être envoyé, cela donne donc:

```c
	  if (rx_half_flag)
	  {
	      rx_half_flag = 0;

	      for (int i = 0; i < AUDIO_HALF_BYTES; i++)
	      {
	          int16_t x = (int16_t)i2s_rx_buf[i];

	          tremolo += trem_step;
	          if (tremolo >= trem_max) { tremolo = trem_max; trem_step = -trem_step; }
	          if (tremolo <= trem_min) { tremolo = trem_min; trem_step = -trem_step; }

	          int32_t y = ((int32_t)x * (int32_t)tremolo) / 32767;

	          if (y > 32767) y = 32767;
	          if (y < -32768) y = -32768;

	          i2s_tx_buf[i] = RC_filter_update(&filter_struct, (uint16_t)(int16_t)y);
	      }

	  }

	  if (rx_full_flag)
	  {
	      rx_full_flag = 0;

	      t0 = DWT->CYCCNT;

	      for (int i = 0; i < AUDIO_HALF_BYTES; i++)
	      {
	          int idx = i + AUDIO_HALF_BYTES;
	          int16_t x = (int16_t)i2s_rx_buf[idx];

	          tremolo += trem_step;
	          if (tremolo >= trem_max) { tremolo = trem_max; trem_step = -trem_step; }
	          if (tremolo <= trem_min) { tremolo = trem_min; trem_step = -trem_step; }

	          int32_t y = ((int32_t)x * (int32_t)tremolo) / 32767;

	          if (y > 32767) y = 32767;
	          if (y < -32768) y = -32768;

	          i2s_tx_buf[idx] = RC_filter_update(&filter_struct, (uint16_t)(int16_t)y);
	      }

	      tps_filter = DWT->CYCCNT - t0;
	  }
```

Nous utilisons une implémentation simple du tremolo, ici notre envellopement est triangulaire, tout ce que l'on fait c'est que on multiplie la valeurs de chaque échantillons par un coefficient qui alterne triangulairement entre une valeurs max et une valeurs minimum voici les valeurs de test: 

```c
int tremolo = 0;
int trem_step = 20;
int trem_min = 4000;
int trem_max = 32767;
```

Nous avons aussi implémenter une fonction dans le shell permettant d'activer ou de désactiver l'effet tremolo:

```c
int fonction_tremolo(h_shell_t * h_shell, int argc, char ** argv)
{
	if(argc>0){
		if(atoi(argv[1])==1){
			tremolo=0;
			trem_step=20;
			int size = snprintf(h_shell->print_buffer, BUFFER_SIZE, "tremolo enabled\r\n");
			h_shell->drv.transmit(h_shell->print_buffer, size);

		}
		else{
			tremolo=32767;
			trem_step=0;
			int size = snprintf(h_shell->print_buffer, BUFFER_SIZE, "tremolo disabled\r\n");
			h_shell->drv.transmit(h_shell->print_buffer, size);

		}

	}
	else{
		int size = snprintf(h_shell->print_buffer, BUFFER_SIZE, "mauvais arg\r\n");
		h_shell->drv.transmit(h_shell->print_buffer, size);
		return 0;
	}



	return 1;
}
```

<img width="368" height="303" alt="image" src="https://github.com/user-attachments/assets/20eb773f-848d-43da-a8c9-d0f387a26514" />


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

## 8. Validation

Chaque étape du TP doit être validée par le professeur avant de passer à la suivante.  
En cas de doute, demander validation avant de modifier le matériel ou le code.

---

## 9. Auteurs

- Mathieu POMMERY
- Lorenzo ROMEO

---

## 10. Références

- Datasheet SGTL5000  
- Documentation STM32L476RG  
- Exemple Shell FreeRTOS : [https://github.com/lfiack/rtos_td_shell](https://github.com/lfiack/rtos_td_shell)
