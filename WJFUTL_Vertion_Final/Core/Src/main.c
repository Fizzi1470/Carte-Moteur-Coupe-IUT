/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * Vertion Ultime N°2 \\\ WJFUTL ///
 *
 * -> CAN semi non fonctionnel
 * -> X & Y non fonctionnel
 *
 * -> vitesse progressive --- V3
 * -> MS : Master System --- V5
 * -> communication CAN --- V3
 * -> tructure AUTO --- V8
 * -> communication USART --- V2
 *
 * -> \\\ vertion global : N°12 ///
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fdcan.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "stdbool.h"      // obligatoire pour utiliser le type "bool"
#include "math.h"         // obligatoire pour utiliser "M_PI" "abs" "fabs"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	float dist_roue_droite_mm;
	float dist_roue_gauche_mm;
	float distance_mm;
	float angle_rad;
	float x;
	float y;
} statut_odometrie_t;

typedef enum {
	STATE_stop, STATE_acc, STATE_dec, STATE_const
} StateType;
StateType currentState = STATE_stop;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PERIMETRE_ROUE_MM (52*M_PI)                                   // modifier le 51.78 = diametre si nouvelle roues
#define TICK_PAR_TOUR 2048                                               // l'encodeur s'incrémente 2000 fois par tour
#define DIAMETRE_ENTRE_LES_ROUES 0.229                                   //en mm
#define PERIMETRE_CERCLE_ENTRE_LES_ROUES (DIAMETRE_ENTRE_LES_ROUES*M_PI) // perimetre en mm
#define TE 0.01                                                          // en s
#define COEF_ROUE_GAUCHE 0.988
#define ECART_ROUE_MM 230//11.25
// en ... jsp

volatile statut_odometrie_t statut_odometrie = { 0 }, precedent_statut = {0};

uint16_t tick_d_precedent = 32768;   // ou 32 440
uint16_t tick_g_precedent = 32768;

//********************************************
float acceleration = 1;
float deceleration = 1;
//********************************************

//********************************************
float vitesse = 0.0;    // en m/s
//********************************************

float erreur_droite = 0;
float erreur_gauche = 0;

uint32_t n_DEC = 0;
uint32_t n_ACC = 0;
uint32_t n_CONST = 0;

int32_t tick_g = 0;
int32_t tick_d = 0;

float NA = 0.0;
float ND = 0.0;
float NC = 0.0;

bool triangle = 0;

float erreur_precedante_droite = 0.0;
float erreur_precedante_gauche = 0.0;

float Delta_droite = 0.0;
float Delta_gauche = 0.0;

//********************************************
//110 -> val OK
//750 -> val OK
float coef_P_d = 110;
float coef_D_d = 750;
float coef_P_g = 110;
float coef_D_g = 750;
//********************************************

bool mouvement_rotation = 0;
bool mouvement_reculer = 0;
bool rotation = 0;

float coef_droit = 1;
float coef_gauche = 1;

int AV = 0;

bool tirette = true;

float floatValue_dist = 0.0;
int16_t uintValue_dist = 0;
float floatValue_angle = 0.0;
int16_t uintValue_angle = 0;

int envoi_fin_auto = 0;
bool tirette_precedente = 0;

float X_actuel = 0.0;
float Y_actuel = 0.0;
float Teta_actuel = 0.0;
float X_debut_m = 0.0;
float Y_debut_m = 0.0;
float Teta_debut_m = 0.0;

float dist_debut_mouvement = 0.0;

int16_t X_actuel_int = 0;
int16_t Y_actuel_int = 0;
int16_t Teta_actuel_int = 0;

float start_roue_droite_mm = 0;
float start_roue_gauche_mm = 0;

//float dist_roue_droite_total_mm = 0.0;
//float dist_roue_gauche_total_mm = 0.0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
int __io_putchar(char c) {
	HAL_UART_Transmit(&huart2, (uint8_t*) &c, 1, HAL_MAX_DELAY);
	return c;
}
*/
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
float delta_dist_roue_droite_mm = 0, delta_dist_roue_gauche_mm = 0;

void update_statut_odometrie(int32_t tick_gauche, int32_t tick_droit) {

	statut_odometrie.dist_roue_droite_mm = coef_droit * (float)tick_droit * PERIMETRE_ROUE_MM / (float)TICK_PAR_TOUR;
	statut_odometrie.dist_roue_gauche_mm = coef_gauche * (float)tick_gauche * PERIMETRE_ROUE_MM / (float)TICK_PAR_TOUR;

	delta_dist_roue_droite_mm = statut_odometrie.dist_roue_droite_mm - precedent_statut.dist_roue_droite_mm;
	delta_dist_roue_gauche_mm = statut_odometrie.dist_roue_gauche_mm - precedent_statut.dist_roue_gauche_mm; //	return statut_actuel;

	float delta_distance_mm = 0.0;
	delta_distance_mm = (delta_dist_roue_droite_mm + delta_dist_roue_gauche_mm) / 2.0;

	// Calcul de l'angle de rotation en radians
	float angle_rad = (statut_odometrie.dist_roue_gauche_mm - statut_odometrie.dist_roue_droite_mm) / ECART_ROUE_MM;

	// Mise à jour de la distance et de l'angle
	statut_odometrie.angle_rad = angle_rad;
	statut_odometrie.x = precedent_statut.x + delta_distance_mm * cos(angle_rad);
	statut_odometrie.y = precedent_statut.y + delta_distance_mm * sin(angle_rad);

	precedent_statut = statut_odometrie;
}
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void send_val_tirette_mise(void) {
	FDCAN_TxHeaderTypeDef header_tirette_mise = { 0 };

	header_tirette_mise.Identifier = 0x211;
	header_tirette_mise.IdType = FDCAN_STANDARD_ID;
	header_tirette_mise.TxFrameType = FDCAN_REMOTE_FRAME;
	header_tirette_mise.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	header_tirette_mise.FDFormat = FDCAN_FD_CAN;
	header_tirette_mise.MessageMarker = 0;
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header_tirette_mise, 0);
}
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void send_val_tirette_retiree(void) {
	FDCAN_TxHeaderTypeDef header_tirette_retiree = { 0 };

	header_tirette_retiree.Identifier = 0x212;
	header_tirette_retiree.IdType = FDCAN_STANDARD_ID;
	header_tirette_retiree.TxFrameType = FDCAN_REMOTE_FRAME;
	header_tirette_retiree.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	header_tirette_retiree.FDFormat = FDCAN_FD_CAN;
	header_tirette_retiree.MessageMarker = 0;
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header_tirette_retiree, 0);
}
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void send_arret_automate(void) {
	FDCAN_TxHeaderTypeDef header_arret = { 0 };

	header_arret.Identifier = 0x10;
	header_arret.IdType = FDCAN_STANDARD_ID;
	header_arret.TxFrameType = FDCAN_REMOTE_FRAME;
	header_arret.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	header_arret.FDFormat = FDCAN_FD_CAN;
	header_arret.MessageMarker = 0;
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header_arret, 0);
}
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void send_odometrie(void) {
	int16_t x = lroundf(statut_odometrie.x);
	int16_t y = lroundf(statut_odometrie.y);
	float angle = -statut_odometrie.angle_rad * 180.f / M_PI * 100.0f;
	int16_t t = lroundf(angle);

	int16_t v = vitesse * 1000;
	uint8_t data_odo[8];

	data_odo[0] = x & 0xFF;
	data_odo[1] = (x >> 8) & 0xFF;
	data_odo[2] = y & 0xFF;
	data_odo[3] = (y >> 8) & 0xFF;
	data_odo[4] = t & 0xFF;
	data_odo[5] = (t >> 8) & 0xFF;
	data_odo[6] = v & 0xFF;
	data_odo[7] = (v >> 8) & 0xFF;

	FDCAN_TxHeaderTypeDef header_odo = { 0 };
	header_odo.Identifier = 0x150;
	header_odo.IdType = FDCAN_STANDARD_ID;
	header_odo.DataLength = 8;
	header_odo.TxFrameType = FDCAN_DATA_FRAME;
	header_odo.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	header_odo.FDFormat = FDCAN_FD_CAN;
	header_odo.MessageMarker = 0;
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header_odo, data_odo);
}
/*
void send_odometrie(void)
{
	X_actuel_int = (int16_t)X_actuel + (int16_t)X_debut_m;
	Y_actuel_int = (int16_t)Y_actuel + (int16_t)Y_debut_m;
	Teta_actuel_int = (int16_t)Teta_actuel + (int16_t)Teta_debut_m;
	Teta_actuel_int *= 100;
	Teta_actuel_int %= 360;
	//printf("Valeur_X : %d\n", X_actuel_int);
	//printf("Valeur_Y : %d\n", Y_actuel_int);
	//printf("Valeur_Teta : %d\n", Teta_actuel_int);
	//printf("\n");
	uint8_t data_odo[6];
	data_odo[0] = (uint8_t)X_actuel_int;
	data_odo[1] = (uint8_t)(X_actuel_int >> 8);
	data_odo[2] = (uint8_t)Y_actuel_int;
	data_odo[3] = (uint8_t)(Y_actuel_int >> 8);
	data_odo[4] = (uint8_t)Teta_actuel_int;
	data_odo[5] = (uint8_t)(Teta_actuel_int >> 8);
	FDCAN_TxHeaderTypeDef  header_odo = {0};
	header_odo.Identifier = 0x150;
	header_odo.IdType = FDCAN_STANDARD_ID;
	header_odo.DataLength = 6;
	header_odo.TxFrameType = FDCAN_DATA_FRAME;
	header_odo.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	header_odo.FDFormat = FDCAN_FD_CAN;
	header_odo.MessageMarker = 0;
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header_odo, data_odo);
}
*/
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
int32_t delta_tick(uint16_t tick, uint16_t tick_precedent, bool direction) {
	int16_t delta_tick;
	if (direction == 0 && (tick_precedent > tick)) {
		delta_tick = (tick - tick_precedent) - 65536;
	} else if (direction == 1 && (tick_precedent < tick)) {
		delta_tick = (tick - tick_precedent) + 65536;
	} else {
		delta_tick = tick - tick_precedent;
	}
	return delta_tick;
}
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void asserv(float consigne_gauche_mm, float consigne_droite_mm) {
    //calculs_avant_asserv();

	erreur_gauche = consigne_gauche_mm - (statut_odometrie.dist_roue_gauche_mm - start_roue_gauche_mm);
	erreur_droite = consigne_droite_mm - (statut_odometrie.dist_roue_droite_mm - start_roue_droite_mm);

	Delta_droite = erreur_droite - erreur_precedante_droite;
	Delta_gauche = erreur_gauche - erreur_precedante_gauche;

	if (erreur_gauche > 0) {
		HAL_GPIO_WritePin(M2inA_GPIO_Port, M2inA_Pin, 0);
		HAL_GPIO_WritePin(M2inB_GPIO_Port, M2inB_Pin, 1);
	} else {
		HAL_GPIO_WritePin(M2inA_GPIO_Port, M2inA_Pin, 1);
		HAL_GPIO_WritePin(M2inB_GPIO_Port, M2inB_Pin, 0);
	}
	if (erreur_droite > 0) {
		HAL_GPIO_WritePin(M1inA_GPIO_Port, M1inA_Pin, 0);
		HAL_GPIO_WritePin(M1inB_GPIO_Port, M1inB_Pin, 1);
	} else {
		HAL_GPIO_WritePin(M1inA_GPIO_Port, M1inA_Pin, 1);
		HAL_GPIO_WritePin(M1inB_GPIO_Port, M1inB_Pin, 0);
	}
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, fabs(coef_P_g * erreur_gauche + coef_D_g * Delta_gauche));
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, fabs(coef_P_d * erreur_droite + coef_D_d * Delta_droite));

	erreur_precedante_droite = erreur_droite;
	erreur_precedante_gauche = erreur_gauche;
}
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void declenchement_automate(float distence_demender, float vitesse_MAX, bool rotation) {
	// we just fuck-up the logic...
	if (distence_demender < 0) {
		mouvement_reculer = 1;
		distence_demender = -distence_demender;
	} else {
		mouvement_reculer = 0;
	}

	if (rotation) {
		distence_demender = PERIMETRE_CERCLE_ENTRE_LES_ROUES * (distence_demender / 360.0);
	}
	float deplacement_triangle = 0.5 * vitesse_MAX * vitesse_MAX / acceleration + 0.5 * vitesse_MAX * vitesse_MAX / deceleration;
	if (deplacement_triangle > distence_demender) {
		deplacement_triangle = distence_demender;
	}

	if (deplacement_triangle < distence_demender) {
		NA = vitesse_MAX / (acceleration * TE);
		ND = vitesse_MAX / (deceleration * TE);
		NC = (distence_demender - deplacement_triangle) / (vitesse_MAX * TE);
		triangle = 0;         // Trapeze
	} else {
		vitesse_MAX = sqrt(deplacement_triangle / ((1 / (2 * acceleration)) + (1 / (2 * deceleration))));
		NA = vitesse_MAX / (acceleration * TE);
		ND = vitesse_MAX / (deceleration * TE);
		NC = 0.0;
		triangle = 1;		// Triangle
	}
	mouvement_rotation = rotation;
	dist_debut_mouvement = 0.0;
	n_DEC = 0;
	n_ACC = 0;
	n_CONST = 0;
	vitesse = 0;

	start_roue_droite_mm = statut_odometrie.dist_roue_droite_mm;
	start_roue_gauche_mm = statut_odometrie.dist_roue_gauche_mm;

	currentState = STATE_acc;
}
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
	FDCAN_RxHeaderTypeDef RxHeader;
	uint8_t RxData[8];
	HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData);
	if (RxHeader.Identifier == 0x101)//_________________________________________________avencer
			{
		uintValue_dist = RxData[0] | (RxData[1] << 8);
		floatValue_dist = (float) uintValue_dist;
		uintValue_angle = RxData[2] | (RxData[3] << 8);
		floatValue_angle = (float) uintValue_angle;
		rotation = 1;

		declenchement_automate(floatValue_angle / 100.0, 0.2, rotation);
	}
	if (RxHeader.Identifier == 0x005) //_______________________________________________________stop
	{
		if(currentState == STATE_acc || currentState == STATE_const) currentState = STATE_dec;

		//declenchement_automate(0, 0, 0);
	}
	if (RxHeader.Identifier == 0x210) //_______________________________________________________tirette
			{
		tirette = HAL_GPIO_ReadPin(tirette_GPIO_Port, tirette_Pin);
		if (tirette == 1) {
			send_val_tirette_mise();
		} else {
			send_val_tirette_retiree();
		}
	}
}
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
uint8_t flag_periode_tim7 = 0;
uint8_t nbr_cycle_10 = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM7) {
		flag_periode_tim7 = 1;
		nbr_cycle_10++;
	}
}
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void lever_drapeau(void) {
	__HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 5950); // position fanion leve
}
void baisser_drapeau(void) {
	__HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 3000); // position fanion baisser
}
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */
	float consigne_gauche_m = 0.0;
	float consigne_droite_m = 0.0;

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_TIM2_Init();
	MX_USART2_UART_Init();
	MX_TIM1_Init();
	MX_TIM4_Init();
	MX_TIM3_Init();
	MX_TIM8_Init();
	MX_FDCAN1_Init();
	MX_TIM7_Init();
	MX_TIM17_Init();
	/* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start_IT(&htim7);

	HAL_FDCAN_Start(&hfdcan1);

	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

	__HAL_TIM_SET_COUNTER(&htim2, 32768); // ou 32 440
	__HAL_TIM_SET_COUNTER(&htim1, 32768);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1); // servo sur PA7

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 3000); // position arret

	HAL_GPIO_WritePin(M1en_GPIO_Port, M1en_Pin, 1);
	HAL_GPIO_WritePin(M1inA_GPIO_Port, M1inA_Pin, 0);
	HAL_GPIO_WritePin(M1inB_GPIO_Port, M1inB_Pin, 0);

	HAL_GPIO_WritePin(M2en_GPIO_Port, M2en_Pin, 1);
	HAL_GPIO_WritePin(M2inA_GPIO_Port, M2inA_Pin, 0);
	HAL_GPIO_WritePin(M2inB_GPIO_Port, M2inB_Pin, 0);

	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, FDCAN_RX_FIFO0);

	//declenchement_automate(45,0.2,1);  // (distence_demender, vitesse_MAX, rotation)
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		uint16_t cod_tick_g = __HAL_TIM_GET_COUNTER(&htim1);
		uint16_t cod_tick_d = __HAL_TIM_GET_COUNTER(&htim2);
		//uint16_t cod_tick_d = __HAL_TIM_GET_COUNTER(&htim2);

		bool direction_g = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1);
		bool direction_d = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);

		int32_t delta_tick_d = delta_tick(cod_tick_d, tick_d_precedent, direction_d);
		int32_t delta_tick_g = delta_tick(cod_tick_g, tick_g_precedent, direction_g);

		tick_d_precedent = cod_tick_d;
		tick_g_precedent = cod_tick_g;

		tick_d += delta_tick_d;
		tick_g += delta_tick_g;

		update_statut_odometrie(tick_g, tick_d * COEF_ROUE_GAUCHE);

	  	//-------------------------------------------------------- MAJ tirette
		//--------------------------------------------------------
		tirette = HAL_GPIO_ReadPin(tirette_GPIO_Port, tirette_Pin);
		if (flag_periode_tim7 == 1 && tirette != tirette_precedente) {
			tirette_precedente = tirette;
			if (tirette == 1) {
				send_val_tirette_mise();
			} else {
				send_val_tirette_retiree();
			}
		}
		//-------------------------------------------------------- fin MAJ tirette
		//-------------------------------------------------------- envoie état tirette
		if (nbr_cycle_10 == 10) {
			send_odometrie();
			nbr_cycle_10 = 0;
		}
		//-------------------------------------------------------- fin envoie état tirette
		//--------------------------------------------------------
		if (flag_periode_tim7 == 1 && tirette == false){
			if (tirette == false) {
				switch (currentState) {
				//___________________________________________________________________________________________________________STOP
				case STATE_stop:
					if (rotation == 1) {
						rotation = 0;
						declenchement_automate((floatValue_dist / 1000.0), 0.6, rotation);
					}
					else {
						asserv(0, 0);
						lever_drapeau();
					}
					if (envoi_fin_auto == 2) {
						envoi_fin_auto = 0;
						send_arret_automate();
					}
					break;
					//___________________________________________________________________________________________________________DEC
				case STATE_dec:
					vitesse = vitesse - deceleration * TE;
					consigne_gauche_m = consigne_gauche_m + vitesse * TE;
					consigne_droite_m = consigne_droite_m + vitesse * TE;
					if (mouvement_rotation) {
						if (mouvement_reculer) {
							asserv(consigne_gauche_m * 1000,-consigne_droite_m * 1000);
							Teta_debut_m = -((consigne_droite_m + consigne_gauche_m) * 180.0) / PERIMETRE_CERCLE_ENTRE_LES_ROUES;
						}
						else {
							asserv(-consigne_gauche_m * 1000,consigne_droite_m * 1000);
							Teta_debut_m = ((consigne_droite_m + consigne_gauche_m) * 180.0) / PERIMETRE_CERCLE_ENTRE_LES_ROUES;
						}
					} else {
						if (mouvement_reculer) {
							asserv(-consigne_gauche_m * 1000, -consigne_droite_m * 1000);
							dist_debut_mouvement = (-consigne_gauche_m * 1000 + (-consigne_droite_m * 1000)) / 2;
							X_debut_m = dist_debut_mouvement * cos((Teta_actuel * M_PI) / 180.0);
							Y_debut_m = dist_debut_mouvement * sin((Teta_actuel * M_PI) / 180.0);
						}
						else {
							asserv(consigne_gauche_m * 1000, consigne_droite_m * 1000);
							dist_debut_mouvement = (consigne_gauche_m * 1000 + consigne_droite_m * 1000) / 2;
							X_debut_m = dist_debut_mouvement * cos((Teta_actuel * M_PI) / 180.0);
							Y_debut_m = dist_debut_mouvement * sin((Teta_actuel * M_PI) / 180.0);
						}
					}
					n_DEC++;
					if (n_DEC >= ND || vitesse <= 0) {
						X_actuel += X_debut_m;
						Y_actuel += Y_debut_m;
						Teta_actuel += Teta_debut_m;
						X_debut_m = 0.0;
						Y_debut_m = 0.0;
						Teta_debut_m = 0.0;

						start_roue_droite_mm = statut_odometrie.dist_roue_droite_mm;
						start_roue_gauche_mm = statut_odometrie.dist_roue_gauche_mm;

						consigne_gauche_m = 0;
						consigne_droite_m = 0;
						envoi_fin_auto++;
						currentState = STATE_stop;
					}
					break;
					//___________________________________________________________________________________________________________AVENCER
				case STATE_acc:
					baisser_drapeau();
					vitesse = vitesse + acceleration * TE;
					consigne_gauche_m = consigne_gauche_m + vitesse * TE;
					consigne_droite_m = consigne_droite_m + vitesse * TE;
					if (mouvement_rotation) {
						if (mouvement_reculer) {
							asserv(consigne_gauche_m * 1000, -consigne_droite_m * 1000);
							Teta_debut_m = -((consigne_droite_m + consigne_gauche_m) * 180.0) / PERIMETRE_CERCLE_ENTRE_LES_ROUES;
						}
						else {
							asserv(-consigne_gauche_m * 1000, consigne_droite_m * 1000);
							Teta_debut_m = ((consigne_droite_m + consigne_gauche_m) * 180.0) / PERIMETRE_CERCLE_ENTRE_LES_ROUES;
						}
					}
					else {
						if (mouvement_reculer) {
							asserv(-consigne_gauche_m * 1000, -consigne_droite_m * 1000);
							dist_debut_mouvement = (-consigne_gauche_m * 1000 + -consigne_droite_m * 1000) / 2;
							X_debut_m = dist_debut_mouvement * cos((Teta_actuel * M_PI) / 180.0);
							Y_debut_m = dist_debut_mouvement * sin((Teta_actuel * M_PI) / 180.0);
						}
						else {
							asserv(consigne_gauche_m * 1000, consigne_droite_m * 1000);
							dist_debut_mouvement = (consigne_gauche_m * 1000 + consigne_droite_m * 1000) / 2;
							X_debut_m = dist_debut_mouvement * cos((Teta_actuel * M_PI) / 180.0);
							Y_debut_m = dist_debut_mouvement * sin((Teta_actuel * M_PI) / 180.0);
						}
					}
					n_ACC++;
					if (n_ACC >= NA) {
						if (triangle)
							currentState = STATE_dec;
						else
							currentState = STATE_const;
					}
					break;
					//___________________________________________________________________________________________________________RECULER
				case STATE_const:
					consigne_gauche_m = consigne_gauche_m + vitesse * TE;
					consigne_droite_m = consigne_droite_m + vitesse * TE;
					if (mouvement_rotation) {
						if (mouvement_reculer) {
							asserv(consigne_gauche_m * 1000, -consigne_droite_m * 1000);
							Teta_debut_m = -((consigne_droite_m + consigne_gauche_m) * 180.0) / PERIMETRE_CERCLE_ENTRE_LES_ROUES;
						} else {
							asserv(-consigne_gauche_m * 1000, consigne_droite_m * 1000);
							Teta_debut_m = ((consigne_droite_m + consigne_gauche_m) * 180.0) / PERIMETRE_CERCLE_ENTRE_LES_ROUES;
						}
					} else {
						if (mouvement_reculer) {
							asserv(-consigne_gauche_m * 1000, -consigne_droite_m * 1000);
							dist_debut_mouvement = (-consigne_gauche_m * 1000 + (-consigne_droite_m * 1000)) / 2;
							X_debut_m = dist_debut_mouvement * cos((Teta_actuel * M_PI) / 180.0);
							Y_debut_m = dist_debut_mouvement * sin((Teta_actuel * M_PI) / 180.0);
						} else {
							asserv(consigne_gauche_m * 1000, consigne_droite_m * 1000);
							dist_debut_mouvement = (consigne_gauche_m * 1000 + consigne_droite_m * 1000) / 2;
							X_debut_m = dist_debut_mouvement * cos((Teta_actuel * M_PI) / 180.0);
							Y_debut_m = dist_debut_mouvement * sin((Teta_actuel * M_PI) / 180.0);
						}
					}
					n_CONST++;
					if (n_CONST >= NC) {
						currentState = STATE_dec;
					}
					break;
				}
			}
			flag_periode_tim7 = 0;
		}
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
	RCC_OscInitStruct.PLL.PLLN = 85;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
// evil floating point bit level hacking
// i  = 0x5f3759df - ( i >> 1 ); what the fuck ?
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/*9 janvier 2007*/
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
