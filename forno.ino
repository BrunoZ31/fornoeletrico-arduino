// =============================================================================================================
// --- Bibliotecas Auxiliares ---  
#include <TimerOne.h>

// =============================================================================================================
// --- Mapeamento de Hardware ---
#define     alert     13                                    //saída para acionamento do alarme
#define     butt       9                                    //entrada para botão de ajuste de offset


// =============================================================================================================
// --- Hardware do MAX6675 ---
const int   thermoDO  = 11;                                 //Pino DO (ou SO)
const int   thermoCS  = 12;                                 //CS
const int   thermoCLK =  6;                                 //Pino CLK (ou SCK)


// MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);        //Cria objeto para termopar e MAX6675


// =============================================================================================================


// =============================================================================================================



// =============================================================================================================
// --- PID VAR---
int teste = 0, teste2 = 0;
float interrupt_s = 1/1000; // tempo do interrupt em s
float kp = 0.01;  //constante ganho proporcional
float Ti = 0.5;   //Tempo Integral
float Td = .1;    //Tempo Integral
float erro = 0;   //Erro
float erro_ant = 0; //erro anterior para ação derivativa
float P = 0;    //Ação Proporcional
float S = 0;    //Integrador
float I = 0;    //Ação Integral
float D = 0;    //Ação Integral
float acao;     //Ação na variável manipulada

int spValue = 0, pvValue = 0;
int sensorValue = 0;
const int pin_mv = 9; // PWM
const int pin_pv = A4; //Pino de entrada para o variável medida
const int pin_sp = A0; //Pino de entrada para o setpoint POTENCIOMETRO
const int lim_int_sup = 29000;
const int lim_int_inf = -29000;
const int lim_pwm_sup = 254;// adotou-se 254 porque a saída do pwm em 255 é muito diferente do valor em 254, podendo gerar instabilidade na malha fechada

bool state = 1;

void amostragem()
{
teste++;

state = !state;
digitalWrite(alert,state);
// Serial.println(state);

teste2++;
if (teste > 2)
	{
      teste = 0;
      spValue = analogRead(pin_sp); //leitura do valor de potenciometro
      pvValue = analogRead(pin_pv); //leitura do valor de setpoint (Sensor temp)
      // Serial.print("spValue: ");
      // Serial.println(spValue);
      // Serial.print("pvValue: ");
      // Serial.println(pvValue);
      erro = pvValue - spValue;
      // Serial.print("erro: ");
      // Serial.println(erro);
      P = erro * kp;
      S += erro;    //fórmula do integrador
      if (S > lim_int_sup) //limitador Integral superior
      {
             S = lim_int_sup;
      }
      if (S < lim_int_inf) //limitador Integral inferior
      {
             S = lim_int_inf;
      }
      I = kp * S / Ti; //fórmula da ação integral
      D = (erro_ant - erro) * Td;
      erro_ant = erro;
      acao = P + I + D;   //cálculo da ação final do controlador PI
      if (acao > lim_pwm_sup) //limitador saída superior pwm
      {
             acao = lim_pwm_sup;
      }
      if (acao < 0) //limitador saída inferior pwm
      {
             acao = 0;
      }
      // Serial.print("I: ");
      // Serial.println(I);
      analogWrite(pin_mv, acao); //linha principal para ação na variável manipulada
	}
if (teste2 > 1000) //atualiza o gráfico do plotter serial a cada 1 segundo
	{
      teste2 = 0;
      Serial.println(); // a linhas a seguir servem para mostrar as variáveis e no plotter serial do arduino
      Serial.print("sp=");
      Serial.print(spValue); //legenda setpoint
      Serial.print(" pv=");
      Serial.print(pvValue); //legenda variável medida
      Serial.print(" mv=");
      Serial.print(acao); //legenda variável manipulada
      Serial.print(" erro=");
      Serial.print(erro); //legenda desvio ou erro
      Serial.println();
      Serial.print(" ");
      Serial.print(spValue); //gráfico setpoint
      Serial.print(" ");
      Serial.print(pvValue); //gráfico variável medida
      Serial.print(" ");
      Serial.print(acao); //gráfico variável manipulada
      Serial.print(" ");
      Serial.print(erro); //gráfico desvio ou erro
	}
}
// =============================================================================================================
// --- Configurações Iniciais ---
void setup() 
{
  pinMode(alert, OUTPUT);                                   //saída para sistema de alerta
  digitalWrite(alert, LOW);                                 //saída alert inicia em LOW
   
  //     // MELHOR FORMA DE IMPLEMENTAR INTERRUPT
  Timer1.initialize(interrupt_s*1000*1000);      // Inicializa o Timer1 e configura para um período de 1 s
  Timer1.attachInterrupt(amostragem); //define a rotina de amostragem da interrupção do PID digital
  
  Serial.begin(9600); //define velocidade serial de 9600bps
  pinMode(pin_mv, OUTPUT); // PWM 
} //end setup

// =============================================================================================================
// --- Loop Infinito ---
void loop() 
{
  
} //end loop
















