// =============================================================================================================
// --- Bibliotecas Auxiliares ---
#include <LiquidCrystal.h>                                  //inclui LCD  
#include <max6675.h>                                        //inclui max6675  
#include <Wire.h>                                           //inclui wire 
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


MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);        //Cria objeto para termopar e MAX6675


// =============================================================================================================
// --- Hardware do LCD ---
LiquidCrystal lcd (8,                                       //RS no digital 8
                   7,                                       //EN no digital 7
                   5,                                       //D4 no digital 5
                   4,                                       //D5 no digital 4
                   3,                                       //D6 no digital 3
                   2);                                      //D7 no digital 2


// =============================================================================================================
// --- Variáveis Globais ---                
short       counterT2   = 0x00,                             //contador auxiliar para o Timer2
            offset      = 0x14,                             //offset (compara temperatura)
            ini_cond    = 0x00;                             //inicial condição para monitorar temperatura

boolean     aux         = 0x01,                             //flag auxiliar para loop finito
            flag        = 0x00;                             //flag auxiliar para botão


// =============================================================================================================
// --- Interrupção ---
ISR(TIMER2_OVF_vect)
{

    TCNT2 = 0x64;                                           //Reinicializa o registrador do Timer2
    ini_cond++;                                             //incrementa ini_cond
    
            
    if(ini_cond > 100)                                      //ini_cond maior que 100?
    {                                                       //sim
      aux = 0x00;                                           //limpa aux (invalida loop finito)
      counterT2++;                                          //incrementa counterT2
    
    } //end if ini_cond


    if(counterT2 == 10)                                     //counterT2 igual a 10?
    {                                                       //sim
      counterT2 = 0x00;                                     //reinicia
      digitalWrite(alert, !digitalRead(alert));             //inverte estado do pino de alerta
     
    } //end if counterT2

      
} //end ISR


// =============================================================================================================
// --- PID VAR---
int teste = 0, teste2 = 0;
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
const int pin_sp = A0; //Pino de entrada para o setpoint
const int lim_int_sup = 29000;
const int lim_int_inf = -29000;
const int lim_pwm_sup = 254;// adotou-se 254 porque a saída do pwm em 255 é muito diferente do valor em 254, podendo gerar instabilidade na malha fechada
// =============================================================================================================
// --- Configurações Iniciais ---
void setup() 
{

  pinMode(butt,   INPUT);                                   //entrada para botão
  pinMode(alert, OUTPUT);                                   //saída para sistema de alerta
  digitalWrite(alert, LOW);                                 //saída alert inicia em LOW
  
  
  // -- Registradores de configuração do Timer2 --
  TCCR2A = 0x00;                                            //Timer2 operando em modo normal
  TCCR2B = 0x07;                                            //Prescaler 1:1024
  TCNT2  = 0x64;                                            //Overflow de aproximadamente 10ms
  TIMSK2 = 0x00;                                            //interrupção do Timer2 inicia desligada
  
  lcd.begin(16, 2);                                         //inicializa LCD 16x2
  lcd.clear();                                              //limpa display

  
      // MELHOR FORMA DE IMPLEMENTAR INTERRUPT
  //Timer1.initialize(1000);      // Inicializa o Timer1 e configura para um período de 1 ms
  //Timer1.attachInterrupt(amostragem); //define a rotina de amostragem da interrupção do PID digital
  
  // -- Loop Finito (aguarda setar flag aux) --
  while(aux)
  {

      if(digitalRead(butt))                                 //botão pressionado?
      {                                                     //sim
         flag = 0x01;                                       //seta flag
         TIMSK2 = 0x01;                                     //liga interrupção do Timer2

      } //end if butt
      
      
      if(!digitalRead(butt) && flag)                        //botão solto e flag setada?
      {                                                     //sim
         flag = 0x00;                                       //limpa flag
         offset++;                                          //incrementa offset
         TIMSK2 = 0x00;                                     //desliga interrupção do Timer2
         ini_cond = 0x00;                                   //zera cond_ini
         
      } //end if 
       
      lcd.setCursor(0,0);                                   //posiciona cursor
      lcd.print(" Ajuste Offset  ");                        //imprime mensagem
      lcd.setCursor(9,1);                                   //posiciona cursor
      lcd.print(offset);                                    //mostra offset atual
    
    
  } //end while


  lcd.clear();                                              //limpa display
  lcd.setCursor(0,0);                                       //posiciona cursor
  lcd.print("  Temp. Forno   ");                            //imprime mensagem
  lcd.setCursor(9,1);                                       //posiciona cursor
  lcd.print(offset);                                        //mostra offset ajustado
  
  Serial.begin(9600); //define velocidade serial de 9600bps
  pinMode(pin_mv, OUTPUT); // PWM 
} //end setup

// =============================================================================================================
// --- PID ---
void amostragem()
{
teste++;
teste2++;
if (teste > 30)
	{
      teste = 0;
      spValue = analogRead(pin_sp); //leitura do valor de setpoint
      pvValue = analogRead(pin_pv); //leitura do valor de setpoint
      erro = pvValue - spValue;
      P = erro * kp;
      S = S + erro;    //fórmula do integrador
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
// --- Loop Infinito ---
void loop() 
{
  
   
  lcd.setCursor(1,1);                                       //posiciona cursor
  lcd.print(thermocouple.readCelsius());                    //imprime temperatura em graus Celsius


  if(thermocouple.readCelsius() > offset) TIMSK2 = 0x01;    //se temperatura maior que offset ajustado, liga interrupção


  else                                                      //senão
  {
     TIMSK2 = 0x00;                                         //mantém interrupção desligada
     digitalWrite(alert, LOW);                              //mantém saída de alerta em LOW
    
  } //end else

 
  delay(1000);                                              //taxa de atualização do display

  
} //end loop
















