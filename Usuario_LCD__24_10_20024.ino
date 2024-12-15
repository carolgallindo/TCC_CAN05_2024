#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PinChangeInterrupt.h>

// Configuração do LCD 20x4
LiquidCrystal_I2C lcd(0x27, 20, 4);

long int tempo_inicial;
int ciclos = 0;

const int R_EN_2 = 8;   //normal
const int L_EN_2 = 9;   //normal
const int R_PWM_2 = 10;  //PWM
const int L_PWM_2 = 11;  //PWM

int limite_pot = 150;

// Definição dos pinos dos botões
const int buttonUpDownPin = A0;  // Botão para cima e para baixo
const int buttonLeftRightPin = A1;  // Botão para esquerda e para direita
const int buttonSelectPin = A2;

// Definição dos pinos
const int fimDeCurso1 = 13; // Usando pino 12
const int fimDeCurso2 = 12; // Usando pino 13

// Variáveis para rastrear os estados
bool isCalibred = false;
volatile bool estadoFimDeCurso1 = false;
volatile bool estadoFimDeCurso2 = false;
volatile bool estadobuttonUpDownPin = false;
volatile bool estadobuttonLeftRightPin = false;
volatile bool estadobuttonSelectPin = false;

// Variáveis para armazenar a opção selecionada, a velocidade e o tempo
int selectedOption = 1; // Começa na segunda linha (Velocidade)
int selectedSpeedOption = 1; // Armazena a opção de velocidade selecionada
int velocidade = 0; // Valor padrão inicial de velocidade
String velcd = "0";
long int tempo = 0; // Valor padrão inicial de tempo
int angulo1 = 0; // Valor padrão inicial do ângulo 1
int angulo2 = 90; // Valor padrão inicial do ângulo 2

// Variáveis para armazenar o estado dos botões
int lastButtonUpDownState = HIGH;
int lastButtonLeftRightState = HIGH;
int lastButtonSelectState = HIGH;

// Variável para rastrear se estamos no menu principal ou em uma sub-tela
bool inSubMenu = false;
bool inSpeedMenu = false;
bool inTimeMenu = false;
bool inAngleMenu = false;
bool isAngle1Menu = false;
bool inSaveMenu = false;

int alvo= 0, alvo_final, erro, previous;
float derivative, integral, output;

float dt = 0.05, kp = 3.7, ki = 0.15, kd= 0.005;
int i =0;
// Filtro e debounce
unsigned long debounceTime = 5; // Tempo de debounce em ms
unsigned long lastInterruptTime = 0;
float alpha = 0.8;  // Constante do filtro complementar

// Variáveis do encoder
volatile long temp, counter = 0; // Variável que aumentará ou diminuirá dependendo da rotação do encoder
volatile long filteredCounter = 0;  // Para o filtro complementar
int lastCounters[5];  // Para a média móvel
int lastCountersIndex = 0;
const int maxJump = 10; // Limite para grandes saltos

void motor(int velocidade) {
  if (velocidade > 0) {
    analogWrite(L_PWM_2, 0);
    analogWrite(R_PWM_2, abs(velocidade));
  } else if (velocidade < 0) {
    analogWrite(L_PWM_2, abs(velocidade));
    analogWrite(R_PWM_2, 0);
  } else {
    analogWrite(L_PWM_2, 0);
    analogWrite(R_PWM_2, 0);
  }
  //Serial.print(abs(velocidade));
}

// Função de interrupção com debounce, filtro complementar e filtro de grandes saltos
void ai0() {
  unsigned long currentTime = millis();
  if ((currentTime - lastInterruptTime) > debounceTime) {
    lastInterruptTime = currentTime;

    // Direção baseada no estado do pino 3
    long newCounterValue;
    if (digitalRead(3) == LOW) {
      newCounterValue = counter + 1;
    } else {
      newCounterValue = counter - 1;
    }

    // Filtro de grandes saltos
    if (abs(newCounterValue - counter) <= maxJump) {
      counter = newCounterValue;
    }
  }
}

// Função de interrupção com debounce, filtro complementar e filtro de grandes saltos
void ai1() {
  unsigned long currentTime = millis();
  if ((currentTime - lastInterruptTime) > debounceTime) {
    lastInterruptTime = currentTime;

    // Direção baseada no estado do pino 2
    long newCounterValue;
    if (digitalRead(2) == LOW) {
      newCounterValue = counter - 1;
    } else {
      newCounterValue = counter + 1;
    }

    // Filtro de grandes saltos
    if (abs(newCounterValue - counter) <= maxJump) {
      counter = newCounterValue;
    }
  }
}

const float inf = 1e20;  // Definindo um valor muito grande como "infinito"
const float neg_inf = -1e20;

void controle(int alvo_final_setado, int ciclos2, float selectedSpeedOption) {
  alvo_final = alvo_final_setado;
  int sobe = 0;
  alvo = counter;
  int botao_focar = 1;
  float alvo_float = counter;
  float valor_x=1;

  if (selectedSpeedOption==1){
    valor_x = 0.25;
  }else if(selectedSpeedOption==2){
    valor_x = 0.5;
  }else if(selectedSpeedOption==3){
    valor_x = 0.75;
  }
  integral=0;
  // Envia o valor de counter se ele mudou desde a última vez
  int alvo_final2 = alvo_final_setado;
  if (alvo_final_setado == 90) {
    alvo_final = inf;
  } else if (alvo_final_setado == 0) {
    alvo_final = neg_inf;
  }else{
    alvo_final = alvo_final_setado*2;
  }

  // Serial.println(alvo_final);
  // Serial.println(counter);
  // Serial.println(!(alvo_final<=counter+2 && alvo_final>=counter-2));

  while(!(alvo_final<=counter+3 && alvo_final>=counter-3)){

  if (alvo < alvo_final){
    alvo_float += (valor_x);
    alvo = (int) floor(alvo_float);
    sobe = 1;
    
  }else if(alvo > alvo_final){
    alvo_float -= (valor_x);
    alvo = (int) floor(alvo_float);
    sobe = 0;
  }

  alvo = (int) floor(alvo_float);
  
  estadoFimDeCurso1 =  !digitalRead(fimDeCurso1);
  estadoFimDeCurso2 =  !digitalRead(fimDeCurso2);
  if(sobe == 1){
    if(estadoFimDeCurso2 == 1){
      counter = 180;
      break;
    }
  }else{
    if(estadoFimDeCurso1 == 1){
      counter = 0;
      break;
    }
  }

  if(alvo - alvo_final > 0 && sobe){
      //alvo = alvo_final;
  }
  if(alvo_final - alvo > 0 && !sobe){
      //alvo = alvo_final;
  }
  
  //alvo=700;
  erro = alvo - counter;

  integral += erro * dt;

  if (integral > 800){
    integral = 800;
  }else if(integral < -800){
    integral = -800;
  }

  derivative = (erro - previous)/dt;

  previous = erro;
  
  output = (kp*erro) + (ki*integral) + (kd*derivative);
  output *= 1;

  // if (erro==0){
  //   if (output > -1){
  //   // output = 0;
  //   integral = 0;
  // }else if(output < 1){
  //   // output = 0;
  //   integral = 0;
  // }
  //}
  
  if(output > limite_pot || output < -limite_pot){
    output = limite_pot * (output/abs(output));
  }

  //display_tempo_controle(ciclos2);

  motor(-output);
  if(abs(output) < 50){
    debounceTime = 6;
  } else if(50 < abs(output) < 100){
    debounceTime = 3;
  } else{
    debounceTime = 1;
  }
  // Serial.print("alvo:");
  // Serial.println(alvo);
  // Serial.println(counter);
  // Serial.print(' ');
  // Serial.println(alvo);
  // Serial.print(' ');
  // Serial.println(output);

  //delay(10);
  display_tempo_controle(ciclos2);

  }
  motor(0);
  
}

void motor00controlado(long int tempo_parado, int alvo_setado, int ciclos2 ) {
  // Envia o valor de counter se ele mudou desde a última vez
  alvo = counter;

  int long tempo_2;
  tempo_2 = millis();
  integral=0;
  while(millis()<tempo_2+tempo_parado){
  //alvo=700;
  erro = alvo - counter;

  integral += erro * dt;

  if (integral > 200){
    integral = 200;
  }else if(integral < -200){
    integral = -200;
  }

  derivative = (erro - previous)/dt;

  previous = erro;
  
  output = (kp*erro) + (ki*integral) + (kd*derivative);
  output *= 1;

  if (erro<-3 and erro>3){
    if (output > -2){
    output = 0;
    integral = 0;
  }else if(output < 2){
    output = 0;
    integral = 0;
  }

  }else{
    output *= 1;
  }

  
  if(output > limite_pot || output < -limite_pot){
    output = limite_pot * (output/abs(output));
  }
  estadoFimDeCurso1 =  !digitalRead(fimDeCurso1);
  estadoFimDeCurso2 =  !digitalRead(fimDeCurso2);
  if(!estadoFimDeCurso1 && !estadoFimDeCurso2){
    motor(-output);
  }else{
    motor(0);
  }
  //Serial.println(erro);
  //Serial.println(counter);
  display_tempo_controle(ciclos2);

  }
}

void velFixa() {
  // Envia o valor de counter se ele mudou desde a última vez
  int long tempo_2;
  tempo_2 = millis();

  alvo = counter - 2;
  estadoFimDeCurso1 =  !digitalRead(fimDeCurso1);
  int tempo = millis();
  float alvo_float = (float) counter - 2;

  while(estadoFimDeCurso1==0){
  
  if(millis() - tempo >= 10){
    tempo = millis();
    alvo_float -= 0.5;
    alvo = floor(alvo_float);
    //Serial.println(alvo);
  }
  
  //alvo=700;
  erro = alvo - counter;

  integral += erro * dt;

  if (integral > 200){
    integral = 200;
  }else if(integral < -200){
    integral = -200;
  }

  derivative = (erro - previous)/dt;

  previous = erro;
  
  output = (kp*erro) + (ki*integral) + (kd*derivative);
  output *= 1;

  if (erro<-3 and erro>3){
    if (output > -2){
    output = 0;
    integral = 0;
  }else if(output < 2){
    output = 0;
    integral = 0;
  }

  }else{
    output *= 1;
  }

  
  if(output > limite_pot || output < -limite_pot){
    output = limite_pot * (output/abs(output));
  }
  estadoFimDeCurso1 =  !digitalRead(fimDeCurso1);
  estadoFimDeCurso2 =  !digitalRead(fimDeCurso2);
  if(!estadoFimDeCurso1){
    motor(-output);
  }else{
    motor(0);
  }
  //Serial.println(erro);
  //Serial.println(counter);
  
  delay(10);

  }

  motor(0);
  delay(350);
  counter = 0;
}

void setup() {
  pinMode(R_EN_2, OUTPUT);
  pinMode(L_EN_2, OUTPUT);
  pinMode(R_PWM_2, OUTPUT);
  pinMode(L_PWM_2, OUTPUT);

  digitalWrite(R_EN_2, HIGH);
  digitalWrite(L_EN_2, HIGH);
  
  pinMode(2, INPUT_PULLUP); // Configura o pino 2 como entrada com pull-up interno (cabo BRANCO)
  pinMode(3, INPUT_PULLUP); // Configura o pino 3 como entrada com pull-up interno (cabo AZUL)

  // Configuração das interrupções
  attachInterrupt(0, ai0, RISING); // Interrupção 0 no pino digital 2 (para o Encoder)
  attachInterrupt(1, ai1, RISING); // Interrupção 1 no pino digital 3 (para o Encoder)

  // Inicializa o LCD
  lcd.init();
  lcd.backlight();

  pinMode(buttonUpDownPin, INPUT_PULLUP); // Resistor pull-up interno
  pinMode(buttonLeftRightPin, INPUT_PULLUP); // Resistor pull-up interno
  pinMode(buttonSelectPin, INPUT_PULLUP); // Resistor pull-up interno


  pinMode(fimDeCurso1, INPUT_PULLUP); // Resistor pull-up interno para o botão 1
  pinMode(fimDeCurso2, INPUT_PULLUP); // Resistor pull-up interno para o botão 2

  // Configuração das interrupções
  // attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(fimDeCurso1), fimDeCurso1Ativado, CHANGE);
  // attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(fimDeCurso2), fimDeCurso2Ativado, CHANGE);

  // Cria o caractere de grau
  createDegreeSymbol();
  
  // Serial.begin(115200);

  // Configura os pinos dos botões como entrada
  pinMode(buttonUpDownPin, INPUT_PULLUP);
  pinMode(buttonLeftRightPin, INPUT_PULLUP);
  pinMode(buttonSelectPin, INPUT_PULLUP);

  motor(0);
  delay(2000);
  motor(0);

    // Inicializa os valores da média móvel
  for (int i = 0; i < 5; i++) {
    lastCounters[i] = 0;
  }

  // Exibe o menu inicial
  displayMenu();
}

//limpar só a segunda linha
void display_tempo_controle(int ciclos2){
  lcd.setCursor(0, 1);
  long tempoRestante = tempo_inicial + tempo * 60000 - millis(); // Cálculo do tempo restante
  if (tempoRestante < 0) {
      lcd.print("00:00");
  } else {
      unsigned long totalSeconds = tempoRestante / 1000;
      unsigned long minutes = totalSeconds / 60;
      unsigned long seconds = totalSeconds % 60;

      // Formatação em duas casas decimais
      String formattedMinutes = String(minutes);
      String formattedSeconds = String(seconds);

      if (formattedMinutes.length() < 2) {
        formattedMinutes = "0" + formattedMinutes; // Adiciona zero à esquerda
      }

      if (formattedSeconds.length() < 2) {
        formattedSeconds = "0" + formattedSeconds; // Adiciona zero à esquerda
      }

      lcd.print(formattedMinutes + ":" + formattedSeconds); // Converte para segundos
  }
  lcd.setCursor(9, 1);
  lcd.print((int) ciclos2);
}

void l555oop(){
  // Envia o estado atual dos botões a cada 500 ms
  int tempo = millis();
  velFixa();
  //Serial.println(millis() - tempo);
  delay(10);
  //Serial.println(counter);
  delay(100220);
}

void loop() {
  // Verifica o estado dos botões
  int buttonUpDownState = digitalRead(buttonUpDownPin);
  int buttonLeftRightState = digitalRead(buttonLeftRightPin);
  int buttonSelectState = digitalRead(buttonSelectPin);

  if (inSubMenu) {
    if (inSpeedMenu) {
      // Controle do submenu Velocidade
      if (buttonLeftRightState == LOW && lastButtonLeftRightState == HIGH) {
        selectedSpeedOption++;
        if (selectedSpeedOption > 3) {
          selectedSpeedOption = 1; // Cicla para Min se passar de Max
        }
        displaySpeedMenu();
      }

      if (buttonUpDownState == LOW && lastButtonUpDownState == HIGH) {
        selectedSpeedOption--;
        if (selectedSpeedOption < 1) {
          selectedSpeedOption = 3; // Cicla para Max se passar de Min
        }
        displaySpeedMenu();
      }

      if (buttonSelectState == LOW && lastButtonSelectState == HIGH) {
        confirmSpeedSelection();
      }
    } else if (inTimeMenu) {
      // Controle do submenu Tempo
      if (buttonLeftRightState == LOW && lastButtonLeftRightState == HIGH) {
        tempo++; // Incrementa o tempo
        displayTimeMenu();
      }

      if (buttonUpDownState == LOW && lastButtonUpDownState == HIGH) {
        if (tempo > 0) {
          tempo--; // Decrementa o tempo se maior que 0
        }
        displayTimeMenu();
      }

      if (buttonSelectState == LOW && lastButtonSelectState == HIGH) {
        confirmTimeSelection();
      }
    } else if (inAngleMenu) {
      // Controle do submenu Ângulos
      if (isAngle1Menu) {
        // Controle do submenu Ângulo 1
        if (buttonLeftRightState == LOW && lastButtonLeftRightState == HIGH) {
          angulo1 += 5; // Incrementa o ângulo 1
          if (angulo1 > 180) { // Limite máximo do ângulo
            angulo1 = 180;
          }
          displayAngleMenu();
        }

        if (buttonUpDownState == LOW && lastButtonUpDownState == HIGH) {
          angulo1 -= 5; // Decrementa o ângulo 1
          if (angulo1 < 0) { // Não permite valores negativos
            angulo1 = 0;
          }
          displayAngleMenu();
        }

        if (buttonSelectState == LOW && lastButtonSelectState == HIGH) {
          confirmAngleSelection();
        }
      } else {
        // Controle do submenu Ângulo 2
        if (buttonLeftRightState == LOW && lastButtonLeftRightState == HIGH) {
          angulo2 += 5; // Incrementa o ângulo 2
          if (angulo2 > 90) { // Limite máximo do ângulo
            angulo2 = 90;
          }
          displayAngleMenu();
        }

        if (buttonUpDownState == LOW && lastButtonUpDownState == HIGH) {
          angulo2 -= 5; // Decrementa o ângulo 2
          if (angulo2 < 0) { // Não permite valores negativos
            angulo2 = 0;
          }
          displayAngleMenu();
        }

        if (buttonSelectState == LOW && lastButtonSelectState == HIGH) {
          confirmAngleSelection();
        }
      }
    } else if (inSaveMenu) {
      // Controle do submenu Salvar
      if (buttonLeftRightState == LOW && lastButtonLeftRightState == HIGH) {
        selectedOption = (selectedOption == 1) ? 2 : 1; // Alterna entre "Voltar" e "Iniciar"
        displaySaveMenu();
      }

      if (buttonUpDownState == LOW && lastButtonUpDownState == HIGH) {
        selectedOption = (selectedOption == 1) ? 2 : 1; // Alterna entre "Voltar" e "Iniciar"
        displaySaveMenu();
      }

      if (buttonSelectState == LOW && lastButtonSelectState == HIGH) {
        if (selectedOption == 1) {

        } else if (selectedOption == 2) {
          // Iniciar
          if (velocidade > 0 && tempo > 0 && angulo2 != 0 && angulo2 != angulo1) {
            lcd.clear();
            motor(0);
            lcd.setCursor(0, 0);
            lcd.print("Tempo:");
            lcd.setCursor(9, 0);
            lcd.print("Ciclos:");
            tempo_inicial = millis();
            ciclos=0;
            
          (tempo*60*1000);
          (tempo_inicial);
            while(millis()<tempo_inicial+tempo*60*1000){
            
            controle(angulo2, ciclos, selectedSpeedOption);

            motor00controlado(5000, angulo2, ciclos);
            controle(angulo1, ciclos, selectedSpeedOption);
            motor00controlado(5000, angulo1, ciclos);
            ciclos++;
            }
            motor(0);
            isCalibred = false;

          } else {
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Falta definir");
            lcd.setCursor(0, 1);
            lcd.print("alguma variavel");
            lcd.setCursor(0, 2);
            lcd.print("Confirme para");
            lcd.setCursor(0, 3);
            lcd.print("voltar ao menu");
            delay(200);
            // Aguardar confirmação para voltar ao menu
            while (digitalRead(buttonSelectPin) == HIGH) {
              delay(100);
            }
          }
        }
          inSubMenu = false;
          inSaveMenu = false;
          displayMenu(); // Volta para o menu
      }
    }
  } else {
    // Controle do menu principal
    if (buttonUpDownState == LOW && lastButtonUpDownState == HIGH) {
      selectedOption--;
      if (selectedOption < 1) {
        selectedOption = 5; // Vai para a última opção se passar da primeira
      }
      displayMenu();
    }

    if (buttonLeftRightState == LOW && lastButtonLeftRightState == HIGH) {
      selectedOption++;
      if (selectedOption > 5) {
        selectedOption = 1; // Volta para a primeira opção se passar da última
      }
      displayMenu();
    }

    if (buttonSelectState == LOW && lastButtonSelectState == HIGH) {
      switch (selectedOption) {
        case 1:
          enterSpeedMenu();
          break;
        case 2:
          enterTimeMenu();
          break;
        case 3:
          enterAngle1Menu();
          break;
        case 4:
          enterAngle2Menu();
          break;
        case 5:
          enterSaveMenu();
          break;
      }
    }
  }

  // Atualiza os estados dos botões
  lastButtonUpDownState = buttonUpDownState;
  lastButtonLeftRightState = buttonLeftRightState;
  lastButtonSelectState = buttonSelectState;

  delay(200); // Debounce simples
}

// Função para criar o caractere personalizado de grau
void createDegreeSymbol() {
  byte degreeSymbol[8] = {
    0b00000,
    0b00100,
    0b00100,
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b00000
  };
  lcd.createChar(0, degreeSymbol); // Cria o caractere no código de índice 0
}

// Função para exibir o menu principal
void displayMenu() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Menu");

  printOption(0, 1, "Velocidade", selectedOption == 1);
  printOption(11, 1, "Tempo", selectedOption == 2);
  printOption(0, 2, "Angulo 1", selectedOption == 3);
  printOption(11, 2, "Angulo 2", selectedOption == 4);
  printOption(0, 3, "Salvar", selectedOption == 5);
}

// Função auxiliar para imprimir uma opção com destaque
void printOption(int col, int row, const char* option, bool isSelected) {
  lcd.setCursor(col, row);

  // Se for a opção selecionada, coloca uma seta ">"
  if (isSelected) {
    lcd.print(">");
  } else {
    lcd.print(" ");
  }

  lcd.print(option);
}

// Função para exibir o menu de velocidade
void enterSpeedMenu() {
  inSubMenu = true;
  inSpeedMenu = true;
  displaySpeedMenu();
}

// Função para mostrar as opções de velocidade
void displaySpeedMenu() {
  lcd.clear();
  
  lcd.setCursor(0, 0);
  lcd.print("Velocidade");

  printOption(0, 1, "Min", selectedSpeedOption == 1);
  printOption(6, 1, "Med", selectedSpeedOption == 2);
  printOption(12, 1, "Max", selectedSpeedOption == 3);

  displayCommonFooter();
}

// Função para confirmar a seleção de velocidade
void confirmSpeedSelection() {
  switch (selectedSpeedOption) {
    case 1:
      velocidade = 20; // Min
      debounceTime = 8; // Tempo de debounce em ms
      velcd = "Minima";
      break;
    case 2:
      velocidade = 40; // Med
      debounceTime = 6; // Tempo de debounce em ms
      velcd = "Media";
      break;
    case 3:
      velocidade = 80; // Max
      velcd = "Maxima";
      break;
  }

  // Após a seleção, volta para o menu principal
  inSubMenu = false;
  inSpeedMenu = false;
  displayMenu();
}

// Função para exibir o menu de tempo
void enterTimeMenu() {
  inSubMenu = true;
  inTimeMenu = true;
  displayTimeMenu();
}

// Função para mostrar o valor do tempo
void displayTimeMenu() {
  lcd.clear();
  
  lcd.setCursor(0, 0);
  lcd.print("Tempo");

  lcd.setCursor(0, 1);
  lcd.print(" ");
  lcd.print(tempo);
  lcd.print(" min");

  displayCommonFooter();
}

// Função para confirmar a seleção de tempo
void confirmTimeSelection() {
  // Após a seleção, volta para o menu principal
  inSubMenu = false;
  inTimeMenu = false;
  displayMenu();
}

// Função para exibir o menu de ângulo 1
void enterAngle1Menu() {
  inSubMenu = true;
  inAngleMenu = true;
  isAngle1Menu = true;
  displayAngleMenu();
}

// Função para exibir o menu de ângulo 2
void enterAngle2Menu() {
  inSubMenu = true;
  inAngleMenu = true;
  isAngle1Menu = false;
  displayAngleMenu();
}

// Função para mostrar o valor do ângulo
void displayAngleMenu() {
  lcd.clear();

  if (isAngle1Menu) {
    lcd.setCursor(0, 0);
    lcd.print("Angulo 1");

    lcd.setCursor(0, 1);
    lcd.print(" ");
    lcd.print(angulo1);
    lcd.write(0); // Mostra o símbolo de grau

    lcd.setCursor(0, 2);
    lcd.print("Confirme para");
    lcd.setCursor(0, 3);
    lcd.print("retornar ao menu");
  } else {
    lcd.setCursor(0, 0);
    lcd.print("Angulo 2");

    lcd.setCursor(0, 1);
    lcd.print(" ");
    lcd.print(angulo2);
    lcd.write(0); // Mostra o símbolo de grau

    lcd.setCursor(0, 2);
    lcd.print("Confirme para");
    lcd.setCursor(0, 3);
    lcd.print("retornar ao menu");
  }
}

// Função para confirmar a seleção de ângulo
void confirmAngleSelection() {
  // Após a seleção, volta para o menu principal
  inSubMenu = false;
  inAngleMenu = false;
  displayMenu();
}

// Função para exibir o menu de salvar
void enterSaveMenu() {
  inSubMenu = true;
  inSaveMenu = true;
  displaySaveMenu();
}

// Função para mostrar o resumo e as opções de salvar
void displaySaveMenu() { 
  if(!isCalibred){
    lcd.clear();
    lcd.setCursor(5, 1);
    lcd.print("CALIBRANDO");
    delay(1000);
    velFixa();
    isCalibred = true;
  }

  motor(0);
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("Resumo:");

  lcd.setCursor(0, 1);
  lcd.print("Vel:  ");
  lcd.print(velcd);
  lcd.setCursor(0, 2);
  lcd.print("Tempo:");
  lcd.print(tempo);
  lcd.print(" min");

  lcd.setCursor(14, 1);
  lcd.print("A1:");
  lcd.print(angulo1);
  lcd.setCursor(14, 2);
  lcd.print("A2:");
  lcd.print(angulo2);

  lcd.setCursor(0, 3);
  printOption(0, 3, "Voltar", selectedOption == 1);
  printOption(10, 3, "Iniciar", selectedOption == 2);
}

// Função comum para exibir o rodapé dos submenus
void displayCommonFooter() {
  lcd.setCursor(0, 2);
  lcd.print("Confirme para");
  lcd.setCursor(0, 3);
  lcd.print("retornar ao menu");
}
