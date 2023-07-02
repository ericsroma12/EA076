/*
---------------------------------------------------------
EA076 - Projeto 2: Gravador de Dados Ambientais (datalogger)
---------------------------------------------------------
Integrantes do grupo:
Nome: Eric Senne Roma						          RA: 215430
Nome: Pedro Henrique Carosso Christensen	RA: 243048
Nome: Nicole Karen Moura de Jesus			    RA: 242539
---------------------------------------------------------

======================
Enunciado - Projeto 2
======================

Dados ambientais são úteis tanto para fins imediatos (por exemplo, para a agricultura) quanto para pesquisas
de longo prazo (por exemplo, a coleta de dados para análise do aquecimento global). Embora seja possível
visitar estações de medição que ficam próximas a lugares habitados (por exemplo, no pátio de uma escola), a
visita regular a alguns lugares é impraticável (por exemplo: a Ilha da Queimada Grande - SP, o topo do Pico
das Agulhas Negras - RJ ou o fundo da Caverna do Janelão - MG). Uma solução possível para a aquisição de
dados ambientais nesses lugares é usar um dispositivo que grava dados ao longo do tempo, e que pode ser
recuperado periodicamente. Esse tipo de dispositivo chama-se datalogger, e será implementado neste projeto.

=======================
Enunciado - Módulo I:
=======================

O objetivo desta atividade é a familiarização dos alunos com as operações de leitura e escrita da
memória 24C16.

======================================
Explicações do Programa do Módulo I:
======================================

No módulo I foram criadas duas funções, uma para escrever em uma posição específica da memória AT24C16 e outra para ler o que está gravado
nessa memória. Para isso tivemos que usar a biblioteca Wire e trabalhar com a comunicação serial I2C, além disso como essa é uma memória de 2^11
endereços que para cada endereço guardamos 8 bits (1 byte) utilizamos a estratégia de na comunicalção I2C colocar os 3 bits MSB do endereço da memória
junto com a ultima part dos bits de endereço do dispositivo (que chamamos de Device_Adress no programa) isso criou um range de 8 endereços diferentes
do dispositivo que vai de 0x50 até 0x57, o que no caso a AT24C16 já se identifica. Fizemos isso para economizar um pulso de 8 bits de comunicação da SDA,
pois ai colocamos os 8 bits LSB do endereço de memória na próxima comunicação. Isso nós fazemos na escrita e na leitura.

=======================
Enunciado - Módulo III:
=======================

O objetivo desta atividade é a familiarização dos alunos com transdutores analógicos.
Especificamente, usaremos um transdutor analógico que converte temperatura em tensão, o sensor de
temperatura LM35. As medidas de temperatura são traduzidas em medidas de tensão usando o
conversor analógico-digital do microcontrolador (ADC, do inglês Analog-to-Digital Converter).

======================================
Explicações do Programa do Módulo III:
======================================

Nesse módulo nós tivemos que lidar com o sensor de temperatura que converte a temperatura em tensão,
primeira preocupação que tivemos foi traduzir a tensão em um valor digital para podermos tratar na lógica
do programa, com isso definimos uma faixa de 0 até 50 graus celsius e colocamos ela sobre a faixa de tensão
de 0 até 1.1 V no máximo para melhorarmos a precisão da medida. Depois de termos convertido a tensão 
em um valor digital agora basta aplicar na fórmula para determinara a temperatura e vincular com o display de
7 segmnentos que vai atualizar a todo momento quanto que o sensor mede.
*/


#include <LiquidCrystal.h>
#include <Wire.h> 
#define PCF8574_ADDR 0x20
#define fixo 0x50 //Endereço fixo de memória

//---------------- Definição das variáveis - Memória ----------------
unsigned int p; //Ponteiro
uint8_t p1; // Primeiro byte do ponteiro
uint8_t p2; // Segundo byte do ponteiro
unsigned int end_memoria, Device_Adress,end;
unsigned char c_memoria = 0x00;

// ============ Declaração de variáveis - Sensor de temperatura ============
int analogPin = A3; 	//Vou do sensor de temperatura conectado no A4
float temperatura;      //Variável que armazena o valor da temperatura lida pelo sensor LM35

//Variáveis pro acionamento dos displays:
int v = 0x7fe;
unsigned char byte1, byte2;
int M = 0;						//Variável que define a casa dos milhares
int Cen = 0; 						//Variável que define a casa das centenas
int D = 0; 						//Variável que define a casa das dezenas
int U = 0; 						//Variável que define a casa das Unidades
int display; 						//Variável que define qual display vai ser ativado (selecionado)
unsigned long codigo; 					//Código binário para salvar valor em hex a ser transmitido
int atualiza_display = 0; 				//Flag para atualizarmos o display em multiplos de 1.6ms
int flag_amostra_temp = 0;      //Flag que indica que passou o período estipulado para uma nova amostragem da temperatura
int contador; 						      //Variável contadora que armazena valores múltiplos de 1.6ms para definir um intervalo de tempo de delay para os displays
int contador_amostra_temp;      //Variável contadora que armazena valores múltiplos de 1.6ms para definir um intervalo de tempo de amostragem da temperatura

// Variáveis que serão usadas na máquina de estados
unsigned int linha;
unsigned int coluna;
int estado = 0;
int C[3] = {1, 1, 1};// C[3] = {C1, C2, C3}
int Ca[3] = {};
int estado_anterior;
int flag_T=0; //Flag definir o delay da leitura do botão do teclado matricial em multiplos de 1.6ms
int contador_teclado; //Variável contadora que armazena valores múltiplos de 1.6ms para definir um intervalo de tempo de delay para leitura
int flag_botao = 0;

//Variáveis da maquina de estados Menu principal:
int estado_menu = 0;
int tecla_pressionada;
int flag_coleta = 0;
int flag_transf_dados = 0;
int n = 11;
int acumuladora;
unsigned int gravado, disponivel;

//Variáveis de definição dos pinos do Arduino conectados ao LCD
LiquidCrystal lcd(12, 11, 10, 8, 9, 13);
// 12 - RS ; 11 - E ; 10 - DB4 ; 8 - DB5 ; 9 - DB6 ; 13 - DB7

// ============ Declaração de Funções ============
void atualiza_C();
void maquina_teclado();
void tecla();
void maquina_menu();
void escrever(unsigned int add, unsigned char dado);
unsigned char ler(unsigned int add);
void separa(float temperatura);
unsigned int armazena_temperatura();
void reset();
void codigo_I2C(unsigned long codigo);
void aciona_display(int M, int Cen, int D, int U, int display);

// ============ Função setup ============

void setup()
{
  Serial.begin(9600);          	//Inicia a interface serial
  analogReference(INTERNAL);	// Coloca a tensão interna de 1.1V como referência
  Wire.begin(); 		// Inicia a biblioteca wire do protocolo I2C.

  p1 = ler(0x7fe); //0x7fe = 2047 (última posição da memória) => Leio o primeiro byte do ponteiro em 2046
  p2 = ler(0x7ff);
  p = p1 << 8 | p2;

  pinMode(5, INPUT_PULLUP);    			// sets the digital pin 5 as input
  pinMode(6, INPUT_PULLUP);    			// sets the digital pin 6 as input
  pinMode(7, INPUT_PULLUP);   			// sets the digital pin 7 as input
  
  pinMode(A0, OUTPUT); 				// Configura o pino A0 como saída
  pinMode(2, OUTPUT);  				// sets the digital pin 2 as output
  pinMode(3, OUTPUT);  				// sets the digital pin 3 as output
  pinMode(4, OUTPUT);  				// sets the digital pin 4 as output

  // Inicializa o LCD com 16 colunas e 2 linhas
  lcd.begin(16, 2);
  
  //Config inicial do LCD
  lcd.setCursor(0, 0); //Define a posição 0 da linha 0 para começar a escrever
  lcd.print("BEM-VINDO       "); //Escreve "PARADO" e apaga tudo na sua frente se houver
  lcd.setCursor(0, 1); //Define a posição 0 da linha 1 para começar a escrever
  lcd.print("                "); // Apaga tudo o que tiver escrito anteriormente nessa linha
  lcd.setCursor(0, 1); //Define a posição 0 da linha 1 para começar a escrever
  lcd.print("ESCOLHA FUNCAO"); //Escreve "DC: " na linha 1

  digitalWrite(A0, LOW); 			// set A0 high
  digitalWrite(2, HIGH);  			// set 2 low
  digitalWrite(3, HIGH); 			// set 3 high
  digitalWrite(4, HIGH); 			// set 4 high 

  cli(); 			// desabilita as interrupções
  configuracao_Timer0(); 	// configura o temporizador
  sei(); 			// habilita as interrupções
}

// ---------------- Função Escrever ----------------

void escrever(unsigned int add, unsigned char dado)
{
    
    unsigned char Device_Adress = fixo + ((add >> 8) & 0xFF); 	//Define o Device_Adress como sendo o valor fixo + os 3 bits MSB de add
    unsigned char end_memoria = (add & 0xFF); 				//Define o end_memoria sendo os 8 bits LSB de add

    Serial.print("endereco:");
    Serial.println(add);
    Serial.print("dado:");
    Serial.println(dado);

    Wire.beginTransmission(Device_Adress); 				// Inicia a comunicação com o AT24C16
    Wire.write(end_memoria); 							//Escreve os 8 bits restantes do endereço da memória
    Wire.write(dado); 								//Escreve os 8 bits de dados
    Wire.endTransmission(); 							//Interrompe a comunicação com o AT24C16
}

// ---------------- Função Ler ----------------

unsigned char ler(unsigned int add)
{
    
    uint8_t Device_Adress = fixo + ((add >> 8) & 0xFF);  	//Define o Device_Adress como sendo o valor fixo + os 3 bits MSB de add
    uint8_t nbytes = 2;							//Define o número de bytes que serão lidos da memória na chamada Wire.requestFrom()
    unsigned char end_memoria = (add & 0xFF);			//Define o end_memoria sendo os 8 bits LSB de add

    Wire.beginTransmission(Device_Adress); 			// Inicia a comunicação com o AT24C16
    Wire.write(end_memoria); 						//Escreve os 8 bits restantes do endereço da memória
    Wire.endTransmission(); 						//Interrompe a comunicação com o AT24C16
    
    Wire.requestFrom(Device_Adress, nbytes); 			//Mestre requisita uma leitura de 8 bits do escravo de endereço Device_Adress
    if (Wire.available())
    {
        c_memoria = Wire.read();   						// Recebe 1 byte como char 				
        
    }
    //Serial.print("endereco:");
    //Serial.println(add);
    //Serial.print("dado:");
    //Serial.println(c_memoria);

    return c_memoria;								//Função ler retorna c sendo o byte lido de leitura
}

void loop()
{
  if (flag_amostra_temp == 1) //Só vai ser amostrada a temperatura quando a flag_amostra_temp estiver em 1 e isso ocorre a cada 2s, isto é, amostro um novo valor de temperatura de 2 em 2 segundos
  {
    ADC = analogRead(analogPin);  		//Lê o valor de um pino analógico especificado
    float voltage = ADC * (1.1/ 1023.0); 	//Equação que converte o valor da cadeia binária da conversão analógica-digital em tensão
    temperatura = (1.1*100*ADC)/(1024); //Equação que converte o valor da cadeia binária da conversão analógica-digital no valor da temperatura medida pelo sensor 
    separa(temperatura); //Chama a função para separar os dígitos do valor lido de temperatura para ser mostrado nos displays da maneira desejada
    if ((flag_coleta == 1) && (p < 2046))
    {
      //Serial.println("Entrei armazena_temperatura");
      p = armazena_temperatura();
    }
    else if ((flag_coleta == 1) && (p > 2045))
    {
      //Config do LCD:
      lcd.setCursor(0, 0); //Define a posição 0 da linha 0 para começar a escrever
      lcd.print("                "); // Apaga tudo o que tiver escrito anteriormente nessa linha
      lcd.print("MEMORIA CHEIA!"); //Escreve "PARADO" e apaga tudo na sua frente se houver
      lcd.setCursor(0, 1); //Define a posição 0 da linha 1 para começar a escrever
      lcd.print("                "); // Apaga tudo o que tiver escrito anteriormente nessa linha
    } 
    flag_amostra_temp =0; 		//A flag amostra_temp é zerada indicando que foi feita a amostragem da temperatura e será esperado que a variávelvel 'contador' seja igual a 1250 novamente (período) para que seja feita uma nova atualização do valor da temperatura nos displays    
    contador_amostra_temp =0; 			//Atualização já feita, a variável 'contador_amostra_temp' é zerada indicando que é preciso passar um novo período (1250*1.6ms = 2s) para fazer uma nova amostragem da temperatura do sensor
  }

  //A Atualização do display é periódica e leva em consideração a contagem da variável "contador" até que, para o período estipulado fosse setado a flag aciona_display 
  if (atualiza_display == 1) //Indica que passou o delay entre uma atualização e outra e será feita uma nova atualização nos displays
  {
    aciona_display(M,0,0,0,1); //atualiza o display 1 (Milhar)
    aciona_display(0,Cen,0,0,2); //atualiza o display 2 (Centena)
    aciona_display(0,0,D,0,3); //atualiza o display 3 (Dezena)
    aciona_display(0,0,0,U,4); //atualiza o display 4 (Unidade)
    atualiza_display =0; //A flag atualiza_display é zerada indicando que foi feita a atualização e será esperado que a variávelvel 'contador' seja igual a 3 novamente (período) para que seja feita uma nova atualização dos displays
    contador =0; //Atualização já feita, a variável 'contador' é zerada indicando que é preciso passar um novo perídoo (3*1.6ms) para fazer uma nova atualização do display
  }
  maquina_teclado();
  maquina_menu();  

    
}

// ============ Função da configuração do temporizador ============//

void configuracao_Timer0(){
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Configuracao Temporizador 0 (8 bits) para gerar interrupcoes periodicas a cada 1.6ms no modo Clear Timer on Compare Match (CTC)
  // Relogio = 16e6 Hz
  // Prescaler = 1024
  // Faixa = 25 (contagem de 0 a OCR0A = 24)
  // Intervalo entre interrupcoes: (Prescaler/Relogio)*Faixa = (1024/16e6)*(24+1) = 1.6ms
  
  // TCCR0A – Timer/Counter Control Register A
  // COM0A1 COM0A0 COM0B1 COM0B0 – – WGM01 WGM00
  // 0      0      0      0          1     0
  TCCR0A = 0x02;

  // OCR0A – Output Compare Register A
  OCR0A = 24;

  // TIMSK0 – Timer/Counter Interrupt Mask Register
  // – – – – – OCIE0B OCIE0A TOIE0
  // – – – – – 0      1      0
  TIMSK0 = 0x02;
  
  // TCCR0B – Timer/Counter Control Register B
  // FOC0A FOC0B – – WGM02 CS02 CS01 CS0
  // 0     0         0     1    0    1
  TCCR0B = 0x05;
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
}

// ============ Rotina de serviço de interrupção do temporizador ============//

ISR(TIMER0_COMPA_vect)
{
  //Entrou aqui dentro eu sei que passou 1.6 ms
  //Contagem para atualização dos displays de 7 segmentos:
  contador++;//contadora incrementa a cada 1.6 ms
  if (contador == 3) //Indica que passou o período estipuladoi de temporização para acionamento dos displays, isto é 3*1.6ms
  {
  	atualiza_display = 1; //Passou um período => atualiza_display = 1, indicando que será feito uma atualização dos displays
  }
  
  contador_amostra_temp++; //contadora incrementa a cada 1.6 ms
  if (contador_amostra_temp == 1250) //Indica que passou o período estipulado de temporização para amostragem da temperatura, isto é 1250*1.6ms = 2s
  {
  	flag_amostra_temp = 1; //Passou um período => flag_amostra_temp = 1, indicando que será feito uma atualização do valor da temperatura nos displays
  }

  contador_teclado++;
  if (contador_teclado == 32) 
  {
  	flag_T = 1; //Passou um período => flag_T = 1, indicando que passou o período de temporização para leitura do botão do teclado matricial
  }
}

// ============ Função que irá separar os digitos da frequêcia em 4 variáveis ============//

// M = Milhar , Cen = Centena , D = Dezena , U = Unidade

void separa(float temperatura) //Separa os digítidos do milhar, centena, dezena e unidade do valor da temperatura amostrado multiplicado por 100. Ex: se a medida da temperatura for 21.35, deve ser exibido o valor 2135 no display => Milhar: 2, Centena: 1, Dezena: 3 e unidade: 5
{
  	int temp; //Definição de uma variável local para armazenar o valor da temperatura que será colocado nos displays
  	temp = int(temperatura*100); //Como o display possui 4 dígitos e iremos medir a temperatura ambiente, deve ser exibido o valor da temperatura em graus celsius vezes 100. Ex: se a medida da temperatura for 21.35, deve ser exibido o valor 2135 no display.
  
  	byte1 = (temp/ 100) % 100;   // Primeiro byte - Ex: 2043 => byte1 = 20
  	byte2 = temp % 100;           // Segundo byte => Ex: 2043 => byte2 = 43

    //Serial.print("byte1:");
  	//Serial.println(byte1);
    //Serial.print("byte2:");
    //Serial.println(byte2);
  
    //Casa dos milhares
	if((temp/1000) < 1){
		M = 0;
	}

	else
  	{
		M = int(temp/1000);
  	}
  
	//Casa das centenas
	temp = temp - (M*1000);

	if((temp/100) < 1){
		Cen = 0;
	}

	else{
		Cen = int(temp/100);
    }

	//Casa das dezenas
	temp = temp - (Cen*100);

	if(temp/10 < 1){
		D = 0;
	}

	else{
		D = int(temp/10);
	}

	//Casa das unidades
	temp = temp - (D*10);

	if(temp < 1){
		U = 0;
	}

	else{
		U = int(temp);
	}

}

unsigned int armazena_temperatura()
{
  uint8_t p3, p4;
  //Serial.print("armazena_temperatura(): ponteiro:");
  //Serial.println(ponteiro);
  if (p!=0)
    p++;
  escrever(p, byte1); //0x7fe = 2047 (última posição da memória) => Escrevo o primeiro byte do ponteiro em 2047
  //Serial.print("Valor byte1: ");
  //Serial.println(ler(ponteiro));
  p++;
  //Serial.print("armazena_temperatura(): ponteiro:");
  //Serial.println(ponteiro); 
  escrever(p, byte2);
  //Serial.print("Valor byte2: ");
  //Serial.println(ler(ponteiro));
  //p++;
  //Serial.print("ponteiro:");
  //Serial.println(ponteiro);
  p1 = (p/ 100) % 100; 
  p2 = p % 100;
  //p++;
  Serial.print("p1: ");
  Serial.println(p1);
  Serial.print("p2: ");
  Serial.println(p2);
  escrever(0x7fe, p1);
  escrever(0x7ff, p2);
  p3 = ler(0x7fe);
  p4 = ler(0x7ff);
  //Serial.print("p3: ");
  //Serial.println(p3);
  //Serial.print("p4: ");
  //Serial.println(p4);
  
  return p; 
}

void reset()
{
  p = 0;
  escrever(0x7fe, 0x00);
  escrever(0x7ff, 0x00);
  //Serial.print("Leitura da posicao 2046");
  //Serial.println(ler(0x7fe));
  //Serial.print("Leitura da posicao 2047");
  //Serial.println(ler(0x7ff));
}

void maquina_teclado()
{
  switch (estado)
  {
    case 0:						//Começa na linha 1 e detecta pressionamento
    {
      if((C[0] == 1) && (C[1] == 1) && (C[2] == 1))	// Caso não haja pressionamento vai para o estado 1
      {
        estado = 1; //L = [1011]
        digitalWrite(A0, HIGH); // set A0 high
        digitalWrite(2, LOW);  // set 2 low
        digitalWrite(3, HIGH); // set 3 high
        digitalWrite(4, HIGH); // set 4 high
      }

      else						// Caso haja pressionamento vai para o estado 4
      {
        estado_anterior = estado;
        estado = 4;
      }
      break;
    }
    
    case 1:						// Está na linha 2 e detecta pressionamento
    {
      if((C[0] == 1) && (C[1] == 1) && (C[2] == 1))	// Caso não haja pressionamento vai para o estado 2
      {
        estado = 2; //L = [1101]
        digitalWrite(A0, HIGH); // set 1 high
        digitalWrite(2, HIGH);  // set 2 high
        digitalWrite(3, LOW); // set 3 low
        digitalWrite(4, HIGH); // set 4 high
      }

      else						// Caso haja pressionamento vai para o estado 4
      {
        estado_anterior = estado;
        estado = 4;
      }
      break;
    }
    
    case 2:						// Está na linha 3 e detecta pressionamento
    {
      if((C[0] == 1) && (C[1] == 1) && (C[2] == 1))	// Caso não haja pressionamento vai para o estado 3
      {
        estado = 3; //L = [1110]
        digitalWrite(A0, HIGH); // set 1 high
        digitalWrite(2, HIGH);  // set 2 high
        digitalWrite(3, HIGH); // set 3 high
        digitalWrite(4, LOW); // set 4 low
      }

      else						// Caso haja pressionamento vai para o estado 4
      {
        estado_anterior = estado;
        estado = 4;
      }
      break;
    }
    
    case 3:						// Está na linhd 4 e detecta pressionamento
    {
      if((C[0] == 1) && (C[1] == 1) && (C[2] == 1))	// Caso não haja pressionamento vai para o estado 0
      {
        estado = 0; //L = [0111]
        digitalWrite(A0, LOW); // set 1 low
        digitalWrite(2, HIGH);  // set 2 high
        digitalWrite(3, HIGH); // set 3 high
        digitalWrite(4, HIGH); // set 4 high
      }

      else						// Caso haja pressionamento vai para o estado 4
      {
        estado_anterior = estado;
        estado = 4;
      }
      break;
    }
    
    case 4:						// Detecta de qual estado direcionou para esse estado e registra o valor de C
    {
      int i;
      for(i=0;i<3;i++)
      {
        Ca[i] = C[i];
      }
      contador_teclado = 0;
      estado = 5;					// Vai para o estado 5
      break;
    }

    case 5:						// Faz a lógica de tempo para evitar o "bounce"
    {
      while (flag_T == 0)
      {
        estado = 5;
      }
      if ((Ca[0] != C[0]) && (Ca[1] != C[1]) && (Ca[2] != C[2]))
      {
        estado = 0; //L = [0111]			// Volta para o estado 0 caso seja um ruído
        digitalWrite(A0, LOW); // set 1 low
        digitalWrite(2, HIGH);  // set 2 high
        digitalWrite(3, HIGH); // set 3 high
        digitalWrite(4, HIGH); // set 4 high
      }
      else
      {
        estado = 6;					// Vai para o estado 6 caso o pressionamento for realmente proposital
      }
      break;
    }
    
    case 6:						// Vai chamar a função tecla para detecção da tecla que foi pressionada
    {
    	tecla();		
      flag_botao = 1;
      estado = 7;
      break;
    }
    
    case 7:						// Vai detectar se o botão continua pressionado ou se já foi solto
    {
      while ((C[0] != 1) && (C[1] != 1) && (C[2] != 1))
      {
        estado = 7;
      }
      contador_teclado = 0;
	    estado = 8;
      break;    
    }
    
    case 8:						// Caso o botão foi detectado como solto no caso 7 ele detecta aqui se foi um "bounce" ou foi solto de fato
    {
      while (flag_T == 0)
      {
        estado = 8;
      }
      if ((C[0] != 1) && (C[1] != 1) && (C[2] != 1))
      {
        estado = 7;
      }
      else
      {
        estado = 0; //L = [0111]			// Volta para o estado 0 e reinicia o ciclo
        digitalWrite(A0, LOW); // set 1 low
        digitalWrite(2, HIGH);  // set 2 high
        digitalWrite(3, HIGH); // set 3 high
        digitalWrite(4, HIGH); // set 4 high 
      }
      break; 
    }
  }
  atualiza_C();						// Chama a função para detectar o valor de C
}

// ============ Função de atualização do valor de C ============
void atualiza_C()
{
  int i;
  for(i=0;i<3;i++)
  	C[i] = digitalRead(5+i);
  //Serial.print(C[0]);
  //Serial.print(C[1]);
  //Serial.println(C[2]);
}

void tecla()
{
  //Serial.print("flag_transf_dados: ");
  //Serial.println(flag_transf_dados);
  //Serial.print("n: ");
  //Serial.println(n);
  //Serial.print("tecla pressionada: ");
  //Serial.println(tecla_pressionada);  
  if ((estado_anterior == 0) && (Ca[0] == 0)) //Linha 1/Coluna 1 
  {
    tecla_pressionada = 1;
    if((flag_botao == 1) && (flag_transf_dados == 1))
    {
      //Serial.print("Cliquei no 1");
      lcd.setCursor(n, 1); //Define a posição 0 da linha 1 para começar a escrever
      lcd.print("1");
      n++;

    }
  }
  
  else if ((estado_anterior == 0) && (Ca[1] == 0)) //Linha 1/Coluna 2
  {
    tecla_pressionada = 2;
    if((flag_botao == 1) && (flag_transf_dados == 1))
    {
      lcd.setCursor(n, 1); //Define a posição 0 da linha 1 para começar a escrever
      lcd.print("2");
      n++;
    } 

  }
  
  else if ((estado_anterior == 0) && (Ca[2] == 0)) //Linha 1/Coluna 3
  {
    tecla_pressionada = 3;
    if((flag_botao == 1) && (flag_transf_dados == 1))
    {
      lcd.setCursor(n, 1); //Define a posição 0 da linha 1 para começar a escrever
      lcd.print("3");
      n++;
    }  
  }
  
  else if ((estado_anterior == 1) && (Ca[0] == 0)) //Linha 2/Coluna 1
  {
    tecla_pressionada = 4;
    if((flag_botao == 1) && (flag_transf_dados == 1))
    {
      lcd.setCursor(n, 1); //Define a posição 0 da linha 1 para começar a escrever
      lcd.print("4");
      n++;
    }
  }
  
  else if ((estado_anterior == 1) && (Ca[1] == 0)) //Linha 2/Coluna 2
  {
    tecla_pressionada = 5;
    if((flag_botao == 1) && (flag_transf_dados == 1))
    {
      lcd.setCursor(n, 1); //Define a posição 0 da linha 1 para começar a escrever
      lcd.print("5");
      n++;
    }
  }
  
  else if ((estado_anterior == 1) && (Ca[2] == 0)) //Linha 2/Coluna 3
  {
    tecla_pressionada = 6;
    if((flag_botao == 1) && (flag_transf_dados == 1))
    {
      lcd.setCursor(n, 1); //Define a posição 0 da linha 1 para começar a escrever
      lcd.print("6");
      n++;
    }
  }
  
  else if ((estado_anterior == 2) && (Ca[0] == 0)) //Linha 3/Coluna 1
  {
    tecla_pressionada = 7;
    if((flag_botao == 1) && (flag_transf_dados == 1))
    {
      lcd.setCursor(n, 1); //Define a posição 0 da linha 1 para começar a escrever
      lcd.print("7");
      n++;
    }
  }
  
  else if ((estado_anterior == 2) && (Ca[1] == 0)) //Linha 3/Coluna 2
  {
    tecla_pressionada = 8;
    if((flag_botao == 1) && (flag_transf_dados == 1))
    {
      lcd.setCursor(n, 1); //Define a posição 0 da linha 1 para começar a escrever
      lcd.print("8");
      n++;
    }
  }
  
  else if ((estado_anterior == 2) && (Ca[2] == 0)) //Linha 3/Coluna 3
  {
    tecla_pressionada = 9;
    if((flag_botao == 1) && (flag_transf_dados == 1))
    {
      lcd.setCursor(n, 1); //Define a posição 0 da linha 1 para começar a escrever
      lcd.print("9");
      n++;
    }
  }
  
  else if ((estado_anterior == 3) && (Ca[0] == 0)) //Linha 4/Coluna 1
  {
    tecla_pressionada = 10; //10 = * = NAO
  }
  
  else if ((estado_anterior == 3) && (Ca[1] == 0)) //Linha 4/Coluna 2
  {
    tecla_pressionada = 0;
    if((flag_botao == 1) && (flag_transf_dados == 1))
    {
      lcd.setCursor(n, 1); //Define a posição 0 da linha 1 para começar a escrever
      lcd.print("0");
      n++;
    }
  }
  
  else if ((estado_anterior == 3) && (Ca[2] == 0)) //Linha 4/Coluna 3
  {
    tecla_pressionada = 11; //11 = # = SIM
  }

  if (n == 12)
  {
    acumuladora = tecla_pressionada * 1000;
  }

  if (n == 13)
  {
    acumuladora += tecla_pressionada * 100;
  }

  if (n == 14)
  {
    acumuladora += tecla_pressionada * 10;
  }

  if (n == 15)
  {
    acumuladora += tecla_pressionada * 1;
    flag_transf_dados = 0;
    n = 0;
  }
  //Serial.print("acumuladora:");
  //Serial.println(acumuladora);
    
}

void maquina_menu()
{
  //Serial.print("estado_menu: ");
  //Serial.println(estado_menu);
  switch(estado_menu)
  {
    case 0:						
        {
          if((flag_botao == 1) && (tecla_pressionada == 1))	// estado 1 da máquina do menu
          {
            estado_menu = 1;
          }

          else if((flag_botao == 1) && (tecla_pressionada == 2))	// estado 1 da máquina do menu
          {
            estado_menu = 2;
          }
          
          else if((flag_botao == 1) && (tecla_pressionada == 3))	// estado 1 da máquina do menu
          {
            estado_menu = 3;
          }

          else if((flag_botao == 1) && (tecla_pressionada == 4))	// estado 1 da máquina do menu
          {
            estado_menu = 4;
          }

          else if((flag_botao == 1) && (tecla_pressionada == 5))	// estado 1 da máquina do menu
          {
            estado_menu = 5;
          }

          else
          {
            estado_menu = 0;
          }

          break;
        }

    case 1:						
        {
          //Config do LCD:
          lcd.setCursor(0, 0); //Define a posição 0 da linha 0 para começar a escrever
          lcd.print("1 - RESET       "); //Escreve "PARADO" e apaga tudo na sua frente se houver
          lcd.setCursor(0, 1); //Define a posição 0 da linha 1 para começar a escrever
          lcd.print("CONFIRMAR?      "); //Escreve "PARADO" e apaga tudo na sua frente se houver

          if((flag_botao == 1) && (tecla_pressionada == 10))	//10 = * = NAO
          {
            estado_menu = 12;
          }

          else if ((flag_botao == 1) && (tecla_pressionada == 11))	//11 = # = SIM
          {
            estado_menu = 6;
          }

          break;
        }
    
    case 2:						
        {
          //Config do LCD:
          lcd.setCursor(0, 0); //Define a posição 0 da linha 0 para começar a escrever
          lcd.print("2 - STATUS      "); //Escreve "PARADO" e apaga tudo na sua frente se houver
          lcd.setCursor(0, 1); //Define a posição 0 da linha 1 para começar a escrever
          lcd.print("CONFIRMAR?      "); //Escreve "PARADO" e apaga tudo na sua frente se houver

          if((flag_botao == 1) && (tecla_pressionada == 10))	//10 = * = NAO
          {
            estado_menu = 12;
          }

          else if ((flag_botao == 1) && (tecla_pressionada == 11))	//11 = # = SIM
          {
            estado_menu = 7;
          }

          break;
        }
    
    case 3:						
        {
          //Config do LCD:
          lcd.setCursor(0, 0); //Define a posição 0 da linha 0 para começar a escrever
          lcd.print("3 - START       "); //Escreve "PARADO" e apaga tudo na sua frente se houver
          lcd.setCursor(0, 1); //Define a posição 0 da linha 1 para começar a escrever
          lcd.print("CONFIRMAR?      "); //Escreve "PARADO" e apaga tudo na sua frente se houver

          if((flag_botao == 1) && (tecla_pressionada == 10))	//10 = * = NAO
          {
            estado_menu = 12;
          }

          else if ((flag_botao == 1) && (tecla_pressionada == 11))	//11 = # = SIM
          {
            estado_menu = 8;
          }

          break;
        }

    case 4:						
        {
          //Config do LCD:
          lcd.setCursor(0, 0); //Define a posição 0 da linha 0 para começar a escrever
          lcd.print("4 - STOP        "); //Escreve "PARADO" e apaga tudo na sua frente se houver
          lcd.setCursor(0, 1); //Define a posição 0 da linha 1 para começar a escrever
          lcd.print("CONFIRMAR?      "); //Escreve "PARADO" e apaga tudo na sua frente se houver

          if((flag_botao == 1) && (tecla_pressionada == 10))	//10 = * = NAO
          {
            estado_menu = 12;
          }

          else if ((flag_botao == 1) && (tecla_pressionada == 11))	//11 = # = SIM
          {
            estado_menu = 9;
          }

          break;
        }
    
    case 5:						
        {
          //Config do LCD:
          lcd.setCursor(0, 0); //Define a posição 0 da linha 0 para começar a escrever
          lcd.print("5 - TRANSF. DADO"); //Escreve "PARADO" e apaga tudo na sua frente se houver
          lcd.setCursor(0, 1); //Define a posição 0 da linha 1 para começar a escrever
          lcd.print("CONFIRMAR?      "); //Escreve "PARADO" e apaga tudo na sua frente se houver

          if((flag_botao == 1) && (tecla_pressionada == 10))	//10 = * = NAO
          {
            estado_menu = 12;
          }

          else if ((flag_botao == 1) && (tecla_pressionada == 11))	//11 = # = SIM
          {
            estado_menu = 11;
          }

          break;
        }
    
    case 6:						
        {
          //Config do LCD:
          lcd.setCursor(0, 0); //Define a posição 0 da linha 0 para começar a escrever
          lcd.print("RESET           "); //Escreve "PARADO" e apaga tudo na sua frente se houver
          lcd.setCursor(0, 1); //Define a posição 0 da linha 1 para começar a escrever
          lcd.print("LIMPANDO MEMORIA"); //Escreve "PARADO" e apaga tudo na sua frente se houver

          reset();
          estado_menu = 0;

          break;
        }
        
    case 7:						
        {
          gravado = p/2;
          //Serial.print("gravado: ");
          Serial.println(gravado);
          disponivel = 1023 - gravado;
          //Serial.print("disponivel: ");
          Serial.println(disponivel);
          //Config do LCD:
          lcd.setCursor(0, 0); //Define a posição 0 da linha 0 para começar a escrever
          lcd.print("GRAVADO: "); //Escreve "PARADO" e apaga tudo na sua frente se houver
          lcd.print(gravado); 
          lcd.setCursor(0, 1); //Define a posição 0 da linha 1 para começar a escrever
          lcd.print("DISPONIVEL: "); //Escreve "PARADO" e apaga tudo na sua frente se houver
          lcd.print(disponivel); 

          estado_menu = 0;

          break;
        }
    
    case 8:						
        {
          //Config do LCD:
          lcd.setCursor(0, 0); //Define a posição 0 da linha 0 para começar a escrever
          lcd.print("START           "); //Escreve "PARADO" e apaga tudo na sua frente se houver 
          lcd.setCursor(0, 1); //Define a posição 0 da linha 1 para começar a escrever
          lcd.print("INICIO DA COLETA"); //Escreve "PARADO" e apaga tudo na sua frente se houver

          flag_coleta = 1;

          estado_menu = 0;

          break;
        }
    
    case 9:						
        {
          unsigned int coletado = 0;
          coletado = p/2 - gravado;
          //Config do LCD:
          lcd.setCursor(0, 0); //Define a posição 0 da linha 0 para começar a escrever
          lcd.print("STOP: FIM       "); //Escreve "PARADO" e apaga tudo na sua frente se houver 
          lcd.setCursor(0, 1); //Define a posição 0 da linha 1 para começar a escrever
          lcd.print("No DADOS: "); //Escreve "PARADO" e apaga tudo na sua frente se houver
          lcd.print(coletado);

          flag_coleta = 0;

          estado_menu = 0;

          break;
        }
    
    case 10:						
        {
          //Serial.println("Entrei no estado 10");
          //Serial.print("acumuladora:");
          //Serial.println(acumuladora);
          break;
        }
    
    case 11:
        {
        
          //Config do LCD:
            lcd.setCursor(0, 0); //Define a posição 0 da linha 0 para começar a escrever
            lcd.print("TRANSF. DADOS   "); //Escreve "PARADO" e apaga tudo na sua frente se houver 
            lcd.setCursor(0, 1); //Define a posição 0 da linha 1 para começar a escrever
            lcd.print("ESC. QTDE: "); //Escreve "PARADO" e apaga tudo na sua frente se houver

            flag_transf_dados = 1;

            estado_menu = 10;

            break;

        }
    case 12:
        {
        
          //Config do LCD:
            lcd.setCursor(0, 0); //Define a posição 0 da linha 0 para começar a escrever
            lcd.print("CANCELADO       "); //Escreve "PARADO" e apaga tudo na sua frente se houver 
            lcd.setCursor(0, 1); //Define a posição 0 da linha 1 para começar a escrever
            lcd.print("ESCOLHA FUNCAO  "); //Escreve "PARADO" e apaga tudo na sua frente se houver

            estado_menu = 0;

            break;

        }
    
  }  
}

// ============ Função que vai enviar as informações para o display de 7 seguimentos via I2C ============

void codigo_I2C(unsigned long codigo)
{
  Wire.beginTransmission(PCF8574_ADDR); 	// Inicia a comunicação com o PCF8574
  Wire.write(codigo); 				//Código em um byte transferido do mestre (microcontrolador) para o escravo PCF8574 para acionar corretamento os displays via interface I2C
  Wire.endTransmission(); 		//Interrompe a comunicação com o PCF8574
}

//Função que seleciona o display e define o que vai ser acionado
void aciona_display(int M, int Cen, int D, int U, int display)
{
  switch(display)
  {
    case 1:
    {
      //Escrevendo no display 1 (casa do milhar)
      switch(M)
      {
        case 0: //Dígito 0
      		codigo_I2C(0xe0); //0xe0 = 11100000 = P7|P6|P5|P4|P3|P2|P1|P0|
            break;
      	case 1: //Dígito 1
      		codigo_I2C(0xe1); //0xe1 = 11100001 = P7|P6|P5|P4|P3|P2|P1|P0|
            break;
      	case 2: //Dígito 2
      		codigo_I2C(0xe2); //0xe2 = 11100010 = P7|P6|P5|P4|P3|P2|P1|P0|
            break;
      	case 3: //Dígito 3
      		codigo_I2C(0xe3); //0xe3 = 11100011 = P7|P6|P5|P4|P3|P2|P1|P0|
            break;
      	case 4: //Dígito 4
      		codigo_I2C(0xe4); //0xe4 = 11100100 = P7|P6|P5|P4|P3|P2|P1|P0|
            break;
      	case 5: //Dígito 5
      		codigo_I2C(0xe5); //0xe5 = 11100101 = P7|P6|P5|P4|P3|P2|P1|P0|
            break;	     
      	case 6: //Dígito 6
      		codigo_I2C(0xe6); //0xe6 = 11100110 = P7|P6|P5|P4|P3|P2|P1|P0|
            break;
      	case 7: //Dígito 7
      		codigo_I2C(0xe7); //0xe7 = 11100111 = P7|P6|P5|P4|P3|P2|P1|P0|
            break;
      	case 8: //Dígito 8
      		codigo_I2C(0xe8); //0xe8 = 11101000 = P7|P6|P5|P4|P3|P2|P1|P0|
            break;
      	case 9: //Dígito 9
      		codigo_I2C(0xe9); //0xe9 = 11101001 = P7|P6|P5|P4|P3|P2|P1|P0|
            break;
      }
    }
    break;
    
    case 2:
    {
      //Seleciona o display 2 (casa da centena)
      switch(Cen)
      {
        case 0: //Dígito 0 
      		codigo_I2C(0xd0); //0xd0 = 11010000 = P7|P6|P5|P4|P3|P2|P1|P0|
            break;
      	case 1: //Dígito 1
      		codigo_I2C(0xd1); //0xd1 = 11010001 = P7|P6|P5|P4|P3|P2|P1|P0|
            break;
      	case 2: //Dígito 2
      		codigo_I2C(0xd2); //0xd2 = 11010010 = P7|P6|P5|P4|P3|P2|P1|P0|
            break;
      	case 3: //Dígito 3
      		codigo_I2C(0xd3); //0xd3 = 11010011 = P7|P6|P5|P4|P3|P2|P1|P0|
            break;
      	case 4: //Dígito 4
      		codigo_I2C(0xd4); //0xd4 = 11010100 = P7|P6|P5|P4|P3|P2|P1|P0|
            break;
      	case 5: //Dígito 5
      		codigo_I2C(0xd5); //0xd5 = 11010101 = P7|P6|P5|P4|P3|P2|P1|P0|
            break;	     
      	case 6: //Dígito 6
      		codigo_I2C(0xd6); //0xd6 = 11010110 = P7|P6|P5|P4|P3|P2|P1|P0|
            break;
      	case 7: //Dígito 7
      		codigo_I2C(0xd7); //0xd7 = 11010111 = P7|P6|P5|P4|P3|P2|P1|P0|
            break;
      	case 8: //Dígito 8
      		codigo_I2C(0xd8); //0xd0 = 11011000 = P7|P6|P5|P4|P3|P2|P1|P0|
            break;
      	case 9: //Dígito 9
      		codigo_I2C(0xd9); //0xd9 = 11011001 = P7|P6|P5|P4|P3|P2|P1|P0|
            break;
      }
    }
    break;
    
    case 3:
    {
      //Seleciona o display 3 (Casa da dezena)
      switch(D)
      {
        case 0: //Dígito 0
      		codigo_I2C(0xb0); //0xb0 = 10110000 = P7|P6|P5|P4|P3|P2|P1|P0|
            break;
      	case 1: //Dígito 1
      		codigo_I2C(0xb1); //0xb1 = 10110001 = P7|P6|P5|P4|P3|P2|P1|P0|
            break;
      	case 2: //Dígito 2
      		codigo_I2C(0xb2); //0xb2 = 10110010 = P7|P6|P5|P4|P3|P2|P1|P0|
            break;
      	case 3: //Dígito 3
      		codigo_I2C(0xb3); //0xb3 = 10110011 = P7|P6|P5|P4|P3|P2|P1|P0|
            break;
      	case 4: //Dígito 4
      		codigo_I2C(0xb4); //0xb4 = 10110100 = P7|P6|P5|P4|P3|P2|P1|P0|
            break;
      	case 5: //Dígito 5
      		codigo_I2C(0xb5); //0xb5 = 10110101 = P7|P6|P5|P4|P3|P2|P1|P0|
            break;	     
      	case 6: //Dígito 6
      		codigo_I2C(0xb6); //0xb6 = 10110110 = P7|P6|P5|P4|P3|P2|P1|P0|
            break;
      	case 7: //Dígito 7
      		codigo_I2C(0xb7); //0xb7 = 10110111 = P7|P6|P5|P4|P3|P2|P1|P0|
            break;
      	case 8: //Dígito 8
      		codigo_I2C(0xb8); //0xb8 = 10111000 = P7|P6|P5|P4|P3|P2|P1|P0|
            break;
      	case 9: //Dígito 9
      		codigo_I2C(0xb9); //0xb9 = 10111001 = P7|P6|P5|P4|P3|P2|P1|P0|
            break;
      }
    }
    break;
    
    case 4:
    {
      //Seleciona o display 4 (Casa da unidade)
      switch(U)
      {
        case 0: //Dígito 0
      		codigo_I2C(0x70); //0x70 = 01110000 = P7|P6|P5|P4|P3|P2|P1|P0|
          codigo_I2C(0xF0); //0xf0  = 11110000 = P7|P6|P5|P4|P3|P2|P1|P0| => Colocamos P7-P4 em 1 = Desativar todos os displays
            break;
      	case 1: //Dígito 1
      		codigo_I2C(0x71); //0x71 = 01110001 = P7|P6|P5|P4|P3|P2|P1|P0|
          codigo_I2C(0xF0); //0xf0  = 11110000 = P7|P6|P5|P4|P3|P2|P1|P0| => Colocamos P7-P4 em 1 = Desativar todos os displays
            break;
      	case 2: //Dígito 2
      		codigo_I2C(0x72); //0x72 = 01110010 = P7|P6|P5|P4|P3|P2|P1|P0|
          codigo_I2C(0xF0); //0xf0  = 11110000 = P7|P6|P5|P4|P3|P2|P1|P0| => Colocamos P7-P4 em 1 = Desativar todos os displays
            break;
      	case 3: //Dígito 3
      		codigo_I2C(0x73); //0x73 = 01110011 = P7|P6|P5|P4|P3|P2|P1|P0|
          codigo_I2C(0xF0); //0xf0  = 11110000 = P7|P6|P5|P4|P3|P2|P1|P0| => Colocamos P7-P4 em 1 = Desativar todos os displays
            break;
      	case 4: //Dígito 4
      		codigo_I2C(0x74); //0x74 = 01110100 = P7|P6|P5|P4|P3|P2|P1|P0|
          codigo_I2C(0xF0); //0xf0  = 11110000 = P7|P6|P5|P4|P3|P2|P1|P0| => Colocamos P7-P4 em 1 = Desativar todos os displays
            break;
      	case 5: //Dígito 5
      		codigo_I2C(0x75); //0x75 = 01110101 = P7|P6|P5|P4|P3|P2|P1|P0|
          codigo_I2C(0xF0); //0xf0  = 11110000 = P7|P6|P5|P4|P3|P2|P1|P0| => Colocamos P7-P4 em 1 = Desativar todos os displays
            break;	     
      	case 6: //Dígito 6
      		codigo_I2C(0x76); //0x76 = 01110110 = P7|P6|P5|P4|P3|P2|P1|P0|
          codigo_I2C(0xF0); //0xf0  = 11110000 = P7|P6|P5|P4|P3|P2|P1|P0| => Colocamos P7-P4 em 1 = Desativar todos os displays
            break;
      	case 7: //Dígito 7
      		codigo_I2C(0x77); //0x77 = 01110111 = P7|P6|P5|P4|P3|P2|P1|P0|
          codigo_I2C(0xF0); //0xf0  = 11110000 = P7|P6|P5|P4|P3|P2|P1|P0| => Colocamos P7-P4 em 1 = Desativar todos os displays
            break;
      	case 8: //Dígito 8
      		codigo_I2C(0x78); //0x78 = 01111000 = P7|P6|P5|P4|P3|P2|P1|P0|
          codigo_I2C(0xF0); //0xf0  = 11110000 = P7|P6|P5|P4|P3|P2|P1|P0| => Colocamos P7-P4 em 1 = Desativar todos os displays
            break;
      	case 9: //Dígito 9
      		codigo_I2C(0x79); //0x79= 01111001 = P7|P6|P5|P4|P3|P2|P1|P0|
          codigo_I2C(0xF0); //0xf0  = 11110000 = P7|P6|P5|P4|P3|P2|P1|P0| => Colocamos P7-P4 em 1 = Desativar todos os displays
            break;
       }
  	}
    break;
	}
}