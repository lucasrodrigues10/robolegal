 int trigger = 7;
int echo = 6;
double tempo;
double distancia_cm;

//motor A - esquerda
int velocidade_esquerda = 3; //eh tio
int IN1 = 2 ; //nao eh tio
int IN2 = 4 ;// nao eh tio
 
//motor B - direita
int IN3 = 9 ;
int IN4 = 10 ;
int velocidade_direita = 11;

//variavel auxiliar
int velocidade = 0;
 
unsigned long previousMillis = 0; 
const long interval = 100;   

void setup() {
  Serial.begin(9600);

  // configura pino TRIGGER como saída
 pinMode(trigger,OUTPUT);
 // deixa pino em LOW
 digitalWrite(trigger,LOW);
 delayMicroseconds(10);
 // configura pino ECHO como entrada
 pinMode(echo,INPUT);
  
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(velocidade_esquerda,OUTPUT);
  pinMode(velocidade_direita,OUTPUT);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);  
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW); 

/*
  analogWrite(velocidade_esquerda, 200);
  analogWrite(velocidade_direita, 200);
  delay(6000);
  analogWrite(velocidade_esquerda, 0);
  analogWrite(velocidade_direita, 0);
  */
  
}

void andarFrente(double valor){
  valor *= 255;
  if(valor < 0){
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);  
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    valor *= -1;
  }else{
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);  
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW); 
  }
  analogWrite(velocidade_esquerda, valor);
  analogWrite(velocidade_direita, valor);
  
}


double medirDistancia(){
  // disparar pulso ultrassônico
 digitalWrite(trigger, HIGH);
 delayMicroseconds(10);
 digitalWrite(trigger, LOW);

 // medir tempo de ida e volta do pulso ultrassônico
 tempo = pulseIn(echo, HIGH);
 // calcular as distâncias em centímetros e polegadas
 distancia_cm = tempo / 29.4 / 2;
  return distancia_cm;
}
int i = 0;

void loop() {
  unsigned long currentMillis = millis();
  
   
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    Serial.print("sen[");
    Serial.print(i++);
    Serial.print("]: ");
    Serial.println(sin(i*0.2));
    Serial.print("Distancia: ");
    Serial.println(medirDistancia());
    andarFrente(sin(i*0.8));

    
  }
  

  


  
  }
