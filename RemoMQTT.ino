//------Código do Esp para projeto do Remo------//

//----------------------------------------------------------------//

//-----------RÓTULO DOS PINOS PARA NÃO ESQUECER-----------//
//------SENSOR-------ESP------//
//-------VCC    ->   VIN------//
//-------GND    ->   GND------//
//-------SCL    ->   D22------//
//-------SDA    ->   D21------//

//---Bibliotecas---//
#include "Adafruit_VL53L0X.h" //Biblioteca de comunicação com o sensor VL53L0X
#include <WiFi.h>             //Biblioteca para conexão Wifi
#include <PubSubClient.h>     //Bibliotexa para conexão com servidor MQTT

//----------------------------------------------------------------//

//---Informações da rede---//
const char* ssid = "Teste";     //Domínio de Wifi
const char* pass = "12345678";//Senha do Wifi

//----------------------------------------------------------------//
/*
#define HOST "10.0.0.211"
#define PORT 80
#define PATH "/ws"
*/
//---Informações para conectar no MQTT do ThingSpeak---//
//const char* clientId = "HjYXDTcQNhY2OzQcKx0YGBo";         //Id do dispositivo MQTT
const char* username = "HjYXDTcQNhY2OzQcKx0YGBo";         //Usuário do dispositivo MQTT
const char* password = "hChdo9lTnAi4w1hTv2izOzDg";        //Senha do dispositivo MQTT

const char* server = "192.168.41.226";              //Servidor do ThingSpeak
const char* channelIdSub = "channels/2679075/subscribe";  //Ip do canal para se inscrever e receber informações (canal Remo_Esp)
const char* channelIdPub = "channels/2679076/subscribe";  //Ip do canal para se publicar informações (canal Esp_Remo)
//----------------------------------------------------------------//

//---Inicializando objetos de rede---//
WiFiClient net;   //Cliente do Wifi
PubSubClient client(net);//Cliente MQTT

//----------------------------------------------------------------// 

//---Declarando sensor de distancia---//
Adafruit_VL53L0X distSensor = Adafruit_VL53L0X();

//---Macros---//
const int mqttPort = 1883;//Porta de rede MQTT (não é a porta segura)
float valor = 0;
int contagem = 0;
unsigned long lastMillis = 0;
const char* V;
const int tresholdMin = 550;
const int tresholdMax = 740;
bool isClose = true;
//----------------------------------------------------------------//
int i = 0;
char str[8];
//---Funções---//

//--Conecta ao Wifi--//
void conectaWifi(){
  Serial.println("Tentando Conectar ao WiFi");                               
  while(WiFi.status() != WL_CONNECTED){       //Enquanto o Wifi não estiver conectado
    WiFi.begin(ssid, pass);                   //Tenta inicializar o Wifi
    Serial.print('.');                        //Imprime para assegurar a funcionalidade do código
    delay(5000);                              //Tempo para esperar conexão
  }
  Serial.println("");
  Serial.println("Conectado ao WiFi");        //Quando conectar imprime que está conectado
}

//--Conecta o cliente ao MQTT--//
void conectaMQTT(){                                 
  if(WiFi.status() != WL_CONNECTED){          //Verifica se está conectado ao Wifi
    conectaWifi();                            //Se não estiver conectado executa //--conectaWifi--//
  }
  Serial.println("Tentando Conectar ao MQTT");
  while (!client.connected()) {
    String client_id = "esp32-client-";
    client_id += String(WiFi.macAddress());
    Serial.printf("The client %s connects to the public MQTT broker\n", client_id.c_str());
    if (client.connect(client_id.c_str(), username, password)) {
      Serial.println("Public EMQX MQTT broker connected");
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }
  Serial.println("");
  Serial.println("Conectado ao MQTT");
}

//--Se inscreve no canal--//
void subscribeToChannel(){
  if(!client.connected()){                    //Verifica se o cliente MQTT está conectado
    conectaMQTT();                            //Se não estiver executa //--conectaMQTT--//
  }
  Serial.print("Tentando se inscrever no canal");
  Serial.println(channelIdSub); 
  while(!client.subscribe(channelIdSub)){     //Tenta se inscrever no canal, enquanto a inscrição retornar falso
    Serial.print(".");                        //Imprime para assegurar a funcionalidade do código
    delay(1000);                              //Tempo para esperar conexão
  }
  Serial.println("");
  Serial.print("Inscrito ao canal: ");        //Quando se inscrever, imprime que está inscrito
  Serial.println(channelIdSub); 
}

//--Mensagem Recebida--//
void callback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Message arrived in topic: ");
    Serial.println(topic);
    Serial.print("Message:");
    for (int i = 0; i < length; i++) {
        Serial.print((char) payload[i]);
    }
    Serial.println();
    Serial.println("-----------------------");  
}

//--Conectar--//
void conectaSensor(){
  Serial.println("Tentando Conectar ao Sensor");
  while (!distSensor.begin()) {
    Serial.print(F("."));
    delay(1000);
  }
  Serial.println("");
  Serial.println("Conectado ao sensor");
  distSensor.startRangeContinuous();
}

//----------------------------------------------------------------//

//---Setup---//
void setup() {
  Serial.begin(115200);                             //Inicializa a comunicação serial
  WiFi.begin(ssid, pass);                           //Inicializa o Wifi com o ssid e pass
  conectaWifi();
  //client.begin(server, net);
  client.setServer(server,mqttPort);
  client.setCallback(callback);
  //client.subscribe(channelIdPub);
  //subscribeToChannel();                             //Executa //--subscribeToChannel--// que utiliza os outros códigos de conexão
  conectaSensor();
}

//----------------------------------------------------------------//

//--Loop--//
void loop() {
  //--Mantem uma conecão estável--//
  client.loop();  //Mantem a conexão MQTT
  delay(10);      // <- fixes some issues with WiFi stability  
  //----------------------------------------------------------------//
  //--Verifica se todos os componente de conexão estão funcionando--//
//--Verifica se todos os componente de conexão estão funcionando--//
    if(!client.connected()){
      //delay(5000);
      //Caso haja falha de algum dos componentes de conexão
      Serial.println("Perda de conexão com servidor");                    //Imprime que ouve falha de conexão
      conectaMQTT();
    }
  if(!distSensor.begin()){
    Serial.println("Perda de conexão com sensor");
    conectaSensor();
  }
  //----------------------------------------------------------------//
  
  if(distSensor.isRangeComplete() && contagem < 3) {
    //Serial.println("Lendo valor do sensor");
    valor += distSensor.readRange();
    contagem += 1;
  }
  
  //----------------------------------------------------------------//
  if(contagem == 3){
    valor = valor/contagem;
    String a = String(valor,2);
    contagem = 0;
    Serial.println("Valor no sensor: "+a);
    if(valor > tresholdMax && isClose){
      isClose = false;
      Serial.println("isClose = false");
    }
    if(valor < tresholdMin && !isClose){
      isClose = true;
      client.publish(channelIdPub, "1");
      lastMillis = millis();
      Serial.println("isClose = true");
    }

    valor = 0;
  }

  if (millis() - lastMillis > 5000){
    lastMillis = millis();
    client.publish(channelIdPub, "ping");
  }
  // put your main code here, to run repeatedly:
  /*
  if (millis() - lastMillis > 200 && contagem==5) {
    valor = valor/contagem;
    String a = String(valor,2);
    V = a.c_str();
    client.publish(channelIdPub, V);
    delay(50);
    lastMillis = millis();
    contagem = 0;
    Serial.println("Valor enviado: "+a);
    Serial.println("____________________________________________________");
    //delay(5);
    valor=0;
  }
  */
}
