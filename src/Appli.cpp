
#include <Arduino.h>
#include <ctype.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
v1.4 08/2026 nouveau core, trame temp variable, messages en queue
v1.3 05/2026 board upesy, humidite absolue, envoi vers serveur
v1.2 03/2026 Surveillance batterie, log 24h en eeprom, OTA à la demande
v1.1 03/2026 copie de plat_esp_chad_gar v1.11 de 3/2026
*/

#include "variables.h"
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <Preferences.h>  // pour nvs eeprom
#include <PID_v1.h>
#include <DHT.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include "ClosedCube_HDC1080.h"
#include "Wire.h"

extern WiFiClient client;
extern Preferences preferences_nvs;  // Déclaration externe

RTC_NOINIT_ATTR uint32_t envoi_6s_prec=0, Mesure_6s_prec;
RTC_NOINIT_ATTR uint32_t Tick_6s;
RTC_NOINIT_ATTR float TintPrec=20;


// variables Detection PIR
RTC_NOINIT_ATTR uint16_t compteur_detection=0;
RTC_NOINIT_ATTR uint16_t compteur_detection_1h=0;

RTC_NOINIT_ATTR uint8_t pause_detection;
RTC_NOINIT_ATTR unsigned long last_detection_time=0;

RTC_NOINIT_ATTR uint16_t Nb_PI[NB_VAL_TAB];

RTC_NOINIT_ATTR uint8_t  WIFI_CHANNEL;
RTC_NOINIT_ATTR uint8_t etat_now;
RTC_NOINIT_ATTR uint16_t Seuil_batt_sonde;  // millivolt
RTC_NOINIT_ATTR uint8_t Nb_jours_Batt_log;
RTC_NOINIT_ATTR uint8_t freq_envoi, cpt_mesure, cpt_nb_val;
RTC_NOINIT_ATTR uint16_t valTemp[NB_VAL_TAB], valHum[NB_VAL_TAB];
RTC_NOINIT_ATTR uint32_t valTick[NB_VAL_TAB];

RTC_NOINIT_ATTR uint8_t compteur_graph;
RTC_NOINIT_ATTR uint16_t compteur_24h;

RTC_NOINIT_ATTR float PIR_24h=0;
RTC_NOINIT_ATTR uint16_t cpt24_PIR=0, PIRV=0;

RTC_NOINIT_ATTR uint8_t Capt_tps_max;
RTC_NOINIT_ATTR uint8_t Capt_seuil_temp;
RTC_NOINIT_ATTR uint8_t Capt_nb_val_max;
RTC_NOINIT_ATTR uint16_t Capt_tps_total_max;
RTC_NOINIT_ATTR uint8_t delai_detection;

RTC_NOINIT_ATTR uint8_t etat_ESP_stop;  // 0:normal 1:arrêté pour batterie faible (deepsleep longue duree)
RTC_NOINIT_ATTR uint8_t cpt24h_batt;


volatile uint8_t ackReceived = false;  // global pour indiquer que le peer a acké
volatile int ackChannel = -1;       // canal où ça a marché
RTC_NOINIT_ATTR uint8_t num_sequentiel;  // pour Ack

extern uint16_t nb_err_reseau;
extern  uint16_t TextV, TintV, HumV, HAV;  // pour stockage dans la partition log_flashG
extern volatile uint8_t ackExpectedSequence;
extern TimerHandle_t gatewayQueueTimer;


void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len);
//void OnDataRecv(const uint8_t *mac, const uint8_t *data, int len);
//void OnDataRecv(const esp_now_peer_info_t * info, const uint8_t *incomingData, int len);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  void OnDataSent(const wifi_tx_info_t* info, esp_now_send_status_t status);
#else
  void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status);
#endif

uint8_t parseMacString(const char* str, uint8_t mac[6]);
float absoluteHumidity(float temperature, float relativeHumidity);



#ifdef Temp_int_DS18B20
  OneWireNg_CurrentPlatform ow(PIN_DS18B20, false);
  OneWireNg_DS18B20 sensor(&ow);

  // Capteur temperature Dallas DS18B20  Temperature intérieure
  typedef uint8_t DeviceAddress[8];
  const int PIN_Tint = 13;      // Tint:Entrée onewire GPIO DS18B20
  //OneWire oneWire(PIN_Tint);
  //DallasTemperature ds(&oneWire);
  int nb_capteurs_temp = 1;  //DS18B20
  DeviceAddress Thermometer[5];
  DeviceAddress adds;
#endif

DHT dht[] = {
  { PIN_Tint22, DHT22 },
};
#ifdef Temp_int_HDC1080
  ClosedCube_HDC1080 hdc1080;
#endif

// Temperature intérieure
float Tint, Text, Humid;
RTC_NOINIT_ATTR uint16_t err_Tint, err_Text, err_Heure;  // compteurs d'erreurs

RTC_NOINIT_ATTR uint16_t calib_hygro1=300, calib_hygro2=800, calib_temp=1000; // calibration hygrométrie HDC1080



// ---------FONCTIONS DEFINIES AILLEURS -----------------

int readLastLogsG(int nombre);



// ----------  FONCTIONS APPLI --------------

void init_10_secondes()
{
}

//setup au debut
void setup_0()
{

   BTN_PIN[0] = PIN_REVEIL;
   BTN_PIN[1] = PIN_REVEIL2;

  #ifdef ESP32_uPesy
    pinMode(PIN_Vbatt, INPUT);
  #endif

  /*if (NB_Graphique==6)
  {
    graphique[0][0] = 180;  //Tint - vert
    graphique[1][0] = 185;
    graphique[2][0] = 190;

    graphique[0][1] = 110;  // Text - bleu
    graphique[1][1] = 80;
    graphique[2][1] = 103;
    graphique[3][1] = 95;

    graphique[0][2] = 150;  // Chaud
    graphique[1][2] = 150;
    graphique[2][2] = 200;
    graphique[3][2] = 200;

    graphique[0][3] = 185;  // Tint moy
    graphique[1][3] = 183;
    graphique[2][3] = 183;
    graphique[3][3] = 195;

    graphique[0][4] = 35;   // Text moy
    graphique[1][4] = 38;
    graphique[2][4] = 42;
    graphique[3][4] = 32;

    graphique[0][5] = 50;  // cout
    graphique[1][5] = 55;
    graphique[2][5] = 48;  
    graphique[3][5] = 52;
  }*/
}



// setup apres la lecture nvs, avant démarrage reseau
void setup_1()
{

    Tint = 15;
    #ifdef Temp_int_HDC1080
      //delay(200);
      //i2cBootRecovery();
      Wire.begin(PIN_SDA, PIN_SCL); // Forçage des pins SDA=8, SCL=9 pour ESP32 S3 DevKit V1

      hdc1080.begin(0x40);
      /*if (i2cDevicePresent(0x40)) {
        Serial.println("HDC1080 détecté");
        hdc1080.begin(0x40);
      } else {
        Serial.println("HDC1080 ABSENT");
      }*/
    #endif

  // initialisation capteur de température intérieur
    #ifdef Temp_int_DHT22
      dht[0].begin();
    #endif
 
 
    #ifdef Temp_int_DS18B20
      ds.begin();  // Startup librairie DS18B20
      nb_capteurs_temp = ds.getDeviceCount();
      Serial.print("Nb Capteurs DS18B20:");
      Serial.println(nb_capteurs_temp);
      if (nb_capteurs_temp > 1) nb_capteurs_temp = 1;
      int j;
      for (j = 0; j < nb_capteurs_temp; j++) {
        Serial.print(" Capteur :");
        ds.getAddress(Thermometer[j], j);
        printAddress(Thermometer[j]);
      }
    #endif

    // lecture initiale temperature interieure
    /*uint8_t Tint_err = lecture_Tint(&Tint);
    if ((Tint < 1) || (Tint > 45)) {
      Tint = 20.0;
      Tint_err = 7;
    }
    if (Tint_err) log_erreur(Code_erreur_Tint, Tint_err, 1);
    else
      Serial.printf("Temp int:%.2f\n\r", Tint);*/

}

// apres demarrage reseau
void setup_2()
{


  #ifdef ESP_TJ_ACTIF

    // lecture des données sauvegardées dans la partition log_flashG
    readLastLogsG(99);

    // Configuration WiFi en mode Station pour ESP-NOW

    if ((mode_reseau==13) )
      WiFi.mode(WIFI_STA);
    
    // 🔍 DIAGNOSTIC: Forcer le canal WiFi
    uint8_t current_channel;
    wifi_second_chan_t second;
    esp_wifi_get_channel(&current_channel, &second);
    Serial.printf("Canal WiFi AVANT config ESP-NOW: %d\n\r", current_channel);
    
    // Forcer le canal si nécessaire (doit correspondre au routeur)
    // esp_wifi_set_promiscuous(true);
    // esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    // esp_wifi_set_promiscuous(false);
    
    if (esp_now_init() != ESP_OK) {
      Serial.println("Erreur initialisation ESP-NOW");
      return;
    }
    //if ((mode_reseau==13) )
    //else
    //  Serial.println(WiFi.softAPmacAddress());
    Serial.println(WiFi.macAddress());

    // Stockage de l'adresse MAC dans le tableau mac_gw[6]
    Serial.printf("   MAC dest : %02X:%02X:%02X:%02X:%02X:%02X\n\r",
            mac_gw[0], mac_gw[1], mac_gw[2],
            mac_gw[3], mac_gw[4], mac_gw[5]);


    Serial.printf("   Canal WiFi: %d\n\r", current_channel);
    Serial.println("   En attente de messages...");
    Serial.println("======================================\n\r");
    delay(2000); // 2 secondes de pause pour lire

  #else  // Si veille
      readLastLogsG(99);

  #endif
}

void setup_3()
{
}

void traitement_recalage_espnow(uint8_t node)
{
}

uint8_t setup_appli()
{
  uint8_t continue_setup=1;  // continue le setup après return sauf si 0

  if (!rtc_valid)  // initialisation si perte d'alim
  {
    last_wifi_channel = WIFI_CHANNEL;  // prochain canal WiFi à utiliser après perte d'alim
    Serial.printf("Prochain canal WiFi à utiliser après perte d'alim: %d\n\r", last_wifi_channel);
  }
  
  // -------------  Capteur/Detecteur : stockage ou envoie infos à la gateway -------------------

  if (type_reveil ==2)   // Detection PIR
  {
    detection_pir();
    uint64_t sleep_time = (uint64_t)periode_cycle * 60 * 1000000;
    if (mode_rapide==12)
    sleep_time = (uint64_t)periode_cycle * 1000000;
    passage_deep_sleep( sleep_time); // 30ULL * 1000000ULL);
  }
  else 
  {
    if (type_reveil == 1)  {
      event_cycle(); // réveil par timer
      continue_setup=0;  // ne continue pas le setup après réveil par timer
    }
    else 
    {
      if (log_detail>=2) Serial.printf("milli H1: %lu\n\r", millis());
      envoi_temp_hygro();  // 30ms
       if (log_detail>=2) Serial.printf("milli H3: %lu\n\r", millis());
    }
    uint8_t etat_BTN0 = digitalRead(BTN_PIN[0]);  // repos-pull_up => 0, appui => 1
    if (log_detail>=2)Serial.printf("Etat BTN0: %i  type_reveil:%i\n\r", etat_BTN0, type_reveil);
    if ((type_reveil != 4) || (!etat_BTN0))  // reveil par BTN0 encore appuyé => pas sleep
    {
      if (gatewayQueueTimer != NULL && xTimerIsTimerActive(gatewayQueueTimer) != pdFALSE)
      {
          if (log_detail>=3) Serial.println("Le timer Envoi_now tourne encore => pas de veille");
      }
      else  // 
      {
        uint64_t sleep_time = (uint64_t)periode_cycle * 60 * 1000000;
        if (mode_rapide==12)
        sleep_time = (uint64_t)periode_cycle * 1000000;
        passage_deep_sleep( sleep_time); // 30ULL * 1000000ULL);
      }
    }
  }
  return continue_setup;
}

void setupRoutes_appli()
{

}

void init_rtc_variables_appli()  // initialisation si perte d'alim
{
  cpt24h_batt=6;   // compteur de jours - permet d'éviter d'attendre 6 jours avant le premier log batterie
  num_sequentiel=0;
  cpt_nb_val=0;
  Tick_6s=0;
  Mesure_6s_prec=0;
  envoi_6s_prec=0;
  compteur_24h=0; 
  TintPrec=0;

}

void init_ram_variables_appli()  // initialisation à chaque démarrage/reset/réveil
{
}


void appli_event_on(systeme_eve_t evt)
{
  Serial.printf("bouton off-Evenement on : %i\n\r", evt.data);

  if (evt.data == 1)  // bouton 1 relaché
  {
      //envoi_temp_hygro();
  }
}

void detection_pir()
{
    compteur_detection++;  // nb de detection des 5 dernières minutes
    compteur_detection_1h++; // nb de detection de la dernière heure
    if (compteur_detection_1h == 1) // premiere detection du cyle 1h
    {
      writeLog('D', 1, 0, 0, "PIR");
      // envoi d'un sms par http pour prevenir d'une presence
    }
}


// log batterie et Temp moyenne toutes les 24h.
void enreg_24h( uint8_t veille)
{
    // Log toutes les jours/semaines : nb d'erreurs wifi et batteries
  float vbatt = readBatteryVoltage();
  if (log_detail>=1) Serial.printf("24h - Nb jours batt log: %i vbatt:%.2f\n\r", Nb_jours_Batt_log, vbatt);

  if (Nb_jours_Batt_log)
  {
    if (log_detail>=3) Serial.printf("Tension Batterie : %.2f V\n\r", vbatt);

    if ((Seuil_batt_arret_ESP) && (vbatt < Seuil_batt_arret_ESP) && (vbatt > 2400))
    {
      writeLog('S', 7, 0, 0, "Batt Stp");
      etat_ESP_stop = 1;
    }
    cpt24h_batt++;
    if (cpt24h_batt >= Nb_jours_Batt_log)  // log chaque X jour 
    {
      cpt24h_batt=0;

      //Serial.printf("Tension Batterie 24h : %.2f %.2f V\n\r", vbatt, Vbatt_Th);
      if (veille)
      {
        writeLog('K', (uint8_t)((vbatt-2.0)*100), 0, 0, "24H_Res");
      }
      else
      {
        if (nb_err_reseau>255) nb_err_reseau=255;
        writeLog('K', (uint8_t)((vbatt-2.0)*100), 0, (uint8_t)nb_err_reseau, "24H_Res");
        nb_err_reseau=0;
      }
    }
  }

  // erreurs Tint, Text, heure
  if (err_Tint>254) err_Tint=255;
  if (err_Tint)
            log_erreur(Code_erreur_Tint, err_Tint,0);
  if (err_Text>254) err_Text=255;
  if (err_Text)
            log_erreur(Code_erreur_Text, err_Text,0);
  if (err_Heure>254) err_Heure=255;
  if (err_Heure)
            log_erreur(Code_erreur_Heure, err_Heure,0);
  err_Tint=0;
  err_Text=0;
  err_Heure=0;

  // graphique des temperatures quotidiennes
  uint16_t tempI=1, tempE=1, Hum=1, HA=1, PIR=1;
  if (cpt24_Tint)  tempI = (uint16_t)(tempI_moy24h/cpt24_Tint*10);
  Serial.printf("tempI: %i cpt24_Tint: %i Temp24h I:%.2f \n\r", tempI, cpt24_Tint, tempI_moy24h);
  if (cpt24_Text)  tempE = (uint16_t)(tempE_moy24h/cpt24_Text*10);
  if (cpt24_Hum) Hum = (uint16_t)(Hum_24h/cpt24_Hum*10);
  if (cpt24_HA) HA = (uint16_t)(HA_moy24h/cpt24_HA*10);
  if (cpt24_PIR) PIR = (uint8_t)(PIR_24h/cpt24_PIR*10);
  if (!tempI) tempI=1;  // permet d'afficher quand meme le point sur le graphique
  if (!tempE) tempE=1;  // permet d'afficher quand meme le point sur le graphique
  if (!Hum) Hum=1;  // permet d'afficher quand meme le point sur le graphique
  if (!HA) HA=1;  // permet d'afficher quand meme le point sur le graphique
  if (!PIR) PIR=1;  // permet d'afficher quand meme le point sur le graphique
  TextV = tempE;
  TintV = tempI;
  HumV = Hum;
  HAV = HA;
  PIRV = PIR;

  //Serial.printf("Temp24h I:%.2f %i E:%.2f %i C:%.2f %i\n\r", tempI_moy24h, cpt24_Tint, tempE_moy24h, cpt24_Text, cout_moy24h, cpt24_Cout);

  tempI_moy24h=0;
  tempE_moy24h=0;
  Hum_24h=0;
  HA_moy24h=0;
  PIR_24h=0;
  cpt24_Tint=0;
  cpt24_Text=0;
  cpt24_Hum=0;
  cpt24_HA=0;
  cpt24_PIR=0;

  uint8_t i;
  for (i = NB_Val_Graph - 1; i; i--) {
    graphique[i][3] = graphique[i - 1][3];
    graphique[i][4] = graphique[i - 1][4];
    graphique[i][5] = graphique[i - 1][5];
  }
  graphique[0][3] = tempI;
  graphique[0][4] = selec_graph(GRAPH2, HA, Hum, PIR);
  graphique[0][5] = selec_graph(GRAPH3, HA, Hum, PIR);  

  if (log_detail>=1) Serial.printf("Graphique 24h : Tint:%i Text:%i Hum:%i HA:%i\n\r", tempI, tempE, Hum, HA);
  writeLogG('G', tempI, HA, Hum); // Enregistrment en Flash des 3 valeurs du graphique

  // Tick, Temp, Hum, Vbatt
  if (action_envoi && esp_now_actif)
  {
    Message_EspNow message;

    message.destinataire = SERVER_ADD | 0x80;  // 0x80 = message hexa
    message.emetteur = add_node;
    message.code = 'C';
    message.code2 = 'J';
    num_sequentiel++;
    message.num_seq = num_sequentiel;

    uint8_t pos = 0;
    message.payload[pos++] = (uint8_t) Tick_6s;
    message.payload[pos++] = (uint8_t) (Tick_6s >> 8);
    message.payload[pos++] = Tick_6s >> 16;
    payloadWrite(message.payload, pos, tempI);
    payloadWrite(message.payload, pos, Hum);
    payloadWrite(message.payload, pos, (uint16_t)(vbatt*100));
    message.longueur = 3 + pos;

    uint8_t result = envoi_data_gateway(message);  // envoi chaque 24h des valeurs à la gateway

  }      
}


void appli_event_off(systeme_eve_t evt)
{
  // Detecteur PIR activé
  if (evt.data == 1)
  {
      envoi_temp_hygro();
      //detection_pir();
  }
  Serial.printf("Bouton on-Evenement off : %i\n\r", evt.data);
}

char* requete_status_appli(char *json_response, char *p, uint8_t type)
{
  p += sprintf(p, "\"PIR_D\":%i,", compteur_detection); // derniere 15min
  p += sprintf(p, "\"PIR_V\":%i,", PIRV); // veille
  return p;
}

// type 1
uint8_t requete_Get_appli(const char* var, float *valeur)
  //uint8_t requete_Get_appli (String var, float *valeur) 
{
  uint8_t res=1;

  if (strncmp(var, "Tint",5) == 0) {
    res = 0;
    *valeur = Tint;
  }
  if (strncmp(var, "Text",5) == 0) {
    res = 0;
    *valeur = Text;
  }
  if (strncmp(var, "codeR_pac",10) == 0) {
    res = 0;
    if (cpt_securite)  *valeur=1;
    else *valeur=0;
  }


  return res;
}




// type 1
uint8_t requete_Set_appli (String param, float valf) 
{
  uint8_t res=1;
  int8_t val = round(valf);

    /*if (param == "consigne")     // Forcage consigne, rajouter duree
    {
      if ((valf >= 6.0) && (valf <= 22.0))  // 6°C à 22°C
      {
          fo_co = round(valf * 10);
          //fo_jus = 10;  // en minutes
          //preferences_nvs.putUChar("Cons", Consigne_G);
          res = 0;
      }
    }*/

    /*if (param == "vbatt")
    {
      res = 0;
      Vbatt_Th = valf;
      Vbatt_Th_I = 1;

      Serial.printf("Réception Vbatt Distante : %.2fV\n\r", Vbatt_Th);
    }*/


  return res;
}

// type 2
uint8_t requete_GetReg_appli(int reg, float *valeur)
{
  uint8_t res=1;

  
  if (reg == 41)  // registre 41 : canal WiFi actuel (dynamic)
  {
    res = 0;
    uint8_t current_channel;
    wifi_second_chan_t second;
    esp_wifi_get_channel(&current_channel, &second);
    *valeur = (float)current_channel;
  }

  return res;
}

// type 2
uint8_t requete_SetReg_appli(int param, float valeurf)
{
  int16_t valeur = int16_t(round(valeurf));
  uint8_t res = 1;

  return res;
}



// type 4
uint8_t requete_Get_String_appli(uint8_t type, String var, char *valeur)
{
  uint8_t res=1;
  int paramV = var.toInt();
  // valeur limité a 50 caractères
  

  return res;
}



// type 4
uint8_t requete_Set_String_appli(int param, const char *texte)
{
  uint8_t res=1;
  IPAddress ip;

    if ((param == 61) && (strlen(texte) <= 20)) // registre 61 : adresse Mac Gateway
    {
      res = 0;
      preferences_nvs.putString("MacGW", texte);
      strncpy(mac_gw_str, texte, sizeof(mac_gw_str) - 1);
      mac_gw_str[sizeof(mac_gw_str) - 1] = '\0';  // Assure la terminaison
    
      if (!parseMacString(texte, mac_gw))
      {
          Serial.println("MAC Serveur invalide");
      }
        else 
          if (log_detail>=3) 
          {
            Serial.printf("MAC Serveur set : %X:%X:%X:%X:%X:%X\n\r", mac_gw[0], mac_gw[1], mac_gw[2], mac_gw[3], mac_gw[4], mac_gw[5]);
            Serial.printf("Mac texte : %s\n\r", mac_gw_str);
          }
      /*else
      {
        preferences_nvs.putString("MacC", texte);
        res = 0;
      }*/
    }

  return res;
}

// type5 : reception message ACTION par uart ou par page web
uint8_t requete_action_appli(const char *reg, const char *data)
{
  uint8_t res=1;

  if (strcmp(reg, "Test1") == 0) 
    { 
      res=0; 
      requete_status(buffer_dmp, 0, 1);
      Serial.println(buffer_dmp);
    }

  if (strcmp(reg, "Tint") == 0) 
    { 
      res=0; 
      uint8_t Tint_erreur = lecture_Tint(&Tint,&Humid);
      Serial.println(Tint_erreur);
      Serial.println(Tint);
    }
  return res;
}


// erreur :0:ok  sinon erreur 2 à 7
uint8_t lecture_Tint(float *mesure, float*humid)
{
  uint8_t Tint_erreur = 7;
  float valeur = 20;
  float valeur2 = 50;


    #ifdef Temp_int_DHT22
      //dht[0].begin();

      if (digitalRead(PIN_Tint22) == HIGH || digitalRead(PIN_Tint22) == LOW)
      {
        valeur =  dht[0].readTemperature();
        if (isnan(valeur))
        {
          valeur = 20.0;
          Tint_erreur = 6;
          Serial.println("---DHT:non numérique");
        }
        else
        {
          Tint_erreur=0;
        }
      }
      else
        Serial.println("---DHT:signal non stable!");
    #endif

    #ifdef Temp_int_HDC1080
      valeur = hdc1080.readTemperature();
      if (isnan(valeur) || (valeur>124)) {
        Serial.println("Reset i2c)");
        resetI2C(); 
        hdc1080.begin(0x40); 
        valeur = hdc1080.readTemperature();

        if (isnan(valeur) || (valeur>124)) {
          Serial.println("Recovery i2c)");
          i2cRecovery();
          hdc1080.begin(0x40); 
          valeur = hdc1080.readTemperature();
          if (isnan(valeur) || (valeur>124)) {
            valeur = 20.0;
            Tint_erreur = 4;
          } else  Tint_erreur=0;
        } else  Tint_erreur=0;

      } else {
        Tint_erreur=0;
      }
      valeur2 = hdc1080.readHumidity();
      if (log_detail>=3) Serial.printf("lecture HDC1080 - Temp: %.2f Humid:%.2f\n\r", valeur, valeur2);
      valeur = valeur + (float)(calib_temp-1000)/100.0;  // calibration temperature
      valeur2 = (valeur2 - (float)calib_hygro1/10.0) * (80.0-30.0) / ((float)calib_hygro2/10.0 - (float)calib_hygro1/10.0) + 30.0;  // calibration hygrométrie
      if (log_detail>=3) Serial.printf("lecture HDC1080M- Temp: %.2f Humid:%.2f\n\r", valeur, valeur2);

    #endif

    #ifdef Temp_int_DS18B20
      valeur = ds.getTemperature();
      Tint_erreur=0;
    #endif


  if (valeur > 50) Tint_erreur = 2;
  if (valeur < -20) Tint_erreur = 3;
  if (log_detail>=3) Serial.printf("lecture Tint : %.2f Humid:%.2f Tint_erreur:%i\n\r", valeur, valeur2, Tint_erreur);
  if (Tint_erreur) {
    valeur = 20.0;
    valeur2 = 50.0;
  }
  *mesure = valeur;
  *humid = valeur2;
  return Tint_erreur;
}



//mesure temperature exterieure
uint8_t lecture_Text(float *mesure) {
  uint8_t Text_erreur = 0;
  int16_t Val_Text = 1600;
  float valeur;

  #ifdef MODBUS
    Text_erreur = read_modbus(2, &Val_Text);  // registre 1-2-3 pour temp exterieure
    valeur = (float)Val_Text / 10;
  #else
    valeur = 18.0;

    /*#ifndef DEBUG_SANS_Sonde_Ext
        Val_Text = analogRead( PIN_Text );  // 0 à 4096
        //Serial.println(Val_Text);
      #endif*/
    // calibration
    // Text1:100(10°C) Text1Val:500
    // Text2:200(20°C) Text2Val:2000
    //valeur = ((float)(Text1Val-Val_Text)/(Text1Val-Text2Val)*(Text2-Text1) + Text1)/10;

    /*float Vmesure = ((float)Val_Text / resolutionADC) * 3.66;
      float Rntc = 15000 * Vmesure / (3.3 - Vmesure);  // Calcul de la résistance de la thermistance
      float T_kelvin = 1.0 / ((1.0 / 298.15) + (1.0 / TBeta) * log(Rntc / Therm0));    // Calcul de la température en Kelvin
      valeur = T_kelvin - 273.15;    // Conversion en °C */
    //Serial.printf("val_text:%i vmesure:%.3f rntc:%.0f T_kelvin:%.1f valeur:%.1f\n\r", Val_Text, Vmesure, Rntc, T_kelvin, valeur);
  #endif

  if ((valeur < -30.0) || (valeur > 60.0)) Text_erreur = 1;
  if (!Val_Text) Text_erreur = 2;

  *mesure = valeur;
  return Text_erreur;
}



uint8_t fetch_internet_temp() {

  uint8_t res=1;

  // 1. Vérifier si le réseau est disponible avant de commencer
  if (WiFi.status() != WL_CONNECTED) {
    // Si vous utilisez l'Ethernet, remplacez par le test approprié
    return res; 
  }

  HTTPClient http;

  // 2. Définir un timeout court (2000ms au lieu des 5-10s par défaut)
  http.setTimeout(2000); 

  char url[150];  // assez grand pour contenir toute l'URL
  sprintf(url, "http://api.open-meteo.com/v1/forecast?latitude=%s&longitude=%s&current=temperature_2m", LATITUDE, LONGITUDE);


  if (http.begin(url)) {
    int httpCode = http.GET();
    if (httpCode == 200) {
      String payload = http.getString();

      // pour éviter les warnings de la librairie ArduinoJson sur les anciennes versions de l'ESP32
      #pragma GCC diagnostic push
      #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
      DynamicJsonDocument doc(512);
      #pragma GCC diagnostic pop

      DeserializationError error = deserializeJson(doc, payload);
      
      if (!error) {
        float temp = doc["current"]["temperature_2m"] | NAN;
        if (!isnan(temp) && temp > -50.0 && temp < 60.0)
        {
          res = 0;
          Text = temp;
          //Serial.printf("Météo Garches : %.1f°C\n\r", Text);
          uint32_t mil = millis();
          if (mil - last_remote_Text_time > 35*60*1000) // le precedent message est vieux de plus de 35 minutes
            err_Text++;
          last_remote_Text_time = mil;
          cpt24_Text++;
          tempE_moy24h += Text;
        }
      } else {
        Serial.printf("Erreur parsing JSON Météo : %s\n\r", error.c_str());
      }
    } else {
      Serial.printf("Erreur HTTP Météo (%d) : %s\n\r", httpCode, http.errorToString(httpCode).c_str());
    }
    http.end();
  }
  return res;
}

uint8_t enreg_valeur()
{

  //Serial.printf("Envoi AAA cpt_nb_val:%d\n\r", cpt_nb_val);
  if (!esp_now_actif) return 0;

  if (cpt_nb_val >= NB_VAL_TAB) cpt_nb_val = 0;
  valTemp[cpt_nb_val] = (uint16_t)((Tint+40) * 100);
  valHum[cpt_nb_val] = (uint16_t)(Humid * 100);
  valTick[cpt_nb_val] = Tick_6s; // chaque 6 secondes
  
  cpt_nb_val++;
  uint8_t envoi=0;

  if (cpt_nb_val >= Capt_nb_val_max) envoi=1;
  if (freq_envoi)  // si frequence fixe periodique : envoi si temps total dépassé
  {
    if ((Tick_6s - envoi_6s_prec)*10 >= Capt_tps_total_max) envoi=1;
  }

  if (envoi)
  {
    //Serial.printf("Envoi BBB %d valeurs Temp-HR-Ecart\n\r", cpt_nb_val);
    Message_EspNow message;

    message.destinataire = SERVER_ADD | 0x80;  // 0x80 = message hexa
    message.emetteur = add_node;
    message.code = 'C';
    message.code2 = 'T';
    num_sequentiel++;
    message.num_seq = num_sequentiel;

    message.payload[0] = cpt_nb_val;
    
    uint8_t pos = 1;
    for (uint8_t cpt = 0; cpt < cpt_nb_val; cpt++)
    {
      message.payload[pos++] = (uint8_t) valTick[cpt];
      message.payload[pos++] = (uint8_t) (valTick[cpt] >> 8);
      message.payload[pos++] = valTick[cpt] >> 16;
      payloadWrite(message.payload, pos, valTemp[cpt]);  
      payloadWrite(message.payload, pos, valHum[cpt]);
      if (log_detail>=4)  Serial.printf("   %.2f°C %.2f%% %d min\n\r", valTemp[cpt]/100.0-40, valHum[cpt]/100.0, valTick[cpt]/10);
    }
    // Taille réelle du message envoyé
    message.longueur = 3 + pos;

    uint8_t result = envoi_data_gateway(message);   // envoi périodique des valeurs à la gateway


    if (result == ESP_OK) {
        if (log_detail>=1) Serial.printf("✅ %d valeurs Tick-Temp-HR envoyées\n\r",  message.payload[0]);
        for (uint8_t cpt = 0; cpt < message.payload[0]; cpt++)
        {
          if (log_detail>=1) Serial.printf("Time: %d min   %.2f°C %.2f%%\n\r", valTick[cpt]/10, valTemp[cpt]/100.0-40, valHum[cpt]/100.0);
        }
        cpt_nb_val=0;
        envoi_6s_prec = Tick_6s;  // mettre à jour le temps du dernier envoi
    } else {
        Serial.printf("❌ Erreur envoi ESP-NOW : %d\n\r", result);
        if (log_detail>=4) 
        {
          activation_writelog();
          writeLog('E', 8, valTemp[0]/10,  valHum[0]/10, "Esp_now");
        }
        cpt_nb_val=0;
        envoi_6s_prec = Tick_6s;  // mettre à jour le temps du dernier envoi
        return 1;
    }
    return 0;
  }
  return 0;
}

uint8_t envoi_valeur_instant(float Tint, float Humid, float HA, float vbatt)
{
  if (!esp_now_actif) return 0;

  Message_EspNow message;

  message.destinataire = SERVER_ADD | 0x80;  // 0x80 = message hexa
  message.emetteur = add_node;
  message.code = 'C';
  message.code2 = 'I';
  num_sequentiel++;
  message.num_seq = num_sequentiel;

  uint8_t pos = 0;
  payloadWrite(message.payload, pos, (uint16_t)(Tint * 100));
  payloadWrite(message.payload, pos, (uint16_t)(Humid * 100));
  payloadWrite(message.payload, pos, (uint16_t)(HA * 100));
  payloadWrite(message.payload, pos, (uint16_t)(vbatt * 100));

  // Taille réelle du message envoyé
  message.longueur = 3+pos;

  uint8_t result = envoi_data_gateway(message);   // envoi valeurs instantanées à la gateway
  return result;
}

void envoi_temp_hygro()
{
  lecture_Tint(&Tint, &Humid);
  if (log_detail>=2) Serial.printf("milli H2: %lu\n\r", millis());

  float HA = absoluteHumidity(Tint, Humid);
  float vbatt = readBatteryVoltage();
  if (log_detail>=2) Serial.printf("Temp int:%.2f Humid:%.2f HA:%.2f VBatt:%.2f\n\r", Tint, Humid, HA, vbatt); 
  envoi_valeur_instant(Tint, Humid, HA, vbatt);
}


void event_cycle()  // toutes les 15 minutes  (Power on, Timer on, inconnu)
{

  if (mode_rapide)  Tick_6s += ((uint16_t)periode_cycle)/6; // ajouter le temps correspondant 
  else  Tick_6s += (uint16_t)periode_cycle*10;

  uint8_t i;
  // chaque 5/15 minutes
  for (i = NB_VAL_TAB - 1; i; i--) {
      Nb_PI[i]= Nb_PI[i - 1];
  }    
  Nb_PI[0] = compteur_detection;
  compteur_detection=0;

  // Récupération de la température extérieure par internet
  //fetch_internet_temp();
  Text = 0;

  // lecture temp-humi
  uint8_t err_Tint = lecture_Tint(&Tint, &Humid);
  if (err_Tint)
    log_erreur(Code_erreur_Tint, err_Tint, 1);
  else
    if (log_detail>=2) Serial.printf("Temp int:%.2f Humid:%.2f\n\r", Tint, Humid); 

  float HA = absoluteHumidity(Tint, Humid);

  float tempI_moy15m, tempE_moy15m, HA_moy15m, Hum_moy15m, PIR_moy15m;
  uint8_t cpt15_Tint, cpt15_Text, cpt15_HA, cpt15_Hum, cpt15_PIR;

  tempI_moy15m += Tint;
  cpt15_Tint++;
  tempE_moy15m += Text;
  cpt15_Text++;
  HA_moy15m += HA;
  cpt15_HA++;
  Hum_moy15m += Humid;
  cpt15_Hum++;
  PIR_moy15m += compteur_detection_1h;
  cpt15_PIR++;

  // chaque 15 minutes
  compteur_graph++;
  if (compteur_graph >= skip_graph)  // 1 valeur sur x
  {
    int16_t tempI_arrondi = 200;
    int16_t tempE_arrondi = 150;
    int16_t HA_arrondi = 10;
    int16_t Hum_arrondi = 500;
    int16_t PIR_arrondi = 0;

     if (cpt15_Tint > 0) {  tempI_arrondi = round(tempI_moy15m / cpt15_Tint * 100);  }
     if (cpt15_Text > 0) {  tempE_arrondi = round(tempE_moy15m / cpt15_Text * 100); }
     if (cpt15_HA > 0) {    HA_arrondi = round(HA_moy15m / cpt15_HA * 100); }
     if (cpt15_Hum > 0) {   Hum_arrondi = round(Hum_moy15m / cpt15_Hum * 100); }
     if (cpt15_PIR > 0) {   PIR_arrondi = round(PIR_moy15m / cpt15_PIR * 10); }

    compteur_graph = 0;
    for (i = NB_Val_Graph - 1; i; i--) {
      graphique[i][0] = graphique[i - 1][0];
      graphique[i][1] = graphique[i - 1][1];
      graphique[i][2] = graphique[i - 1][2];
    }
    graphique[0][0] = tempI_arrondi; // 20°C => 2000
    graphique[0][1] = selec_graph(GRAPH2, HA_arrondi, Hum_arrondi, PIR_arrondi);
    graphique[0][2] = selec_graph(GRAPH3, HA_arrondi, Hum_arrondi, PIR_arrondi);

    if (compteur_detection_1h>1)  // au moins 2
    {
      uint8_t tot = (uint8_t)compteur_detection_1h;
      if (compteur_detection_1h > 255) tot = 255;
      writeLog('D', 1, tot, 0, "PIR 1h");
    }
    compteur_detection_1h = 0;

    if (action_envoi) 
    {
      if (freq_envoi)  // envoi periodique activé
      {
        cpt_mesure++;
        if (cpt_mesure >= freq_envoi)  // mesure toutes les x valeurs (x*skip_graph*15 minutes)
        {
          if (log_detail>=3) Serial.printf("Mesure periodique toutes les %i valeurs (cpt_mesure=%i)\n\r", freq_envoi, cpt_mesure);
          cpt_mesure=0;
          enreg_valeur();  // enreg puis envoi  par ESP-NOW
          TintPrec = Tint;
        }
      }
      else // envoi lorsque les valeurs évoluent
      {
        uint8_t enreg=0;
        float diffTint = (Tint - TintPrec)*100;  // en 0,01°C
        if (diffTint > Capt_seuil_temp) enreg=1;
        Serial.printf("Temp Int: %.2f , Temp Int Prec: %.2f , Seuil: %i\n\r", Tint, TintPrec, Capt_seuil_temp);
        if (-diffTint > Capt_seuil_temp) enreg=1;
        if ((Tick_6s - Mesure_6s_prec) > Capt_tps_max*10) enreg=1;
        Serial.printf("Tick_6s : %i, Mesure_6s_prec: %i, Capt_tps_max: %i\n\r", Tick_6s, Mesure_6s_prec, Capt_tps_max);
        if (enreg) {
          Serial.printf("Envoi  (cpt_nb_val=%i)\n\r", cpt_nb_val);
          enreg_valeur();  // enreg puis envoi  par ESP-NOW
          TintPrec = Tint;
          Mesure_6s_prec = Tick_6s;
        }
      }
    }
    tempI_moy15m = 0;
    tempE_moy15m = 0;
    HA_moy15m = 0;
    Hum_moy15m = 0;
    PIR_moy15m = 0;
    cpt15_Tint = 0;
    cpt15_Text = 0;
    cpt15_HA = 0; 
    cpt15_Hum = 0;
    cpt15_PIR = 0;
  }
  tempI_moy24h += Tint;
  cpt24_Tint++;
  tempE_moy24h += Text;
  cpt24_Text++;
  HA_moy24h += HA;
  cpt24_HA++;
  Hum_24h += Humid;
  cpt24_Hum++;
  PIR_24h += compteur_detection_1h;
  cpt24_PIR++;

  //Serial.printf("fin cycle :reveil:%i cpt:%i %i tint:%i 24h:%i\n\r", type_reveil, compteur_graph, skip_graph, graphique[0][0], compteur_24h);
  if (log_detail>=1) Serial.printf("compteur 24h:%i\n\r", compteur_24h);

  if (type_reveil != 10)      // si diff de toujours actif => compteur 24h
  {
    compteur_24h++;
    if (compteur_24h >= (uint16_t)24*60/periode_cycle) // )  // toutes les 24h
    {
      Serial.println("24h");
      activation_writelog();
      enreg_24h(1);  // et envoi si actif
      compteur_24h=0;
    }
  }
}



float readBatteryVoltage() {
  // Lecture ADC (0-4095) sur PIN_Vbatt
  // Sur ESP32 DevKit V1, l'ADC est calibré par défaut
  int raw = analogRead(PIN_Vbatt);
  float voltage = 0.0;  
  // Conversion:
  // raw / 4095.0 * 3.3V (tension ref approx) * 2 (pont diviseur) * 1.1 (facteur corection empirique souvent nécessaire sur ESP32)
  // On commence sans facteur 1.1 pour tester
  #ifdef ESP32_Fire2
     voltage = (raw / 4095.0) * 3.3 * 2.5; 
  #endif

  #ifdef ESP32_uPesy
     voltage = (raw / 4095.0) * 3.3 * 1.411 ;
  #endif

  return voltage;
}

// 1:Emission, ascii message, N° queue, Canal, Ack
// 2: +
// 3: +
// 4: +
/*3:Message ajoute au buffer RTC (taille: 14, octets utilises: 37, head: 30, tail: 67)                                                        
4:WiFi mode set to STA for ESP-NOW                                            
4:ðŸ” Esp_now canal 3)                                   
3:--- Essai canal 3 ---                     
1:STA non connectÃ© : channel actuel: 3
1:C8 42 12 43 54 03 02 58 18 9D 14 00 00 0A 5A 18 9D 14 00 00 14                                   
3: ðŸ“¥ ========== RECEPTION ESP-NOW ==========                
3:14:C1:9F:28:A5:AC                                      
4:   Canal WiFi actuel: 3                               
2:   Taille recue: 6 octets                                    
2:C2 48 03 41 00 03                             
2:Message de gateway node:H                                              
3:Ack recu                        
1:Numero de sequence Ackcorrect                            
3:âœ… Ack Recu en 57 ms                                                
3:etat_now:2        
3:Traitement buffer gateway : result=0, longueur=18
2:Messages restants dans le buffer RTC : 1
2:âœ… 1 valeurs Tick-Temp-HR envoyees         
2:Time: 3 min   22.35Â°C 52.77%               
2:Etat BTN0: 0  type_reveil:1
2:Le timer Envoi_now tourne encore => pas de veille */

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
//void OnDataRecv(const esp_now_peer_info_t * info, const uint8_t *incomingData, int len) {
  // 🔍 DIAGNOSTIC: Afficher infos de réception
  if (log_detail>=3) 
  {
    Serial.println("\n📥 ========== RECEPTION ESP-NOW OnDataRecv=======");
    Serial.printf("   Adresse MAC source: ");
    for (int i = 0; i < 6; i++) {
      Serial.printf("%02X", info->src_addr[i]);
      if (i < 5) Serial.print(":");
    }
    Serial.println();
  }
  
  // Afficher le canal WiFi actuel
  uint8_t current_channel;
  wifi_second_chan_t second;
  esp_wifi_get_channel(&current_channel, &second);
  if (log_detail>=4) Serial.printf("   Canal WiFi actuel: %d\n\r", current_channel);
  if (log_detail>=3)Serial.printf("   Taille recue: %d octets\n\r", len);
  
  if (len > sizeof(Message_EspNow)) {
    Serial.println("⚠️ message trop long");
    return;
  }
  Message_EspNow msg;
  memcpy(&msg, data, sizeof(msg));
  uint8_t renvoi_ack=0;

  if (log_detail>=3)
  {
    Serial.printf("   Donnees reçues: ");
    for (uint8_t i = 0; i < len; i++) Serial.printf("%02X ", data[i]);
    Serial.println();
  }
  if ((msg.destinataire & 0x7F) == add_node)
  {
    if (log_detail>=3) Serial.printf("Message de gateway node:%c\n\r", msg.emetteur);
    if ((msg.emetteur & 0x7F) == SERVER_ADD)
    {
      if (msg.code == 'A')
      {
        if (log_detail>=3) Serial.println("Ack recu");
        if (ackExpectedSequence == msg.num_seq)
        {
          if (log_detail>=2) Serial.println("Ack recu correct");
          ackReceived = true;
        }
        else
        {
          Serial.println("Numéro d'Ack incorrect");
        }
      }
    }
  }
}

void traitement_espnow_recv(EspNowRecvMsg_t &recv) {
  Message_EspNow &msg = recv.msg;
  int len = recv.len;
  uint8_t *src_addr = recv.src_addr;

  // 🔍 DIAGNOSTIC: Afficher infos de réception
  if (log_detail>=2) 
  {
    Serial.println("\n📥 ========== RECEPTION ESP-NOW ==========");
    for (int i = 0; i < 6; i++) {
      Serial.printf("%02X", src_addr[i]);
      if (i < 5) Serial.print(":");
    }
    Serial.println();
  }
}

static uint8_t envoi_now_legacy(uint8_t channel, esp_now_peer_info_t * peerInfo, Message_EspNow * message)
{
  uint8_t result = false;

  // Fixer le canal
  if (log_detail>=2) Serial.printf("\n--- Essai canal %d ---\n\r", channel);
  uint8_t actual_channel = 0;
  wifi_second_chan_t second;

  // Une STA connectée est obligatoirement synchronisée sur le canal du point
  // d'accès => on ne change donc pas de canal
  if (WiFi.status() != WL_CONNECTED)
  {
    esp_err_t err = esp_wifi_set_promiscuous(true);
    if (err == ESP_OK) {
      err = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
      esp_err_t promiscuous_err = esp_wifi_set_promiscuous(false);
      if (err == ESP_OK) err = promiscuous_err;
    }

    esp_err_t get_channel_err = esp_wifi_get_channel(&actual_channel, &second);
    if (err != ESP_OK || get_channel_err != ESP_OK || actual_channel != channel)
    {
      Serial.printf("⚠️ Echec changement canal (demandé:%d, actuel:%d, erreur:%s)\n\r",
                    channel, actual_channel,
                    esp_err_to_name(err != ESP_OK ? err : get_channel_err));
      return false;
    }
    
    delay(50); // Délai pour stabilisation du canal

    // Ajouter le peer sur ce canal
    if (esp_now_is_peer_exist(mac_gw)) {
      esp_now_del_peer(mac_gw);
    }
    peerInfo->channel = actual_channel; // Utiliser le canal réel
    if (esp_now_add_peer(peerInfo) != ESP_OK){
      Serial.println("❌ Échec ajout peer");
    }
    Serial.printf("STA non connecté : channel actuel: %d\n\r", actual_channel);
  }
  else {
    // Une STA connectée reste sur le canal de l'AP
    actual_channel = WiFi.channel();
    peerInfo->channel = 0;  // canal Wi-Fi courant
    Serial.printf("STA connecté : channel actuel: %d\n\r", actual_channel);
  }
  
  if (!esp_now_is_peer_exist(mac_gw))
  {
    esp_err_t err = esp_now_add_peer(peerInfo);
    if (err != ESP_OK) {
        Serial.printf("Erreur ajout peer: %s\n", esp_err_to_name(err));
        return false;
    }
  }
  
  ackReceived=0;
  ackChannel = -1;
  esp_err_t resulta = esp_now_send(mac_gw, (uint8_t *) message, message->longueur+3);

  // impression du message envoyé pour diagnostic
  if (log_detail>=2)
  {
      for (int i = 0; i < message->longueur+3; i++) {
      Serial.printf("%02X ", ((uint8_t*)message)[i]);
    }
    Serial.println();
  }

  if (resulta == ESP_OK)
  {
    //Serial.printf("Envoye sur canal %d\n\r", actual_channel);

    // attendre la réponse max 100 ms
    int wait = 0;
    while (!ackReceived && wait < 60) { // 60 * 3ms = 180ms
        delay(3);
        wait++;
    }

    if (ackReceived) // canal trouvé
    {
      result = true; 
      if (log_detail>=2) Serial.printf("✅ Ack Recu en %i ms\n\r", wait * 3);
      if (last_wifi_channel != actual_channel)
      {
        last_wifi_channel = actual_channel;
        requete_SetReg(42, last_wifi_channel, 1);  // Enreg WIFI_CHANNEL
        WIFI_CHANNEL = last_wifi_channel;  // Mettre à jour le canal WiFi valide
        Serial.printf("🔄 Mise à jour canal WiFi valide: %d\n\r", WIFI_CHANNEL);
      }
    }
  }
  else Serial.println("❌ Echec d'envoi");

  return result;
}
