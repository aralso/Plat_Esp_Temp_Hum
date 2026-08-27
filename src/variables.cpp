#include "../include/variables.h"

// Definition of the PARAMS array (kept in a single translation unit to avoid multiple definition linker errors)
Param PARAMS[] = {
  {"LogD", 1, U8, 0, 5, 0, 0, nullptr, &log_detail, 0},             // registre 1 : détail logs
  {"reseau", 2, U8,  11, 14, 11, 0, nullptr, &mode_reseau, 0},
  {"nb_reset", 3, U16, 0, 65535, 0, 0, nullptr, &nb_reset, 0},

  // cycle and network-related registers mapped to SetReg order numbers
  {"Rap", 5, U8, 0, 255, 0, 0, nullptr, &mode_rapide, 0},           // registre 5 : cycle rapide
  {"cycle", 6, U8, 10, 120, 15, 0, nullptr, &periode_cycle, 0},    // registre 4 : période du cycle (min)
  {"DelWS", 7, U8,  1, 30, 1, 0, nullptr, &DelaiWebsocket, 0},      // registre 6 : délai écoute websocket (s)
  {"Skip", 8, U8, 1, 50, 2,0, nullptr, &skip_graph, 0},             // registre 7 : skip graph

  // Parameters from requete_SetReg_appli
  {"SeBa", 9, U16, 1800, 4500, 3000, 0, nullptr, &Seuil_batt_sonde, 0},  // registre 9 : seuil batterie sonde (mV)
  {"FrBL", 10, U8, 1, 15, 7, 0, nullptr, &Nb_jours_Batt_log, 0},         // registre 10 : nb jours log batterie
  // 11:reglage date, 12:reglage heure, 13:reglage OTA
  {"Allu", 15, U8, 0, 1, 0, 0, nullptr, &pas_de_veille, 0},   // 0:veille 1:pas de mise en veille
  {"PVei", 16, U16, 15, 600, 30, 0, nullptr, &prolong_veille, 0}, 
          // registre 16 : duree allumage (s)

  // Application settings
  {"AcSt", 17, U8, 0, 1, 0, 0, nullptr, &action_stockage, 0},        // action stockage
  {"AcEn", 18, U8, 0, 1, 0, 0, nullptr, &action_envoi, 0},           // action envoi
  {"FrEn", 19, U8, 0, 15, 2, 0, nullptr, &freq_envoi, 0},        // frequence envoi (par mesure) 0:var, >1:fixe
  {"BooRap", 20, U8, 0, 3, 1, 0, nullptr, &boot_rapide, 0},          // registre 11 : boot rapide (0:lent 1:normal 2:rapide 3:très rapide(pas LogG-ota))



  {"latitude", 34, STR, 0, 0, 0, 0, "48.8461", &latitude, 16},
  {"longitude", 35, STR, 0, 0, 0, 0, "2.3469", &longitude, 16},


  // WiFi channel (SetReg_appli uses 41/42)
  {"Esp", 40, U8, 0, 1, 1, 0, nullptr, &esp_now_actif, 0},         // registre 40 : activation esp_now
  {"lWc", 41, U8, 0, 13, 0, 0, nullptr, &last_wifi_channel, 0},         // registre 41 : last_wifi_channel (not persisted)
  {"WifiC", 42, U8, 1, 13, 1, 0, nullptr, &WIFI_CHANNEL, 0},         // registre 42 : canal wifi preferentiel (persisted)
  // 43:puissance wifi, 44:mode wifi
  
  // IPv4 addresses stored as four bytes
  {"ipAdd", 50, IP, 0, 0xFFFFFFFFu, 192, 0, nullptr, local_ip, 4},
  {"ipGat", 51, IP, 0, 0xFFFFFFFFu, 192, 0, nullptr, gateway, 4},
  {"ipSub", 52, IP, 0, 0xFFFFFFFFu, 255, 0, nullptr, subnet, 4},          // 255.255.255.0
  {"ipDNS", 53, IP, 0, 0xFFFFFFFFu, 8, 0, nullptr, primaryDNS, 4},        // 8.8.8.8
  {"ipDNS2", 54, IP, 0, 0xFFFFFFFFu, 8, 0, nullptr, secondaryDNS, 4},     // 8.8.4.4
  {"Rout", 55, STR, 0, 0, 0, 0,  "garches", nom_routeur, 16},             // nom routeur  
  {"Mdp", 56, STR, 0, 0, 0, 0, "196492380", mdp_routeur, 25},                
  {"WSOn", 57, U8, 0, 2, 1, 0, nullptr, &websocket_on, 0},                 // 0 ou 1
  {"WSock", 58, STR, 0, 0, 0, 0, "websocket", ip_websocket, 40},          // ws://webcam.hd.free.fr:8081
  {"WSId", 59, U8, 0, 9, 9, 0,nullptr, &id_websocket, 0},                 // 1, 2, 3
  {"MacGW", 61, STR, 0, 0, 0, 0,"00:00:00:00:00:00", mac_gw_str, 20},      // adresse mac gateway

  // Variables Appli :
  {"CaTpsMx", 70, U8, 1, 240, 15, 0,nullptr, &Capt_tps_max, 0},           // tps max entre 2 mesures (minutes) 1min à 4h  
  {"CaSeuil", 71, U8, 0, 100, 30, 0,nullptr, &Capt_seuil_temp, 0},        // 0:pas de seuil(chaque lecture), 1:seuil 0,01°C, 30:0,3°C  
  {"CaNb", 72, U8, 1, 12, 2, 0,nullptr, &Capt_nb_val_max, 0},             // 0:inactif 1:à chaque lecture, 2:2val max
  {"CaTps", 73, U16, 10, 600, 300, 0,nullptr, &Capt_tps_total_max, 0},    // tps envoi max en minutes : 10min à 10h
  {"DelD", 74, U8, 1, 60, 10, 0, nullptr, &delai_detection, 0},        // registre 14 : delai entre detections (s)
  {"CalT", 75, U16, 1, 5000, 1000, 0, nullptr, &calib_temp, 0},        // registre 65 : calibration_temperature (0:pas de calibration, 1:calibration)
  {"CalH30", 76, U16, 100, 500, 300, 0, nullptr, &calib_hygro1, 0},        // registre 66 : calibration_hygro30 (0:pas de calibration, 1:calibration)
  {"CalH80", 77, U16, 600, 1000, 800, 0, nullptr, &calib_hygro2, 0},        // registre 67 : calibration_hygro80 (0:pas de calibration, 1:calibration)

};

// Provide number of entries for other translation units
const size_t PARAMS_COUNT = sizeof(PARAMS) / sizeof(PARAMS[0]);