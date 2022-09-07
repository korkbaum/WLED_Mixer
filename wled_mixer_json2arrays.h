#define MAX_FX 160
#define MAX_PAL 75
#define MAX_LAMPS 4

DynamicJsonDocument doc(12288); 

// declare a bunch of global arrays to hold several relevant info per lamp from the respective json files
bool state_on [MAX_LAMPS] = {0};
int state_bri [MAX_LAMPS] = {0};
bool state_PWRon [MAX_LAMPS] = {0};
int state_PWRbri [MAX_LAMPS] = {0};
int state_col_R [MAX_LAMPS] = {0};  // to save the current solid color
int state_col_G [MAX_LAMPS] = {0};
int state_col_B [MAX_LAMPS] = {0};
int state_seg_fx [MAX_LAMPS] = {0};
int state_seg_sx [MAX_LAMPS] = {0};
int state_seg_ix [MAX_LAMPS] = {0};
int state_seg_pal [MAX_LAMPS] = {0};
const char* info_name[MAX_LAMPS] = {0};
int info_udpport [MAX_LAMPS] = {0};
int info_ws [MAX_LAMPS] = {0};
int info_fxcount [MAX_LAMPS] = {0};
int info_palcount [MAX_LAMPS] = {0};
const char* info_ip[MAX_LAMPS] = {0};
const char* fx[MAX_LAMPS][MAX_FX] = {0};
const char* pal[MAX_LAMPS][MAX_PAL] = {0};

// const char * array values have to be taken to char arrays to work
char info_name_char[MAX_LAMPS][25] = {0};
char info_ip_char[MAX_LAMPS][16] = {0};
char fx_char[MAX_LAMPS][MAX_FX][25] = {0};
char pal_char[MAX_LAMPS][MAX_PAL][25] = {0};

int fxchar_ids[MAX_LAMPS][MAX_FX] = {0};    // to retain the original fx ids after array was sorted
int palchar_ids[MAX_LAMPS][MAX_PAL] = {0};    // to retain the original pal ids after array was sorted
unsigned int color[MAX_LAMPS] = {0};                   // consolidates the 3 RGB values into one hue value 0-65535

//********************************************************************

void sort_fxchar_array(int lamp_id, int array_length) {
  int i, j;
  char temp[30] = {0};
  
  for( i = 1; i < array_length; i++ ){            // start with i=1 to skip solid color
      for( j = 1; j < array_length-1; j++ ){
          if( strcmp(fx_char[lamp_id][j], fx_char[lamp_id][j+1]) > 0 ) {
            strcpy(temp, fx_char[lamp_id][j]);
            strcpy(fx_char[lamp_id][j], fx_char[lamp_id][j+1]);
            strcpy(fx_char[lamp_id][j+1], temp);
          }
      }
  }
}

void sort_palchar_array(int lamp_id, int array_length) {
  int i, j; 
  char temp[30] = {0};
  
  for(i = 1; i < array_length; i++){                  // start with 1 to skip default
      for(j = 1; j < array_length-1; j++){
          if( strcmp(pal_char[lamp_id][j], pal_char[lamp_id][j+1]) > 0 ) {
            strcpy(temp, pal_char[lamp_id][j]);
            strcpy(pal_char[lamp_id][j], pal_char[lamp_id][j+1]);
            strcpy(pal_char[lamp_id][j+1], temp);
          }
      }
  }
}


int search_fxchar_id_array( int lamp_id, int arraysize, int findme ) {   // identify the fx id + name based on ID in sorted array
  for ( int i=0; i < arraysize; i++ ) {
    if ( fxchar_ids[lamp_id][i] == findme ) return i;    // position of correct fx id/name found
  }
  return -1;
}


int search_palchar_id_array( int lamp_id, int arraysize, int findme ) {   // identify the fx id + name based on ID in sorted array
  for ( int i=0; i < arraysize; i++ ) {
    if ( palchar_ids[lamp_id][i] == findme ) return i;    // position of correct fx id/name found
  }
  return -1;
}


void json2array(int lamp_id, bool all_data) {     // all data fills in all json data if true, if 0 only a smaller subset
 
  JsonObject state = doc["state"];
    state_on[lamp_id] = state["on"];
    state_bri[lamp_id] = state["bri"];
    state_PWRon[lamp_id] = state["PWRon"];
    state_PWRbri[lamp_id] = state["PWRbri"];

  JsonObject state_seg_0 = state["seg"][0];
    state_seg_fx[lamp_id] = state_seg_0["fx"];;
    state_seg_sx[lamp_id] = state_seg_0["sx"];
    state_seg_ix[lamp_id] = state_seg_0["ix"];
    state_seg_pal[lamp_id] = state_seg_0["pal"];

  JsonArray state_seg_0_col = state_seg_0["col"];
  JsonArray state_seg_0_col_0 = state_seg_0_col[0];
    state_col_R[lamp_id] = state_seg_0_col_0[0];
    state_col_G[lamp_id] = state_seg_0_col_0[1];
    state_col_B[lamp_id] = state_seg_0_col_0[2];

  if (all_data) {
    JsonObject info = doc["info"];
    	info_name[lamp_id] = info["name"];
      sprintf(info_name_char[lamp_id],"%s",info_name[lamp_id]);
    
      info_udpport[lamp_id] = info["udpport"];
      info_ws[lamp_id] = info["ws"];
      info_fxcount[lamp_id] = info["fxcount"];
      info_palcount[lamp_id] = info["palcount"];
    
    JsonObject info_fs = info["fs"];
     info_ip[lamp_id] = info["ip"];
     sprintf(info_ip_char[lamp_id],"%s",info_ip[lamp_id]);
    
    JsonArray effects = doc["effects"];
        
  	for (int i = 0; i < info_fxcount[lamp_id]; i++) {	
  		fx[lamp_id][i] = effects[i];
      sprintf(fx_char[lamp_id][i],"%s_%03d", fx[lamp_id][i], i);   //add the id of the effect (which is just the position in the json) to its name to easily extract the ID later 
      //Serial.printf("json2array vor sort:  fxchar_[%d][%d] = %s\r\n", lamp_id, i, fx_char[lamp_id][i]);
  	}
        // skip solid id 0 somehow
    sort_fxchar_array(lamp_id, info_fxcount[lamp_id]);            // the effect names are unsorted in the json source
    
    for (int i = 0; i < info_fxcount[lamp_id]; i++) {             // extract IDs from freshly sorted fxchar_array and store in separate array
      int fxlen = strlen(fx_char[lamp_id][i]);
      char buf[4] = {0};
      sprintf(buf, "%c%c%c", fx_char[lamp_id][i][fxlen-3], fx_char[lamp_id][i][fxlen-2], fx_char[lamp_id][i][fxlen-1] );
      fxchar_ids[lamp_id][i] = atoi(buf);
      //Serial.printf("json2array nach sort buf: %s fxchar_ids[%d][%d]: %d  fx_char[%d][%d] name: %s\r\n", buf, lamp_id, i, fxchar_ids[lamp_id][i], lamp_id, i, fx_char[lamp_id][i]);
    }
  
    JsonArray palettes = doc["palettes"];
  	for (int i = 0; i < info_palcount[lamp_id]; i++) {
 		  pal[lamp_id][i] = palettes[i];
      sprintf(pal_char[lamp_id][i],"%s_%03d", pal[lamp_id][i], i);   //add the id of the palette (which is just the position in the json) to its name to easily extract the ID later
  	}
    sort_palchar_array(lamp_id, info_palcount[lamp_id]);   // the effect names are unsorted in the json source
    
    for (int i = 0; i < info_palcount[lamp_id]; i++) {             // extract IDs from freshly sorted fxchar_array and store in separate array
      int pallen = strlen(pal_char[lamp_id][i]);
      char buf[4] = {0};
      sprintf(buf, "%c%c%c", pal_char[lamp_id][i][pallen-3], pal_char[lamp_id][i][pallen-2], pal_char[lamp_id][i][pallen-1] );
      palchar_ids[lamp_id][i] = atoi(buf);
    }
  }
}





//// complete json structure for reference or later extensions

/*
JsonObject state = doc["state"];
bool state_on = state["on"]; // true
int state_bri = state["bri"]; // 128
int state_transition = state["transition"]; // 7
int state_ps = state["ps"]; // -1
int state_pl = state["pl"]; // -1
bool state_PWRon = state["PWRon"]; // true
int state_PWRbri = state["PWRbri"]; // 16

JsonObject state_ccnf = state["ccnf"];
int state_ccnf_min = state_ccnf["min"]; // 1
int state_ccnf_max = state_ccnf["max"]; // 5
int state_ccnf_time = state_ccnf["time"]; // 12

JsonObject state_nl = state["nl"];
bool state_nl_on = state_nl["on"]; // false
int state_nl_dur = state_nl["dur"]; // 60
bool state_nl_fade = state_nl["fade"]; // true
int state_nl_mode = state_nl["mode"]; // 1
int state_nl_tbri = state_nl["tbri"]; // 0
int state_nl_rem = state_nl["rem"]; // -1

bool state_udpn_send = state["udpn"]["send"]; // false
bool state_udpn_recv = state["udpn"]["recv"]; // true

int state_lor = state["lor"]; // 0
int state_mainseg = state["mainseg"]; // 0

JsonObject state_seg_0 = state["seg"][0];
int state_seg_0_id = state_seg_0["id"]; // 0
int state_seg_0_start = state_seg_0["start"]; // 0
int state_seg_0_stop = state_seg_0["stop"]; // 30
int state_seg_0_len = state_seg_0["len"]; // 30
int state_seg_0_grp = state_seg_0["grp"]; // 1
int state_seg_0_spc = state_seg_0["spc"]; // 0
bool state_seg_0_on = state_seg_0["on"]; // true
int state_seg_0_bri = state_seg_0["bri"]; // 255

JsonArray state_seg_0_col = state_seg_0["col"];

JsonArray state_seg_0_col_0 = state_seg_0_col[0];
int state_seg_0_col_0_0 = state_seg_0_col_0[0]; // 43
int state_seg_0_col_0_1 = state_seg_0_col_0[1]; // 255
int state_seg_0_col_0_2 = state_seg_0_col_0[2]; // 0

JsonArray state_seg_0_col_1 = state_seg_0_col[1];
int state_seg_0_col_1_0 = state_seg_0_col_1[0]; // 0
int state_seg_0_col_1_1 = state_seg_0_col_1[1]; // 0
int state_seg_0_col_1_2 = state_seg_0_col_1[2]; // 0

JsonArray state_seg_0_col_2 = state_seg_0_col[2];
int state_seg_0_col_2_0 = state_seg_0_col_2[0]; // 0
int state_seg_0_col_2_1 = state_seg_0_col_2[1]; // 0
int state_seg_0_col_2_2 = state_seg_0_col_2[2]; // 0

int state_seg_0_fx = state_seg_0["fx"]; // 0
int state_seg_0_sx = state_seg_0["sx"]; // 128
int state_seg_0_ix = state_seg_0["ix"]; // 128
int state_seg_0_f1x = state_seg_0["f1x"]; // 128
int state_seg_0_f2x = state_seg_0["f2x"]; // 128
int state_seg_0_f3x = state_seg_0["f3x"]; // 128
int state_seg_0_pal = state_seg_0["pal"]; // 0
bool state_seg_0_sel = state_seg_0["sel"]; // true
bool state_seg_0_rev = state_seg_0["rev"]; // false
bool state_seg_0_mi = state_seg_0["mi"]; // false

JsonObject info = doc["info"];
const char* info_ver = info["ver"]; // "0.12.1-b1"
long info_vid = info["vid"]; // 2200005

JsonObject info_leds = info["leds"];
int info_leds_count = info_leds["count"]; // 30
bool info_leds_rgbw = info_leds["rgbw"]; // false
bool info_leds_wv = info_leds["wv"]; // false

int info_leds_pin_0 = info_leds["pin"][0]; // 16

int info_leds_pwr = info_leds["pwr"]; // 406
int info_leds_fps = info_leds["fps"]; // 3
int info_leds_maxpwr = info_leds["maxpwr"]; // 850
int info_leds_maxseg = info_leds["maxseg"]; // 16
bool info_leds_seglock = info_leds["seglock"]; // false

bool info_str = info["str"]; // false
const char* info_name = info["name"]; // "WhiteWheel"
int info_udpport = info["udpport"]; // 21324
bool info_live = info["live"]; // false
const char* info_lm = info["lm"]; // nullptr
const char* info_lip = info["lip"]; // nullptr
int info_ws = info["ws"]; // 0
int info_fxcount = info["fxcount"]; // 154
int info_palcount = info["palcount"]; // 70

JsonObject info_wifi = info["wifi"];
const char* info_wifi_bssid = info_wifi["bssid"]; // "E0:28:6D:0D:56:72"
int info_wifi_rssi = info_wifi["rssi"]; // -68
int info_wifi_signal = info_wifi["signal"]; // 64
int info_wifi_channel = info_wifi["channel"]; // 1

JsonObject info_fs = info["fs"];
int info_fs_u = info_fs["u"]; // 16
int info_fs_t = info_fs["t"]; // 1507
int info_fs_pmt = info_fs["pmt"]; // 0

int info_ndc = info["ndc"]; // -1
const char* info_arch = info["arch"]; // "esp32"
const char* info_core = info["core"]; // "v3.3.5-1-g85c43024c"
int info_lwip = info["lwip"]; // 0
long info_freeheap = info["freeheap"]; // 130332
int info_uptime = info["uptime"]; // 1043
int info_opt = info["opt"]; // 79
const char* info_brand = info["brand"]; // "WLED"
const char* info_product = info["product"]; // "FOSS"
const char* info_mac = info["mac"]; // "0cb815d88f14"
const char* info_ip = info["ip"]; // "192.168.3.29"

JsonArray effects = doc["effects"];
const char* effects_0 = effects[0]; // "Solid"
const char* effects_1 = effects[1]; // "Blink"
const char* effects_2 = effects[2]; // "Breathe"
const char* effects_3 = effects[3]; // "Wipe"
const char* effects_4 = effects[4]; // "Wipe Random"
const char* effects_5 = effects[5]; // "Random Colors"
const char* effects_6 = effects[6]; // "Sweep"
const char* effects_7 = effects[7]; // "Dynamic"
const char* effects_8 = effects[8]; // "Colorloop"
const char* effects_9 = effects[9]; // "Rainbow"
const char* effects_10 = effects[10]; // "Scan"
const char* effects_11 = effects[11]; // "Scan Dual"
const char* effects_12 = effects[12]; // "Fade"
const char* effects_13 = effects[13]; // "Theater"
const char* effects_14 = effects[14]; // "Theater Rainbow"
const char* effects_15 = effects[15]; // "Running"
const char* effects_16 = effects[16]; // "Saw"
const char* effects_17 = effects[17]; // "Twinkle"
const char* effects_18 = effects[18]; // "Dissolve"
const char* effects_19 = effects[19]; // "Dissolve Rnd"
const char* effects_20 = effects[20]; // "Sparkle"
const char* effects_21 = effects[21]; // "Sparkle Dark"
const char* effects_22 = effects[22]; // "Sparkle+"
const char* effects_23 = effects[23]; // "Strobe"
const char* effects_24 = effects[24]; // "Strobe Rainbow"
const char* effects_25 = effects[25]; // "Strobe Mega"
const char* effects_26 = effects[26]; // "Blink Rainbow"
const char* effects_27 = effects[27]; // "Android"
const char* effects_28 = effects[28]; // "Chase"
const char* effects_29 = effects[29]; // "Chase Random"
const char* effects_30 = effects[30]; // "Chase Rainbow"
const char* effects_31 = effects[31]; // "Chase Flash"
const char* effects_32 = effects[32]; // "Chase Flash Rnd"
const char* effects_33 = effects[33]; // "Rainbow Runner"
const char* effects_34 = effects[34]; // "Colorful"
const char* effects_35 = effects[35]; // "Traffic Light"
const char* effects_36 = effects[36]; // "Sweep Random"
const char* effects_37 = effects[37]; // "Running 2"
const char* effects_38 = effects[38]; // "Aurora"
const char* effects_39 = effects[39]; // "Stream"
const char* effects_40 = effects[40]; // "Scanner"
const char* effects_41 = effects[41]; // "Lighthouse"
const char* effects_42 = effects[42]; // "Fireworks"
const char* effects_43 = effects[43]; // "Rain"
const char* effects_44 = effects[44]; // "Tetrix"
const char* effects_45 = effects[45]; // "Fire Flicker"
const char* effects_46 = effects[46]; // "Gradient"
const char* effects_47 = effects[47]; // "Loading"
const char* effects_48 = effects[48]; // "Police"
const char* effects_49 = effects[49]; // "Police All"
const char* effects_50 = effects[50]; // "Two Dots"
const char* effects_51 = effects[51]; // "Two Areas"
const char* effects_52 = effects[52]; // "Running Dual"
const char* effects_53 = effects[53]; // "Halloween"
const char* effects_54 = effects[54]; // "Tri Chase"
const char* effects_55 = effects[55]; // "Tri Wipe"
const char* effects_56 = effects[56]; // "Tri Fade"
const char* effects_57 = effects[57]; // "Lightning"
const char* effects_58 = effects[58]; // "ICU"
const char* effects_59 = effects[59]; // "Multi Comet"
const char* effects_60 = effects[60]; // "Scanner Dual"
const char* effects_61 = effects[61]; // "Stream 2"
const char* effects_62 = effects[62]; // "Oscillate"
const char* effects_63 = effects[63]; // "Pride 2015"
const char* effects_64 = effects[64]; // "Juggle"
const char* effects_65 = effects[65]; // "Palette"
const char* effects_66 = effects[66]; // "Fire 2012"
const char* effects_67 = effects[67]; // "Colorwaves"
const char* effects_68 = effects[68]; // "Bpm"
const char* effects_69 = effects[69]; // "Fill Noise"
const char* effects_70 = effects[70]; // "Noise 1"
const char* effects_71 = effects[71]; // "Noise 2"
const char* effects_72 = effects[72]; // "Noise 3"
const char* effects_73 = effects[73]; // "Noise 4"
const char* effects_74 = effects[74]; // "Colortwinkles"
const char* effects_75 = effects[75]; // "Lake"
const char* effects_76 = effects[76]; // "Meteor"
const char* effects_77 = effects[77]; // "Meteor Smooth"
const char* effects_78 = effects[78]; // "Railway"
const char* effects_79 = effects[79]; // "Ripple"
const char* effects_80 = effects[80]; // "Twinklefox"
const char* effects_81 = effects[81]; // "Twinklecat"
const char* effects_82 = effects[82]; // "Halloween Eyes"
const char* effects_83 = effects[83]; // "Solid Pattern"
const char* effects_84 = effects[84]; // "Solid Pattern Tri"
const char* effects_85 = effects[85]; // "Spots"
const char* effects_86 = effects[86]; // "Spots Fade"
const char* effects_87 = effects[87]; // "Glitter"
const char* effects_88 = effects[88]; // "Candle"
const char* effects_89 = effects[89]; // "Fireworks Starburst"
const char* effects_90 = effects[90]; // "Fireworks 1D"
const char* effects_91 = effects[91]; // "Bouncing Balls"
const char* effects_92 = effects[92]; // "Sinelon"
const char* effects_93 = effects[93]; // "Sinelon Dual"
const char* effects_94 = effects[94]; // "Sinelon Rainbow"
const char* effects_95 = effects[95]; // "Popcorn"
const char* effects_96 = effects[96]; // "Drip"
const char* effects_97 = effects[97]; // "Plasma"
const char* effects_98 = effects[98]; // "Percent"
const char* effects_99 = effects[99]; // "Ripple Rainbow"
const char* effects_100 = effects[100]; // "Heartbeat"
const char* effects_101 = effects[101]; // "Pacifica"
const char* effects_102 = effects[102]; // "Candle Multi"
const char* effects_103 = effects[103]; // "Solid Glitter"
const char* effects_104 = effects[104]; // "Sunrise"
const char* effects_105 = effects[105]; // "Phased"
const char* effects_106 = effects[106]; // "Phased Noise"
const char* effects_107 = effects[107]; // "TwinkleUp"
const char* effects_108 = effects[108]; // "Noise Pal"
const char* effects_109 = effects[109]; // "Sine"
const char* effects_110 = effects[110]; // "Flow"
const char* effects_111 = effects[111]; // "Chunchun"
const char* effects_112 = effects[112]; // "Dancing Shadows"
const char* effects_113 = effects[113]; // "Washing Machine"
const char* effects_114 = effects[114]; // "Candy Cane"
const char* effects_115 = effects[115]; // "Blends"
const char* effects_116 = effects[116]; // "TV Simulator"
const char* effects_117 = effects[117]; // "Dynamic Smooth"
const char* effects_118 = effects[118]; // "* Pixels"
const char* effects_119 = effects[119]; // "* Pixelwave"
const char* effects_120 = effects[120]; // "* Juggles"
const char* effects_121 = effects[121]; // "* Matripix"
const char* effects_122 = effects[122]; // "* Gravimeter"
const char* effects_123 = effects[123]; // "* Plasmoid"
const char* effects_124 = effects[124]; // "* Puddles"
const char* effects_125 = effects[125]; // "* Midnoise"
const char* effects_126 = effects[126]; // "* Noisemeter"
const char* effects_127 = effects[127]; // "** Freqwave"
const char* effects_128 = effects[128]; // "** Freqmatrix"
const char* effects_129 = effects[129]; // "** 2D GEQ"
const char* effects_130 = effects[130]; // "** Waterfall"
const char* effects_131 = effects[131]; // "** Freqpixels"
const char* effects_132 = effects[132]; // "** Binmap"
const char* effects_133 = effects[133]; // "* Noisefire"
const char* effects_134 = effects[134]; // "* Puddlepeak"
const char* effects_135 = effects[135]; // "** Noisemove"
const char* effects_136 = effects[136]; // "2D Plasma"
const char* effects_137 = effects[137]; // "Perlin Move"
const char* effects_138 = effects[138]; // "* Ripple Peak"
const char* effects_139 = effects[139]; // "2D FireNoise"
const char* effects_140 = effects[140]; // "2D Squared Swirl"
const char* effects_141 = effects[141]; // "2D Fire2012"
const char* effects_142 = effects[142]; // "2D DNA"
const char* effects_143 = effects[143]; // "2D Matrix"
const char* effects_144 = effects[144]; // "2D Meatballs"
const char* effects_145 = effects[145]; // "** Freqmap"
const char* effects_146 = effects[146]; // "* Gravcenter"
const char* effects_147 = effects[147]; // "* Gravcentric"
const char* effects_148 = effects[148]; // "** Gravfreq"
const char* effects_149 = effects[149]; // "** DJ Light"
const char* effects_150 = effects[150]; // "** 2D Funky Plank"
const char* effects_151 = effects[151]; // "** 2D CenterBars"
const char* effects_152 = effects[152]; // "2D Julia"
const char* effects_153 = effects[153]; // "** Blurz"

JsonArray palettes = doc["palettes"];
const char* palettes_0 = palettes[0]; // "Default"
const char* palettes_1 = palettes[1]; // "* Random Cycle"
const char* palettes_2 = palettes[2]; // "* Color 1"
const char* palettes_3 = palettes[3]; // "* Colors 1&2"
const char* palettes_4 = palettes[4]; // "* Color Gradient"
const char* palettes_5 = palettes[5]; // "* Colors Only"
const char* palettes_6 = palettes[6]; // "Party"
const char* palettes_7 = palettes[7]; // "Cloud"
const char* palettes_8 = palettes[8]; // "Lava"
const char* palettes_9 = palettes[9]; // "Ocean"
const char* palettes_10 = palettes[10]; // "Forest"
const char* palettes_11 = palettes[11]; // "Rainbow"
const char* palettes_12 = palettes[12]; // "Rainbow Bands"
const char* palettes_13 = palettes[13]; // "Sunset"
const char* palettes_14 = palettes[14]; // "Rivendell"
const char* palettes_15 = palettes[15]; // "Breeze"
const char* palettes_16 = palettes[16]; // "Red & Blue"
const char* palettes_17 = palettes[17]; // "Yellowout"
const char* palettes_18 = palettes[18]; // "Analogous"
const char* palettes_19 = palettes[19]; // "Splash"
const char* palettes_20 = palettes[20]; // "Pastel"
const char* palettes_21 = palettes[21]; // "Sunset 2"
const char* palettes_22 = palettes[22]; // "Beech"
const char* palettes_23 = palettes[23]; // "Vintage"
const char* palettes_24 = palettes[24]; // "Departure"
const char* palettes_25 = palettes[25]; // "Landscape"
const char* palettes_26 = palettes[26]; // "Beach"
const char* palettes_27 = palettes[27]; // "Sherbet"
const char* palettes_28 = palettes[28]; // "Hult"
const char* palettes_29 = palettes[29]; // "Hult 64"
const char* palettes_30 = palettes[30]; // "Drywet"
const char* palettes_31 = palettes[31]; // "Jul"
const char* palettes_32 = palettes[32]; // "Grintage"
const char* palettes_33 = palettes[33]; // "Rewhi"
const char* palettes_34 = palettes[34]; // "Tertiary"
const char* palettes_35 = palettes[35]; // "Fire"
const char* palettes_36 = palettes[36]; // "Icefire"
const char* palettes_37 = palettes[37]; // "Cyane"
const char* palettes_38 = palettes[38]; // "Light Pink"
const char* palettes_39 = palettes[39]; // "Autumn"
const char* palettes_40 = palettes[40]; // "Magenta"
const char* palettes_41 = palettes[41]; // "Magred"
const char* palettes_42 = palettes[42]; // "Yelmag"
const char* palettes_43 = palettes[43]; // "Yelblu"
const char* palettes_44 = palettes[44]; // "Orange & Teal"
const char* palettes_45 = palettes[45]; // "Tiamat"
const char* palettes_46 = palettes[46]; // "April Night"
const char* palettes_47 = palettes[47]; // "Orangery"
const char* palettes_48 = palettes[48]; // "C9"
const char* palettes_49 = palettes[49]; // "Sakura"
const char* palettes_50 = palettes[50]; // "Aurora"
const char* palettes_51 = palettes[51]; // "Atlantica"
const char* palettes_52 = palettes[52]; // "C9 2"
const char* palettes_53 = palettes[53]; // "C9 New"
const char* palettes_54 = palettes[54]; // "Temperature"
const char* palettes_55 = palettes[55]; // "Aurora 2"
const char* palettes_56 = palettes[56]; // "Retro Clown"
const char* palettes_57 = palettes[57]; // "Candy"
const char* palettes_58 = palettes[58]; // "Toxy Reaf"
const char* palettes_59 = palettes[59]; // "Fairy Reaf"
const char* palettes_60 = palettes[60]; // "Semi Blue"
const char* palettes_61 = palettes[61]; // "Pink Candy"
const char* palettes_62 = palettes[62]; // "Red Reaf"
const char* palettes_63 = palettes[63]; // "Red & Flash"
const char* palettes_64 = palettes[64]; // "YBlue"
const char* palettes_65 = palettes[65]; // "Lite Light"
const char* palettes_66 = palettes[66]; // "Pink Plasma"
const char* palettes_67 = palettes[67]; // "Blink Red"
const char* palettes_68 = palettes[68]; // "Yellow 2 Blue"
const char* palettes_69 = palettes[69]; // "Yellow 2 Red"
const char* palettes_70 = palettes[70]; // "Candy2"
 
 */
