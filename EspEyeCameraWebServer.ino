
#include <AsyncTelegram2.h>
#include <time.h>
#include <WiFi.h>    
#include <HTTPClient.h>
#include <UrlEncode.h>
#include <WiFiClient.h>
#include <SSLClient.h>
#include <WiFiClientSecure.h>
#include "tg_certificate.h"
#include "esp_camera.h"
#include <ArduinoJson.h>
#include <SPIFFS.h>
#define FILESYSTEM SPIFFS
#define CAMERA_MODEL_ESP_EYE 
#include "camera_pins.h"


File file;
const char filename[] = "/recording12.wav";//Select audio name
const int headerSize = 44;
TBMessage msg;
bool pendingAudio = false;
long usedSpiffsBytes = 0;

// ===========================
// Enter your WiFi credentials
// ===========================
const char* ssid = "***** Put Here Your Wifi SSID *****"; 
const char* password = "******** Put Here Your Wifi PassWord ********";

WiFiClientSecure mClient;

// ===========================
// Enter your Telegram credentials
// ===========================
const char* token = "**************** Put Here BotFather Given API Key ****************";
String chatId = "************";//Telegram User

AsyncTelegram2 myBot(mClient);
#define MYTZ "CET-1CEST,M3.5.0,M10.5.0/3"

//MIC AUDIO RECORD
#include <driver/i2s.h>
#include <SPIFFS.h>
#define I2S_WS 32
#define I2S_SD 33
#define I2S_SCK 26
#define I2S_PORT I2S_NUM_1
#define I2S_SAMPLE_RATE  16000
#define I2S_SAMPLE_BITS   (16)
#define I2S_READ_LEN     (1024*70)
#define RECORD_TIME        6 //Seconds
#define I2S_CHANNEL_NUM   (1)
#define FLASH_RECORD_SIZE (I2S_CHANNEL_NUM * I2S_SAMPLE_RATE * I2S_SAMPLE_BITS / 8 * RECORD_TIME)

//CAM RED LED
#define MEMORY_FORMAT_LED 21



size_t sendPicture(TBMessage& msg) {

  // Take picture with Camera and send to Telegram
  digitalWrite(LED_GPIO_NUM, HIGH);
  delay(100);
  camera_fb_t* fb = esp_camera_fb_get();
  digitalWrite(LED_GPIO_NUM, LOW);
  if (!fb) {
    Serial.println("Camera capture failed");
    return 0;
  }
  size_t len = fb->len;
  myBot.sendPhoto(msg, fb->buf, fb->len);

  // Clear buffer
  esp_camera_fb_return(fb);
  return len;
}

void startAudioRecord(){
  xTaskCreate(i2s_adc, "i2s_adc", 4096, NULL, 1, NULL);
}


void camaraConfig(){
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_SVGA;//800x600 Resolution
  config.pixel_format = PIXFORMAT_JPEG; // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;//CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  
  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if(config.pixel_format == PIXFORMAT_JPEG){
    if(psramFound()){
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }
#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
     return;
  }

  sensor_t * s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1); // flip it back
    s->set_brightness(s, 1); // up the brightness just a bit
    s->set_saturation(s, -2); // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  if(config.pixel_format == PIXFORMAT_JPEG){
    s->set_framesize(s, FRAMESIZE_SVGA);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif*/
}


void sendDocument(TBMessage &msg,
                  AsyncTelegram2::DocumentType fileType,
                  const char* filename,
                  const char* caption = nullptr )
  {

  File file3 = FILESYSTEM.open("/recording12.wav", "r");
  if (file3) {
    myBot.sendDocument(msg, file3, file3.size(), fileType, file3.name(), caption);
    file3.close();
  }
  else {
    Serial.println("Can't open the file. Upload \"data\" folder to filesystem");
  }
}

void setup() {

  Serial.begin(9600);
  SPIFFSInit();

  pinMode(LED_GPIO_NUM, OUTPUT);//Flash pin for photos
  pinMode(MEMORY_FORMAT_LED, OUTPUT);//Format SPIFFS LED
  
  camaraConfig();
  

  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // Sync time with NTP
  configTzTime(MYTZ, "time.google.com", "time.windows.com", "pool.ntp.org");
  mClient.setCACert(telegram_cert);

  // Set the Telegram bot properies
  myBot.setUpdateTime(2000);//Time in ms to refresh Telegram queries
  myBot.setTelegramToken(token);


  myBot.begin() ? Serial.println("OK") : Serial.println("NOK");

  Serial.println("Bot name: @" + String(myBot.getBotName()));

}


void loop() 
{
    if (myBot.getNewMessage(msg)) {
      String msgText = msg.text;
      String chat_id = String(msg.chatId);
      if (chat_id != chatId){
        myBot.sendMessage(msg, "Unauthorized user", "");
      }else{
          if (msgText.equals("/foto")) {               
            myBot.sendMessage(msg, "📸 Di Patata!");
            delay(1500); 
            sendPicture(msg);
          }else if (msgText.equals("/rec_on")) {     
            i2sInit();
            myBot.sendMessage(msg, "💽 Grabando Audio");
            startAudioRecord();    
        }
        else if (msgText.equals("/start")) {     
            String from_name = msg.sender.firstName;
            String welcome = "Bienvenido 😄👋 , " + from_name + "\n";
            welcome += "Usa los siguientes comandos para interacturar con TWL iCAM \n";
            welcome += "/foto : Saca una foto\n";
            welcome += "/rec_on : Graba un audio \n";
            myBot.sendMessage(msg, welcome);            
        }    
      }
  }
    if(pendingAudio)
    {
      pendingAudio = false;
      sendDocument(msg, AsyncTelegram2::DocumentType::VOICE, "/recording12.wav" );
      SPIFFS.end();
      SPIFFSInit();
    }  

}
void SPIFFSInit(){

  if(!SPIFFS.begin(true)){
    Serial.println("SPIFFS initialisation failed!");
  }

 if (SPIFFS.exists(filename)){
    usedSpiffsBytes += SPIFFS.usedBytes();
    Serial.println( usedSpiffsBytes);
    if(usedSpiffsBytes >= 1300000)//1799921 max size configured for SPIFFS  Tools-->Partition Scheme
    {
      myBot.sendMessage(msg, "Formateando Memoria espere unos segundos ␡🎬 ...");
      //Prevents memory corruption, memory sectors not guarantee to be erased with remove see espressif Docs
      usedSpiffsBytes = 0;
      digitalWrite(MEMORY_FORMAT_LED, HIGH);
      SPIFFS.format();
      digitalWrite(MEMORY_FORMAT_LED, LOW);
    }else{
      if(SPIFFS.remove(filename)){
        Serial.println("Fichero borrado!");
      }
    }
 }
  file = SPIFFS.open(filename, FILE_WRITE);
  if(!file){
    Serial.println("File is not available!");
  }
  byte header[headerSize];
  wavHeader(header, FLASH_RECORD_SIZE);

  file.write(header, headerSize);

}

void i2sInit(){

  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = I2S_SAMPLE_RATE,
    .bits_per_sample = i2s_bits_per_sample_t(I2S_SAMPLE_BITS),
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = 0,
    .dma_buf_count = 12,
    .dma_buf_len = 1024,
    .use_apll = 1
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);//I2S0 Used by Camera

  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_SD
  };
  i2s_set_pin(I2S_PORT, &pin_config);
}


void i2s_adc_data_scale(uint8_t * d_buff, uint8_t* s_buff, uint32_t len)
{
    uint32_t j = 0;
    uint32_t dac_value = 0;
    for (int i = 0; i < len; i += 2) {
        dac_value = ((((uint16_t) (s_buff[i + 1] & 0xf) << 8) | ((s_buff[i + 0]))));
        d_buff[j++] = 0;
        d_buff[j++] = dac_value * 256 / 2048;
    }
}

void i2s_adc(void *arg)
{

    long i2s_read_len = I2S_READ_LEN;
    long flash_wr_size = 0;
    size_t bytes_read;

    char* i2s_read_buff = (char*) calloc(i2s_read_len, sizeof(char));
    uint8_t* flash_write_buff = (uint8_t*) calloc(i2s_read_len, sizeof(char));


    Serial.println(" *** Recording Start *** ");
    while (flash_wr_size < FLASH_RECORD_SIZE) {
        //read data from I2S bus, in this case, from ADC.
        i2s_read(I2S_PORT, (void*) i2s_read_buff, i2s_read_len, &bytes_read, portMAX_DELAY);
        i2s_adc_data_scale(flash_write_buff, (uint8_t*)i2s_read_buff, i2s_read_len);
        file.write((const byte*) flash_write_buff, i2s_read_len);
        flash_wr_size += i2s_read_len;
    }
    file.close();

    free(i2s_read_buff);
    i2s_read_buff = NULL;
    free(flash_write_buff);
    flash_write_buff = NULL;
    pendingAudio = true; 

    i2s_driver_uninstall(I2S_PORT);
    vTaskDelete(NULL);
 
}


void wavHeader(byte* header, int wavSize){
  header[0] = 'R';
  header[1] = 'I';
  header[2] = 'F';
  header[3] = 'F';
  unsigned int fileSize = wavSize + headerSize - 8;
  header[4] = (byte)(fileSize & 0xFF);
  header[5] = (byte)((fileSize >> 8) & 0xFF);
  header[6] = (byte)((fileSize >> 16) & 0xFF);
  header[7] = (byte)((fileSize >> 24) & 0xFF);
  header[8] = 'W';
  header[9] = 'A';
  header[10] = 'V';
  header[11] = 'E';
  header[12] = 'f';
  header[13] = 'm';
  header[14] = 't';
  header[15] = ' ';
  header[16] = 0x10;
  header[17] = 0x00;
  header[18] = 0x00;
  header[19] = 0x00;
  header[20] = 0x01;
  header[21] = 0x00;
  header[22] = 0x01;
  header[23] = 0x00;
  header[24] = 0x80;
  header[25] = 0x3E;
  header[26] = 0x00;
  header[27] = 0x00;
  header[28] = 0x00;
  header[29] = 0x7D;
  header[30] = 0x00;
  header[31] = 0x00;
  header[32] = 0x02;
  header[33] = 0x00;
  header[34] = 0x10;
  header[35] = 0x00;
  header[36] = 'd';
  header[37] = 'a';
  header[38] = 't';
  header[39] = 'a';
  header[40] = (byte)(wavSize & 0xFF);
  header[41] = (byte)((wavSize >> 8) & 0xFF);
  header[42] = (byte)((wavSize >> 16) & 0xFF);
  header[43] = (byte)((wavSize >> 24) & 0xFF);
  
}


void listSPIFFS(void) {
  Serial.println(F("\r\nListing SPIFFS files:"));
  static const char line[] PROGMEM =  "=================================================";

  Serial.println(FPSTR(line));
  Serial.println(F("  File name  Size"));
  Serial.println(FPSTR(line));

  fs::File root = SPIFFS.open("/");
  if (!root) {
    Serial.println(F("Failed to open directory"));
    return;
  }
  if (!root.isDirectory()) {
    Serial.println(F("Not a directory"));
    return;
  }

  fs::File file2 = root.openNextFile();
  while (file2) {

    if (file2.isDirectory()) {
      Serial.print("DIR : ");
      String fileName = file2.name();
      Serial.print(fileName);
    } else {
      String fileName = file2.name();
      Serial.print("  " + fileName);
      // File path can be 31 characters maximum in SPIFFS
      int spaces = 33 - fileName.length(); // Tabulate nicely
      if (spaces < 1) spaces = 1;
      while (spaces--) Serial.print(" ");
      String fileSize = (String) file2.size();
      spaces = 10 - fileSize.length(); // Tabulate nicely
      if (spaces < 1) spaces = 1;
      while (spaces--) Serial.print(" ");
     Serial.println(fileSize + " bytes");
                          // Serial.print("Deleting ... ");
                          // String comFileName = "/"+fileName;
                          //SPIFFS.remove(comFileName);

    }

    file2 = root.openNextFile();
  }
  Serial.println(FPSTR(line));
  Serial.println();
  delay(1000);
}


