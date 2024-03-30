Sd2Card card;
SdVolume volume;
File root;
SdFile SDroot;
File myFile;

char SDmsgbuf[128];
File dataLogFile;

void SDinit(){
  sendInfo("Initializing SD card...");
  if (!card.init(SPI_HALF_SPEED, SDCSPin)) {
    sendError("card.init initialization failed.");
  } else {
    if (!SD.begin(SDCSPin)) {
    sendError("SD.begin initialization failed.");
  }
    sendInfo("SD card initialized.");
  }

  // print the type of card

  switch (card.type()) {
    case SD_CARD_TYPE_SD1:
      sendInfo("Card type SD1");
      break;
    case SD_CARD_TYPE_SD2:
      sendInfo("Card type SD2");
      break;
    case SD_CARD_TYPE_SDHC:
      sendInfo("Card type SDHC");
      break;
    default:
      sendInfo("Card type unknown");
  }

  // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
  if (!volume.init(card)) {
    sendError("Could not find FAT16/FAT32 partition.");
  }

  sprintf(SDmsgbuf, "Clusters: %u", volume.clusterCount());
  sendInfo(SDmsgbuf);

  sprintf(SDmsgbuf, "Blocks x Cluster: %u", volume.blocksPerCluster());
  sendInfo(SDmsgbuf);

  sprintf(SDmsgbuf, "Total Blocks: %u", volume.blocksPerCluster() * volume.clusterCount());
  sendInfo(SDmsgbuf);

  sprintf(SDmsgbuf, "Volume type is: FAT%u", volume.fatType());
  sendInfo(SDmsgbuf);

  uint32_t volumesize;
  volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
  volumesize *= volume.clusterCount();       // we'll have a lot of clusters
  volumesize /= 2;                           // SD card blocks are always 512 bytes (2 blocks are 1KB)
  volumesize /= 1024;
  sprintf(SDmsgbuf, "Volume size (Mb): %u", volumesize);
  sendInfo(SDmsgbuf);


  if(!SD.exists("/LOGS")){
    SD.mkdir("LOGS");
    sendInfo("Created LOGS folder.");
  }

  root = SD.open("/LOGS");
  char nextfile[20];
  sprintf(nextfile, "LOGS/%i.TXT", nextFileNo(root));
  dataLogFile = SD.open(nextfile, FILE_WRITE);

  char logstartmsg[128];
  sprintf(logstartmsg, "Log started: %s", nextfile);
  sendInfo(logstartmsg);

  //root = SD.open("/");
  //printDirectory(root);

}

void logSD(char *msg){
  if(dataLogFile){
    dataLogFile.println(msg);
    dataLogFile.flush();
  }
}

int printDirectory(File dir) {
  int filecount;
  while (true) {
    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    filecount++;
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println("/");
      printDirectory(entry);
    } else {
      // files have sizes, directories do not
      Serial.print(",");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
  }
  return filecount;
}

int nextFileNo(File dir) {
  int filecount;
  while (true) {
    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    filecount++;
    entry.close();
  }
  return filecount + 1;
}