Sd2Card card;
SdVolume volume;
File root;
SdFile SDroot;
File myFile;

char SDmsgbuf[128];
File dataLogFile;
char nextfile[20];

int DeletedCount = 0;
int FolderDeleteCount = 0;
int FailCount = 0;
String rootpath = "/";

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
    char logsdmsg[150];
    sprintf(logsdmsg, "%lu>%s", millis(), msg);
    dataLogFile.println(logsdmsg);
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

int dumpLogFile(int fileno) {
  dataLogFile.close(); //close current datalog file

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("LOGS/2.TXT");

  // if the file is available, write to it:
  if (dataFile) {
    Serial.print("<<<BEGIN ");
    Serial.print(fileno);
    Serial.println(".TXT>>>");
    while (dataFile.available()) {
      Serial.write(dataFile.read());
    }
    dataFile.close();
    Serial.print("<<<END ");
    Serial.print(fileno);
    Serial.println(".TXT>>>");
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }

  dataLogFile = SD.open(nextfile, FILE_WRITE);//reopen old datalog file
}

void rm(File dir, String tempPath) {
  while(true) {
    File entry =  dir.openNextFile();
    String localPath;

    if (entry) {
      if ( entry.isDirectory() )
      {
        localPath = tempPath + entry.name() + rootpath + '\0';
        char folderBuf[localPath.length()];
        localPath.toCharArray(folderBuf, localPath.length() );
        rm(entry, folderBuf);


        if( SD.rmdir( folderBuf ) )
        {
          Serial.print("Deleted folder ");
          Serial.println(folderBuf);
          FolderDeleteCount++;
        } 
        else
        {
          Serial.print("Unable to delete folder ");
          Serial.println(folderBuf);
          FailCount++;
        }
      } 
      else
      {
        localPath = tempPath + entry.name() + '\0';
        char charBuf[localPath.length()];
        localPath.toCharArray(charBuf, localPath.length() );

        if( SD.remove( charBuf ) )
        {
          Serial.print("Deleted ");
          Serial.println(localPath);
          DeletedCount++;
        } 
        else
        {
          Serial.print("Failed to delete ");
          Serial.println(localPath);
          FailCount++;
        }

      }
    } 
    else {
      // break out of recursion
      break;
    }
  }
}

void deleteAll(){
  dataLogFile.close(); //close current datalog file

  root = SD.open("/");
  delay(2000);

  rm(root, rootpath);

  if( !DeletedCount && !FailCount && !FolderDeleteCount ){

  } 
  else{
    Serial.print("Deleted ");
    Serial.print(DeletedCount);
    Serial.print(" file");
    if( DeletedCount != 1 ){
      Serial.print("s");
    }
    Serial.print(" and ");
    Serial.print(FolderDeleteCount);
    Serial.print(" folder");
    if ( FolderDeleteCount != 1 ){
      Serial.print("s");
    }
    Serial.println(" from SD card.");
    if( FailCount > 0 ){
      Serial.print("Failed to delete ");
      Serial.print(FailCount);
      Serial.print(" item");
      if( FailCount != 1 ){
        Serial.print("s");
      }
    }
  }
}