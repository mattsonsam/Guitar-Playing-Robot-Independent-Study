#include <SdFat.h>
#include <MD_MIDIFile.h>


const uint8_t SD_SELECT = 53;

// states for the state machine
enum fsm_state { STATE_BEGIN, STATE_PROMPT, STATE_READ_FNAME, STATE_GET_TRACK, STATE_LOAD, STATE_PROCESS, STATE_CLOSE };

SdFat   SD;
MD_MIDIFile SMF;

int tempo;
int us_per_tick;
int ms_per_tick;
uint8_t track_request;

void midiCallback(midi_event *pev) {
  if (pev->track == track_request) {
    Serial.print("Track: ");
    Serial.print(pev->track);
    Serial.print(" time: ");
    Serial.print( SMF.getTickTime() / 1000);
    Serial.print(" ms ");
    if (pev->data[0] == 0x80) {
      Serial.print("Note off ");
    }
    else if (pev->data[0] == 0x90) {
      Serial.print("Note on ");
    }
    else {
      Serial.print("N/A ");
    }
    Serial.println(pev->data[1]);
  }
}


void setup() {
  Serial.begin(57600);
  Serial.println("[MIDI_File_Dumper]");


  if (!SD.begin(SD_SELECT, SPI_FULL_SPEED))
  {
    Serial.println("SD init failed!");
    while (true) ;
  }

  SMF.begin(&SD);
  SMF.setMidiHandler(midiCallback);

}

void loop(void)
{
  int  err;
  static fsm_state state = STATE_BEGIN;
  static char fname[50];

  switch (state)
  {
    case STATE_BEGIN:
    case STATE_PROMPT:
      Serial.print("\nEnter file name: ");
      state = STATE_READ_FNAME;
      break;

    case STATE_READ_FNAME:
      {
        uint8_t len = 0;
        char c;

        // read until end of line
        do
        {
          while (!Serial.available())
            ;  // wait for the next character
          c = Serial.read();
          fname[len++] = c;
        } while (c != '\n');

        // properly terminate
        --len;
        fname[len++] = '\0';

        Serial.println(fname);
        state = STATE_LOAD;
      }
      break;

    case STATE_LOAD:
      err = SMF.load(fname);
      if (err != MD_MIDIFile::E_OK)
      {
        Serial.print("SMF load Error ");
        Serial.println(err);
        state = STATE_PROMPT;
      }
      else
      {
        Serial.println("Loaded file");
        //SMF.dump();  ///-------------------this is where I can modify code to do other things to process the midi
        tempo = SMF.getTempo();
        Serial.println("the tempo is: ");
        Serial.println(tempo);

        us_per_tick = SMF.getTickTime();
        ms_per_tick = us_per_tick / 1000;
        Serial.print("milliseconds per tick is: ");
        Serial.println(ms_per_tick);
        Serial.print("Number of tracks on file: ");
        Serial.println(SMF.getTrackCount());


        //SMF.getNextEvent();  // ----- this is where I can also modify code

        state = STATE_GET_TRACK;
      }
      break;

    case STATE_GET_TRACK:
      Serial.println("Enter desired track number: ");
      while (Serial.available() == 0) {}          // wait for user input
      track_request = Serial.parseInt();                    //Read user input and hold it in a variable

 
 
      state = STATE_PROCESS;

      break;

    case STATE_PROCESS:
      if (!SMF.isEOF())
        SMF.getNextEvent();
      else
        state = STATE_CLOSE;
      break;


    case STATE_CLOSE:
      SMF.close();
      state = STATE_BEGIN;
      break;

    default:
      state = STATE_BEGIN;  // reset in case things go awry
  }
}
