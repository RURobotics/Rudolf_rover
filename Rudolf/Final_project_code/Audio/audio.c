#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/wait.h>

void play_audio(int song_key);

int main() {
    while(1) {
      printf("play siren:\n");
      play_audio(10);
      wait(NULL);
      printf("Siren end:\n");
    }
    
    

return 0;
}

//play_audio(0); - play's audio file 0. Followed by wait(NULL); if the audio should finish before process is continued.
//kill(pid, SIGKILL); To turn of audio process before it has finished.
void play_audio(int audio_key) {
    pid_t pid;
    pid = fork(); ///Create new process

    if(pid < 0) {      ///Check if succeeded
        perror("Fork failed\n");
        exit(1);
    }
    else if(pid == 0){
        ///Inside new process

        switch(audio_key){
        case 0:
        execlp("mpg123", "mpg123", "-q" ,"./sound/0_power.mp3" , NULL); /// POWER
        break;

        case 1:
        execlp("mpg123", "mpg123", "-q", "./sound/1_battery_low.mp3", NULL); /// Battery Low
        break;

        case 2:
        execlp("mpg123", "mpg123", "-q", "./sound/2_something_doing.mp3", NULL); /// Something to doing
        break;

        case 3:
        execlp("mpg123", "mpg123", "-q", "./sound/3_work.mp3", NULL); /// Work Work
        break;

        case 4:
        execlp("mpg123", "mpg123", "-q", "./sound/4_busy.mp3", NULL); ///Me busy, leave me alone! 
        break;

        case 5:
        execlp("mpg123", "mpg123", "-q", "./sound/5_happy_to.mp3", NULL); /// I be happy to
        break;

        case 6:
        execlp("mpg123", "mpg123", "-q", "./sound/6_ready_to_work.mp3", NULL); /// Ready to work
        break;

        case 7:
        execlp("mpg123", "mpg123", "-q", "./sound/7_no_time_play.mp3", NULL); /// No time to play!
        break;

        case 8:
        execlp("mpg123", "mpg123", "-q", "./sound/8_TRIPOLSKI.mp3", NULL); /// TRIPOLOWSKI
        break;

        case 9:
        execlp("mpg123", "mpg123", "-q", "./sound/9_goat.mp3", NULL); /// Goat scream
        break;

        case 10:
        execlp("mpg123", "mpg123", "-q", "./sound/10_siren.mp3", NULL); /// Goat scream
        break;

        case 11:
        execlp("mpg123", "mpg123", "-q", "./sound/11_dosimeter.mp3", NULL); /// Goat scream
        break;
        }
    }
}